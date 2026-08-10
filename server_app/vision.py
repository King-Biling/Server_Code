# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.vision
 模块职责: 摄像头管理、视觉检测主循环、画面叠加与设备绑定
--------------------------------------------------------------------------------
 本模块承载服务器端全部“本地视觉”职责：

   CameraManager            物理摄像头的独立抓帧线程管理，缓存最新帧与 JPEG 编码，
                            并串行化共享 USB 总线上的摄像头切换
   draw_vision_overlay      在帧上绘制机器人编号 / 误差 / 制导速度文本
   apply_vision_overlay     在原始帧上轻量叠加标签框 + 文本（供 MJPEG 显示线程）
   update_vision_snapshot   生成 base64 图像快照（供不支持 MJPEG 的旧接口）
   maybe_update_snapshot_lowfreq  低频节流地更新快照，避免拖慢检测循环
   mark_waiting_image_started     标记当前待拼接车已开始上报图像
   vision_loop              视觉主循环：切换目标相机 -> 检测解算 -> 更新 overlay/制导
   设备选择/绑定辅助函数     get_available_car_ids / parse_car_selection /
                            prompt_deployed_cars / bind_vision_until_ready

 显示与检测解耦（关键设计）:
   检测线程只把“角点/误差/是否有 Tag”写入共享 overlay 状态，不做 JPEG 编码；
   MJPEG 显示线程读取后在原始采集帧上轻量绘制并编码输出。这样显示帧率与检测
   帧率彻底独立，标签入画导致的检测耗时尖峰不再拖累画面流畅度。

 依赖关系:
   - 依赖 config（快照间隔等）、state（视觉/重构状态与锁）。
   - 依赖视觉引擎 Car_vision_system（DeviceBinder / PoseEstimator / CONFIG），
     由 app 模块在装配阶段注入到 state.vision_state。
   - 制导相关调用（ensure_guide_controller_started / update_guide_controller）
     在函数内部延迟导入，打破 vision <-> reconstruct 循环依赖。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import os
import sys
import time
import base64
import threading
import traceback

import numpy as np
import cv2

from . import config
from . import state


class CameraManager:
    """管理物理摄像头的独立抓帧线程并缓存最新帧与 JPEG 编码。

    避免频繁 open/release 与重复编码；并串行化共享 USB 总线上的摄像头切换，
    保证“同一时刻只打开一路相机”，防止多路争用带宽导致黑屏/打开失败。
    """

    def __init__(self):
        self._cams = {}  # index -> {thread, cap, lock, frame, frame_ts, jpeg_b64, jpeg_bytes, jpeg_ts, running}
        self._lock = threading.Lock()
        self._switch_lock = threading.Lock()  # 串行化摄像头切换，避免共享 USB 总线上多路同时打开
        self._processed_frames = {}  # index -> {jpeg_bytes, jpeg_ts} 存放带检测叠加的处理帧

    def cache_processed_frame(self, index, frame, quality=80):
        """缓存某相机带检测叠加的处理帧（JPEG 字节）。"""
        try:
            ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
            if ok:
                bts = buf.tobytes()
                with self._lock:
                    self._processed_frames[index] = {
                        'jpeg_bytes': bts,
                        'jpeg_ts': time.time()
                    }
        except Exception:
            pass

    def get_processed_jpeg_bytes(self, index):
        """取回缓存的处理帧字节（带检测叠加），无则返回 None。"""
        with self._lock:
            if index in self._processed_frames:
                return self._processed_frames[index].get('jpeg_bytes')
        return None

    def stop_all_except(self, keep_index):
        """同步停止除 keep_index 之外的所有摄像头。

        共享 USB 总线下必须先释放旧相机再打开新相机，
        否则两路同时占用带宽会导致新相机黑屏/打开失败。
        """
        with self._lock:
            indices = list(self._cams.keys())
        for index in indices:
            if index != keep_index:
                try:
                    self.stop_camera(index)
                except Exception:
                    pass

    def switch_to(self, index, binder, cooldown=0.35, retries=3):
        """串行化的摄像头切换：先关闭其它相机 -> USB 冷却 -> 打开目标相机（带重试）。

        共享 USB 总线下这是保证“同时只开一路”的唯一安全路径。返回是否成功打开目标相机。
        """
        with self._switch_lock:
            # 目标已在运行则无需切换，仅确保其它相机已关闭
            with self._lock:
                already_running = index in self._cams and self._cams[index].get('running')
            if already_running:
                self.stop_all_except(index)
                return True

            # 先释放其它相机，给共享 USB 总线让出带宽和端点
            self.stop_all_except(index)
            if cooldown > 0:
                time.sleep(cooldown)

            for attempt in range(max(1, retries)):
                if self.start_camera(index, binder):
                    return True
                # 打开失败：再等一个冷却周期后重试
                time.sleep(cooldown)
            print(f" [CameraManager] 摄像头索引 {index} 多次打开失败")
            return False

    def start_camera(self, index, binder):
        """启动某相机的抓帧线程（已在运行则直接返回 True）。"""
        with self._lock:
            if index in self._cams and self._cams[index].get('running'):
                return True
        try:
            cap = binder._open_camera(index)
        except Exception:
            cap = None
        if not cap or not cap.isOpened():
            try:
                if cap:
                    cap.release()
            except Exception:
                pass
            return False

        lock = threading.Lock()
        cam_state = {
            'thread': None,
            'cap': cap,
            'lock': lock,
            'frame': None,
            'frame_ts': 0,
            'jpeg_b64': None,
            'jpeg_bytes': None,
            'jpeg_ts': 0,
            'running': True
        }
        with self._lock:
            self._cams[index] = cam_state

        def _capture_loop():
            c = cam_state['cap']
            try:
                c.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            # 丢掉启动后的旧帧，减少切换时的残影和陈旧画面
            for _ in range(3):
                try:
                    c.grab()
                except Exception:
                    break
            while cam_state['running']:
                try:
                    ret, frm = c.read()
                    if not ret or frm is None:
                        time.sleep(0.01)
                        continue
                    with cam_state['lock']:
                        cam_state['frame'] = frm
                        cam_state['frame_ts'] = time.time()
                        cam_state['jpeg_ts'] = 0
                except Exception:
                    time.sleep(0.05)
            try:
                c.release()
            except Exception:
                pass

        t = threading.Thread(target=_capture_loop, daemon=True)
        cam_state['thread'] = t
        t.start()
        # 不在这里停止其他摄像头；调用者可选择调用 stop_all_except 保持平滑切换
        return True

    def stop_camera(self, index):
        """停止某相机的抓帧线程并释放设备。"""
        with self._lock:
            st = self._cams.get(index)
        if not st:
            return
        st['running'] = False
        try:
            if st.get('thread'):
                st['thread'].join(timeout=0.5)
        except Exception:
            pass
        try:
            if st.get('cap'):
                st['cap'].release()
        except Exception:
            pass
        with self._lock:
            self._cams.pop(index, None)

    def get_frame(self, index, timeout=0.8):
        """获取某相机最新帧的副本与时间戳；超时返回 (None, 0)。"""
        with self._lock:
            st = self._cams.get(index)
        if not st:
            return None, 0
        deadline = time.time() + timeout
        while time.time() < deadline:
            with st['lock']:
                if st['frame'] is not None:
                    return st['frame'].copy(), st['frame_ts']
            time.sleep(0.01)
        return None, 0

    def encode_frame(self, frame, quality=80):
        """把帧编码为 (base64字符串, 原始字节)；失败返回 (None, None)。"""
        try:
            ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
            if not ok:
                return None, None
            bts = buf.tobytes()
            b64 = base64.b64encode(bts).decode('utf-8')
            return b64, bts
        except Exception:
            return None, None

    def get_jpeg_bytes(self, index, quality=80):
        """获取某相机最新帧的 JPEG 字节（带编码缓存，帧未更新则复用）。"""
        with self._lock:
            st = self._cams.get(index)
        if not st:
            return None
        with st['lock']:
            frame = st.get('frame')
            frame_ts = st.get('frame_ts', 0)
            if frame is None:
                return None
            if st.get('jpeg_ts', 0) < frame_ts or st.get('jpeg_bytes') is None:
                try:
                    ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
                    if ok:
                        st['jpeg_bytes'] = buf.tobytes()
                        st['jpeg_b64'] = None
                        st['jpeg_ts'] = frame_ts
                except Exception:
                    return None
            return st.get('jpeg_bytes')

    def get_jpeg_b64(self, index, quality=80):
        """获取某相机最新帧的 base64 JPEG（带编码缓存，帧未更新则复用）。"""
        with self._lock:
            st = self._cams.get(index)
        if not st:
            return None
        with st['lock']:
            frame = st.get('frame')
            frame_ts = st.get('frame_ts', 0)
            if frame is None:
                return None
            if st.get('jpeg_ts', 0) < frame_ts:
                try:
                    ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
                    if ok:
                        bts = buf.tobytes()
                        st['jpeg_bytes'] = bts
                        st['jpeg_b64'] = base64.b64encode(bts).decode('utf-8')
                        st['jpeg_ts'] = frame_ts
                except Exception:
                    return None
            return st.get('jpeg_b64')


# 摄像头管理器单例
camera_manager = CameraManager()


def draw_vision_overlay(frame, car_id, error_x, error_y, error_yaw):
    """在帧上绘制机器人编号、位姿误差与（若较新的）制导速度文本。"""
    if frame is None:
        return frame

    try:
        height, width = frame.shape[:2]
        robot_suffix = car_id[3:] if isinstance(car_id, str) and car_id.upper().startswith("CAR") and len(car_id) > 3 else str(car_id or "")
        robot_label = f"ROBOT-{robot_suffix}"
        pose_text = f"X:{error_x:.1f}cm Y:{error_y:.1f}cm Yaw:{error_yaw:.1f}deg"

        label_font = cv2.FONT_HERSHEY_SIMPLEX
        label_scale = 0.7
        label_thickness = 2
        pose_scale = 0.65
        pose_thickness = 2

        label_size = cv2.getTextSize(robot_label, label_font, label_scale, label_thickness)[0]
        pose_size = cv2.getTextSize(pose_text, label_font, pose_scale, pose_thickness)[0]

        label_top_left = (10, 10)
        label_bottom_right = (label_top_left[0] + label_size[0] + 18, label_top_left[1] + label_size[1] + 18)
        cv2.rectangle(frame, label_top_left, label_bottom_right, (0, 0, 0), -1)
        cv2.putText(frame, robot_label, (label_top_left[0] + 8, label_top_left[1] + label_size[1] + 6), label_font, label_scale, (0, 165, 255), label_thickness)

        pose_bottom_y = max(20 + pose_size[1], height - 12)
        pose_top_y = max(0, pose_bottom_y - pose_size[1] - 14)
        pose_bottom_right = (10 + pose_size[0] + 18, min(height, pose_bottom_y + 8))
        cv2.rectangle(frame, (10, pose_top_y), pose_bottom_right, (0, 0, 0), -1)
        cv2.putText(frame, pose_text, (18, pose_bottom_y), label_font, pose_scale, (255, 255, 255), pose_thickness)

        # 制导速度实时叠加：仅在有较新的制导指令时显示（1s 内）
        try:
            with state.reconstruct_lock:
                gv = state.reconstruct_state.get("guide_velocity", {}).get(car_id)
            if gv and (time.time() - gv.get("ts", 0) < 1.0):
                vel_text = f"VX:{gv['vx']:+.3f} VY:{gv['vy']:+.3f} VZ:{gv['vz']:+.3f} m/s"
                vel_color = (0, 255, 0) if gv.get("done") == 1 else (0, 255, 255)
                vel_scale = 0.6
                vel_thickness = 2
                vel_size = cv2.getTextSize(vel_text, label_font, vel_scale, vel_thickness)[0]
                # 放在姿势文本上方一行
                vel_bottom_y = max(0, pose_top_y - 6)
                vel_top_y = max(0, vel_bottom_y - vel_size[1] - 12)
                vel_bottom_right = (10 + vel_size[0] + 18, vel_bottom_y + 4)
                cv2.rectangle(frame, (10, vel_top_y), vel_bottom_right, (0, 0, 0), -1)
                cv2.putText(frame, vel_text, (18, vel_bottom_y - 2), label_font, vel_scale, vel_color, vel_thickness)
        except Exception:
            pass

        # 横向扫描状态指示
        try:
            with state.reconstruct_lock:
                scan_st = state.reconstruct_state.get("scan_state", {}).get(car_id)
            if scan_st and scan_st.get("active", False):
                scan_text = "SCANNING >>>" if scan_st.get("direction", 1) > 0 else "<<< SCANNING"
                scan_color = (0, 255, 255)  # yellow
                cv2.putText(frame, scan_text, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, scan_color, 2)
        except Exception:
            pass
    except Exception:
        pass

    return frame


def apply_vision_overlay(frame, car_id, corners, error_tuple, has_tag):
    """在原始采集帧上轻量叠加：标签框(若有) + 文本(机器人名/误差/制导速度)。

    该函数由 MJPEG 显示线程调用，输入为采集线程的原始帧副本，与检测循环解耦：
    检测线程只把“角点/误差/是否有 Tag”写入共享状态，这里负责把它们画出来。
    绘制成本很低(几条线 + 几段文字)，不含检测/PnP，因此显示帧率不受检测耗时影响。
    """
    if frame is None:
        return frame
    try:
        ex, ey, eyaw = (error_tuple if error_tuple and len(error_tuple) >= 3 else (0.0, 0.0, 0.0))
        # 先画标签框(绿色四边形 + 角点)，仅当本次检测确有 Tag 且角点可用
        if has_tag and corners is not None:
            try:
                pts = np.asarray(corners, dtype=np.int32).reshape(-1, 2)
                if pts.shape[0] >= 4:
                    cv2.polylines(frame, [pts[:4]], True, (0, 255, 0), 2)
                    for (px, py) in pts[:4]:
                        cv2.circle(frame, (int(px), int(py)), 3, (0, 0, 255), -1)
            except Exception:
                pass
        # 再叠加文本(机器人名/误差/速度)，复用已有实现
        frame = draw_vision_overlay(frame, car_id, ex, ey, eyaw)
    except Exception:
        pass
    return frame


def update_vision_snapshot(car_id, frame, error_tuple, has_tag):
    """把当前帧编码为 base64 并写入视觉/重构共享状态（供旧的轮询接口）。"""
    try:
        # 使用 camera_manager 的编码缓存（若可用），避免每次重复编码
        image_base64 = None
        try:
            if camera_manager:
                b64, _ = camera_manager.encode_frame(frame)
                image_base64 = b64
        except Exception:
            image_base64 = None
        if not image_base64:
            _, buffer = cv2.imencode('.jpg', frame)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
    except Exception as e:
        _log_reconstruct_event(f"图像编码失败 ({car_id}): {e}")
        return

    ts = time.time()
    with state.vision_lock:
        state.vision_state["current_image"] = image_base64
        state.vision_state["current_error"] = error_tuple
        state.vision_state["current_error_car"] = car_id
        state.vision_state["current_has_tag"] = has_tag
        state.vision_state["image_timestamp"] = ts

    with state.reconstruct_lock:
        state.reconstruct_state["current_image"] = image_base64
        state.reconstruct_state["current_image_car"] = car_id
        state.reconstruct_state["current_error"] = error_tuple
        state.reconstruct_state["current_error_car"] = car_id
        state.reconstruct_state["current_has_tag"] = has_tag
        state.reconstruct_state["image_timestamp"] = ts


def maybe_update_snapshot_lowfreq(cam_index, car_id):
    """低频(~5Hz)生成 base64 图像快照，仅供不支持 MJPEG 的旧接口
    (/api/vision/image、/api/reconstruct/image) 轮询使用。

    关键：base64 编码较重，若每帧都做会拖慢检测循环。这里从采集线程缓存里取“干净”的
    原始帧并轻量叠加 overlay 后编码，且做时间节流，避免影响检测/制导更新率。
    """
    now = time.time()
    if now - state.snapshot_lowfreq_state["last_ts"] < config.SNAPSHOT_LOWFREQ_INTERVAL:
        return
    state.snapshot_lowfreq_state["last_ts"] = now
    try:
        if cam_index is None:
            return
        frame, _ts = camera_manager.get_frame(cam_index, timeout=0.05)
        if frame is None:
            return
        # 在快照上也叠加 overlay（与 MJPEG 显示一致），随后编码
        with state.vision_lock:
            corners = state.vision_state.get("overlay_corners")
            err = state.vision_state.get("overlay_error", (0.0, 0.0, 0.0))
            has_tag = state.vision_state.get("overlay_has_tag", False)
        out = apply_vision_overlay(frame, car_id, corners, err, has_tag)
        update_vision_snapshot(car_id, out, err, has_tag)
    except Exception:
        pass


def mark_waiting_image_started(car_id):
    """标记当前待拼接车已开始上报图像，并推进 subphase。"""
    with state.reconstruct_lock:
        if state.reconstruct_state.get("phase") != "assembling":
            return
        if state.reconstruct_state.get("waiting_car_id") != car_id:
            return
        # 延迟导入：make_step_runtime 属于 reconstruct 模块
        from .reconstruct import make_step_runtime
        runtime = state.reconstruct_state.setdefault("step_runtime", {}).setdefault(car_id, make_step_runtime())
        if not runtime.get("image_started", False):
            runtime["image_started"] = True
            runtime["img_meta_ts"] = time.time()
            if runtime.get("assembling_started", False):
                state.reconstruct_state["subphase"] = "GUIDING"
            else:
                state.reconstruct_state["subphase"] = "WAIT_FIRST_IMAGE"


def _log_reconstruct_event(message):
    """转调重构事件日志（延迟导入，避免模块级循环依赖）。"""
    try:
        from .reconstruct import log_reconstruct_event
        log_reconstruct_event(message)
    except Exception:
        pass


def vision_loop():
    """视觉主循环：切换目标相机 -> 检测解算 -> 更新 overlay 与制导。

    与显示/编码彻底解耦：本循环只做检测并把“角点/误差/有无 Tag”写入共享状态，
    JPEG 编码与叠加显示由 MJPEG 线程独立完成，避免检测耗时拖累画面。
    """
    # 延迟导入：打破 vision <-> reconstruct 循环依赖
    from .reconstruct import ensure_guide_controller_started, update_guide_controller

    active_car = None
    active_cam_index = None
    last_active_car = None
    switch_time = 0.0
    empty_frame_count = 0

    while True:
        try:
            with state.reconstruct_lock:
                assembling = state.reconstruct_state.get("phase") == "assembling"
                waiting_car = state.reconstruct_state.get("waiting_car_id")

            with state.vision_lock:
                monitor_car = state.vision_state.get("monitor_car_id")
                bound_cameras = dict(state.vision_state.get("bound_cameras", {}))
                binder = state.vision_state.get("binder")
                estimator = state.vision_state.get("estimator")

            target_car = waiting_car if assembling and waiting_car else monitor_car

            if target_car != active_car:
                target_cam_index = bound_cameras.get(target_car) if target_car else None
                if binder and target_cam_index is not None:
                    # 共享 USB 总线：先释放旧相机 -> 冷却 -> 打开目标相机（串行、带重试）
                    if camera_manager.switch_to(target_cam_index, binder):
                        active_cam_index = target_cam_index
                    else:
                        # 打开失败：不提交 active_car，下一轮循环重试，避免“卡住打不开”
                        active_cam_index = None
                        with state.vision_lock:
                            state.vision_state["last_warning"] = f"摄像头打开失败: {target_car}"
                        time.sleep(0.3)
                        continue
                else:
                    active_cam_index = None

                previous_cam = active_cam_index
                active_car = target_car
                with state.vision_lock:
                    state.vision_state["active_car_id"] = active_car
                last_active_car = active_car
                switch_time = time.time()
                empty_frame_count = 0

            if not target_car or not binder or not estimator:
                time.sleep(0.05)
                continue

            cam_index = bound_cameras.get(target_car)
            if cam_index is None:
                with state.vision_lock:
                    state.vision_state["last_warning"] = f"未绑定摄像头: {target_car}"
                time.sleep(0.2)
                continue

            # 从 CameraManager 获取最新帧，避免频繁 open/release
            frame, frame_ts = camera_manager.get_frame(cam_index, timeout=0.8)
            if frame is None:
                empty_frame_count += 1
                if empty_frame_count >= 30:
                    print(" 检测到连续空帧/黑屏，摄像头可能异常，稍后重试...")
                    empty_frame_count = 0
                time.sleep(0.02)
                continue
            empty_frame_count = 0

            # 短暂 warm-up（比之前小），仍让算法稳定
            if time.time() - switch_time < 0.5:
                # 预热期：只清空 overlay（让 MJPEG 显示原始帧 + WARMING 字样），不做检测
                with state.vision_lock:
                    state.vision_state["overlay_cam_index"] = cam_index
                    state.vision_state["overlay_car_id"] = target_car
                    state.vision_state["overlay_corners"] = None
                    state.vision_state["overlay_error"] = (0.0, 0.0, 0.0)
                    state.vision_state["overlay_has_tag"] = False
                    state.vision_state["overlay_ts"] = time.time()
                time.sleep(0.02)
                continue

            # === 只做检测/解算，绝不在这里编码或叠加显示（编码交给 MJPEG 线程做）===
            # 注意：传入 frame 的副本无意义（process_frame 只读灰度），但我们不让它改动
            # 采集线程缓存的原始帧——process_frame 内部对传入 frame 会画框，因此这里传一份拷贝，
            # 保证 MJPEG 读到的原始帧保持“干净”，叠加由 MJPEG 线程独立完成。
            success, z_dist, x_offset, yaw_angle, _ = estimator.process_frame(frame.copy(), target_car)
            # 映射到小车坐标：前进误差使用 z_dist，横向误差使用 -x_offset
            error_x = float(z_dist) * 100.0 if success else 0.0  # 前进误差
            error_y = -float(x_offset) * 100.0 if success else 0.0  # 横向误差
            error_yaw = float(yaw_angle) if success else 0.0  # 角度误差

            # 把检测“数据”写入共享 overlay 状态，供 MJPEG 线程轻量绘制（不做 JPEG 编码）
            corners_copy = None
            try:
                if success and estimator.last_tag_corners is not None:
                    corners_copy = estimator.last_tag_corners.copy()
            except Exception:
                corners_copy = None
            with state.vision_lock:
                state.vision_state["overlay_cam_index"] = cam_index
                state.vision_state["overlay_car_id"] = target_car
                state.vision_state["overlay_corners"] = corners_copy
                state.vision_state["overlay_error"] = (error_x, error_y, error_yaw)
                state.vision_state["overlay_has_tag"] = bool(success)
                state.vision_state["overlay_ts"] = time.time()
                # 兼容旧的弹窗/状态查询：仍记录误差与 tag 标志（不再在这里做 base64 编码，
                # base64 快照改由低频的 maybe_update_snapshot_lowfreq 生成）
                state.vision_state["current_error"] = (error_x, error_y, error_yaw)
                state.vision_state["current_error_car"] = target_car
                state.vision_state["current_has_tag"] = bool(success)
                state.vision_state["image_timestamp"] = time.time()

            mark_waiting_image_started(target_car)

            # 低频生成 base64 快照（供不支持 MJPEG 的旧弹窗/接口），避免每帧重复编码拖慢检测
            maybe_update_snapshot_lowfreq(cam_index, target_car)

            if assembling and waiting_car == target_car:
                ensure_guide_controller_started(target_car)
                if success:
                    # 真正检测到 AprilTag，误差可信，可作为到达/前进依据
                    update_guide_controller(target_car, (error_x, error_y, error_yaw), has_tag=True)
                else:
                    # 无Tag：仅保活刷新，绝不能作为到达依据（否则会误判 DONE=1）
                    update_guide_controller(target_car, (0.0, 0.0, 0.0), has_tag=False)

            # 检测循环节流：制导需要足够更新率，但不必跑满 CPU。20~30ms 一轮即可。
            time.sleep(0.02)

        except Exception as e:
            _log_reconstruct_event(f"视觉线程错误: {e}")
            try:
                print("视觉线程异常堆栈:\n" + traceback.format_exc())
            except Exception:
                pass
            time.sleep(0.1)


# ------------------------------------------------------------------------------
# 设备选择与绑定辅助
# ------------------------------------------------------------------------------
def get_available_car_ids():
    """从视觉配置的 CHANNEL_MAP 中提取可用小车编号（大写、去重、排序）。"""
    # 视觉配置在 app 装配阶段注入到 state.vision_state["vision_config"]
    vision_config = state.vision_state.get("vision_config", {}) or {}
    channel_map = vision_config.get("CHANNEL_MAP", {})
    car_ids = sorted({str(v).upper() for v in channel_map.values()})
    return car_ids


def parse_car_selection(raw_input, available_ids):
    """把用户输入（如 "CAR2,CAR3" 或 "all"）解析为规范化的小车编号列表。"""
    if not raw_input:
        return []
    text = raw_input.strip().upper()
    if text in {"ALL", "*"}:
        return list(available_ids)

    tokens = []
    normalized = text.replace(";", ",").replace(" ", ",")
    for part in normalized.split(","):
        item = part.strip()
        if not item:
            continue
        if item.isdigit():
            tokens.append(f"CAR{item}")
        else:
            tokens.append(item)

    selected = []
    for token in tokens:
        if not token.startswith("CAR"):
            token = f"CAR{token}"
        selected.append(token)

    normalized_list = []
    for car_id in selected:
        if car_id in available_ids:
            normalized_list.append(car_id)
    return sorted(set(normalized_list))


def prompt_deployed_cars(available_ids):
    """交互式提示用户输入本次部署的小车编号；非交互环境按需默认全绑。"""
    if not available_ids:
        print(" 未发现可用车辆配置，跳过交互选择")
        return []
    force_prompt = config.FORCE_VISION_PROMPT_SOFT or os.getenv("FORCE_VISION_PROMPT") == "1"
    if not sys.stdin.isatty() and not force_prompt:
        print(" 非交互环境，默认绑定全部车辆")
        return list(available_ids)
    if force_prompt:
        print(" FORCE_VISION_PROMPT=1，强制启用交互输入")

    tips = ", ".join(available_ids)
    print("\n请输入本次部署的小车编号，使用逗号分隔。")
    print(f"可选: {tips}，例如: CAR2,CAR3 (或输入 all)")
    while True:
        try:
            raw_input_text = input("部署车辆> ").strip()
        except EOFError:
            print(" 读取输入失败，默认绑定全部车辆")
            return list(available_ids)

        selected = parse_car_selection(raw_input_text, available_ids)
        if selected:
            return selected
        print(" 输入无效，请重新输入。")


def bind_vision_until_ready(binder, estimator, selected_cars):
    """将 binder/estimator 写入视觉状态，不再自动扫描绑定（改为手动绑定）。"""
    with state.vision_lock:
        state.vision_state["binder"] = binder
        state.vision_state["estimator"] = estimator
        state.vision_state["bound_cameras"] = {}
        state.vision_state["last_warning"] = "请在控制面板中手动绑定摄像头"
    return True
