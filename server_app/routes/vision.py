# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.routes.vision
 模块职责: 视觉监控图像、目标选择、设备绑定与 MJPEG 视频流接口
--------------------------------------------------------------------------------
 提供本地视觉相关的全部 HTTP 接口:

   GET  /api/vision/image          获取视觉监控图像（base64，供前端弹窗）
   POST /api/vision/select         选择要监控的小车
   GET  /api/vision/status         查询视觉监控状态（监控目标/绑定关系/告警）
   POST /api/vision/bind/scan      触发重新扫描并绑定摄像头
   GET  /stream/mjpeg/<car_id>     某辆车的 MJPEG 实时视频流

 显示与检测解耦（关键设计）:
   MJPEG 流直接取采集线程的原始帧，自己轻量叠加 overlay（标签框/编号/误差/速度）
   后编码输出，与检测循环完全独立，检测再慢也不拖慢显示帧率。

 依赖关系:
   - 依赖 state（视觉状态与锁）、vision（摄像头管理器与叠加函数）。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import time
import threading
import base64

import numpy as np
import cv2
from flask import Blueprint, request, jsonify, Response

from .. import state
from .. import vision as vision_ops

vision_bp = Blueprint('vision', __name__)


def _capture_camera_frame(binder, cam_index, retries=5):
    """打开指定摄像头、抓取一帧并返回 frame_b64，失败返回 None。"""
    cap = binder._open_camera(cam_index)
    if not cap or not cap.isOpened():
        if cap:
            cap.release()
        return None
    try:
        for _ in range(5):
            cap.grab()
        best_frame = None
        for _ in range(retries):
            ret, frame = cap.read()
            if not ret or frame is None:
                continue
            best_frame = frame
        if best_frame is None:
            return None
        small = cv2.resize(best_frame, (640, 480))
        ok, buf = cv2.imencode('.jpg', small, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        frame_b64 = base64.b64encode(buf.tobytes()).decode('utf-8') if ok else None
        return {"frame_b64": frame_b64}
    finally:
        cap.release()


@vision_bp.route('/api/vision/image')
def get_vision_image():
    """获取视觉监控图像（用于前端弹窗）。"""
    with state.vision_lock:
        if state.vision_state["current_image"]:
            return jsonify({
                'success': True,
                'image': state.vision_state["current_image"],
                'car_id': state.vision_state["current_error_car"],
                'timestamp': state.vision_state["image_timestamp"],
                'error': state.vision_state.get("current_error"),
                'has_tag': state.vision_state.get("current_has_tag", False),
                'warning': state.vision_state.get("last_warning")
            })
        return jsonify({
            'success': False,
            'message': '暂无图像数据',
            'warning': state.vision_state.get("last_warning")
        })


@vision_bp.route('/api/vision/select', methods=['POST'])
def select_vision_car():
    """选择要监控的小车（写入 monitor_car_id）。"""
    data = request.json or {}
    car_id = data.get('car_id')
    with state.vision_lock:
        state.vision_state["monitor_car_id"] = car_id
    return jsonify({'success': True, 'car_id': car_id})


@vision_bp.route('/api/vision/status')
def get_vision_status():
    """查询视觉监控状态：监控目标、激活目标、绑定关系与告警。"""
    with state.vision_lock:
        return jsonify({
            'monitor_car_id': state.vision_state.get("monitor_car_id"),
            'active_car_id': state.vision_state.get("active_car_id"),
            'bound_cameras': state.vision_state.get("bound_cameras", {}),
            'warning': state.vision_state.get("last_warning")
        })



@vision_bp.route('/api/vision/bind/manual', methods=['POST'])
def manual_bind_vision_devices():
    """手动绑定摄像头到小车。请求体: {"binding": {"CAR1": 0, "CAR2": 1, ...}}"""
    data = request.get_json(silent=True) or {}
    binding_map = data.get('binding', {})
    if not binding_map:
        return jsonify({'success': False, 'message': '缺少绑定映射 binding'}), 400

    parsed = {}
    for car_id, cam_index in binding_map.items():
        try:
            parsed[str(car_id).upper()] = int(cam_index)
        except (ValueError, TypeError):
            return jsonify({'success': False, 'message': f'无效的绑定参数: {car_id}={cam_index}'}), 400

    with state.vision_lock:
        binder = state.vision_state.get("binder")
    if not binder:
        return jsonify({'success': False, 'message': '视觉模块未初始化'}), 500

    camera_manager = vision_ops.camera_manager
    camera_manager.stop_all_except(keep_index=-1)
    time.sleep(0.3)
    result = binder.manual_bind(parsed)
    with state.vision_lock:
        state.vision_state["bound_cameras"] = result
        state.vision_state["last_warning"] = None if result else "手动绑定失败"
    if result and binder:
        first_cam = next(iter(result.values()), None)
        if first_cam is not None:
            camera_manager.switch_to(first_cam, binder)

    return jsonify({'success': True, 'message': '手动绑定完成', 'bound_cameras': result})


@vision_bp.route('/api/vision/cameras')
def list_available_cameras():
    """列出系统中可用的摄像头索引和分辨率。"""
    with state.vision_lock:
        binder = state.vision_state.get("binder")
    if not binder:
        return jsonify({'success': False, 'cameras': []}), 500

    try:
        cameras = binder.list_available_cameras()
        return jsonify({'success': True, 'cameras': cameras})
    except Exception as e:
        return jsonify({'success': False, 'cameras': [], 'error': str(e)}), 500


@vision_bp.route('/stream/mjpeg/<car_id>')
def mjpeg_stream(car_id):
    """某辆车的 MJPEG 实时视频流；未绑定相机时回退占位画面。"""
    camera_manager = vision_ops.camera_manager

    with state.vision_lock:
        bound = dict(state.vision_state.get('bound_cameras', {}))
        binder = state.vision_state.get('binder')
    cam_index = bound.get(car_id)
    if cam_index is None:
        def _fallback_gen():
            try:
                placeholder_img = 255 * np.ones((240, 320, 3), dtype=np.uint8)
                cv2.putText(placeholder_img, 'CAMERA NOT BOUND', (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                ok, buf = cv2.imencode('.jpg', placeholder_img, [int(cv2.IMWRITE_JPEG_QUALITY), int(60)])
                jpg = buf.tobytes() if ok else b''
            except Exception:
                jpg = b''
            while True:
                if not jpg:
                    time.sleep(0.2)
                    continue
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
                time.sleep(0.5)

        return Response(_fallback_gen(), mimetype='multipart/x-mixed-replace; boundary=frame')
    q = request.args.get('q')
    try:
        quality = int(q) if q else 80
        if quality < 10:
            quality = 10
        if quality > 95:
            quality = 95
    except Exception:
        quality = 80

    # 尝试确保摄像头已启动，减少空白流的发生（串行切换，共享 USB 总线下始终只开一路）
    try:
        with state.vision_lock:
            binder = state.vision_state.get('binder')
        if binder is not None:
            camera_manager.switch_to(cam_index, binder)
    except Exception:
        pass

    # 预生成占位 JPEG（避免客户端长时间等待）
    try:
        placeholder_img = 255 * np.ones((240, 320, 3), dtype=np.uint8)
        cv2.putText(placeholder_img, 'NO FRAME', (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        ok, buf = cv2.imencode('.jpg', placeholder_img, [int(cv2.IMWRITE_JPEG_QUALITY), int(60)])
        placeholder_jpg = buf.tobytes() if ok else b''
    except Exception:
        placeholder_jpg = b''

    def gen():
        # 显示线程：直接取采集线程的原始帧，自己轻量叠加 overlay（标签框+编号+误差+速度）后编码输出。
        # 与检测循环完全解耦——检测再慢也不会拖慢这里的显示帧率（这是解决“标签入画卡顿”的关键）。
        while True:
            frame, frame_ts = camera_manager.get_frame(cam_index, timeout=0.5)
            if frame is None:
                if placeholder_jpg:
                    try:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + placeholder_jpg + b'\r\n')
                    except Exception:
                        pass
                time.sleep(0.05)
                continue

            # 仅当本相机正是当前检测目标、且 overlay 数据新鲜(1s 内)时，才叠加检测结果。
            overlay_frame = frame
            try:
                with state.vision_lock:
                    ov_cam = state.vision_state.get("overlay_cam_index")
                    ov_car = state.vision_state.get("overlay_car_id")
                    ov_corners = state.vision_state.get("overlay_corners")
                    ov_err = state.vision_state.get("overlay_error", (0.0, 0.0, 0.0))
                    ov_has_tag = state.vision_state.get("overlay_has_tag", False)
                    ov_ts = state.vision_state.get("overlay_ts", 0)
                # 即使没有 Tag，也叠加“编号+误差”文本（只在本相机是当前目标时）；有 Tag 再加框+速度。
                if ov_cam == cam_index and (time.time() - ov_ts) < 1.0:
                    overlay_frame = vision_ops.apply_vision_overlay(frame.copy(), ov_car or car_id,
                                                                    ov_corners, ov_err, ov_has_tag)
                else:
                    # 本相机不是当前检测目标：仍叠加编号，方便辨识画面属于哪辆车
                    overlay_frame = vision_ops.apply_vision_overlay(frame.copy(), car_id,
                                                                    None, (0.0, 0.0, 0.0), False)
            except Exception:
                overlay_frame = frame

            try:
                ok, buf = cv2.imencode('.jpg', overlay_frame,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
                if not ok:
                    time.sleep(0.02)
                    continue
                jpg = buf.tobytes()
            except Exception:
                time.sleep(0.02)
                continue

            try:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
            except Exception:
                time.sleep(0.02)
                continue
            time.sleep(0.03)

    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


@vision_bp.route('/api/vision/bind/manual/start', methods=['POST'])
def manual_bind_start():
    """启动手动绑定流程：枚举所有摄像头，逐个展示画面供操作人员绑定。"""
    with state.vision_lock:
        binder = state.vision_state.get("binder")
    if not binder:
        return jsonify({'success': False, 'message': '视觉模块未初始化'}), 500

    def _start_task():
        cameras = binder.list_available_cameras()
        with state.vision_lock:
            state.manual_bind_state["active"] = True
            state.manual_bind_state["cameras"] = cameras
            state.manual_bind_state["current_index"] = 0
            state.manual_bind_state["current_frame_b64"] = None
            state.manual_bind_state["bound_so_far"] = {}
            state.manual_bind_state["finished"] = False
        if cameras:
            cam = cameras[0]
            result = _capture_camera_frame(binder, cam["index"])
            with state.vision_lock:
                if result:
                    state.manual_bind_state["current_frame_b64"] = result["frame_b64"]
                else:
                    state.manual_bind_state["current_frame_b64"] = None

    threading.Thread(target=_start_task, daemon=True).start()
    return jsonify({'success': True, 'message': '手动绑定流程已启动'})


@vision_bp.route('/api/vision/bind/manual/status')
def manual_bind_status():
    """查询手动绑定流程的当前状态。"""
    with state.vision_lock:
        mbs = state.manual_bind_state
        cameras = mbs.get("cameras", [])
        cur_idx = mbs.get("current_index", 0)
        cur_cam = cameras[cur_idx] if cur_idx < len(cameras) else None
        return jsonify({
            'active': mbs.get("active", False),
            'finished': mbs.get("finished", False),
            'total_cameras': len(cameras),
            'current_index': cur_idx,
            'current_camera': cur_cam,
            'current_frame': mbs.get("current_frame_b64"),

            'bound_so_far': mbs.get("bound_so_far", {}),
        })


@vision_bp.route('/api/vision/bind/manual/confirm', methods=['POST'])
def manual_bind_confirm():
    """确认当前摄像头绑定到指定小车，绑定成功后自动切换到下一个摄像头。"""
    data = request.get_json(silent=True) or {}
    car_id = data.get('car_id', '')
    if not car_id:
        return jsonify({'success': False, 'message': '缺少 car_id'}), 400

    with state.vision_lock:
        mbs = state.manual_bind_state
        if not mbs.get("active"):
            return jsonify({'success': False, 'message': '手动绑定流程未启动'}), 400
        cameras = mbs.get("cameras", [])
        cur_idx = mbs.get("current_index", 0)
        if cur_idx >= len(cameras):
            return jsonify({'success': False, 'message': '没有更多摄像头'}), 400
        cur_cam = cameras[cur_idx]
        cam_index = cur_cam["index"]
        mbs["bound_so_far"][str(car_id).upper()] = cam_index
        next_idx = cur_idx + 1
        if next_idx >= len(cameras):
            mbs["finished"] = True
            mbs["current_index"] = next_idx
            bound_result = dict(mbs["bound_so_far"])
            binder = state.vision_state.get("binder")
        else:
            mbs["current_index"] = next_idx
            binder = state.vision_state.get("binder")

    if mbs["finished"]:
        if binder:
            camera_manager = vision_ops.camera_manager
            camera_manager.stop_all_except(keep_index=-1)
            time.sleep(0.3)
            result = binder.manual_bind(dict(mbs["bound_so_far"]))
            with state.vision_lock:
                state.vision_state["bound_cameras"] = result
                state.vision_state["last_warning"] = None if result else "手动绑定失败"
                state.manual_bind_state["active"] = False
            if result:
                first_cam = next(iter(result.values()), None)
                if first_cam is not None:
                    camera_manager.switch_to(first_cam, binder)
        return jsonify({
            'success': True,
            'message': '全部摄像头绑定完成',
            'finished': True,
            'bound_cameras': dict(mbs["bound_so_far"]),
        })

    next_cam = cameras[next_idx]
    result = _capture_camera_frame(binder, next_cam["index"])
    with state.vision_lock:
        if result:
            state.manual_bind_state["current_frame_b64"] = result["frame_b64"]
        else:
            state.manual_bind_state["current_frame_b64"] = None

    return jsonify({
        'success': True,
        'message': f'摄像头 {cam_index} 已绑定到 {car_id}，已切换到下一个',
        'finished': False,
        'current_index': next_idx,
        'total_cameras': len(cameras),
    })


@vision_bp.route('/api/vision/bind/manual/skip', methods=['POST'])
def manual_bind_skip():
    """跳过当前摄像头（不绑定），切换到下一个。"""
    with state.vision_lock:
        mbs = state.manual_bind_state
        if not mbs.get("active"):
            return jsonify({'success': False, 'message': '手动绑定流程未启动'}), 400
        cameras = mbs.get("cameras", [])
        cur_idx = mbs.get("current_index", 0)
        next_idx = cur_idx + 1
        if next_idx >= len(cameras):
            mbs["finished"] = True
            mbs["current_index"] = next_idx
            binder = state.vision_state.get("binder")
        else:
            mbs["current_index"] = next_idx
            binder = state.vision_state.get("binder")

    if mbs["finished"]:
        bound_result = dict(mbs["bound_so_far"])
        if bound_result and binder:
            camera_manager = vision_ops.camera_manager
            camera_manager.stop_all_except(keep_index=-1)
            time.sleep(0.3)
            result = binder.manual_bind(bound_result)
            with state.vision_lock:
                state.vision_state["bound_cameras"] = result
                state.vision_state["last_warning"] = None if result else "手动绑定失败"
                state.manual_bind_state["active"] = False
        else:
            with state.vision_lock:
                state.manual_bind_state["active"] = False
        return jsonify({
            'success': True,
            'message': '已跳过，全部摄像头处理完毕',
            'finished': True,
            'bound_cameras': dict(mbs["bound_so_far"]),
        })

    next_cam = cameras[next_idx]
    result = _capture_camera_frame(binder, next_cam["index"])
    with state.vision_lock:
        if result:
            state.manual_bind_state["current_frame_b64"] = result["frame_b64"]
        else:
            state.manual_bind_state["current_frame_b64"] = None

    return jsonify({
        'success': True,
        'message': '已跳过当前摄像头',
        'finished': False,
        'current_index': next_idx,
        'total_cameras': len(cameras),
    })


@vision_bp.route('/api/vision/bind/manual/refresh', methods=['POST'])
def manual_bind_refresh():
    """刷新当前摄像头的画面（重新抓帧）。"""
    with state.vision_lock:
        mbs = state.manual_bind_state
        if not mbs.get("active"):
            return jsonify({'success': False, 'message': '手动绑定流程未启动'}), 400
        cameras = mbs.get("cameras", [])
        cur_idx = mbs.get("current_index", 0)
        if cur_idx >= len(cameras):
            return jsonify({'success': False, 'message': '没有当前摄像头'}), 400
        cur_cam = cameras[cur_idx]
        binder = state.vision_state.get("binder")

    if not binder:
        return jsonify({'success': False, 'message': '视觉模块未初始化'}), 500

    result = _capture_camera_frame(binder, cur_cam["index"], retries=8)
    with state.vision_lock:
        if result:
            state.manual_bind_state["current_frame_b64"] = result["frame_b64"]
        else:
            state.manual_bind_state["current_frame_b64"] = None

    return jsonify({
        'success': True,
        'message': '画面已刷新',
        'frame': result["frame_b64"] if result else None,
    })


@vision_bp.route('/api/vision/bind/manual/cancel', methods=['POST'])
def manual_bind_cancel():
    """取消手动绑定流程。"""
    with state.vision_lock:
        state.manual_bind_state["active"] = False
        state.manual_bind_state["finished"] = False
        state.manual_bind_state["cameras"] = []
        state.manual_bind_state["current_index"] = 0
        state.manual_bind_state["current_frame_b64"] = None
        state.manual_bind_state["bound_so_far"] = {}
    return jsonify({'success': True, 'message': '手动绑定流程已取消'})


@vision_bp.route('/api/vision/channel-map')
def get_channel_map():
    """获取频道映射关系（供操作人员查看，无需查表）。"""
    with state.vision_lock:
        binder = state.vision_state.get("binder")
    if not binder:
        return jsonify({'success': False, 'message': '视觉模块未初始化'}), 500

    channel_map = binder.config.get("CHANNEL_MAP", {})
    bound_cameras = {}
    with state.vision_lock:
        bound_cameras = dict(state.vision_state.get("bound_cameras", {}))

    reverse_map = {}
    for channel, car_id in channel_map.items():
        cam_index = bound_cameras.get(car_id)
        reverse_map[channel] = {
            "car_id": car_id,
            "camera_index": cam_index,
        }

    car_to_channel = {}
    for channel, car_id in channel_map.items():
        if car_id not in car_to_channel:
            car_to_channel[car_id] = []
        car_to_channel[car_id].append(channel)

    return jsonify({
        'success': True,
        'channel_map': channel_map,
        'reverse_map': reverse_map,
        'car_to_channel': car_to_channel,
        'bound_cameras': bound_cameras,
    })
