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

import numpy as np
import cv2
from flask import Blueprint, request, jsonify, Response

from .. import state
from .. import vision as vision_ops

vision_bp = Blueprint('vision', __name__)


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


@vision_bp.route('/api/vision/bind/scan', methods=['POST'])
def scan_and_bind_vision_devices():
    """后台线程触发重新扫描并绑定摄像头。"""
    def _scan_task():
        with state.vision_lock:
            binder = state.vision_state.get("binder")
        if not binder:
            return
        camera_manager = vision_ops.camera_manager
        camera_manager.stop_all_except(keep_index=-1)
        time.sleep(0.3)
        mapping = binder.scan_and_bind()
        with state.vision_lock:
            state.vision_state["bound_cameras"] = mapping
            state.vision_state["last_warning"] = None if mapping else "未绑定到任何摄像头"
        if mapping and binder:
            first_cam = next(iter(mapping.values()), None)
            if first_cam is not None:
                camera_manager.switch_to(first_cam, binder)

    threading.Thread(target=_scan_task, daemon=True).start()
    return jsonify({'success': True, 'message': '已开始重新扫描摄像头'})


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
