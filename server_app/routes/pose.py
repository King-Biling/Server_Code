# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.routes.pose
 模块职责: 小车位姿数据保存接口
--------------------------------------------------------------------------------
 提供 POST /api/save_pose，开启一次定时位姿采样保存会话：
   - 仅支持 10 / 20 / 30 秒三种时长；
   - 小车必须处于已连接状态；
   - 采样与落盘由 pose_save 模块的后台线程完成。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

from flask import Blueprint, request, jsonify

from .. import state
from .. import pose_save

pose_bp = Blueprint('pose', __name__)


@pose_bp.route('/api/save_pose', methods=['POST'])
def save_pose():
    """开启一次定时位姿保存会话（10/20/30 秒）。"""
    data = request.json
    car_id = data.get('car_id')
    duration = int(data.get('duration', 10))

    if not car_id:
        return jsonify({'success': False, 'error': '缺少car_id'})
    if duration not in (10, 20, 30):
        return jsonify({'success': False, 'error': '仅支持 10/20/30 秒'})

    with state.car_lock:
        if car_id not in state.cars or not state.cars[car_id].connected:
            return jsonify({'success': False, 'error': f'小车 {car_id} 未连接'})

    success, result = pose_save.start_pose_save(car_id, duration)
    if not success:
        return jsonify({'success': False, 'error': result})

    return jsonify({
        'success': True,
        'message': f'已开始保存 {car_id} 位姿数据，持续 {duration}s',
        'filename': result["filename"],
        'save_path': result["save_path"],
        'duration': duration
    })
