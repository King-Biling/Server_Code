# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.routes.broadcast
 模块职责: 广播开关与参数设置接口
--------------------------------------------------------------------------------
 提供三个接口，控制服务器周期性广播小车状态的行为：

   POST /api/broadcast              开启 / 关闭广播
   POST /api/broadcast/interval     设置广播间隔（秒）
   POST /api/broadcast/group_size   设置广播分组大小（每组小车数）

 说明:
   broadcast_enabled / broadcast_interval / broadcast_group_size 属于运行时
   可变状态，统一存放于 state 模块。本模块通过 state.xxx 直接赋值，
   使广播循环线程能立即感知到变更。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

from flask import Blueprint, request, jsonify

from .. import state

broadcast_bp = Blueprint('broadcast', __name__)


@broadcast_bp.route('/api/broadcast', methods=['POST'])
def toggle_broadcast():
    """开启或关闭周期性广播。"""
    data = request.json
    enable = data.get('enable', True)
    state.broadcast_enabled = enable
    status = "开启" if enable else "关闭"
    print(f" 广播功能 {status}")
    return jsonify({
        'success': True,
        'message': f'广播功能已{status}',
        'broadcast_enabled': state.broadcast_enabled
    })


@broadcast_bp.route('/api/broadcast/interval', methods=['POST'])
def set_broadcast_interval():
    """设置广播间隔（秒），必须大于 0。"""
    data = request.json
    interval = data.get('interval', 0.05)
    if interval <= 0:
        return jsonify({'success': False, 'error': '间隔必须大于0'})
    state.broadcast_interval = interval
    return jsonify({
        'success': True,
        'message': f'广播间隔已更新为{interval}秒',
        'broadcast_interval': interval
    })


@broadcast_bp.route('/api/broadcast/group_size', methods=['POST'])
def set_broadcast_group_size():
    """设置广播分组大小（每组最多多少辆小车），必须大于 0。"""
    data = request.json
    group_size = data.get('group_size', 2)
    if group_size <= 0:
        return jsonify({'success': False, 'error': '分组大小必须大于0'})
    state.broadcast_group_size = group_size
    return jsonify({
        'success': True,
        'message': f'广播分组大小已更新为{group_size}辆小车',
        'broadcast_group_size': group_size
    })
