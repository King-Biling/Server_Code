# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.routes.control
 模块职责: 单车位置控制与实时速度遥控接口
--------------------------------------------------------------------------------
 提供两个接口，用于对单辆小车下发运动指令：

   POST /api/control_position   下发导航指令 [C,CARn,x,y,yaw]（可靠重发）
   POST /api/control_velocity   下发实时速度指令 [M,CARn,vx,vy,vz]（低延迟单发）

 安全拦截（与原实现一致）:
   - 编队执行中禁止单车独立位置控制；实时遥控仅允许操作领航者。
   - 矩形轨迹实验进行中，禁止单车位置控制与遥控。
   编队/矩形实验状态通过 formation_controller 的查询函数获取。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

from flask import Blueprint, request, jsonify

from .. import state
from formation_controller import get_formation_info, get_rectangle_experiment_info

control_bp = Blueprint('control', __name__)


@control_bp.route('/api/control_position', methods=['POST'])
def control_car_position():
    """下发单车导航（目标位置）指令，编队/矩形实验期间拒绝。"""
    # 核心拦截逻辑：如果在编队执行中，严禁独立位置控制
    formation_info = get_formation_info()
    if formation_info.get('enabled'):
        return jsonify({'success': False, 'error': '编队执行中，已锁定单车独立控制！'})

    rect_info = get_rectangle_experiment_info()
    if rect_info.get('active'):
        return jsonify({'success': False, 'error': '矩形轨迹实验进行中，已锁定单车位置控制！'})

    data = request.json
    car_id = data.get('car_id')
    position = data.get('position')
    heading = data.get('heading', 0)
    if not car_id or not position:
        return jsonify({'success': False, 'error': '缺少参数'})
    cmd_str = f"[C,{car_id},{position.get('x', 0):.2f},{position.get('y', 0):.2f},{heading:.1f}]"
    success = state.udp_server.send_to_car_reliable(car_id, cmd_str, max_retries=4)
    if success:
        return jsonify({'success': True, 'message': f'导航指令已发送到小车 {car_id}'})
    else:
        return jsonify({'success': False, 'error': f'小车 {car_id} 未连接'})


@control_bp.route('/api/control_velocity', methods=['POST'])
def control_car_velocity():
    """实时速度控制接口（用于键盘遥控），编队时仅允许遥控领航者。"""
    data = request.json
    car_id = data.get('car_id')

    # 核心拦截逻辑：如果在编队中，且控制对象不是领航者，则拒绝遥控
    formation_info = get_formation_info()
    if formation_info.get('enabled') and car_id != formation_info.get('leader'):
        return jsonify({'success': False, 'error': '编队执行中，仅允许遥控领航者！'})

    rect_info = get_rectangle_experiment_info()
    if rect_info.get('active'):
        return jsonify({'success': False, 'error': '矩形轨迹实验进行中，已锁定遥控！'})

    vx = data.get('vx', 0.0)
    vy = data.get('vy', 0.0)
    vz = data.get('vz', 0.0)

    if not car_id:
        return jsonify({'success': False, 'error': '缺少car_id'})

    # 构建实时运动指令: [M,CAR1,vx,vy,vz]
    cmd_str = f"[M,{car_id},{vx:.3f},{vy:.3f},{vz:.3f}]"

    # 低延迟遥控，不重传，直接单发
    success = state.udp_server.send_to_car(car_id, cmd_str)
    return jsonify({'success': success})
