# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.routes.topology
 模块职责: 通信拓扑设置与查询接口
--------------------------------------------------------------------------------
 提供四个接口，用于设置/查询多车之间的通信拓扑（谁能收到谁的状态广播）:

   POST /api/topology                设置 4x4 拓扑矩阵并启用/禁用，随后广播下发
   GET  /api/topology/status         查询当前拓扑矩阵与启用状态
   POST /api/topology/toggle         仅启用/禁用拓扑过滤（不改矩阵）
   GET  /api/topology/visible/<id>   查询某辆车当前能收到哪些车的状态

 说明:
   communication_topology / topology_enabled 属于运行时可变状态，存放于 state；
   本模块通过 state.xxx 直接读写，确保广播循环线程能感知变更。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

from flask import Blueprint, request, jsonify

from .. import state
from .. import topology as topology_ops

topology_bp = Blueprint('topology', __name__)


@topology_bp.route('/api/topology', methods=['POST'])
def set_topology():
    """设置 4x4 通信拓扑矩阵并启用/禁用，随后广播下发给小车。"""
    data = request.json
    topology_matrix = data.get('topology')
    enable = data.get('enable', False)
    if topology_matrix:
        if (isinstance(topology_matrix, list) and len(topology_matrix) == 4 and
                all(isinstance(row, list) and len(row) == 4 for row in topology_matrix)):
            state.communication_topology = topology_matrix
            state.topology_enabled = enable
            topology_ops.update_topology_cache()
            topology_flat = []
            for row in state.communication_topology:
                topology_flat.extend(row)
            topology_str = ','.join(str(cell) for cell in topology_flat)
            topology_cmd = f"[T,M,{topology_str}]"
            success = state.udp_server.broadcast_global_command(topology_cmd)
            print(f" 通信拓扑已更新: {state.communication_topology}")
            print(f" 发送拓扑指令: {topology_cmd}")
            return jsonify({
                'success': True,
                'message': f'通信拓扑已{"启用" if enable else "禁用"}',
                'topology': state.communication_topology,
                'topology_enabled': state.topology_enabled,
                'broadcast_success': success,
                'topology_string': topology_str
            })
        else:
            return jsonify({'success': False, 'error': '无效的拓扑矩阵格式'})
    else:
        return jsonify({'success': False, 'error': '缺少拓扑矩阵'})


@topology_bp.route('/api/topology/status')
def get_topology_status():
    """查询当前拓扑矩阵与启用状态。"""
    return jsonify({
        'topology': state.communication_topology,
        'topology_enabled': state.topology_enabled
    })


@topology_bp.route('/api/topology/toggle', methods=['POST'])
def toggle_topology():
    """仅启用/禁用拓扑过滤（不改矩阵），并广播启用/禁用指令。"""
    data = request.json
    enable = data.get('enable', False)
    state.topology_enabled = enable
    status = "启用" if enable else "禁用"
    topology_ops.update_topology_cache()
    toggle_cmd = f"[T,E,{1 if enable else 0}]"
    broadcast_success = state.udp_server.broadcast_global_command(toggle_cmd)
    print(f" 拓扑通信 {status}")
    return jsonify({
        'success': True,
        'message': f'拓扑通信已{status}',
        'topology_enabled': state.topology_enabled,
        'broadcast_success': broadcast_success
    })


@topology_bp.route('/api/topology/visible/<car_id>')
def get_visible_cars(car_id):
    """查询某辆车当前能收到哪些车的状态广播。"""
    visible_cars = state.udp_server._get_visible_cars_for_car(car_id)
    return jsonify({
        'car_id': car_id,
        'visible_cars': visible_cars,
        'topology_enabled': state.topology_enabled
    })
