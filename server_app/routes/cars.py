# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.routes.cars
 模块职责: 小车列表查询接口
--------------------------------------------------------------------------------
 提供 GET /api/cars，返回当前所有已知小车的状态快照。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

from flask import Blueprint, jsonify

from .. import state

cars_bp = Blueprint('cars', __name__)


@cars_bp.route('/api/cars')
def get_cars():
    """返回所有已知小车的状态快照列表。"""
    with state.car_lock:
        car_list = []
        for car_id, car in state.cars.items():
            car_list.append({
                'id': car_id,
                'mac_address': car.mac_address,
                'position': car.position,
                'heading': car.heading,
                'battery': car.battery,
                'velocity': car.velocity,
                'speed': car.speed,
                'connected': car.connected,
                'status': car.status,
                'last_update': car.last_update,
                'update_count': car.update_count,
                'connection_attempts': car.connection_attempts
            })
        return jsonify(car_list)
