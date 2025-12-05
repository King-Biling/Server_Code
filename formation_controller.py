import json
import time
from flask import Blueprint, request, jsonify
formation_bp = Blueprint('formation', __name__)
formation_enabled = False
formation_leader = None
formation_type = "line"  # line, Diamond, square, custom
formation_params = {}
cars_dict = {} 
udp_server = None 
FORMATION_CONFIGS = {
    "line": {
        "CAR1": {"x": 0, "y": 0, "yaw": 0},
        "CAR2": {"x": -0.7, "y": 0, "yaw": 0},
        "CAR3": {"x": -1.4, "y": 0, "yaw": 0},
        "CAR4": {"x": -2.1, "y": 0, "yaw": 0}
    },
    "Diamond": {
        "CAR1": {"x": 0, "y": 0, "yaw": 0},
        "CAR2": {"x": -0.7, "y": -0.7, "yaw": 0},
        "CAR3": {"x": -0.7, "y": 0.7, "yaw": 0},
        "CAR4": {"x": -1.4, "y": 0, "yaw": 0}
    },
    "square": {
        "CAR1": {"x": 0, "y": 0, "yaw": 0},
        "CAR2": {"x": 0, "y": -0.7, "yaw": 0},
        "CAR3": {"x": -0.7, "y": -0.7, "yaw": 0},
        "CAR4": {"x": -0.7, "y": 0, "yaw": 0}
    }
}
def init_formation_controller(cars, server):
    global cars_dict, udp_server
    cars_dict = cars
    udp_server = server
    print("🔧 编队控制器初始化完成")
def send_formation_command(car_id, command):
    if udp_server:
        return udp_server.send_to_car_reliable(car_id, command, max_retries=4)
    else:
        print(f"❌ UDP服务器未初始化，无法发送指令给 {car_id}")
        return False
@formation_bp.route('/api/formation/start', methods=['POST'])
def start_formation():
    global formation_enabled, formation_leader, formation_type
    data = request.json
    leader_id = data.get('leader_id')
    formation_type = data.get('formation_type', 'line')
    if not leader_id:
        return jsonify({'success': False, 'error': '需要指定领航者'})
    if leader_id not in cars_dict or not cars_dict[leader_id].connected:
        return jsonify({'success': False, 'error': f'领航者 {leader_id} 未连接'})
    print(f"🚀 启动编队控制 - 领航者: {leader_id}, 队形: {formation_type}")
    if formation_type in FORMATION_CONFIGS:
        formation_offsets = FORMATION_CONFIGS[formation_type]
    else:
        formation_offsets = FORMATION_CONFIGS["line"]
    old_leader = formation_leader
    formation_leader = leader_id
    formation_enabled = True
    print(f"🎯 直接启动编队，不发送停止指令")
    success_count = 0
    total_cars = 0
    for car_id in cars_dict:
        if not cars_dict[car_id].connected:
            continue
        total_cars += 1
        if car_id == leader_id:
            start_cmd = f"[F,S,{leader_id},{formation_type}]"
            leader_role_cmd = f"[F,L,{car_id}]"
            if send_formation_command(car_id, start_cmd):
                print(f"🎯 向领航者 {car_id} 发送开始指令: {start_cmd}")
                if send_formation_command(car_id, leader_role_cmd):
                    print(f"🎯 向领航者 {car_id} 发送角色指令: {leader_role_cmd}")
                    success_count += 1
        else:
            start_cmd = f"[F,S,{leader_id},{formation_type}]"
            offset = formation_offsets.get(car_id, {"x": 0, "y": 0, "yaw": 0})
            follower_cmd = f"[F,F,{leader_id},{offset['x']},{offset['y']},{offset['yaw']}]"
            if send_formation_command(car_id, start_cmd):
                print(f"🎯 向跟随者 {car_id} 发送开始指令: {start_cmd}")
                if send_formation_command(car_id, follower_cmd):
                    print(f"🎯 向跟随者 {car_id} 发送偏移指令: {follower_cmd}")
                    success_count += 1
    if old_leader and old_leader != leader_id and old_leader in cars_dict:
        if cars_dict[old_leader].connected:
            start_cmd = f"[F,S,{leader_id},{formation_type}]"
            offset = formation_offsets.get(car_id, {"x": 0, "y": 0, "yaw": 0})
            follower_cmd = f"[F,F,{leader_id},{offset['x']},{offset['y']},{offset['yaw']}]"
            if send_formation_command(old_leader, start_cmd) and send_formation_command(old_leader, follower_cmd):
                print(f"🔄 原领航者 {old_leader} 转换为跟随者")
    unicast_success_rate = (success_count / total_cars * 100) if total_cars > 0 else 0
    return jsonify({
        'success': True,
        'message': f'编队控制已启动 - 领航者: {leader_id}, 队形: {formation_type}',
        'formation_leader': formation_leader,
        'formation_type': formation_type,
        'formation_offsets': formation_offsets,
        'unicast_success_count': success_count,
        'total_cars': total_cars,
        'success_rate': f'{unicast_success_rate:.1f}%'
    })
@formation_bp.route('/api/formation/stop', methods=['POST'])
def stop_formation():
    global formation_enabled
    stop_cmd = "[F,T]"
    success_count = 0
    total_cars = 0
    for car_id in cars_dict:
        if cars_dict[car_id].connected:
            total_cars += 1
            if send_formation_command(car_id, stop_cmd):
                success_count += 1
    formation_enabled = False
    unicast_success_rate = (success_count / total_cars * 100) if total_cars > 0 else 0
    print(f"🛑 编队控制已停止，单播发送停止指令: {success_count}/{total_cars} 成功")
    return jsonify({
        'success': True,
        'message': '编队控制已停止',
        'unicast_success_count': success_count,
        'total_cars': total_cars,
        'success_rate': f'{unicast_success_rate:.1f}%'
    })
@formation_bp.route('/api/formation/status')
def get_formation_status():
    return jsonify({
        'formation_enabled': formation_enabled,
        'formation_leader': formation_leader,
        'formation_type': formation_type
    })
@formation_bp.route('/api/formation/custom', methods=['POST'])
def set_custom_formation():
    global formation_enabled, formation_leader
    data = request.json
    custom_offsets = data.get('offsets', {})
    leader_id = data.get('leader_id')
    if not custom_offsets or not leader_id:
        return jsonify({'success': False, 'error': '需要提供领航者ID和编队偏移量'})
    if leader_id not in cars_dict or not cars_dict[leader_id].connected:
        return jsonify({'success': False, 'error': f'领航者 {leader_id} 未连接'})
    formation_leader = leader_id
    formation_enabled = True
    print(f"🔧 设置自定义编队 - 领航者: {leader_id}, 偏移量: {custom_offsets}")
    success_count = 0
    total_cars = 0
    for car_id in cars_dict:
        if not cars_dict[car_id].connected:
            continue
        total_cars += 1
        if car_id == leader_id:
            start_cmd = f"FORMATION:CUSTOM,{leader_id}"
            leader_cmd = "FORMATION:LEADER,CUSTOM"
            if send_formation_command(car_id, start_cmd) and send_formation_command(car_id, leader_cmd):
                success_count += 1
        else:
            start_cmd = f"FORMATION:CUSTOM,{leader_id}"
            offset = custom_offsets.get(car_id, {"x": 0, "y": 0, "yaw": 0})
            follower_cmd = f"FORMATION:FOLLOWER,{leader_id},{offset['x']},{offset['y']},{offset['yaw']}"
            if send_formation_command(car_id, start_cmd) and send_formation_command(car_id, follower_cmd):
                success_count += 1
    unicast_success_rate = (success_count / total_cars * 100) if total_cars > 0 else 0
    return jsonify({
        'success': True,
        'message': '自定义编队已设置',
        'formation_leader': formation_leader,
        'formation_offsets': custom_offsets,
        'unicast_success_count': success_count,
        'total_cars': total_cars,
        'success_rate': f'{unicast_success_rate:.1f}%'
    })
@formation_bp.route('/api/formation/configs')
def get_formation_configs():
    return jsonify({
        'success': True,
        'formation_configs': FORMATION_CONFIGS
    })
@formation_bp.route('/api/formation/update_offsets', methods=['POST'])
def update_formation_offsets():
    global formation_enabled, formation_leader
    if not formation_enabled:
        return jsonify({'success': False, 'error': '编队控制未启动'})
    data = request.json
    new_offsets = data.get('offsets', {})
    if not new_offsets:
        return jsonify({'success': False, 'error': '需要提供新的偏移量'})
    print(f"🔄 更新编队偏移量: {new_offsets}")
    success_count = 0
    total_cars = 0
    for car_id, offset in new_offsets.items():
        if car_id in cars_dict and car_id != formation_leader and cars_dict[car_id].connected:
            total_cars += 1
            update_cmd = f"[F,U,{formation_leader},{offset['x']},{offset['y']},{offset['yaw']}]"
            if send_formation_command(car_id, update_cmd):
                print(f"🔄 向小车 {car_id} 发送偏移更新: {update_cmd}")
                success_count += 1
    success_rate = (success_count / total_cars * 100) if total_cars > 0 else 0
    return jsonify({
        'success': True,
        'message': f'编队偏移量已更新，通知了 {success_count}/{total_cars} 辆小车',
        'updated_cars': success_count,
        'success_rate': f'{success_rate:.1f}%'
    })
def get_formation_info():
    return {
        'enabled': formation_enabled,
        'leader': formation_leader,
        'type': formation_type
    }