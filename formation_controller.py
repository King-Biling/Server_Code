import csv
import json
import math
import os
import threading
import time
from datetime import datetime
from flask import Blueprint, request, jsonify, send_file

formation_bp = Blueprint('formation', __name__)

formation_enabled = False
formation_leader = None
formation_type = "line"  # line, Diamond, square, custom
formation_params = {}
cars_dict = {} 
udp_server = None 

RECT_POINTS = [
    {"x": 1.40, "y": 1.70, "yaw": 0.0},
    {"x": 3.40, "y": 1.70, "yaw": 90.0},
    {"x": 3.40, "y": 4.90, "yaw": 180.0},
    {"x": 1.40, "y": 4.90, "yaw": -90.0}
]
RECT_DISTANCE_THRESHOLD = 0.15
RECT_YAW_THRESHOLD_DEG = 5.0
RECT_COMMAND_HZ = 10.0
RECT_DATA_HZ = 10.0
RECT_LOG_DIRNAME = "logs"

rect_lock = threading.Lock()
rect_state = {
    "active": False,
    "stop_requested": False,
    "phase": "idle",
    "leader_id": None,
    "start_time": None,
    "idx": None,
    "segment_idx": 0,
    "speed": 0.2,
    "last_cmd": {},
    "records": {},
    "thread": None,
    "data_thread": None,
    "finalized": False,
    "last_files": [],
    "last_error": None
}

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

def _build_offsets_for_leader(formation_type_name, leader_id):
    base_offsets = FORMATION_CONFIGS.get(formation_type_name, FORMATION_CONFIGS["line"])
    leader_base = base_offsets.get(leader_id)
    if not leader_base:
        return base_offsets
    

    adjusted = {}
    for car_id, offset in base_offsets.items():
        adjusted[car_id] = {
            "x": offset["x"] - leader_base["x"],
            "y": offset["y"] - leader_base["y"],
            "yaw": offset.get("yaw", 0) - leader_base.get("yaw", 0)
        }
    return adjusted

def init_formation_controller(cars, server):
    global cars_dict, udp_server
    cars_dict = cars
    udp_server = server
    print("🔧 编队控制器初始化完成")

def _ensure_rect_log_dir():
    log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), RECT_LOG_DIRNAME))
    os.makedirs(log_dir, exist_ok=True)
    return log_dir

def _wrap_angle_deg(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle

def _get_leader_pose(leader_id):
    if not leader_id or leader_id not in cars_dict:
        return None
    car = cars_dict.get(leader_id)
    if not car or not car.connected:
        return None
    return car.position.get("x", 0.0), car.position.get("y", 0.0), car.heading

def _send_rectangle_command(leader_id, vx, vy, target_yaw):
    if not udp_server:
        return False
    cmd = f"[T,{leader_id},{vx:.3f},{vy:.3f},{target_yaw:.1f}]"
    return udp_server.send_to_car(leader_id, cmd)

def _update_rect_last_cmd(car_id, vx, vy):
    with rect_lock:
        rect_state["last_cmd"][car_id] = {"vx": float(vx), "vy": float(vy)}

def _select_nearest_point(x, y):
    best_idx = 0
    best_dist = None
    for idx, point in enumerate(RECT_POINTS):
        dx = point["x"] - x
        dy = point["y"] - y
        dist = (dx * dx + dy * dy) ** 0.5
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_idx = idx
    return best_idx

def _finalize_rect_experiment(reason=None):
    with rect_lock:
        if rect_state["finalized"]:
            return list(rect_state.get("last_files", []))
        rect_state["finalized"] = True
        rect_state["active"] = False
        rect_state["stop_requested"] = False
        rect_state["phase"] = "idle"
        if reason in ("completed", "manual_stop", "stopped"):
            rect_state["last_error"] = None
        else:
            rect_state["last_error"] = reason
        records_by_car = dict(rect_state["records"])

    log_dir = _ensure_rect_log_dir()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    saved_paths = []

    for car_id, rows in records_by_car.items():
        if not rows:
            continue
        filename = f"rect_traj_{car_id}_{timestamp}.csv"
        save_path = os.path.join(log_dir, filename)
        try:
            with open(save_path, "w", newline="", encoding="utf-8") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(["timestamp", "car_id", "x", "y", "yaw", "set_vx", "set_vy"])
                for row in rows:
                    writer.writerow(row)
            saved_paths.append(save_path)
        except Exception as exc:
            with rect_lock:
                rect_state["last_error"] = str(exc)
            return None

    with rect_lock:
        rect_state["last_files"] = list(saved_paths)
        rect_state["records"] = {}
        rect_state["last_cmd"] = {}

    return list(saved_paths)

def _rect_data_loop():
    interval = 1.0 / RECT_DATA_HZ
    while True:
        with rect_lock:
            if not rect_state["active"]:
                break
            last_cmd_snapshot = dict(rect_state["last_cmd"])
        current_ts = time.time()
        ts_str = datetime.fromtimestamp(current_ts).isoformat(timespec="milliseconds")
        rows_by_car = {}
        for car_id, car in list(cars_dict.items()):
            if not car.connected:
                continue
            cmd = last_cmd_snapshot.get(car_id, {"vx": 0.0, "vy": 0.0})
            rows_by_car.setdefault(car_id, []).append([
                ts_str,
                car_id,
                round(car.position.get("x", 0.0), 4),
                round(car.position.get("y", 0.0), 4),
                round(car.heading, 2),
                round(cmd["vx"], 4),
                round(cmd["vy"], 4)
            ])
        if rows_by_car:
            with rect_lock:
                for car_id, rows in rows_by_car.items():
                    rect_state["records"].setdefault(car_id, []).extend(rows)
        time.sleep(interval)

def _rect_state_loop():
    interval = 1.0 / RECT_COMMAND_HZ
    while True:
        with rect_lock:
            if not rect_state["active"]:
                break
            if rect_state["stop_requested"]:
                break
            leader_id = rect_state["leader_id"]
            phase = rect_state["phase"]
            idx = rect_state["idx"]
            segment_idx = rect_state["segment_idx"]
            speed = rect_state["speed"]

        pose = _get_leader_pose(leader_id)
        if not pose:
            _finalize_rect_experiment("leader_disconnected")
            break
        x, y, yaw = pose

        yaw_rad = math.radians(yaw)
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)

        vx, vy = 0.0, 0.0
        target_yaw = yaw

        abs_speed = abs(speed)

        if phase == "homing":
            target = RECT_POINTS[idx]
            dx = target["x"] - x
            dy = target["y"] - y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < RECT_DISTANCE_THRESHOLD:
                with rect_lock:
                    rect_state["phase"] = "rotate"
                    rect_state["segment_idx"] = 0
            else:
                if dist > 0:
                    forward = cos_yaw * dx + sin_yaw * dy
                    left = -sin_yaw * dx + cos_yaw * dy
                    vx = abs_speed * forward / dist
                    vy = abs_speed * left / dist
        elif phase == "rotate":
            current = RECT_POINTS[(idx + segment_idx) % len(RECT_POINTS)]
            target_yaw = current["yaw"]
            yaw_error = _wrap_angle_deg(target_yaw - yaw)
            if abs(yaw_error) <= RECT_YAW_THRESHOLD_DEG:
                with rect_lock:
                    rect_state["phase"] = "move"
            else:
                vx, vy = 0.0, 0.0
        elif phase == "move":
            current = RECT_POINTS[(idx + segment_idx) % len(RECT_POINTS)]
            target_yaw = current["yaw"]
            next_point = RECT_POINTS[(idx + segment_idx + 1) % len(RECT_POINTS)]
            dx = next_point["x"] - x
            dy = next_point["y"] - y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist < RECT_DISTANCE_THRESHOLD:
                next_segment = segment_idx + 1
                if next_segment >= len(RECT_POINTS):
                    _finalize_rect_experiment("completed")
                    break
                with rect_lock:
                    rect_state["segment_idx"] = next_segment
                    rect_state["phase"] = "rotate"
            else:
                if dist > 0:
                    forward = cos_yaw * dx + sin_yaw * dy
                    left = -sin_yaw * dx + cos_yaw * dy
                    vx = abs_speed * forward / dist
                    vy = abs_speed * left / dist

        _update_rect_last_cmd(leader_id, vx, vy)
        _send_rectangle_command(leader_id, vx, vy, target_yaw)
        time.sleep(interval)

    _finalize_rect_experiment("stopped")

def _start_rectangle_experiment(speed):
    leader_id = formation_leader
    pose = _get_leader_pose(leader_id)
    if not pose:
        return False, "leader_unavailable"

    x, y, _ = pose
    idx = _select_nearest_point(x, y)

    with rect_lock:
        if rect_state["active"]:
            return False, "already_running"
        rect_state["active"] = True
        rect_state["stop_requested"] = False
        rect_state["phase"] = "homing"
        rect_state["leader_id"] = leader_id
        rect_state["start_time"] = time.time()
        rect_state["idx"] = idx
        rect_state["segment_idx"] = 0
        rect_state["speed"] = speed
        rect_state["records"] = {}
        rect_state["last_cmd"] = {}
        rect_state["finalized"] = False
        rect_state["last_files"] = []
        rect_state["last_error"] = None

    rect_thread = threading.Thread(target=_rect_state_loop, daemon=True)
    data_thread = threading.Thread(target=_rect_data_loop, daemon=True)
    with rect_lock:
        rect_state["thread"] = rect_thread
        rect_state["data_thread"] = data_thread
    rect_thread.start()
    data_thread.start()
    return True, {"idx": idx, "leader_id": leader_id}

def _stop_rectangle_experiment():
    with rect_lock:
        if not rect_state["active"]:
            return False, "not_running"
        rect_state["stop_requested"] = True
        leader_id = rect_state["leader_id"]

    pose = _get_leader_pose(leader_id)
    if pose:
        _, _, yaw = pose
        _send_rectangle_command(leader_id, 0.0, 0.0, yaw)
        _update_rect_last_cmd(leader_id, 0.0, 0.0)

    deadline = time.time() + 2.5
    while time.time() < deadline:
        with rect_lock:
            if not rect_state["active"]:
                break
        time.sleep(0.05)

    save_paths = _finalize_rect_experiment("manual_stop")
    return True, {"save_paths": save_paths or []}

def get_rectangle_experiment_info():
    with rect_lock:
        return {
            "active": rect_state["active"],
            "leader_id": rect_state["leader_id"],
            "phase": rect_state["phase"],
            "last_files": list(rect_state["last_files"]),
            "last_error": rect_state["last_error"]
        }

def send_formation_command(car_id, command):
    if udp_server:
        return udp_server.send_to_car_reliable(car_id, command, max_retries=4)
    else:
        print(f"⚠️ UDP服务器未初始化，无法发送指令给 {car_id}")
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
    
    formation_offsets = _build_offsets_for_leader(formation_type, leader_id)
        
    old_leader = formation_leader
    formation_leader = leader_id
    formation_enabled = True
    print("🎯 直接启动编队，不发送停止指令")
    
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
            offset = formation_offsets.get(old_leader, {"x": 0, "y": 0, "yaw": 0})
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

@formation_bp.route('/api/formation/rectangle/start', methods=['POST'])
def start_rectangle_experiment():
    data = request.json or {}
    speed = float(data.get('speed', 0.2))

    if not formation_leader:
        return jsonify({'success': False, 'error': '未设置领航者'}), 400

    with rect_lock:
        if rect_state["active"]:
            return jsonify({'success': False, 'error': '矩形实验进行中'}), 400

    speed = abs(speed)
    speed = max(0.05, min(speed, 0.7))
    success, result = _start_rectangle_experiment(speed)
    if not success:
        return jsonify({'success': False, 'error': result}), 400

    return jsonify({
        'success': True,
        'message': '矩形轨迹实验已启动',
        'leader_id': result["leader_id"],
        'start_idx': result["idx"],
        'speed': speed
    })

@formation_bp.route('/api/formation/rectangle/stop', methods=['POST'])
def stop_rectangle_experiment():
    success, result = _stop_rectangle_experiment()
    if not success:
        return jsonify({'success': False, 'error': result}), 400

    save_paths = result.get("save_paths", [])
    files = [os.path.basename(path) for path in save_paths if path]
    download_urls = [f"/api/formation/rectangle/download/{name}" for name in files]

    return jsonify({
        'success': True,
        'message': '矩形轨迹实验已停止并保存',
        'files': files,
        'download_urls': download_urls
    })

@formation_bp.route('/api/formation/rectangle/status')
def get_rectangle_experiment_status():
    info = get_rectangle_experiment_info()
    files = [os.path.basename(path) for path in info.get("last_files", []) if path]
    download_urls = [f"/api/formation/rectangle/download/{name}" for name in files]
    info.update({'download_urls': download_urls, 'files': files})
    return jsonify(info)

@formation_bp.route('/api/formation/rectangle/download/<filename>')
def download_rectangle_file(filename):
    safe_name = os.path.basename(filename)
    if not safe_name:
        return jsonify({'success': False, 'error': '无效文件名'}), 400
    log_dir = _ensure_rect_log_dir()
    file_path = os.path.join(log_dir, safe_name)
    if not os.path.exists(file_path):
        return jsonify({'success': False, 'error': '文件不存在'}), 404
    return send_file(file_path, as_attachment=True, download_name=safe_name)

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
        'message': f'编队偏移量已更新，通知 {success_count}/{total_cars} 辆小车',
        'updated_cars': success_count,
        'success_rate': f'{success_rate:.1f}%'
    })

def get_formation_info():
    return {
        'enabled': formation_enabled,
        'leader': formation_leader,
        'type': formation_type
    }