import socket
import threading
import time
import json
import random 
from flask import Flask, request, jsonify, render_template
from flask_cors import CORS
# 增加导入 get_formation_info
from formation_controller import formation_bp, init_formation_controller, get_formation_info

app = Flask(__name__)
CORS(app)
app.register_blueprint(formation_bp)  

cars = {}
car_lock = threading.Lock()

UDP_HOST = '0.0.0.0'
UDP_PORT = 8080
WEB_PORT = 5000
BROADCAST_PORT = 8081 

broadcast_enabled = False
broadcast_interval = 0.07  
broadcast_group_size = 1  

communication_topology = [
    [0, 1, 1, 1],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
]
topology_enabled = False
topology_cache = {}

def get_subnet_broadcast():
    try:
        import netifaces
        for interface in netifaces.interfaces():
            addrs = netifaces.ifaddresses(interface)
            if netifaces.AF_INET in addrs:
                for addr_info in addrs[netifaces.AF_INET]:
                    ip = addr_info['addr']
                    if ip.startswith('127.') or ip.startswith('169.254.'):
                        continue  
                    if 'broadcast' in addr_info:
                        broadcast_addr = addr_info['broadcast']
                        print(f"🌐 发现广播地址: {broadcast_addr} (接口: {interface})")
                        return broadcast_addr
                    netmask = addr_info.get('netmask', '255.255.255.0')
                    ip_parts = list(map(int, ip.split('.')))
                    mask_parts = list(map(int, netmask.split('.')))
                    broadcast_parts = []
                    for i in range(4):
                        broadcast_parts.append(str(ip_parts[i] | (~mask_parts[i] & 0xFF)))
                    calculated_broadcast = '.'.join(broadcast_parts)
                    print(f"🌐 计算得到广播地址: {calculated_broadcast} (接口: {interface})")
                    return calculated_broadcast
        fallback_broadcast = "192.168.31.255"
        print(f"⚠️ 无法自动获取广播地址，使用默认: {fallback_broadcast}")
        return fallback_broadcast
    except ImportError:
        print("⚠️ 未安装netifaces库，使用默认广播地址")
        return "192.168.31.255"
    except Exception as e:
        print(f"❌ 获取广播地址失败: {e}，使用默认广播地址")
        return "192.168.31.255"

class Car:
    def __init__(self, car_id, address):
        self.car_id = car_id
        self.mac_address = f"CAR_{car_id}"
        self.address = address
        self.position = {"x": 0, "y": 0}
        self.heading = 0
        self.battery = 100
        self.velocity = {"vx": 0, "vy": 0, "vz": 0}
        self.speed = 0
        self.connected = True
        self.last_update = time.time()
        self.status = "正常"
        self.update_count = 0
        self.last_broadcast_time = 0
        self.connection_attempts = 0

class BroadcastServer:
    def __init__(self, port=8081):
        self.port = port
        self.socket = None
        self.running = False
        self.broadcast_address = "192.168.31.255" 

    def start(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind(('', self.port))
            self.running = True
            print(f"📢 广播服务器启动成功，绑定端口 {self.port}")
            print(f"🌐 使用子网广播地址: {self.broadcast_address}")
            return True
        except Exception as e:
            print(f"❌ 广播服务器启动失败: {e}")
            return False

    def broadcast_data(self, data):
        try:
            target = (self.broadcast_address, self.port)
            self.socket.sendto(data.encode('utf-8'), target)
            print(f"📢 广播数据: {data} -> {self.broadcast_address}:{self.port}")
            return True
        except Exception as e:
            print(f"❌ 广播发送失败: {e}")
            return False

    def broadcast_command_reliable(self, command, retries=5, delay=0.04):
        success_count = 0
        for i in range(retries):
            if self.broadcast_data(command):
                success_count += 1
                if i < retries - 1:
                    time.sleep(delay)
        print(f"📢 广播指令 '{command}' 发送 {success_count}/{retries} 次")
        return success_count > 0

    def stop(self):
        self.running = False
        if self.socket:
            self.socket.close()

class UDPServer:
    def __init__(self, host='0.0.0.0', port=8080):
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        self.broadcast_sequence = 0
        self.last_debug_log = 0
        self.broadcast_server = BroadcastServer(BROADCAST_PORT)

    def start(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 128 * 1024)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 128 * 1024)
            self.socket.bind((self.host, self.port))
            self.running = True
            print(f"🚀 UDP服务器启动在 {self.host}:{self.port}")
            print("等待小车连接...")

            receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            receive_thread.start()

            broadcast_thread = threading.Thread(target=self._broadcast_loop, daemon=True)
            broadcast_thread.start()

            cleanup_thread = threading.Thread(target=self._cleanup_loop, daemon=True)
            cleanup_thread.start()

            health_thread = threading.Thread(target=self._health_check_loop, daemon=True)
            health_thread.start()

            if not self.broadcast_server.start():
                print("❌ 广播服务器启动失败，但UDP服务器继续运行")

            return True
        except Exception as e:
            print(f"❌ UDP服务器启动失败: {e}")
            return False

    def _receive_loop(self):
        while self.running:
            try:
                data, addr = self.socket.recvfrom(1024)
                if data:
                    self._handle_car_data(data.decode('utf-8', errors='ignore'), addr)
            except BlockingIOError:
                time.sleep(0.001)
            except Exception as e:
                print(f"❌ UDP接收错误: {e}")
                time.sleep(0.01)

    def _handle_car_data(self, data, addr):
        try:
            data = data.strip()
            if not data:
                return
            parts = data.split(':')
            if len(parts) != 2:
                return
            car_id = parts[0]
            values = parts[1].split(',')
            
            if len(values) >= 7:
                x = float(values[0])
                y = float(values[1])
                yaw = float(values[2])
                voltage = float(values[3])
                vx = float(values[4])
                vy = float(values[5])
                vz = float(values[6])
                
                current_time = time.time()
                reconnect_event = False
                
                with car_lock:
                    if car_id in cars:
                        car = cars[car_id]
                        old_address = car.address
                        if car.address != addr:
                            print(f"🔄 小车 {car_id} 地址变化: {car.address} -> {addr}")
                            car.address = addr
                            reconnect_event = True
                        if not car.connected:
                            print(f"🎉 小车 {car_id} 重新连接! 从 {old_address} 到 {addr}")
                            car.connected = True
                            reconnect_event = True
                            car.connection_attempts = 0
                            
                        car.position = {"x": x, "y": y}
                        car.heading = yaw
                        car.battery = voltage
                        car.velocity = {"vx": vx, "vy": vy, "vz": vz}
                        car.speed = (vx ** 2 + vy ** 2) ** 0.5
                        car.last_update = current_time
                        car.update_count += 1
                    else:
                        cars[car_id] = Car(car_id, addr)
                        car = cars[car_id]
                        car.position = {"x": x, "y": y}
                        car.heading = yaw
                        car.battery = voltage
                        car.velocity = {"vx": vx, "vy": vy, "vz": vz}
                        car.speed = (vx ** 2 + vy ** 2) ** 0.5
                        car.last_update = current_time
                        print(f"🚗 新小车连接: {car_id} from {addr}")
                        reconnect_event = True

                if reconnect_event:
                    self._send_reconnect_ack(car_id)
                    print(f"🚀 立即为新连接的小车 {car_id} 触发广播")
                    threading.Thread(target=self._broadcast_all_cars_data, daemon=True).start()

        except Exception as e:
            print(f"❌ 处理小车数据失败: {e}")

    def _send_reconnect_ack(self, car_id):
        ack_msg = f"RECONNECT_ACK:{car_id},SERVER_READY"
        try:
            with car_lock:
                if car_id in cars and cars[car_id].connected:
                    self.socket.sendto(ack_msg.encode('utf-8'), cars[car_id].address)
                    print(f"📤 向 {car_id} 发送重连确认")
        except Exception as e:
            print(f"❌ 发送重连确认失败: {e}")

    def _broadcast_loop(self):
        last_broadcast = 0
        debug_counter = 0
        while self.running:
            try:
                current_time = time.time()
                if broadcast_enabled and (current_time - last_broadcast >= broadcast_interval):
                    with car_lock:
                        connected_count = sum(1 for car in cars.values() if car.connected)
                    
                    success = self._broadcast_all_cars_data()
                    last_broadcast = current_time
                    debug_counter += 1
                    
                    if debug_counter >= 20: 
                        print(f"📡 广播统计: 成功={success}, 周期={debug_counter}")
                        debug_counter = 0
                        
                sleep_time = max(0.001, broadcast_interval - (time.time() - last_broadcast))
                time.sleep(sleep_time)
            except Exception as e:
                print(f"❌ 广播循环错误: {e}")
                time.sleep(0.01)

    def _split_cars_into_groups(self, car_list):
        groups = []
        car_ids = sorted(car_list.keys()) 
        for i in range(0, len(car_ids), broadcast_group_size):
            group_car_ids = car_ids[i:i + broadcast_group_size]
            group_cars = {car_id: car_list[car_id] for car_id in group_car_ids}
            groups.append(group_cars)
        return groups

    def _broadcast_all_cars_data(self):
        current_time = time.time()
        connected_cars = {}
        with car_lock:
            for car_id, car in cars.items():
                if car.connected and current_time - car.last_update < 3.0:
                    connected_cars[car_id] = car
                    
        if not connected_cars:
            return False
            
        try:
            cars_to_broadcast = {}
            if topology_enabled:
                car_mapping = {"CAR1": 0, "CAR2": 1, "CAR3": 2, "CAR4": 3}
                for car_id, car in connected_cars.items():
                    if car_id not in car_mapping:
                        cars_to_broadcast[car_id] = car
                        continue
                    car_index = car_mapping[car_id]
                    row_sum = sum(communication_topology[car_index])
                    if row_sum > 0:
                        cars_to_broadcast[car_id] = car
            else:
                cars_to_broadcast = connected_cars
                
            if not cars_to_broadcast:
                return True
                
            car_groups = self._split_cars_into_groups(cars_to_broadcast)
            total_groups = len(car_groups)
            all_success = True
            
            for group_index, group_cars in enumerate(car_groups):
                broadcast_parts = [f"[{len(group_cars)}"]
                for car_id, car in group_cars.items():
                    short_id = f"C{car_id[-1]}"
                    car_data = (f"{short_id} {car.position['x']:.2f} {car.position['y']:.2f} "
                                f"{car.heading:.1f} {car.velocity['vx']:.4f} "
                                f"{car.velocity['vy']:.4f} {car.velocity['vz']:.4f}")
                    broadcast_parts.append(car_data)
                broadcast_msg = " ".join(broadcast_parts) + "]"
                
                success = self.broadcast_server.broadcast_data(broadcast_msg)
                if not success:
                    all_success = False
                for car in group_cars.values():
                    car.last_broadcast_time = current_time
                if group_index < total_groups - 1:
                    time.sleep(0.01)
                    
            return all_success
        except Exception as e:
            print(f"❌ 广播所有小车数据失败: {e}")
            return False

    def _get_visible_cars_for_car(self, target_car_id):
        if not topology_enabled:
            return ["CAR1", "CAR2", "CAR3", "CAR4"]
        return topology_cache.get(target_car_id, [])

    def _health_check_loop(self):
        while self.running:
            try:
                current_time = time.time()
                disconnected_cars = []
                with car_lock:
                    for car_id, car in cars.items():
                        if current_time - car.last_update > 5.0:
                            if car.connected:
                                disconnected_cars.append(car_id)
                                car.connected = False
                for car_id in disconnected_cars:
                    print(f"⚠️ 小车 {car_id} 超时未更新，标记为断开")
                time.sleep(2.0)
            except Exception as e:
                print(f"❌ 健康检查错误: {e}")
                time.sleep(1.0)

    def _cleanup_loop(self):
        while self.running:
            try:
                current_time = time.time()
                cleanup_cars = []
                with car_lock:
                    for car_id, car in list(cars.items()):
                        if not car.connected and current_time - car.last_update > 60.0:
                            cleanup_cars.append(car_id)
                for car_id in cleanup_cars:
                    with car_lock:
                        if car_id in cars:
                            del cars[car_id]
                            print(f"🗑️ 清理长时间离线小车: {car_id}")
                time.sleep(10.0)
            except Exception as e:
                print(f"❌ 清理循环错误: {e}")
                time.sleep(1.0)

    def send_to_car(self, car_id, message):
        with car_lock:
            if car_id in cars:
                car = cars[car_id]
                if car.connected:
                    try:
                        if not message.endswith('\n'):
                            message += '\n'
                        self.socket.sendto(message.encode('utf-8'), car.address)
                        print(f"📤 向 {car_id} 发送: {message.strip()}")
                        return True
                    except Exception as e:
                        print(f"❌ 向 {car_id} 发送失败: {e}")
                        car.connected = False
                        return False
                else:
                    print(f"⚠️ 小车 {car_id} 已断开连接")
            else:
                print(f"⚠️ 小车 {car_id} 不存在")
        return False

    def send_to_car_reliable(self, car_id, message, max_retries=4):
        for attempt in range(max_retries):
            if self.send_to_car(car_id, message):
                return True
            time.sleep(0.05)
        return False

    def broadcast_global_command(self, command):
        return self.broadcast_server.broadcast_command_reliable(command, retries=5, delay=0.01)

    def stop(self):
        self.running = False
        if self.socket:
            self.socket.close()
        self.broadcast_server.stop()

udp_server = UDPServer(UDP_HOST, UDP_PORT)

def update_topology_cache():
    global topology_cache
    topology_cache = {}
    car_ids = ["CAR1", "CAR2", "CAR3", "CAR4"]
    car_mapping = {"CAR1": 0, "CAR2": 1, "CAR3": 2, "CAR4": 3}
    for target_car in car_ids:
        visible_cars = []
        target_index = car_mapping[target_car]
        for other_car in car_ids:
            if other_car != target_car:
                other_index = car_mapping[other_car]
                if communication_topology[other_index][target_index] == 1:
                    visible_cars.append(other_car)
        topology_cache[target_car] = visible_cars
    print(f"🔧 拓扑缓存已更新: {topology_cache}")

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/cars')
def get_cars():
    with car_lock:
        car_list = []
        for car_id, car in cars.items():
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

@app.route('/api/broadcast', methods=['POST'])
def toggle_broadcast():
    global broadcast_enabled
    data = request.json
    enable = data.get('enable', True)
    broadcast_enabled = enable
    status = "开启" if enable else "关闭"
    print(f"📢 广播功能 {status}")
    return jsonify({
        'success': True,
        'message': f'广播功能已{status}',
        'broadcast_enabled': broadcast_enabled
    })

@app.route('/api/broadcast/interval', methods=['POST'])
def set_broadcast_interval():
    global broadcast_interval
    data = request.json
    interval = data.get('interval', 0.05)
    if interval <= 0:
        return jsonify({'success': False, 'error': '间隔必须大于0'})
    broadcast_interval = interval
    return jsonify({
        'success': True,
        'message': f'广播间隔已更新为{interval}秒',
        'broadcast_interval': interval
    })

@app.route('/api/broadcast/group_size', methods=['POST'])
def set_broadcast_group_size():
    global broadcast_group_size
    data = request.json
    group_size = data.get('group_size', 2)
    if group_size <= 0:
        return jsonify({'success': False, 'error': '分组大小必须大于0'})
    broadcast_group_size = group_size
    return jsonify({
        'success': True,
        'message': f'广播分组大小已更新为{group_size}辆小车',
        'broadcast_group_size': group_size
    })

@app.route('/api/control_position', methods=['POST'])
def control_car_position():
    # 核心拦截逻辑：如果在编队执行中，严禁独立位置控制
    formation_info = get_formation_info()
    if formation_info.get('enabled'):
        return jsonify({'success': False, 'error': '编队执行中，已锁定单车独立控制！'})
        
    data = request.json
    car_id = data.get('car_id')
    position = data.get('position')
    heading = data.get('heading', 0)
    if not car_id or not position:
        return jsonify({'success': False, 'error': '缺少参数'})
    cmd_str = f"[C,{car_id},{position.get('x', 0):.2f},{position.get('y', 0):.2f},{heading:.1f}]"
    success = udp_server.send_to_car_reliable(car_id, cmd_str, max_retries=4)
    if success:
        return jsonify({'success': True, 'message': f'导航指令已发送到小车 {car_id}'})
    else:
        return jsonify({'success': False, 'error': f'小车 {car_id} 未连接'})

@app.route('/api/control_velocity', methods=['POST'])
def control_car_velocity():
    """实时速度控制接口 (用于键盘遥控)"""
    data = request.json
    car_id = data.get('car_id')
    
    # 核心拦截逻辑：如果在编队中，且控制对象不是领航者，则拒绝遥控
    formation_info = get_formation_info()
    if formation_info.get('enabled') and car_id != formation_info.get('leader'):
        return jsonify({'success': False, 'error': '编队执行中，仅允许遥控领航者！'})
        
    vx = data.get('vx', 0.0)
    vy = data.get('vy', 0.0)
    vz = data.get('vz', 0.0)
    
    if not car_id:
        return jsonify({'success': False, 'error': '缺少car_id'})
        
    # 构建实时运动指令: [M,CAR1,vx,vy,vz]
    cmd_str = f"[M,{car_id},{vx:.3f},{vy:.3f},{vz:.3f}]"
    
    # 低延迟遥控，不重传，直接单发
    success = udp_server.send_to_car(car_id, cmd_str)
    return jsonify({'success': success})

@app.route('/api/topology', methods=['POST'])
def set_topology():
    global communication_topology, topology_enabled
    data = request.json
    topology_matrix = data.get('topology')
    enable = data.get('enable', False)
    if topology_matrix:
        if (isinstance(topology_matrix, list) and len(topology_matrix) == 4 and
                all(isinstance(row, list) and len(row) == 4 for row in topology_matrix)):
            communication_topology = topology_matrix
            topology_enabled = enable
            update_topology_cache()
            topology_flat = []
            for row in communication_topology:
                topology_flat.extend(row)
            topology_str = ','.join(str(cell) for cell in topology_flat)
            topology_cmd = f"[T,M,{topology_str}]"
            success = udp_server.broadcast_global_command(topology_cmd)
            print(f"✅ 通信拓扑已更新: {communication_topology}")
            print(f"📤 发送拓扑指令: {topology_cmd}")
            return jsonify({
                'success': True,
                'message': f'通信拓扑已{"启用" if enable else "禁用"}',
                'topology': communication_topology,
                'topology_enabled': topology_enabled,
                'broadcast_success': success,
                'topology_string': topology_str
            })
        else:
            return jsonify({'success': False, 'error': '无效的拓扑矩阵格式'})
    else:
        return jsonify({'success': False, 'error': '缺少拓扑矩阵'})

@app.route('/api/topology/status')
def get_topology_status():
    return jsonify({
        'topology': communication_topology,
        'topology_enabled': topology_enabled
    })

@app.route('/api/topology/toggle', methods=['POST'])
def toggle_topology():
    global topology_enabled
    data = request.json
    enable = data.get('enable', False)
    topology_enabled = enable
    status = "启用" if enable else "禁用"
    update_topology_cache()
    toggle_cmd = f"[T,E,{1 if enable else 0}]"
    broadcast_success = udp_server.broadcast_global_command(toggle_cmd)
    print(f"🔗 拓扑通信 {status}")
    return jsonify({
        'success': True,
        'message': f'拓扑通信已{status}',
        'topology_enabled': topology_enabled,
        'broadcast_success': broadcast_success
    })

@app.route('/api/topology/visible/<car_id>')
def get_visible_cars(car_id):
    visible_cars = udp_server._get_visible_cars_for_car(car_id)
    return jsonify({
        'car_id': car_id,
        'visible_cars': visible_cars,
        'topology_enabled': topology_enabled
    })

def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "无法获取IP"

def get_network_info():
    try:
        import netifaces
        interfaces = netifaces.interfaces()
        print("🌐 网络接口信息:")
        for interface in interfaces:
            addrs = netifaces.ifaddresses(interface)
            if netifaces.AF_INET in addrs:
                for addr_info in addrs[netifaces.AF_INET]:
                    print(f"  {interface}: {addr_info['addr']} - 广播地址: {addr_info.get('broadcast', 'N/A')}")
    except ImportError:
        print("⚠️ 无法获取详细网络信息，请安装 netifaces 库")

if __name__ == '__main__':
    get_network_info()
    update_topology_cache()
    
    if udp_server.start():
        print("✅ UDP服务器启动成功")
        init_formation_controller(cars, udp_server)
        print(f"📡 广播频率: {1 / broadcast_interval:.0f}Hz ({broadcast_interval * 1000:.0f}ms间隔)")
        print(f"📡 广播分组大小: 每组最多 {broadcast_group_size} 辆小车")
        print(f"📢 使用子网广播地址，端口: {BROADCAST_PORT}")
        local_ip = get_local_ip()
        print(f"🌐 服务器本地IP地址: {local_ip}")
        print(f"💡 访问 http://{local_ip}:{WEB_PORT} 打开控制界面")
        app.run(host='0.0.0.0', port=WEB_PORT, debug=False, use_reloader=False, threaded=True)
    else:
        print("❌ UDP服务器启动失败")