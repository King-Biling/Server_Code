import socket
import threading
import time
import json
import random 
from flask import Flask, request, jsonify, render_template
from flask_cors import CORS
from formation_controller import formation_bp, init_formation_controller  # 新增导入

app = Flask(__name__)
CORS(app)
app.register_blueprint(formation_bp)  # 注册编队控制器蓝图

# 存储小车信息的字典，key为小车ID
cars = {}
car_lock = threading.Lock()

# 服务器配置ll
UDP_HOST = '0.0.0.0'
UDP_PORT = 8080
WEB_PORT = 5000
BROADCAST_PORT = 8081  # 新增广播端口

# 广播配置
broadcast_enabled = False
broadcast_interval = 0.07  # 50ms
broadcast_group_size = 1  # 每组最多广播的小车数量

# 通信拓扑配置
communication_topology = [
    [0, 1, 1, 1],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
]

topology_enabled = False
topology_cache = {}


def get_subnet_broadcast():
    """获取子网广播地址"""
    try:
        import netifaces
        for interface in netifaces.interfaces():
            addrs = netifaces.ifaddresses(interface)
            if netifaces.AF_INET in addrs:
                for addr_info in addrs[netifaces.AF_INET]:
                    ip = addr_info['addr']
                    if ip.startswith('127.') or ip.startswith('169.254.'):
                        continue  # 跳过回环和链路本地地址

                    if 'broadcast' in addr_info:
                        broadcast_addr = addr_info['broadcast']
                        print(f"🌐 发现广播地址: {broadcast_addr} (接口: {interface})")
                        return broadcast_addr

                    # 如果没有广播地址，计算一个
                    netmask = addr_info.get('netmask', '255.255.255.0')
                    ip_parts = list(map(int, ip.split('.')))
                    mask_parts = list(map(int, netmask.split('.')))
                    broadcast_parts = []
                    for i in range(4):
                        broadcast_parts.append(str(ip_parts[i] | (~mask_parts[i] & 0xFF)))
                    calculated_broadcast = '.'.join(broadcast_parts)
                    print(f"🌐 计算得到广播地址: {calculated_broadcast} (接口: {interface})")
                    return calculated_broadcast

        # 如果所有方法都失败，使用常见的子网广播地址
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
        self.broadcast_address = "192.168.31.255"  # 获取子网广播地址

    def start(self):
        """启动广播服务器"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # 直接绑定到广播端口
            self.socket.bind(('', self.port))
            self.running = True

            print(f"📢 广播服务器启动成功，绑定端口 {self.port}")
            print(f"🌐 使用子网广播地址: {self.broadcast_address}")
            return True

        except Exception as e:
            print(f"❌ 广播服务器启动失败: {e}")
            return False

    def broadcast_data(self, data):
        """广播数据到所有小车 - 使用子网广播地址"""
        try:
            # 发送到子网广播地址
            target = (self.broadcast_address, self.port)
            self.socket.sendto(data.encode('utf-8'), target)
            print(f"📢 广播数据: {data} -> {self.broadcast_address}:{self.port}")
            return True
        except Exception as e:
            print(f"❌ 广播发送失败: {e}")
            return False

    def broadcast_command_reliable(self, command, retries=5, delay=0.04):
        """可靠地广播指令，重复发送指定次数"""
        success_count = 0
        for i in range(retries):
            if self.broadcast_data(command):
                success_count += 1
                if i < retries - 1:  # 不是最后一次发送
                    time.sleep(delay)
        print(f"📢 广播指令 '{command}' 发送 {success_count}/{retries} 次")
        return success_count > 0

    def stop(self):
        """停止广播服务器"""
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

        # 新增广播服务器实例
        self.broadcast_server = BroadcastServer(BROADCAST_PORT)

    def start(self):
        """启动UDP服务器"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # 增大缓冲区
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 128 * 1024)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 128 * 1024)

            self.socket.bind((self.host, self.port))
            self.running = True

            print(f"🚀 UDP服务器启动在 {self.host}:{self.port}")
            print("等待小车连接...")

            # 启动接收线程
            receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            receive_thread.start()

            # 启动广播线程
            broadcast_thread = threading.Thread(target=self._broadcast_loop, daemon=True)
            broadcast_thread.start()

            # 启动清理线程
            cleanup_thread = threading.Thread(target=self._cleanup_loop, daemon=True)
            cleanup_thread.start()

            # 启动连接健康检查线程
            health_thread = threading.Thread(target=self._health_check_loop, daemon=True)
            health_thread.start()

            # 启动广播服务器
            if not self.broadcast_server.start():
                print("❌ 广播服务器启动失败，但UDP服务器继续运行")

            return True

        except Exception as e:
            print(f"❌ UDP服务器启动失败: {e}")
            return False

    def _receive_loop(self):
        """UDP数据接收循环"""
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
        """处理小车数据"""
        try:
            data = data.strip()
            if not data:
                return

            # 解析小车数据
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
                    # 检查小车是否已经存在
                    if car_id in cars:
                        car = cars[car_id]
                        old_address = car.address
                        old_connected = car.connected

                        # 检查是否重连（地址变化或从断开状态恢复）
                        if car.address != addr:
                            print(f"🔄 小车 {car_id} 地址变化: {car.address} -> {addr}")
                            car.address = addr
                            reconnect_event = True

                        if not car.connected:
                            print(f"🎉 小车 {car_id} 重新连接! 从 {old_address} 到 {addr}")
                            car.connected = True
                            reconnect_event = True
                            car.connection_attempts = 0

                        # 更新小车状态
                        car.position = {"x": x, "y": y}
                        car.heading = yaw
                        car.battery = voltage
                        car.velocity = {"vx": vx, "vy": vy, "vz": vz}
                        car.speed = (vx ** 2 + vy ** 2) ** 0.5
                        car.last_update = current_time
                        car.update_count += 1

                    else:
                        # 新小车连接
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

                # 如果是重连事件，发送确认消息
                if reconnect_event:
                    self._send_reconnect_ack(car_id)

            # 立即触发一次广播，让新连接的小车尽快收到数据
            if reconnect_event:
                print(f"🚀 立即为新连接的小车 {car_id} 触发广播")
                threading.Thread(target=self._broadcast_all_cars_data, daemon=True).start()

        except Exception as e:
            print(f"❌ 处理小车数据失败: {e}")

    def _send_reconnect_ack(self, car_id):
        """发送重连确认消息"""
        ack_msg = f"RECONNECT_ACK:{car_id},SERVER_READY"
        try:
            with car_lock:
                if car_id in cars and cars[car_id].connected:
                    self.socket.sendto(ack_msg.encode('utf-8'), cars[car_id].address)
                    print(f"📤 向 {car_id} 发送重连确认")
        except Exception as e:
            print(f"❌ 发送重连确认失败: {e}")

    def _broadcast_loop(self):
        """UDP广播循环 - 使用子网广播"""
        last_broadcast = 0
        debug_counter = 0

        while self.running:
            try:
                current_time = time.time()
                if broadcast_enabled and (current_time - last_broadcast >= broadcast_interval):
                    # 添加调试信息
                    with car_lock:
                        connected_count = sum(1 for car in cars.values() if car.connected)
                    print(f"📡 开始广播周期，当前连接小车数量: {connected_count}")

                    success = self._broadcast_all_cars_data()
                    last_broadcast = current_time

                    debug_counter += 1
                    if debug_counter >= 20:  # 每20次打印一次
                        print(f"📡 广播统计: 成功={success}, 周期={debug_counter}")
                        debug_counter = 0

                sleep_time = max(0.001, broadcast_interval - (time.time() - last_broadcast))
                time.sleep(sleep_time)

            except Exception as e:
                print(f"❌ 广播循环错误: {e}")
                time.sleep(0.01)

    def _split_cars_into_groups(self, car_list):
        """将小车列表分成多个组，每组最多 broadcast_group_size 辆小车"""
        groups = []
        car_ids = sorted(car_list.keys())  # 按ID排序确保分组稳定

        for i in range(0, len(car_ids), broadcast_group_size):
            group_car_ids = car_ids[i:i + broadcast_group_size]
            group_cars = {car_id: car_list[car_id] for car_id in group_car_ids}
            groups.append(group_cars)

        return groups

    def _broadcast_all_cars_data(self):
        """使用子网广播发送所有小车数据 - 根据拓扑矩阵过滤不需要的数据"""
        current_time = time.time()
        connected_cars = {}

        # 收集连接的小车
        with car_lock:
            for car_id, car in cars.items():
                if car.connected and current_time - car.last_update < 3.0:
                    connected_cars[car_id] = car

        print(f"📡 准备广播，连接的小车: {list(connected_cars.keys())}")

        if not connected_cars:
            print("📡 没有连接的小车，跳过广播")
            return False

        try:
            # 根据拓扑矩阵过滤需要广播的小车
            cars_to_broadcast = self._filter_cars_by_topology(connected_cars)
            
            if not cars_to_broadcast:
                print("📡 根据拓扑矩阵，没有需要广播的小车数据")
                return True

            print(f"📡 拓扑过滤后需要广播的小车: {list(cars_to_broadcast.keys())}")

            # 将需要广播的小车分成多个组
            car_groups = self._split_cars_into_groups(cars_to_broadcast)
            total_groups = len(car_groups)

            print(f"📡 将 {len(cars_to_broadcast)} 辆小车分成 {total_groups} 组进行广播")

            all_success = True

            # 依次广播每个组
            for group_index, group_cars in enumerate(car_groups):
                # 构建包含组内小车数据的广播消息
                broadcast_parts = [f"[{len(group_cars)}"]
                for car_id, car in group_cars.items():
                    # 使用小车期望的格式
                    # 使用极简ID：C1 C2 C3
                    short_id = f"C{car_id[-1]}"
                    car_data = (f"{short_id} {car.position['x']:.2f} {car.position['y']:.2f} "
                                f"{car.heading:.1f} {car.velocity['vx']:.4f} "
                                f"{car.velocity['vy']:.4f} {car.velocity['vz']:.4f}")
                    broadcast_parts.append(car_data)

                broadcast_msg = " ".join(broadcast_parts) + "]"
                print(f"📡 广播第 {group_index + 1}/{total_groups} 组小车数据: {broadcast_msg}")

                # 发送广播消息 - 使用子网广播地址
                success = self.broadcast_server.broadcast_data(broadcast_msg)
                if not success:
                    all_success = False

                # 更新组内小车的最后广播时间
                for car in group_cars.values():
                    car.last_broadcast_time = current_time

                # 如果不是最后一组，稍微延迟一下再发送下一组
                if group_index < total_groups - 1:
                    time.sleep(0.01)  # 10ms延迟

            print(f"📡 分组广播完成: {'全部成功' if all_success else '部分失败'}")
            return all_success

        except Exception as e:
            print(f"❌ 广播所有小车数据失败: {e}")
            return False

    def _filter_cars_by_topology(self, connected_cars):
        """根据拓扑矩阵过滤需要广播的小车数据"""
        if not topology_enabled:
            # 拓扑未启用，广播所有小车
            return connected_cars
        
        # 如果拓扑启用，检查哪些小车的数据需要被广播
        # 规则：如果某辆小车在拓扑矩阵中对应的行全为0，说明没有小车需要它的数据，就不广播
        cars_to_broadcast = {}
        
        # 拓扑矩阵映射
        car_mapping = {"CAR1": 0, "CAR2": 1, "CAR3": 2, "CAR4": 3}
        
        for car_id, car in connected_cars.items():
            if car_id not in car_mapping:
                # 未知的小车ID，默认广播
                cars_to_broadcast[car_id] = car
                continue
                
            car_index = car_mapping[car_id]
            
            # 检查拓扑矩阵中是否有其他小车需要这辆小车的数据
            # 即检查该小车对应的列是否有1（其他小车能看到这辆小车）
            has_visible = False
            for i in range(4):  # 遍历所有行
                if i != car_index and communication_topology[i][car_index] == 1:
                    has_visible = True
                    break
            
            if has_visible:
                cars_to_broadcast[car_id] = car
                print(f"📡 拓扑过滤: {car_id} 被其他小车需要，包含在广播中")
            else:
                print(f"📡 拓扑过滤: {car_id} 没有被任何小车需要，跳过广播")
        
        return cars_to_broadcast

    def _get_visible_cars_for_car(self, target_car_id):
        """获取目标小车可以看到的其他小车列表"""
        if not topology_enabled:
            return ["CAR1", "CAR2", "CAR3", "CAR4"]
        return topology_cache.get(target_car_id, [])

    def _health_check_loop(self):
        """连接健康检查循环"""
        while self.running:
            try:
                current_time = time.time()
                disconnected_cars = []

                with car_lock:
                    for car_id, car in cars.items():
                        # 如果小车超过3秒没有更新，标记为断开
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
        """清理离线小车"""
        while self.running:
            try:
                current_time = time.time()
                cleanup_cars = []

                with car_lock:
                    for car_id, car in list(cars.items()):
                        # 如果小车断开超过60秒，清理资源
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
        """向指定小车发送消息"""
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
        """可靠地向指定小车发送消息 - 重复4次"""
        for attempt in range(max_retries):
            if self.send_to_car(car_id, message):
                return True
            time.sleep(0.05)
        return False

    def broadcast_global_command(self, command):
        """广播全局指令（重复5次）"""
        return self.broadcast_server.broadcast_command_reliable(command, retries=5, delay=0.01)

    def stop(self):
        """停止服务器"""
        self.running = False
        if self.socket:
            self.socket.close()
        self.broadcast_server.stop()


# 创建全局UDP服务器实例
udp_server = UDPServer(UDP_HOST, UDP_PORT)


# 拓扑缓存更新函数
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


# 编队控制变量
formation_enabled = False
formation_leader = None
formation_type = "line"

# 编队配置
FORMATION_CONFIGS = {
    "line": {
        "CAR1": {"x": 0, "y": 0, "yaw": 0},
        "CAR2": {"x": -0.7, "y": 0, "yaw": 0},
        "CAR3": {"x": -1.4, "y": 0, "yaw": 0},
        "CAR4": {"x": -2.1, "y": 0, "yaw": 0}
    },
    "triangle": {
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


# Flask路由
@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/cars')
def get_cars():
    """获取所有小车状态"""
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

    # 移除初始化广播测试
    # if enable:
    #     print("🚀 手动触发广播测试")
    #     threading.Thread(target=udp_server._broadcast_all_cars_data, daemon=True).start()

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
    data = request.json
    car_id = data.get('car_id')
    position = data.get('position')
    heading = data.get('heading', 0)

    if not car_id or not position:
        return jsonify({'success': False, 'error': '缺少参数'})

    # 新格式: [C,CAR1,1.5,2.3,45.0]
    cmd_str = f"[C,{car_id},{position.get('x', 0):.2f},{position.get('y', 0):.2f},{heading:.1f}]"
    success = udp_server.send_to_car_reliable(car_id, cmd_str, max_retries=4)

    if success:
        return jsonify({'success': True, 'message': f'导航指令已发送到小车 {car_id}'})
    else:
        return jsonify({'success': False, 'error': f'小车 {car_id} 未连接'})


# 拓扑相关API - 使用广播发送
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

            # 新格式: [T,M,1,1,1,1,1,0,1,1,1,1,0,1,1,1,1,0]
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

     # 新格式: [T,E,1] 或 [T,E,0]
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
    """获取网络信息"""
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
    # 显示网络信息
    get_network_info()

    update_topology_cache()

    if udp_server.start():
        print("✅ UDP服务器启动成功")

        # 初始化编队控制器
        init_formation_controller(cars, udp_server)

        print(f"📡 广播频率: {1 / broadcast_interval:.0f}Hz ({broadcast_interval * 1000:.0f}ms间隔)")
        print(f"📡 广播分组大小: 每组最多 {broadcast_group_size} 辆小车")
        print(f"📢 使用子网广播地址，端口: {BROADCAST_PORT}")
        print("💡 全局指令使用广播重复5次，特定指令使用单播重复4次")

        local_ip = get_local_ip()
        print(f"🌐 服务器本地IP地址: {local_ip}")
        print(f"💡 请确保小车配置中的SERVER_IP设置为: {local_ip}")
        print(f"💡 访问 http://{local_ip}:{WEB_PORT} 打开控制界面")

        app.run(host='0.0.0.0', port=WEB_PORT, debug=False, use_reloader=False, threaded=True)
    else:
        print("❌ UDP服务器启动失败")