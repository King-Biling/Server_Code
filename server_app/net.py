# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.net
 模块职责: UDP 遥测/指令服务器、子网广播服务器及网络辅助函数
--------------------------------------------------------------------------------
 本模块承载服务器端全部“网络收发”职责：

   get_subnet_broadcast     自动探测本机子网广播地址
   _normalize_car_address   规范化小车回包地址（固定发往小车监听端口）
   BroadcastServer          子网广播发送器（可靠重发）
   UDPServer                UDP 主服务器：接收遥测/重构报文、周期广播小车状态、
                            连接健康检查、离线清理、STEP 守护等

 依赖关系:
   - 依赖 config（端口/超时常量）、state（小车表/广播开关/拓扑）、
     models（Car）、pose_save（位姿采样）。
   - 与 reconstruct 层存在相互调用（接收报文 -> 交给重构解析；守护线程 ->
     调用重构推进）。为打破循环导入，这些调用在函数内部使用延迟导入，
     且严格保持原有调用时序与锁使用方式。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import time
import socket
import threading

from . import config
from . import state
from .models import Car
from . import pose_save


def get_subnet_broadcast():
    """自动探测本机可用的子网广播地址；失败时回退到默认地址。"""
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
                        print(f" 发现广播地址: {broadcast_addr} (接口: {interface})")
                        return broadcast_addr
                    netmask = addr_info.get('netmask', '255.255.255.0')
                    ip_parts = list(map(int, ip.split('.')))
                    mask_parts = list(map(int, netmask.split('.')))
                    broadcast_parts = []
                    for i in range(4):
                        broadcast_parts.append(str(ip_parts[i] | (~mask_parts[i] & 0xFF)))
                    calculated_broadcast = '.'.join(broadcast_parts)
                    print(f" 计算得到广播地址: {calculated_broadcast} (接口: {interface})")
                    return calculated_broadcast
        fallback_broadcast = "192.168.31.255"
        print(f" 无法自动获取广播地址，使用默认: {fallback_broadcast}")
        return fallback_broadcast
    except ImportError:
        print(" 未安装netifaces库，使用默认广播地址")
        return "192.168.31.255"
    except Exception as e:
        print(f" 获取广播地址失败: {e}，使用默认广播地址")
        return "192.168.31.255"


def _normalize_car_address(addr):
    """使用最近上报源IP，回包固定发往小车监听端口。"""
    if not addr or len(addr) < 2:
        return addr
    return (addr[0], config.CAR_COMMAND_PORT)


class BroadcastServer:
    """子网广播发送器：负责向广播地址发送指令/状态，支持可靠重发。"""

    def __init__(self, port=config.BROADCAST_PORT):
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
            print(f"广播服务器启动成功，绑定端口 {self.port}")
            print(f"使用子网广播地址: {self.broadcast_address}")
            return True
        except Exception as e:
            print(f"广播服务器启动失败: {e}")
            return False

    def broadcast_data(self, data):
        try:
            target = (self.broadcast_address, self.port)
            self.socket.sendto(data.encode('utf-8'), target)
            print(f"广播数据: {data} -> {self.broadcast_address}:{self.port}")
            return True
        except Exception as e:
            print(f"广播发送失败: {e}")
            return False

    def broadcast_command_reliable(self, command, retries=5, delay=0.04):
        success_count = 0
        for i in range(retries):
            if self.broadcast_data(command):
                success_count += 1
                if i < retries - 1:
                    time.sleep(delay)
        print(f"广播指令 '{command}' 发送 {success_count}/{retries} 次")
        return success_count > 0

    def stop(self):
        self.running = False
        if self.socket:
            self.socket.close()


class UDPServer:
    """UDP 主服务器：遥测接收、状态广播、健康检查、离线清理与 STEP 守护。"""

    def __init__(self, host=config.UDP_HOST, port=config.UDP_PORT):
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        self.broadcast_sequence = 0
        self.last_debug_log = 0
        self.broadcast_server = BroadcastServer(config.BROADCAST_PORT)

    def start(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 128 * 1024)
            self.socket.bind((self.host, self.port))
            self.running = True
            print(f"UDP服务器启动在 {self.host}:{self.port}")
            print("等待小车连接...")

            receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            receive_thread.start()

            broadcast_thread = threading.Thread(target=self._broadcast_loop, daemon=True)
            broadcast_thread.start()

            cleanup_thread = threading.Thread(target=self._cleanup_loop, daemon=True)
            cleanup_thread.start()

            health_thread = threading.Thread(target=self._health_check_loop, daemon=True)
            health_thread.start()

            # PREP/STEP 守护线程（测试阶段停用 PREP 守护，防止超时自动流转）
            # prep_watchdog_thread = threading.Thread(target=self._prep_watchdog_loop, daemon=True)
            # prep_watchdog_thread.start()
            step_watchdog_thread = threading.Thread(target=self._step_watchdog_loop, daemon=True)
            step_watchdog_thread.start()

            if not self.broadcast_server.start():
                print(" 广播服务器启动失败，但UDP服务器继续运行")

            return True
        except Exception as e:
            print(f" UDP服务器启动失败: {e}")
            return False

    def _receive_loop(self):
        while self.running:
            try:
                data, addr = self.socket.recvfrom(65536)
                if data:
                    # 先传递原始字节，避免在这里提前 decode 破坏图像分片负载
                    self._handle_car_data(data, addr)
            except BlockingIOError:
                time.sleep(0.001)
            except Exception as e:
                print(f" UDP接收错误: {e}")
                time.sleep(0.01)

    def _handle_car_data(self, data, addr):
        # 延迟导入：打破 net <-> reconstruct 循环依赖
        from .reconstruct import handle_reconstruct_report
        try:
            car_id = None

            # =====================================================
            # 以下是原有的重构和遥测数据处理逻辑
            # =====================================================
            car_id = None
            # 字节流先交给重构解析器（可处理 IMG_CHUNK 的二进制载荷）
            if isinstance(data, bytes):
                # 尝试从二进制分片头提取 car_id
                if b'IMG_CHUNK' in data and b'[R,' in data:
                    try:
                        header_end = data.find(b'\n')
                        if header_end > 0:
                            header = data[:header_end].decode('utf-8', errors='ignore')
                            if header.startswith('[R,IMG_CHUNK,'):
                                parts = header.split(',')
                                if len(parts) > 2:
                                    car_id = parts[2]
                    except Exception:
                        pass
                if handle_reconstruct_report(data):
                    # 更新 car 的 last_update && connected 标志，防止假掉线
                    if car_id:
                        current_time = time.time()
                        with state.car_lock:
                            if car_id in state.cars:
                                state.cars[car_id].last_update = current_time
                                state.cars[car_id].connected = True
                    return
                data = data.decode('utf-8', errors='ignore')

            # 处理文本数据
            data = data.strip()
            if not data:
                return

            # 检查是否是重构相关消息（包含图像数据）
            if data.startswith('[R,'):
                # 提取 car_id
                try:
                    parts = data[1:].split(']')[0].split(',')
                    if len(parts) > 2:
                        car_id = parts[2]
                except Exception:
                    pass
                if handle_reconstruct_report(data):
                    # 更新 car 的 last_update && connected 标志
                    if car_id:
                        current_time = time.time()
                        with state.car_lock:
                            if car_id in state.cars:
                                state.cars[car_id].last_update = current_time
                                state.cars[car_id].connected = True
                    return

            # 检查是否是普通遥测数据
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
                heartbeat_event = False
                normalized_addr = _normalize_car_address(addr)

                with state.car_lock:
                    if car_id in state.cars:
                        car = state.cars[car_id]
                        old_address = car.address
                        if car.address != normalized_addr:
                            print(f" 小车 {car_id} 地址变化: {car.address} -> {normalized_addr}")
                            car.address = normalized_addr
                            reconnect_event = True
                        if not car.connected:
                            print(f" 小车 {car_id} 重新连接! 从 {old_address} 到 {normalized_addr}")
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
                        heartbeat_event = True
                    else:
                        state.cars[car_id] = Car(car_id, normalized_addr)
                        car = state.cars[car_id]
                        car.position = {"x": x, "y": y}
                        car.heading = yaw
                        car.battery = voltage
                        car.velocity = {"vx": vx, "vy": vy, "vz": vz}
                        car.speed = (vx ** 2 + vy ** 2) ** 0.5
                        car.last_update = current_time
                        print(f" 新小车连接: {car_id} from {normalized_addr}")
                        reconnect_event = True
                        heartbeat_event = True

                    pose_save.record_pose_sample(car_id, current_time, x, y, yaw)

                if heartbeat_event:
                    with state.reconstruct_lock:
                        ev = state.reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
                        ev["last_heartbeat"] = current_time

                if reconnect_event:
                    self._send_reconnect_ack(car_id)
                    print(f" 立即为新连接的小车 {car_id} 触发广播")
                    threading.Thread(target=self._broadcast_all_cars_data, daemon=True).start()

        except Exception as e:
            print(f" 处理小车数据失败: {e}")

    def _send_reconnect_ack(self, car_id):
        ack_msg = f"RECONNECT_ACK:{car_id},SERVER_READY"
        try:
            with state.car_lock:
                if car_id in state.cars and state.cars[car_id].connected:
                    self.socket.sendto(ack_msg.encode('utf-8'), state.cars[car_id].address)
                    print(f" 向 {car_id} 发送重连确认")
        except Exception as e:
            print(f" 发送重连确认失败: {e}")

    def _broadcast_loop(self):
        last_broadcast = 0
        debug_counter = 0
        while self.running:
            try:
                current_time = time.time()
                if state.broadcast_enabled and (current_time - last_broadcast >= state.broadcast_interval):
                    success = self._broadcast_all_cars_data()
                    last_broadcast = current_time
                    debug_counter += 1

                    if debug_counter >= 20:
                        print(f" 广播统计: 成功={success}, 周期={debug_counter}")
                        debug_counter = 0

                sleep_time = max(0.001, state.broadcast_interval - (time.time() - last_broadcast))
                time.sleep(sleep_time)
            except Exception as e:
                print(f" 广播循环错误: {e}")
                time.sleep(0.01)

    def _split_cars_into_groups(self, car_list):
        groups = []
        car_ids = sorted(car_list.keys())
        for i in range(0, len(car_ids), state.broadcast_group_size):
            group_car_ids = car_ids[i:i + state.broadcast_group_size]
            group_cars = {car_id: car_list[car_id] for car_id in group_car_ids}
            groups.append(group_cars)
        return groups

    def _broadcast_all_cars_data(self):
        current_time = time.time()
        connected_cars = {}
        with state.car_lock:
            for car_id, car in state.cars.items():
                if car.connected and current_time - car.last_update < 3.0:
                    connected_cars[car_id] = car

        if not connected_cars:
            return False

        try:
            cars_to_broadcast = {}
            if state.topology_enabled:
                car_mapping = config.CAR_INDEX_MAP
                for car_id, car in connected_cars.items():
                    if car_id not in car_mapping:
                        cars_to_broadcast[car_id] = car
                        continue
                    car_index = car_mapping[car_id]
                    row_sum = sum(state.communication_topology[car_index])
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
                print(f"📡 广播第 {group_index + 1}/{total_groups} 组小车数据: {broadcast_msg}")

                # 发送广播消息 - 使用子网广播地址
                success = self.broadcast_server.broadcast_data(broadcast_msg)
                if not success:
                    all_success = False
                for car in group_cars.values():
                    car.last_broadcast_time = current_time
                if group_index < total_groups - 1:
                    time.sleep(0.01)

            return all_success
        except Exception as e:
            print(f" 广播所有小车数据失败: {e}")
            return False

    def _get_visible_cars_for_car(self, target_car_id):
        if not state.topology_enabled:
            return list(config.ALL_CAR_IDS)
        return state.topology_cache.get(target_car_id, [])

    def _health_check_loop(self):
        while self.running:
            try:
                current_time = time.time()
                disconnected_cars = []
                with state.car_lock:
                    for car_id, car in state.cars.items():
                        if current_time - car.last_update > 5.0 and car.connected:
                            disconnected_cars.append(car_id)
                            car.connected = False
                for car_id in disconnected_cars:
                    print(f" 小车 {car_id} 超时未更新，标记为断开")
                time.sleep(2.0)
            except Exception as e:
                print(f" 健康检查错误: {e}")
                time.sleep(1.0)

    def _cleanup_loop(self):
        while self.running:
            try:
                current_time = time.time()
                cleanup_cars = []
                with state.car_lock:
                    for car_id, car in list(state.cars.items()):
                        if not car.connected and current_time - car.last_update > 60.0:
                            cleanup_cars.append(car_id)
                for car_id in cleanup_cars:
                    with state.car_lock:
                        if car_id in state.cars:
                            del state.cars[car_id]
                            print(f" 清理长时间离线小车: {car_id}")
                time.sleep(10.0)
            except Exception as e:
                print(f" 清理循环错误: {e}")
                time.sleep(1.0)

    def _prep_watchdog_loop(self):
        """监控 PREP 阶段，处理长时间未 PREP_OK 的车辆：先检查前车可见性，再重试 PREP 或重发 ORDER。"""
        # 延迟导入：打破 net <-> reconstruct 循环依赖
        from . import reconstruct
        while self.running:
            try:
                retry_actions = []
                with state.car_lock:
                    car_last_updates = {
                        cid: car.last_update
                        for cid, car in state.cars.items()
                        if car.connected
                    }
                with state.reconstruct_lock:
                    if state.reconstruct_state.get("phase") == "preparing":
                        order = list(state.reconstruct_state.get("order", []))
                        now = time.time()
                        prepared_map = state.reconstruct_state.get("prepared", {})
                        prepared = {
                            cid for cid, ts in prepared_map.items()
                            if now - ts <= config.PREP_OK_VALID_WINDOW
                        }

                        for car_id in order:
                            if car_id in prepared:
                                continue

                            entry = state.reconstruct_state.setdefault("prep_waiting", {}).setdefault(
                                car_id, {"last_prep_ok": 0, "last_retry": 0, "retries": 0}
                            )
                            last_prep_ok = entry.get("last_prep_ok", 0)

                            if now - last_prep_ok <= config.PREP_RETRY_TIMEOUT:
                                continue

                            try:
                                idx = order.index(car_id)
                            except ValueError:
                                idx = None

                            front_visible = False
                            if idx is None or idx == 0:
                                front_visible = True
                            else:
                                front_car = order[idx - 1]
                                if front_car in car_last_updates and now - car_last_updates[front_car] < 3.0:
                                    front_visible = True

                            last_retry = entry.get("last_retry", 0)
                            if front_visible and now - last_retry > config.PREP_RETRY_PREP_INTERVAL:
                                retry_actions.append(("prep", car_id, now))
                            elif (not front_visible) and now - last_retry > config.PREP_RETRY_ORDER_INTERVAL:
                                retry_actions.append(("order", car_id, now))

                # 注意：广播和日志不在锁内执行，避免阻塞 PREP 回包和其它重构流程
                for action_type, car_id, now in retry_actions:
                    if action_type == "prep":
                        self.broadcast_global_command("[R,PREP]")
                        reconstruct.log_reconstruct_event(f"重试 PREP（目标 {car_id} 未 PREP_OK，前车可见）")
                    else:
                        with state.reconstruct_lock:
                            order = list(state.reconstruct_state.get("order", []))
                        self.broadcast_global_command(f"[R,ORDER,{','.join(order)}]")
                        reconstruct.log_reconstruct_event(f"重发 ORDER（目标 {car_id} 未 PREP_OK，前车不可见）")

                    with state.reconstruct_lock:
                        entry = state.reconstruct_state.setdefault("prep_waiting", {}).setdefault(
                            car_id, {"last_prep_ok": 0, "last_retry": 0, "retries": 0}
                        )
                        entry["last_retry"] = now
                        entry["retries"] = entry.get("retries", 0) + 1

                time.sleep(0.5)
            except Exception as e:
                print(f" PREP 守护线程错误: {e}")
                time.sleep(0.5)

    def _step_watchdog_loop(self):
        """监控 STEP_ACK 与首帧图像超时，必要时重发 STEP。"""
        # 延迟导入：打破 net <-> reconstruct 循环依赖
        from . import reconstruct
        while self.running:
            try:
                action = None
                with state.reconstruct_lock:
                    if state.reconstruct_state.get("phase") == "assembling":
                        car_id = state.reconstruct_state.get("waiting_car_id")
                        index = state.reconstruct_state.get("current_index")
                        order = list(state.reconstruct_state.get("order", []))

                        if car_id and index is not None and order:
                            runtime = state.reconstruct_state.setdefault("step_runtime", {}).setdefault(car_id, reconstruct.make_step_runtime())
                            now = time.time()
                            diag = runtime.get("last_diag") or {}
                            diag_code = runtime.get("last_diag_code") or diag.get("code")
                            diag_hold_until = runtime.get("diag_hold_until_ts", 0)

                            if runtime.get("step_sent_ts", 0) > 0 and not runtime.get("assembling_started", False):
                                if diag_code == "ASM_STAGGER_WAIT" and now < diag_hold_until:
                                    continue
                                if now - runtime["step_sent_ts"] > config.STEP_ACK_TIMEOUT:
                                    retry_count = runtime.get("step_retry_count", 0)
                                    if retry_count < config.STEP_RETRY_MAX:
                                        runtime["step_retry_count"] = retry_count + 1
                                        runtime["last_retry_ts"] = now
                                        action = ("retry_ack", car_id, index, len(order), runtime["step_retry_count"])
                                    else:
                                        reconstruct.enter_error_state(f"STEP_ACK_TIMEOUT:{car_id}")
                                        action = ("error", f"{car_id} STEP_ACK 超时次数超过上限")

                            elif runtime.get("assembling_started", False) and not runtime.get("image_started", False):
                                image_timeout = config.FIRST_IMAGE_TIMEOUT
                                if diag_code == "ASM_STAGGER_WAIT":
                                    image_timeout += config.ASM_STAGGER_WAIT_GRACE
                                elif diag_code == "ASM_NO_FIRST_IMAGE":
                                    image_timeout = 0.2

                                if now - runtime.get("step_ack_ts", now) > image_timeout:
                                    retry_count = runtime.get("step_retry_count", 0)
                                    if retry_count < config.STEP_RETRY_MAX:
                                        runtime["step_retry_count"] = retry_count + 1
                                        runtime["last_retry_ts"] = now
                                        action = ("retry_img", car_id, index, len(order), runtime["step_retry_count"])
                                    else:
                                        reconstruct.enter_error_state(f"FIRST_IMAGE_TIMEOUT:{car_id}")
                                        action = ("error", f"{car_id} 首图超时次数超过上限")

                if action:
                    action_type = action[0]
                    if action_type == "retry_ack":
                        _, car_id, index, total, retry_count = action
                        reconstruct.log_reconstruct_event(f"{car_id} 未收到 STEP_ACK，重发 STEP（第{retry_count}次）")
                        # 死锁A自愈：小车可能因 ASM_START 丢包仍停在 READY，导致它忽略 STEP（handle_step 要求 s_state==ASSEMBLING）。
                        # 因此重发 STEP 前先补一发 ASM_START，把落队的小车拉进 ASSEMBLING 后再握手。
                        reconstruct.burst_broadcast_command("[R,ASM_START]")
                        reconstruct.send_reconstruct_step(car_id, index, total, is_separation=False)
                    elif action_type == "retry_img":
                        _, car_id, index, total, retry_count = action
                        reconstruct.log_reconstruct_event(f"{car_id} 已 STEP_ACK 但无首图，重发 STEP（第{retry_count}次）")
                        reconstruct.send_reconstruct_step(car_id, index, total, is_separation=False)
                    elif action_type == "error":
                        _, msg = action
                        reconstruct.log_reconstruct_event(f" {msg}")

                time.sleep(0.1)
            except Exception as e:
                print(f" STEP 守护线程错误: {e}")
                time.sleep(0.2)

    def send_to_car(self, car_id, message):
        with state.car_lock:
            if car_id in state.cars:
                car = state.cars[car_id]
                if car.connected:
                    try:
                        if not message.endswith('\n'):
                            message += '\n'
                        self.socket.sendto(message.encode('utf-8'), car.address)
                        print(f" 向 {car_id} 发送: {message.strip()}")
                        return True
                    except Exception as e:
                        print(f" 向 {car_id} 发送失败: {e}")
                        car.connected = False
                        return False
                else:
                    print(f" 小车 {car_id} 已断开连接")
            else:
                print(f" 小车 {car_id} 不存在")
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
