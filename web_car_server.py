# -*- coding: utf-8 -*-
import socket
import threading
import time
import json
import random 
import os
import csv
import base64
import sys
import numpy as np
import cv2
import gzip
from datetime import datetime
from flask import Flask, request, jsonify, render_template
from flask_cors import CORS
# 本地视觉引擎
VISION_MODULE_DIR = os.path.join(os.path.dirname(__file__), "Car_vision_system")
if VISION_MODULE_DIR not in sys.path:
    sys.path.append(VISION_MODULE_DIR)

try:
    from car_vision_system import DeviceBinder, PoseEstimator, CONFIG as VISION_CONFIG
except Exception as e:
    DeviceBinder = None
    PoseEstimator = None
    VISION_CONFIG = {}
    print(f"⚠️ 本地视觉模块导入失败: {e}")
# 增加导入 get_formation_info
from formation_controller import formation_bp, init_formation_controller, get_formation_info

def _configure_utf8_console():
    """尽量让 Windows 控制台和 Python 标准输出统一使用 UTF-8。"""
    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(encoding="utf-8", errors="replace")
        if hasattr(sys.stderr, "reconfigure"):
            sys.stderr.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    if os.name == "nt":
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            kernel32.SetConsoleOutputCP(65001)
            kernel32.SetConsoleCP(65001)
        except Exception:
            pass


_configure_utf8_console()

app = Flask(__name__)
CORS(app)
app.register_blueprint(formation_bp)  

cars = {}
car_lock = threading.Lock()

reconstruct_lock = threading.RLock()
reconstruct_state = {
    "active": False,
    "phase": "idle",
    "order": [],
    "prepared": {},
    "current_index": None,
    "waiting_car_id": None,
    "last_error": None,
    "prev_topology": None,
    "prev_topology_enabled": None,
    "event_log": [],
    "latest_frames": {},
    "guide_controllers": {},
    "guide_frequency": 20,  # Hz
    "guide_timeout": 0.3,   # 秒
    "target_distance": 5.0, # 5cm
    "current_image": None,   # 当前显示的图像
    "current_image_car": None, # 当前图像对应的小车
    "image_timestamp": 0,    # 图像时间戳
    "current_error": None,   # 当前误差 (x, y, yaw)
    "current_error_car": None,
    "current_has_tag": False,
    "debug_logs": []         # 调试日志
    ,"prep_waiting": {}
    ,"events": {}
    ,"step_runtime": {}
    ,"subphase": "idle"
}

RECONSTRUCT_POS_LABELS = ["HEAD", "MID2", "MID3", "TAIL"]
MAX_RECONSTRUCT_EVENTS = 120

# PREP 重试策略（秒）
PREP_RETRY_TIMEOUT = 3.0
PREP_RETRY_PREP_INTERVAL = 1.0
PREP_RETRY_ORDER_INTERVAL = 2.0
PREP_OK_VALID_WINDOW = 3.0

# STEP/IMG 握手超时（秒）
STEP_ACK_TIMEOUT = 1.5
FIRST_IMAGE_TIMEOUT = 1.5
STEP_RETRY_MAX = 3
ASM_STAGGER_WAIT_GRACE = 2.5

# 末端制导参数
GUIDE_SPEED_LIMIT = 0.15  # m/s
# 误差输入单位为 cm，因此增益按 cm 缩放
GUIDE_P_GAIN_X = 0.005
GUIDE_P_GAIN_Y = 0.005
GUIDE_P_GAIN_YAW = 0.3

# AprilTag参数
APRILTAG_SIZE = 0.06  # 60mm
PIXEL_TO_METER_RATIO = 0.0005  # 需要根据实际摄像头标定调整

save_lock = threading.Lock()
active_pose_saves = {}
POSE_SAVE_DIRNAME = "pose_data"
LOG_DIRNAME = "logs"

file_log_lock = threading.Lock()

def _ensure_log_dir():
    log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), LOG_DIRNAME))
    try:
        os.makedirs(log_dir, exist_ok=True)
    except Exception:
        pass
    return log_dir

def _write_persistent_event(event_type, car_id=None, payload=None):
    """将事件以 JSONL 形式追加到按日切分的日志文件中。"""
    try:
        log_dir = _ensure_log_dir()
        date_str = datetime.now().strftime("%Y%m%d")
        filename = os.path.join(log_dir, f"reconstruct-{date_str}.jsonl")
        entry = {
            "ts": datetime.now().isoformat(timespec='milliseconds'),
            "type": event_type,
            "car_id": car_id,
            "payload": payload or {}
        }
        line = json.dumps(entry, ensure_ascii=False)
        with file_log_lock:
            with open(filename, 'a', encoding='utf-8') as f:
                f.write(line + "\n")
    except Exception as e:
        # 持久化日志失败不阻塞主流程，仅在控制台记录
        print(f"⚠️ 写持久化日志失败: {e}")

def _write_readable_log(event_type, car_id=None, **kwargs):
    """以纯文本友好格式追加日志（支持记事本直接打开查看）。"""
    try:
        log_dir = _ensure_log_dir()
        date_str = datetime.now().strftime("%Y%m%d")
        filename = os.path.join(log_dir, f"reconstruct-readable-{date_str}.log")
        ts_str = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # 构建友好的日志行
        car_part = f" [{car_id}]" if car_id else ""
        payload_part = " ".join(f"{k}={v}" for k, v in kwargs.items())
        line = f"[{ts_str}] {event_type}{car_part}"
        if payload_part:
            line += f" {payload_part}"
        
        with file_log_lock:
            with open(filename, 'a', encoding='utf-8') as f:
                f.write(line + "\n")
    except Exception:
        pass

# 自动归档（自动下载）设置
LOG_ARCHIVE_DIRNAME = os.path.join(LOG_DIRNAME, "auto_downloads")
LOG_ARCHIVE_INTERVAL_S = 10 * 60  # 默认每10分钟归档一次，可调整

def _ensure_archive_dir():
    archive_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), LOG_ARCHIVE_DIRNAME))
    try:
        os.makedirs(archive_dir, exist_ok=True)
    except Exception:
        pass
    return archive_dir

def _archive_current_log():
    """把当前日期的 jsonl 日志压缩为 .jsonl.gz 并保存到 auto_downloads 目录。"""
    try:
        log_dir = _ensure_log_dir()
        archive_dir = _ensure_archive_dir()
        date_str = datetime.now().strftime("%Y%m%d")
        src = os.path.join(log_dir, f"reconstruct-{date_str}.jsonl")
        if not os.path.exists(src):
            return False, "no_source"
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        dst_name = f"reconstruct-{date_str}-{ts}.jsonl.gz"
        dst = os.path.join(archive_dir, dst_name)
        # 进行压缩复制
        with open(src, 'rb') as f_in:
            with gzip.open(dst, 'wb') as f_out:
                while True:
                    chunk = f_in.read(65536)
                    if not chunk:
                        break
                    f_out.write(chunk)
        print(f"📥 自动归档日志: {dst}")
        _write_persistent_event("archive", None, {"file": os.path.basename(dst)})
        return True, dst
    except Exception as e:
        print(f"⚠️ 自动归档失败: {e}")
        return False, str(e)

def _archive_worker_loop(interval_s=LOG_ARCHIVE_INTERVAL_S):
    while True:
        try:
            _archive_current_log()
        except Exception as e:
            print(f"❌ 归档守护错误: {e}")
        time.sleep(interval_s)

UDP_HOST = '0.0.0.0'
UDP_PORT = 8080
WEB_PORT = 5000
BROADCAST_PORT = 8081 
CAR_COMMAND_PORT = 8081

broadcast_enabled = False
broadcast_interval = 0.07  
broadcast_group_size = 1  

communication_topology = [
    [0, 1, 1, 1],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
]
DEFAULT_COMMUNICATION_TOPOLOGY = [row[:] for row in communication_topology]
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


def _normalize_car_address(addr):
    """使用最近上报源IP，回包固定发往小车监听端口。"""
    if not addr or len(addr) < 2:
        return addr
    return (addr[0], CAR_COMMAND_PORT)

class GuideController:
    """末端制导控制器"""
    def __init__(self, car_id):
        self.car_id = car_id
        self.active = False
        self.last_guide_time = 0
        self.target_reached = False
        self.last_pose_error = None
        self.guide_thread = None
        
    def start_guidance(self):
        """开始制导"""
        if self.active:
            return
        self.active = True
        self.target_reached = False
        self.guide_thread = threading.Thread(target=self._guidance_loop, daemon=True)
        self.guide_thread.start()
        
    def stop_guidance(self):
        """停止制导"""
        self.active = False
        
    def _guidance_loop(self):
        """制导循环"""
        guide_interval = 1.0 / reconstruct_state["guide_frequency"]
        
        while self.active:
            try:
                # 没有可用位姿时仍保持刷新，避免车端超时
                if self.last_pose_error is None:
                    self._send_guide_command(0, 0, 0, done=0)
                    time.sleep(guide_interval)
                    continue
                    
                # 计算制导速度
                vx, vy, vz, done = self._calculate_guide_velocity()
                
                # 发送制导指令
                self._send_guide_command(vx, vy, vz, done)
                
                # 如果目标已到达，停止制导
                if done == 1:
                    self.target_reached = True
                    self.active = False
                    _advance_reconstruct_assembly(self.car_id)
                    break
                    
                time.sleep(guide_interval)
                
            except Exception as e:
                print(f"❌ 制导循环错误 ({self.car_id}): {e}")
                time.sleep(guide_interval)
                
    def _calculate_guide_velocity(self):
        """计算制导速度"""
        if self.last_pose_error is None:
            return 0, 0, 0, 0
            
        error_x, error_y, error_yaw = self.last_pose_error
        
        # 检查是否到达目标
        if (abs(error_x) < 2.0 and abs(error_y) < 10.0 and abs(error_yaw) < 1.5):
            return 0, 0, 0, 1  # 目标到达
            
        # PID控制（简化版，只有P项）
        vx = GUIDE_P_GAIN_X * error_x
        vy = GUIDE_P_GAIN_Y * error_y
        # 航向控制改用小车自身航向角与前车航向角
        vz = GUIDE_P_GAIN_YAW * _get_front_heading_error(self.car_id)
        
        # 速度限制
        speed_magnitude = (vx**2 + vy**2)**0.5
        if speed_magnitude > GUIDE_SPEED_LIMIT:
            scale = GUIDE_SPEED_LIMIT / speed_magnitude
            vx *= scale
            vy *= scale
            
        return vx, vy, vz, 0
        
    def _send_guide_command(self, vx, vy, vz, done):
        """发送制导指令"""
        # 命名字段格式（推荐）
        cmd = f"[R,GUIDE,{self.car_id},VX={vx:.3f},VY={vy:.3f},VZ={vz:.3f},DONE={done}]"
        success = udp_server.send_to_car(self.car_id, cmd)
        
        if success:
            self.last_guide_time = time.time()
            # 记录最后一次下发 GUIDE 时间到全局事件表
            try:
                with reconstruct_lock:
                    ev = reconstruct_state.setdefault("events", {}).setdefault(self.car_id, {})
                    ev["last_guide_sent"] = time.time()
                    if done == 1:
                        ev["last_guide_done"] = time.time()
                        reconstruct_state["subphase"] = "WAIT_STEP_OK"
            except Exception:
                pass
            if done == 0:
                print(f"🎯 制导指令 ({self.car_id}): vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}")
            else:
                print(f"✅ 制导完成 ({self.car_id})")
        
    def update_pose_error(self, error_x, error_y, error_yaw):
        """更新位姿误差"""
        self.last_pose_error = (error_x, error_y, error_yaw)


class PoseSaveSession:
    def __init__(self, car_id, duration_sec, filename):
        self.car_id = car_id
        self.duration_sec = duration_sec
        self.filename = filename
        self.save_path = ""
        self.start_time = time.time()
        self.end_time = self.start_time + duration_sec
        self.records = []

    def add_record(self, timestamp, x, y, yaw):
        self.records.append({
            "timestamp": timestamp,
            "x": x,
            "y": y,
            "yaw": yaw
        })

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
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
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

            # PREP 监控守护线程：检查未返回 PREP_OK 的车辆并按策略重试 PREP/ORDER
            prep_watchdog_thread = threading.Thread(target=self._prep_watchdog_loop, daemon=True)
            prep_watchdog_thread.start()

            # STEP 监控守护线程：ACK/首图超时重试
            step_watchdog_thread = threading.Thread(target=self._step_watchdog_loop, daemon=True)
            step_watchdog_thread.start()

            if not self.broadcast_server.start():
                print("❌ 广播服务器启动失败，但UDP服务器继续运行")

            return True
        except Exception as e:
            print(f"❌ UDP服务器启动失败: {e}")
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
                print(f"❌ UDP接收错误: {e}")
                time.sleep(0.01)

    def _handle_car_data(self, data, addr):
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
                        with car_lock:
                            if car_id in cars:
                                cars[car_id].last_update = current_time
                                cars[car_id].connected = True
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
                        with car_lock:
                            if car_id in cars:
                                cars[car_id].last_update = current_time
                                cars[car_id].connected = True
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
                
                with car_lock:
                    if car_id in cars:
                        car = cars[car_id]
                        old_address = car.address
                        if car.address != normalized_addr:
                            print(f"🔄 小车 {car_id} 地址变化: {car.address} -> {normalized_addr}")
                            car.address = normalized_addr
                            reconnect_event = True
                        if not car.connected:
                            print(f"🎉 小车 {car_id} 重新连接! 从 {old_address} 到 {normalized_addr}")
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
                        cars[car_id] = Car(car_id, normalized_addr)
                        car = cars[car_id]
                        car.position = {"x": x, "y": y}
                        car.heading = yaw
                        car.battery = voltage
                        car.velocity = {"vx": vx, "vy": vy, "vz": vz}
                        car.speed = (vx ** 2 + vy ** 2) ** 0.5
                        car.last_update = current_time
                        print(f"🚗 新小车连接: {car_id} from {normalized_addr}")
                        reconnect_event = True
                        heartbeat_event = True

                    record_pose_sample(car_id, current_time, x, y, yaw)

                if heartbeat_event:
                    with reconstruct_lock:
                        ev = reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
                        ev["last_heartbeat"] = current_time
                    try:
                        _write_persistent_event("heartbeat", car_id, {"ts": current_time})
                        _write_readable_log("HEARTBEAT", car_id, x=f"{x:.2f}", y=f"{y:.2f}", yaw=f"{yaw:.1f}")
                    except Exception:
                        pass

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
                        if current_time - car.last_update > 5.0 and car.connected:
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

    def _prep_watchdog_loop(self):
        """监控 PREP 阶段，处理长时间未 PREP_OK 的车辆：先检查前车可见性，再重试 PREP 或重发 ORDER"""
        while self.running:
            try:
                retry_actions = []
                with car_lock:
                    car_last_updates = {
                        cid: car.last_update
                        for cid, car in cars.items()
                        if car.connected
                    }
                with reconstruct_lock:
                    if reconstruct_state.get("phase") == "preparing":
                        order = list(reconstruct_state.get("order", []))
                        now = time.time()
                        prepared_map = reconstruct_state.get("prepared", {})
                        prepared = {
                            cid for cid, ts in prepared_map.items()
                            if now - ts <= PREP_OK_VALID_WINDOW
                        }

                        for car_id in order:
                            if car_id in prepared:
                                continue

                            entry = reconstruct_state.setdefault("prep_waiting", {}).setdefault(
                                car_id, {"last_prep_ok": 0, "last_retry": 0, "retries": 0}
                            )
                            last_prep_ok = entry.get("last_prep_ok", 0)

                            if now - last_prep_ok <= PREP_RETRY_TIMEOUT:
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
                            if front_visible and now - last_retry > PREP_RETRY_PREP_INTERVAL:
                                retry_actions.append(("prep", car_id, now))
                            elif (not front_visible) and now - last_retry > PREP_RETRY_ORDER_INTERVAL:
                                retry_actions.append(("order", car_id, now))

                # 注意：广播和日志不在锁内执行，避免阻塞 PREP 回包和其它重构流程
                for action_type, car_id, now in retry_actions:
                    if action_type == "prep":
                        self.broadcast_global_command("[R,PREP]")
                        _log_reconstruct_event(f"重试 PREP（目标 {car_id} 未 PREP_OK，前车可见）")
                    else:
                        with reconstruct_lock:
                            order = list(reconstruct_state.get("order", []))
                        self.broadcast_global_command(f"[R,ORDER,{','.join(order)}]")
                        _log_reconstruct_event(f"重发 ORDER（目标 {car_id} 未 PREP_OK，前车不可见）")

                    with reconstruct_lock:
                        entry = reconstruct_state.setdefault("prep_waiting", {}).setdefault(
                            car_id, {"last_prep_ok": 0, "last_retry": 0, "retries": 0}
                        )
                        entry["last_retry"] = now
                        entry["retries"] = entry.get("retries", 0) + 1

                time.sleep(0.5)
            except Exception as e:
                print(f"❌ PREP 守护线程错误: {e}")
                time.sleep(0.5)

    def _step_watchdog_loop(self):
        """监控 STEP_ACK 与首帧图像超时，必要时重发 STEP"""
        while self.running:
            try:
                action = None
                with reconstruct_lock:
                    if reconstruct_state.get("phase") == "assembling":
                        car_id = reconstruct_state.get("waiting_car_id")
                        index = reconstruct_state.get("current_index")
                        order = list(reconstruct_state.get("order", []))

                        if car_id and index is not None and order:
                            runtime = reconstruct_state.setdefault("step_runtime", {}).setdefault(car_id, _make_step_runtime())
                            now = time.time()
                            diag = runtime.get("last_diag") or {}
                            diag_code = runtime.get("last_diag_code") or diag.get("code")
                            diag_hold_until = runtime.get("diag_hold_until_ts", 0)

                            if runtime.get("step_sent_ts", 0) > 0 and not runtime.get("assembling_started", False):
                                if diag_code == "ASM_STAGGER_WAIT" and now < diag_hold_until:
                                    continue
                                if now - runtime["step_sent_ts"] > STEP_ACK_TIMEOUT:
                                    retry_count = runtime.get("step_retry_count", 0)
                                    if retry_count < STEP_RETRY_MAX:
                                        runtime["step_retry_count"] = retry_count + 1
                                        runtime["last_retry_ts"] = now
                                        action = ("retry_ack", car_id, index, len(order), runtime["step_retry_count"])
                                    else:
                                        reconstruct_state["phase"] = "error"
                                        reconstruct_state["last_error"] = f"STEP_ACK_TIMEOUT:{car_id}"
                                        action = ("error", f"{car_id} STEP_ACK 超时次数超过上限")

                            elif runtime.get("assembling_started", False) and not runtime.get("image_started", False):
                                image_timeout = FIRST_IMAGE_TIMEOUT
                                if diag_code == "ASM_STAGGER_WAIT":
                                    image_timeout += ASM_STAGGER_WAIT_GRACE
                                elif diag_code == "ASM_NO_FIRST_IMAGE":
                                    image_timeout = 0.2

                                if now - runtime.get("step_ack_ts", now) > image_timeout:
                                    retry_count = runtime.get("step_retry_count", 0)
                                    if retry_count < STEP_RETRY_MAX:
                                        runtime["step_retry_count"] = retry_count + 1
                                        runtime["last_retry_ts"] = now
                                        action = ("retry_img", car_id, index, len(order), runtime["step_retry_count"])
                                    else:
                                        reconstruct_state["phase"] = "error"
                                        reconstruct_state["last_error"] = f"FIRST_IMAGE_TIMEOUT:{car_id}"
                                        action = ("error", f"{car_id} 首图超时次数超过上限")

                if action:
                    action_type = action[0]
                    if action_type == "retry_ack":
                        _, car_id, index, total, retry_count = action
                        _log_reconstruct_event(f"{car_id} 未收到 STEP_ACK，重发 STEP（第{retry_count}次）")
                        _send_reconstruct_step(car_id, index, total, is_separation=False)
                    elif action_type == "retry_img":
                        _, car_id, index, total, retry_count = action
                        _log_reconstruct_event(f"{car_id} 已 STEP_ACK 但无首图，重发 STEP（第{retry_count}次）")
                        _send_reconstruct_step(car_id, index, total, is_separation=False)
                    elif action_type == "error":
                        _, msg = action
                        _log_reconstruct_event(f"❌ {msg}")

                time.sleep(0.1)
            except Exception as e:
                print(f"❌ STEP 守护线程错误: {e}")
                time.sleep(0.2)

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

# 本地视觉状态
vision_lock = threading.Lock()
vision_state = {
    "binder": None,
    "estimator": None,
    "bound_cameras": {},
    "monitor_car_id": None,
    "active_car_id": None,
    "current_image": None,
    "current_error": None,
    "current_error_car": None,
    "current_has_tag": False,
    "image_timestamp": 0,
    "last_warning": None,
}

udp_server = UDPServer(UDP_HOST, UDP_PORT)

def _update_vision_snapshot(car_id, frame, error_tuple, has_tag):
    try:
        _, buffer = cv2.imencode('.jpg', frame)
        image_base64 = base64.b64encode(buffer).decode('utf-8')
    except Exception as e:
        _log_reconstruct_event(f"图像编码失败 ({car_id}): {e}")
        return

    ts = time.time()
    with vision_lock:
        vision_state["current_image"] = image_base64
        vision_state["current_error"] = error_tuple
        vision_state["current_error_car"] = car_id
        vision_state["current_has_tag"] = has_tag
        vision_state["image_timestamp"] = ts

    with reconstruct_lock:
        reconstruct_state["current_image"] = image_base64
        reconstruct_state["current_image_car"] = car_id
        reconstruct_state["current_error"] = error_tuple
        reconstruct_state["current_error_car"] = car_id
        reconstruct_state["current_has_tag"] = has_tag
        reconstruct_state["image_timestamp"] = ts

def _mark_waiting_image_started(car_id):
    with reconstruct_lock:
        if reconstruct_state.get("phase") != "assembling":
            return
        if reconstruct_state.get("waiting_car_id") != car_id:
            return
        runtime = reconstruct_state.setdefault("step_runtime", {}).setdefault(car_id, _make_step_runtime())
        if not runtime.get("image_started", False):
            runtime["image_started"] = True
            runtime["img_meta_ts"] = time.time()
            reconstruct_state["subphase"] = "GUIDING"

def _vision_loop():
    cap = None
    active_car = None

    while True:
        try:
            with reconstruct_lock:
                assembling = reconstruct_state.get("phase") == "assembling"
                waiting_car = reconstruct_state.get("waiting_car_id")

            with vision_lock:
                monitor_car = vision_state.get("monitor_car_id")
                bound_cameras = dict(vision_state.get("bound_cameras", {}))
                binder = vision_state.get("binder")
                estimator = vision_state.get("estimator")

            target_car = waiting_car if assembling and waiting_car else monitor_car

            if target_car != active_car:
                if cap:
                    cap.release()
                    cap = None
                active_car = target_car
                with vision_lock:
                    vision_state["active_car_id"] = active_car

            if not target_car or not binder or not estimator:
                time.sleep(0.1)
                continue

            cam_index = bound_cameras.get(target_car)
            if cam_index is None:
                with vision_lock:
                    vision_state["last_warning"] = f"未绑定摄像头: {target_car}"
                time.sleep(0.5)
                continue

            if cap is None:
                cap = binder._open_camera(cam_index)
                if not cap or not cap.isOpened():
                    if cap:
                        cap.release()
                    cap = None
                    with vision_lock:
                        vision_state["last_warning"] = f"摄像头打开失败: {target_car} (USB {cam_index})"
                    time.sleep(0.5)
                    continue

            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.02)
                continue

            success, z_dist, x_offset, yaw_angle, out_frame = estimator.process_frame(frame, target_car)
            # 映射到小车坐标：前进误差使用 z_dist，横向误差使用 -x_offset
            error_x = float(z_dist) * 100.0 if success else 0.0
            error_y = -float(x_offset) * 100.0 if success else 0.0
            error_yaw = float(yaw_angle) if success else 0.0

            _update_vision_snapshot(target_car, out_frame, (error_x, error_y, error_yaw), success)
            _mark_waiting_image_started(target_car)

            if assembling and waiting_car == target_car and success:
                _update_guide_controller(target_car, (error_x, error_y, error_yaw))

            time.sleep(0.02)

        except Exception as e:
            _log_reconstruct_event(f"视觉线程错误: {e}")
            time.sleep(0.1)

def record_pose_sample(car_id, timestamp, x, y, yaw):
    with save_lock:
        session = active_pose_saves.get(car_id)
        if not session:
            return
        if timestamp > session.end_time:
            return
        session.add_record(timestamp, x, y, yaw)

def finalize_pose_save(car_id):
    with save_lock:
        session = active_pose_saves.pop(car_id, None)
    if not session:
        return

    try:
        save_path = session.save_path or os.path.join(os.path.dirname(__file__), session.filename)
        with open(save_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["timestamp", "elapsed_s", "x", "y", "yaw"])
            for item in session.records:
                ts = datetime.fromtimestamp(item["timestamp"]).isoformat(timespec="milliseconds")
                elapsed = item["timestamp"] - session.start_time
                writer.writerow([ts, round(elapsed, 3), item["x"], item["y"], item["yaw"]])
        print(f"✅ 位姿数据保存完成: {save_path} (共 {len(session.records)} 条)")
    except Exception as e:
        print(f"❌ 位姿数据保存失败: {e}")

def start_pose_save(car_id, duration_sec):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{car_id}_{timestamp}.csv"
    save_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), POSE_SAVE_DIRNAME))
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, filename)

    with save_lock:
        if car_id in active_pose_saves:
            return False, "该小车正在保存中"
        session = PoseSaveSession(car_id, duration_sec, filename)
        session.save_path = save_path
        active_pose_saves[car_id] = session

    def _finalize_after_delay():
        time.sleep(duration_sec)
        finalize_pose_save(car_id)

    threading.Thread(target=_finalize_after_delay, daemon=True).start()
    return True, {"filename": filename, "save_path": save_path}

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

def _copy_topology(matrix):
    return [row[:] for row in matrix]

def _flatten_topology(matrix):
    return ','.join(str(cell) for row in matrix for cell in row)

def _apply_topology(matrix, enable):
    global communication_topology, topology_enabled
    communication_topology = _copy_topology(matrix)
    update_topology_cache()

    if enable and not topology_enabled:
        udp_server.broadcast_global_command("[T,E,1]")

    topology_cmd = f"[T,M,{_flatten_topology(communication_topology)}]"
    udp_server.broadcast_global_command(topology_cmd)

    if not enable and topology_enabled:
        udp_server.broadcast_global_command("[T,E,0]")

    topology_enabled = enable

def _build_reconstruct_topology(order):
    car_mapping = {"CAR1": 0, "CAR2": 1, "CAR3": 2, "CAR4": 3}
    matrix = [[0 for _ in range(4)] for _ in range(4)]
    if not order or len(order) < 2:
        return matrix
    for i in range(1, len(order)):
        source = order[i - 1]
        target = order[i]
        if source in car_mapping and target in car_mapping:
            matrix[car_mapping[source]][car_mapping[target]] = 1
    return matrix

def _get_reconstruct_pos_label(index, total):
    if total == 4 and 0 <= index < 4:
        return RECONSTRUCT_POS_LABELS[index]
    return f"P{index + 1}"

def _log_reconstruct_event(message):
    """记录重构事件日志"""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    log_entry = f"[{timestamp}] {message}"

    with reconstruct_lock:
        reconstruct_state["debug_logs"].append(log_entry)
        # 保持日志数量在合理范围内
        if len(reconstruct_state["debug_logs"]) > 100:
            reconstruct_state["debug_logs"] = reconstruct_state["debug_logs"][-50:]

    # 控制台输出
    print(f"📝 {log_entry}")

    # 追加持久化日志（异步安全的文件写入）
    try:
        _write_persistent_event("log", None, {"message": message, "short_ts": timestamp})
        _write_readable_log("LOG", None, msg=message)
    except Exception:
        pass


def _log_reconstruct_request(action, order=None):
    remote_addr = request.headers.get("X-Forwarded-For") or request.remote_addr
    user_agent = request.headers.get("User-Agent", "unknown")
    detail = f"{action} from {remote_addr} ua={user_agent}"
    if order:
        detail += f" order={order}"
    _log_reconstruct_event(detail)


def _reset_reconstruct_state():
    # 停止所有制导控制器
    for car_id, controller in reconstruct_state["guide_controllers"].items():
        controller.stop_guidance()
    
    reconstruct_state.update({
        "active": False,
        "phase": "idle",
        "subphase": "idle",
        "order": [],
        "prepared": {},
        "current_index": None,
        "waiting_car_id": None,
        "last_error": None,
        "prev_topology": None,
        "prev_topology_enabled": None,
        "guide_controllers": {},
        "step_runtime": {},
        "current_image": None,
        "current_image_car": None,
        "image_timestamp": 0,
        "current_error": None,
        "current_error_car": None,
        "current_has_tag": False,
        "debug_logs": []
    })


def _all_prep_ok_in_window(order):
    now = time.time()
    prepared_map = reconstruct_state.get("prepared", {})
    for car_id in order:
        ts = prepared_map.get(car_id)
        if ts is None:
            return False
        if now - ts > PREP_OK_VALID_WINDOW:
            return False
    return True


def _make_step_runtime():
    return {
        "step_sent_ts": 0,
        "step_ack_ts": 0,
        "img_meta_ts": 0,
        "assembling_started": False,
        "image_started": False,
        "step_retry_count": 0,
        "last_diag": None,
        "last_diag_code": None,
        "diag_hold_until_ts": 0,
        "last_retry_ts": 0,
    }

def _send_reconstruct_step(car_id, index, total, is_separation=False):
    pos_label = _get_reconstruct_pos_label(index, total)
    with reconstruct_lock:
        runtime = reconstruct_state.setdefault("step_runtime", {}).setdefault(car_id, _make_step_runtime())
        runtime["step_sent_ts"] = time.time()
        runtime["step_ack_ts"] = 0
        runtime["img_meta_ts"] = 0
        runtime["assembling_started"] = False
        runtime["image_started"] = False
        reconstruct_state["subphase"] = "WAIT_STEP_ACK"

    if is_separation:
        cmd = f"[R,SEP_STEP,{car_id},POS={pos_label}]"
        _log_reconstruct_event(f"发送分离指令: {cmd}")
    else:
        cmd = f"[R,STEP,{car_id},POS={pos_label}]"
        _log_reconstruct_event(f"发送拼接指令: {cmd}")
    return udp_server.send_to_car_reliable(car_id, cmd, max_retries=4)

def _advance_reconstruct_assembly(car_id):
    with reconstruct_lock:
        if reconstruct_state["phase"] != "assembling":
            return
        if reconstruct_state["waiting_car_id"] != car_id:
            return

        order = reconstruct_state["order"]
        index = reconstruct_state["current_index"]
        if index is None:
            return

        next_index = index + 1
        if next_index >= len(order):
            reconstruct_state["phase"] = "assembled"
            reconstruct_state["current_index"] = None
            reconstruct_state["waiting_car_id"] = None
            _log_reconstruct_event(f"所有小车拼接完成! 广播重构完成指令")
            udp_server.broadcast_global_command("[R,DONE]")
            _restore_previous_topology()
            return

        reconstruct_state["current_index"] = next_index
        reconstruct_state["waiting_car_id"] = order[next_index]
        _log_reconstruct_event(f"小车 {car_id} 拼接完成，开始下一辆: {order[next_index]}")
        _send_reconstruct_step(order[next_index], next_index, len(order), is_separation=False)

def _advance_reconstruct_separation(car_id):
    with reconstruct_lock:
        if reconstruct_state["phase"] != "separating":
            return
        if reconstruct_state["waiting_car_id"] != car_id:
            return

        order = reconstruct_state["order"]
        index = reconstruct_state["current_index"]
        if index is None:
            return

        next_index = index - 1
        if next_index < 0:
            _log_reconstruct_event(f"所有小车分离完成! 广播分离完成指令")
            udp_server.broadcast_global_command("[R,SEP_DONE]")
            _reset_reconstruct_state()
            return

        reconstruct_state["current_index"] = next_index
        reconstruct_state["waiting_car_id"] = order[next_index]
        _log_reconstruct_event(f"小车 {car_id} 分离完成，开始下一辆: {order[next_index]}")
        _send_reconstruct_step(order[next_index], next_index, len(order), is_separation=True)

def _run_separation_sequence(order):
    for idx in range(len(order) - 1, -1, -1):
        car_id = order[idx]
        with reconstruct_lock:
            reconstruct_state["waiting_car_id"] = car_id
            reconstruct_state["current_index"] = idx
            reconstruct_state["subphase"] = "SEPARATING"

        back_cmd = f"[M,{car_id},0,-0.15,0]"
        udp_server.send_to_car(car_id, back_cmd)
        _log_reconstruct_event(f"分离后退: {car_id} 5s")

        time.sleep(5.0)

        stop_cmd = f"[M,{car_id},0,0,0]"
        udp_server.send_to_car(car_id, stop_cmd)
        _log_reconstruct_event(f"分离停止: {car_id}")

    with reconstruct_lock:
        reconstruct_state["phase"] = "idle"
        reconstruct_state["subphase"] = "idle"
        reconstruct_state["active"] = False
        reconstruct_state["waiting_car_id"] = None
        reconstruct_state["current_index"] = None

    _restore_previous_topology()

def _wrap_angle_deg(angle_deg):
    """Wrap angle to (-180, 180]."""
    while angle_deg <= -180:
        angle_deg += 360
    while angle_deg > 180:
        angle_deg -= 360
    return angle_deg

def _get_front_heading_error(car_id):
    """Heading error = front car heading - current car heading (deg)."""
    with reconstruct_lock:
        order = list(reconstruct_state.get("order", []))
        waiting = reconstruct_state.get("waiting_car_id")

    if not order or waiting != car_id:
        return 0.0

    try:
        idx = order.index(car_id)
    except ValueError:
        return 0.0

    if idx <= 0:
        return 0.0

    front_car_id = order[idx - 1]
    with car_lock:
        if car_id not in cars or front_car_id not in cars:
            return 0.0
        if not cars[car_id].connected or not cars[front_car_id].connected:
            return 0.0
        current_heading = cars[car_id].heading
        front_heading = cars[front_car_id].heading

    return _wrap_angle_deg(front_heading - current_heading)

def _update_guide_controller(car_id, pose_error):
    """更新制导控制器"""
    with reconstruct_lock:
        if reconstruct_state["phase"] != "assembling":
            return
            
        if reconstruct_state["waiting_car_id"] != car_id:
            return
            
        # 获取或创建制导控制器
        if car_id not in reconstruct_state["guide_controllers"]:
            reconstruct_state["guide_controllers"][car_id] = GuideController(car_id)
            
        controller = reconstruct_state["guide_controllers"][car_id]
        reconstruct_state["subphase"] = "GUIDING"
        
        # 如果控制器未激活，启动它
        if not controller.active:
            controller.start_guidance()
            print(f" 启动制导控制器 ({car_id})")
            
        # 更新位姿误差
        controller.update_pose_error(*pose_error)


def handle_reconstruct_report(data):
    """处理重构相关报告"""
    # 1) 二进制图像分片优先：旧协议忽略
    if isinstance(data, bytes) and b'IMG_CHUNK' in data and b'\n' in data:
        return True

    # 2) 非图像二进制包再做安全文本解码
    if isinstance(data, bytes):
        data = data.decode('utf-8', errors='ignore')

    if not isinstance(data, str):
        return False

    data = data.strip()
    if not data:
        return False

    # 3) 文本重构指令处理
    if not (data.startswith('[') and data.endswith(']')):
        return False

    content = data[1:-1]
    parts = [part.strip() for part in content.split(',') if part.strip()]
    if len(parts) < 2 or parts[0] != "R":
        return False

    command = parts[1]
    car_id = parts[2] if len(parts) > 2 else None

    if command == "IMG_META" and car_id:
        return True

    if command == "PREP_OK" and car_id:
        with reconstruct_lock:
            if reconstruct_state["phase"] != "preparing":
                return True
            if car_id not in reconstruct_state["order"]:
                return True
            reconstruct_state["prepared"][car_id] = time.time()
            # 记录 PREP_OK 时间
            entry = reconstruct_state.setdefault("prep_waiting", {}).setdefault(car_id, {"last_prep_ok": 0, "last_retry": 0, "retries": 0})
            now = time.time()
            entry["last_prep_ok"] = now
            # 记录事件时间用于诊断
            ev = reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
            ev["last_prep_ok"] = now
            try:
                _write_persistent_event("prep_ok", car_id, {"ts": now})
                _write_readable_log("PREP_OK", car_id, status="准备完成")
            except Exception:
                pass
            # PREP_OK 支持重复上报，按有效窗口判定全车是否准备好
            if _all_prep_ok_in_window(reconstruct_state["order"]):
                reconstruct_state["phase"] = "assembling"
                reconstruct_state["subphase"] = "SEND_STEP"
                reconstruct_state["current_index"] = 0
                reconstruct_state["waiting_car_id"] = reconstruct_state["order"][0]
                _send_reconstruct_step(reconstruct_state["order"][0], 0, len(reconstruct_state["order"]), is_separation=False)

                # 启动第一辆车的制导控制器
                first_car = reconstruct_state["order"][0]
                reconstruct_state["guide_controllers"][first_car] = GuideController(first_car)
        return True

    if command == "STEP_ACK" and car_id:
        with reconstruct_lock:
            if reconstruct_state.get("phase") != "assembling":
                return True
            if reconstruct_state.get("waiting_car_id") != car_id:
                return True
            runtime = reconstruct_state.setdefault("step_runtime", {}).setdefault(car_id, _make_step_runtime())
            runtime["assembling_started"] = True
            runtime["step_ack_ts"] = time.time()
            reconstruct_state["subphase"] = "WAIT_FIRST_IMAGE"
            ev = reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
            ev["last_step_ack"] = runtime["step_ack_ts"]
            try:
                _write_persistent_event("step_ack", car_id, {"ts": runtime["step_ack_ts"]})
                _write_readable_log("STEP_ACK", car_id, status="开始组装")
            except Exception:
                pass
            _log_reconstruct_event(f"收到 STEP_ACK: {car_id}")
        return True

    if command == "DIAG" and car_id:
        params = {}
        for part in parts[3:]:
            if '=' in part:
                key, value = part.split('=', 1)
                params[key] = value
        level = params.get("L", "I")
        code = params.get("C", "UNKNOWN")
        diag_ts = time.time()
        with reconstruct_lock:
            runtime = reconstruct_state.setdefault("step_runtime", {}).setdefault(car_id, _make_step_runtime())
            runtime["last_diag"] = {"level": level, "code": code, "ts": diag_ts}
            runtime["last_diag_code"] = code
            if code == "ASM_STAGGER_WAIT":
                runtime["diag_hold_until_ts"] = max(runtime.get("diag_hold_until_ts", 0), diag_ts + ASM_STAGGER_WAIT_GRACE)
            elif code == "ASM_NO_FIRST_IMAGE":
                runtime["diag_hold_until_ts"] = 0
            ev = reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
            ev["last_diag_ts"] = diag_ts
        try:
            _write_persistent_event("diag", car_id, {"level": level, "code": code, "ts": diag_ts})
            _write_readable_log("DIAG", car_id, level=level, code=code)
        except Exception:
            pass
        _log_reconstruct_event(f"DIAG {car_id}: L={level}, C={code}")
        return True

    if command == "STEP_OK" and car_id:
        # 停止当前车的制导控制器
        with reconstruct_lock:
            if reconstruct_state.get("phase") != "assembling":
                return True
            if reconstruct_state.get("waiting_car_id") != car_id:
                return True
            if car_id in reconstruct_state["guide_controllers"]:
                reconstruct_state["guide_controllers"][car_id].stop_guidance()

        # 记录 STEP_OK 时间
        with reconstruct_lock:
            ev = reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
            ev["last_step_ok"] = time.time()
            reconstruct_state["subphase"] = "SEND_STEP"
            try:
                _write_persistent_event("step_ok", car_id, {"ts": ev["last_step_ok"]})
                _write_readable_log("STEP_OK", car_id, status="拼接完成")
            except Exception:
                pass

        _advance_reconstruct_assembly(car_id)
        return True

    if command == "SEP_OK" and car_id:
        _advance_reconstruct_separation(car_id)
        return True

    if command == "STEP_FAIL" and car_id:
        with reconstruct_lock:
            ev = reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
            ev["last_step_fail"] = time.time()
            if car_id in reconstruct_state.get("guide_controllers", {}):
                reconstruct_state["guide_controllers"][car_id].stop_guidance()

            try:
                _write_persistent_event("step_fail", car_id, {"ts": ev["last_step_fail"]})
                _write_readable_log("STEP_FAIL", car_id, status="拼接失败")
            except Exception:
                pass

            if reconstruct_state.get("phase") == "assembling":
                waiting_car = reconstruct_state.get("waiting_car_id")
                index = reconstruct_state.get("current_index")
                order = reconstruct_state.get("order", [])
                if waiting_car == car_id and index is not None:
                    _log_reconstruct_event(f"收到 STEP_FAIL，重试 STEP: {car_id}")
                    reconstruct_state["subphase"] = "SEND_STEP"
                    _send_reconstruct_step(car_id, index, len(order), is_separation=False)
                    return True

            reconstruct_state["phase"] = "error"
            reconstruct_state["last_error"] = data
        return True

    if command == "SEP_FAIL":
        with reconstruct_lock:
            reconstruct_state["phase"] = "error"
            reconstruct_state["last_error"] = data
            if car_id:
                ev = reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
                ev["last_step_fail"] = time.time()
                if car_id in reconstruct_state.get("guide_controllers", {}):
                    reconstruct_state["guide_controllers"][car_id].stop_guidance()
        return True

    return False

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

@app.route('/api/reconstruct/start', methods=['POST'])
def start_reconstruct():
    formation_info = get_formation_info()
    if formation_info.get('enabled'):
        return jsonify({'success': False, 'error': '编队执行中，无法启动重构'}), 400

    data = request.json or {}
    order = data.get('order') or ["CAR1", "CAR2", "CAR3", "CAR4"]
    if not isinstance(order, list) or len(order) < 1:
        return jsonify({'success': False, 'error': '需要提供至少1辆小车的重构顺序'}), 400
    if len(set(order)) != len(order):
        return jsonify({'success': False, 'error': '重构顺序中包含重复小车'}), 400

    _log_reconstruct_request("reconstruct_start", order=order)

    with car_lock:
        missing = [
            car_id for car_id in order
            if car_id not in cars or not cars[car_id].connected
        ]
    if missing:
        return jsonify({'success': False, 'error': f'小车 {missing[0]} 未连接'}), 400

    with reconstruct_lock:
        if reconstruct_state["phase"] not in ("idle", "assembled"):
            return jsonify({'success': False, 'error': '重构流程进行中，请先结束或中止'}), 400
        reconstruct_state["active"] = True
        reconstruct_state["phase"] = "preparing"
        reconstruct_state["subphase"] = "WAIT_PREP_OK_ALL"
        reconstruct_state["order"] = order
        reconstruct_state["prepared"] = {}
        reconstruct_state["step_runtime"] = {}
        reconstruct_state["current_index"] = None
        reconstruct_state["waiting_car_id"] = None
        reconstruct_state["last_error"] = None
        reconstruct_state["prev_topology"] = _copy_topology(communication_topology)
        reconstruct_state["prev_topology_enabled"] = topology_enabled

        # 初始化 prep_waiting 表
        prep_map = {}
        now = time.time()
        for car_id in order:
            prep_map[car_id] = {"last_prep_ok": 0, "last_retry": 0, "retries": 0}
        reconstruct_state["prep_waiting"] = prep_map

    # 进入重构模式后立即切换到重构拓扑，方便准备阶段获取前后车关系
    _switch_to_reconstruct_topology()

    udp_server.broadcast_global_command("[R,PREP]")
    udp_server.broadcast_global_command(f"[R,ORDER,{','.join(order)}]")

    return jsonify({
        'success': True,
        'message': '重构流程已启动，等待准备完成',
        'order': order,
        'waiting_car_id': None
    })

@app.route('/api/reconstruct/separate', methods=['POST'])
def separate_reconstruct():
    with reconstruct_lock:
        if reconstruct_state["phase"] != "assembled":
            return jsonify({'success': False, 'error': '当前未处于拼接完成状态'}), 400
        order = reconstruct_state["order"]
        if not order:
            return jsonify({'success': False, 'error': '未找到重构顺序'}), 400

        reconstruct_state["phase"] = "separating"
        reconstruct_state["current_index"] = len(order) - 1
        reconstruct_state["waiting_car_id"] = order[-1]
        reconstruct_state["last_error"] = None

    threading.Thread(target=_run_separation_sequence, args=(list(order),), daemon=True).start()

    return jsonify({
        'success': True,
        'message': '分离流程已启动',
        'order': order,
        'waiting_car_id': order[-1]
    })

@app.route('/api/reconstruct/abort', methods=['POST'])
def abort_reconstruct():
    _log_reconstruct_request("reconstruct_abort")
    udp_server.broadcast_global_command("[R,ABORT]")
    with reconstruct_lock:
        _restore_previous_topology()
        _reset_reconstruct_state()
    return jsonify({
        'success': True,
        'message': '重构流程已中止'
    })

@app.route('/api/reconstruct/status')
def get_reconstruct_status():
    with reconstruct_lock:
        now = time.time()
        prepared_valid = [
            car_id for car_id, ts in reconstruct_state["prepared"].items()
            if now - ts <= PREP_OK_VALID_WINDOW
        ]
        waiting_car = reconstruct_state["waiting_car_id"]
        waiting_runtime = {}
        if waiting_car:
            waiting_runtime = reconstruct_state.get("step_runtime", {}).get(waiting_car, {})

        state_snapshot = {
            'active': reconstruct_state["active"],
            'phase': reconstruct_state["phase"],
            'subphase': reconstruct_state.get("subphase", "idle"),
            'order': list(reconstruct_state["order"]),
            'prepared': prepared_valid,
            'waiting_car_id': reconstruct_state["waiting_car_id"],
            'waiting_runtime': waiting_runtime,
            'last_error': reconstruct_state["last_error"],
            'current_image_car': reconstruct_state["current_image_car"],
            'image_timestamp': reconstruct_state["image_timestamp"],
            'current_error': reconstruct_state.get("current_error"),
            'current_error_car': reconstruct_state.get("current_error_car"),
            'current_has_tag': reconstruct_state.get("current_has_tag"),
            'debug_logs': reconstruct_state["debug_logs"][-20:]  # 返回最近20条日志
        }
    return jsonify(state_snapshot)


@app.route('/api/reconstruct/events')
def get_reconstruct_events():
    """获取每车事件时间戳"""
    with reconstruct_lock:
        events_snapshot = reconstruct_state.get("events", {})
    return jsonify({'events': events_snapshot})


@app.route('/api/reconstruct/params', methods=['GET', 'POST'])
def get_or_set_reconstruct_params():
    """获取或设置重构参数（超时、窗口、GUIDE 频率等）"""
    global PREP_RETRY_TIMEOUT, PREP_RETRY_PREP_INTERVAL, PREP_RETRY_ORDER_INTERVAL

    if request.method == 'GET':
        return jsonify({
            'prep_retry_timeout_s': PREP_RETRY_TIMEOUT,
            'prep_retry_prep_interval_s': PREP_RETRY_PREP_INTERVAL,
            'prep_retry_order_interval_s': PREP_RETRY_ORDER_INTERVAL,
            'guide_frequency_hz': reconstruct_state.get("guide_frequency"),
            'guide_timeout_s': reconstruct_state.get("guide_timeout")
        })

    data = request.json or {}
    with reconstruct_lock:
        if 'prep_retry_timeout_s' in data:
            PREP_RETRY_TIMEOUT = float(data['prep_retry_timeout_s'])
        if 'prep_retry_prep_interval_s' in data:
            PREP_RETRY_PREP_INTERVAL = float(data['prep_retry_prep_interval_s'])
        if 'prep_retry_order_interval_s' in data:
            PREP_RETRY_ORDER_INTERVAL = float(data['prep_retry_order_interval_s'])
        if 'guide_frequency_hz' in data:
            reconstruct_state["guide_frequency"] = float(data['guide_frequency_hz'])
        if 'guide_timeout_s' in data:
            reconstruct_state["guide_timeout"] = float(data['guide_timeout_s'])

    return jsonify({'success': True})


@app.route('/api/reconstruct/image')
def get_reconstruct_image():
    """获取当前重构图像"""
    with reconstruct_lock:
        if reconstruct_state["current_image"]:
            return jsonify({
                'success': True,
                'image': reconstruct_state["current_image"],
                'car_id': reconstruct_state["current_image_car"],
                'timestamp': reconstruct_state["image_timestamp"],
                'error': reconstruct_state.get("current_error"),
                'has_tag': reconstruct_state.get("current_has_tag", False)
            })
        else:
            return jsonify({
                'success': False,
                'message': '暂无图像数据'
            })


@app.route('/api/vision/image')
def get_vision_image():
    """获取视觉监控图像（用于前端弹窗）"""
    with vision_lock:
        if vision_state["current_image"]:
            return jsonify({
                'success': True,
                'image': vision_state["current_image"],
                'car_id': vision_state["current_error_car"],
                'timestamp': vision_state["image_timestamp"],
                'error': vision_state.get("current_error"),
                'has_tag': vision_state.get("current_has_tag", False),
                'warning': vision_state.get("last_warning")
            })
        return jsonify({
            'success': False,
            'message': '暂无图像数据',
            'warning': vision_state.get("last_warning")
        })


@app.route('/api/vision/select', methods=['POST'])
def select_vision_car():
    data = request.json or {}
    car_id = data.get('car_id')
    with vision_lock:
        vision_state["monitor_car_id"] = car_id
    return jsonify({'success': True, 'car_id': car_id})


@app.route('/api/vision/status')
def get_vision_status():
    with vision_lock:
        return jsonify({
            'monitor_car_id': vision_state.get("monitor_car_id"),
            'active_car_id': vision_state.get("active_car_id"),
            'bound_cameras': vision_state.get("bound_cameras", {}),
            'warning': vision_state.get("last_warning")
        })


@app.route('/api/vision/bind/scan', methods=['POST'])
def scan_and_bind_vision_devices():
    def _scan_task():
        with vision_lock:
            binder = vision_state.get("binder")
        if not binder:
            return
        mapping = binder.scan_and_bind()
        with vision_lock:
            vision_state["bound_cameras"] = mapping
            vision_state["last_warning"] = None if mapping else "未绑定到任何摄像头"

    threading.Thread(target=_scan_task, daemon=True).start()
    return jsonify({'success': True, 'message': '已开始重新扫描摄像头'})


@app.route('/api/reconstruct/logs')
def get_reconstruct_logs():
    """获取重构日志"""
    with reconstruct_lock:
        return jsonify({
            'logs': reconstruct_state["debug_logs"],
            'count': len(reconstruct_state["debug_logs"])
        })


@app.route('/api/reconstruct/logfile')
def get_reconstruct_logfile():
    """下载或查看按日持久化的重构日志。参数: date=YYYYMMDD (默认今天), lines=整数(返回尾部N行)"""
    date = request.args.get('date')
    try:
        if not date:
            date = datetime.now().strftime('%Y%m%d')
        lines = int(request.args.get('lines', '200'))
    except Exception:
        return jsonify({'success': False, 'error': '无效参数'}), 400

    log_dir = _ensure_log_dir()
    filename = os.path.join(log_dir, f"reconstruct-{date}.jsonl")
    if not os.path.exists(filename):
        return jsonify({'success': False, 'error': '日志文件不存在', 'file': filename}), 404

    try:
        with file_log_lock:
            with open(filename, 'r', encoding='utf-8') as f:
                all_lines = f.read().splitlines()
        tail = all_lines[-lines:]
        # 解析为 JSON 对象列表（如果可能）
        parsed = []
        for ln in tail:
            try:
                parsed.append(json.loads(ln))
            except Exception:
                parsed.append({'raw': ln})
        return jsonify({'success': True, 'file': os.path.basename(filename), 'lines': parsed})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)}), 500

def _switch_to_reconstruct_topology():
    order = reconstruct_state.get("order") or []
    if len(order) < 2:
        return
    reconstruct_topology = _build_reconstruct_topology(order)
    _log_reconstruct_event(f"切换到重构拓扑: {order}")
    _log_reconstruct_event(f"重构拓扑矩阵: {reconstruct_topology}")
    _apply_topology(reconstruct_topology, enable=True)

def _restore_previous_topology():
    prev_topology = reconstruct_state.get("prev_topology")
    prev_enabled = reconstruct_state.get("prev_topology_enabled")

    if prev_topology is None or prev_enabled is None:
        return

    if prev_enabled:
        _apply_topology(prev_topology, enable=True)
        return

    _apply_topology(DEFAULT_COMMUNICATION_TOPOLOGY, enable=False)

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

@app.route('/api/save_pose', methods=['POST'])
def save_pose():
    data = request.json
    car_id = data.get('car_id')
    duration = int(data.get('duration', 10))

    if not car_id:
        return jsonify({'success': False, 'error': '缺少car_id'})
    if duration not in (10, 20, 30):
        return jsonify({'success': False, 'error': '仅支持 10/20/30 秒'})

    with car_lock:
        if car_id not in cars or not cars[car_id].connected:
            return jsonify({'success': False, 'error': f'小车 {car_id} 未连接'})

    success, result = start_pose_save(car_id, duration)
    if not success:
        return jsonify({'success': False, 'error': result})

    return jsonify({
        'success': True,
        'message': f'已开始保存 {car_id} 位姿数据，持续 {duration}s',
        'filename': result["filename"],
        'save_path': result["save_path"],
        'duration': duration
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
        print("⚠️ 无法获取详细网络信息，请安装 netifaces")

if __name__ == '__main__':
    get_network_info()
    update_topology_cache()

    if DeviceBinder and PoseEstimator:
        try:
            if "TEMPLATE_DIR" in VISION_CONFIG:
                VISION_CONFIG["TEMPLATE_DIR"] = os.path.join(
                    os.path.dirname(__file__), "Car_vision_system", "templates"
                )
            binder = DeviceBinder(VISION_CONFIG)
            estimator = PoseEstimator(VISION_CONFIG)
            bound = {}
            try:
                bound = binder.scan_and_bind()
            except Exception as e:
                print(f"⚠️ 摄像头绑定失败，继续运行: {e}")
            with vision_lock:
                vision_state["binder"] = binder
                vision_state["estimator"] = estimator
                vision_state["bound_cameras"] = bound
                vision_state["last_warning"] = None if bound else "未绑定到任何摄像头"

            threading.Thread(target=_vision_loop, daemon=True).start()
            print("👁️ 本地视觉线程已启动")
        except Exception as e:
            print(f"⚠️ 初始化本地视觉失败，继续运行: {e}")
    else:
        print("⚠️ 本地视觉模块不可用，跳过视觉线程")
    
    if udp_server.start():
        print("✅ UDP服务器启动成功")
        init_formation_controller(cars, udp_server)
        print(f"📡 广播频率: {1 / broadcast_interval:.0f}Hz ({broadcast_interval * 1000:.0f}ms间隔)")
        print(f"📡 广播分组大小: 每组最多 {broadcast_group_size} 辆小车")
        print(f"📢 使用子网广播地址，端口: {BROADCAST_PORT}")
        local_ip = get_local_ip()
        print(f"🌐 服务器本地IP地址: {local_ip}")
        print(f"💡 访问 http://{local_ip}:{WEB_PORT} 打开控制界面")
        # 启动自动归档守护线程（自动下载日志到 logs/auto_downloads）
        try:
            archive_thread = threading.Thread(target=_archive_worker_loop, args=(LOG_ARCHIVE_INTERVAL_S,), daemon=True)
            archive_thread.start()
            print(f"🔁 自动归档守护已启动，间隔 {LOG_ARCHIVE_INTERVAL_S}s，目录: {os.path.join(os.path.dirname(__file__), LOG_ARCHIVE_DIRNAME)}")
        except Exception as e:
            print(f"⚠️ 启动自动归档守护失败: {e}")
        app.run(host='0.0.0.0', port=WEB_PORT, debug=False, use_reloader=False, threaded=True)
    else:
        print("❌ UDP服务器启动失败，无法运行应用")