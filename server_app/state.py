# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.state
 模块职责: 跨模块共享的可变状态与线程锁集中定义
--------------------------------------------------------------------------------
 本模块存放整个服务器端在运行期间会被读写的全部“共享状态”，包括：

   1. 小车表 cars 及其锁 car_lock
   2. 重构状态机 reconstruct_state 及其可重入锁 reconstruct_lock
   3. 视觉状态 vision_state 及其锁 vision_lock
   4. 位姿保存会话表 active_pose_saves 及其锁 save_lock
   5. 通信拓扑矩阵 communication_topology / topology_enabled / topology_cache
   6. 广播开关与参数 broadcast_enabled / broadcast_interval / broadcast_group_size
   7. 日志文件写入锁 file_log_lock
   8. UDP 服务器单例 udp_server（在 app 装配阶段赋值）

 重要约定（保持行为不变的关键）:
   本模块中“会被重新赋值”的标量（broadcast_enabled、topology_enabled、
   communication_topology、udp_server 等），其它模块必须以
   `state.broadcast_enabled` 的形式读写，禁止使用
   `from state import broadcast_enabled` 拷贝取值，
   否则运行时对这些量的修改无法跨模块传播，会破坏原有行为。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import threading

from . import config

# ------------------------------------------------------------------------------
# 小车表：car_id -> Car 实例
# ------------------------------------------------------------------------------
cars = {}
car_lock = threading.Lock()

# ------------------------------------------------------------------------------
# 重构状态机（拼接 / 分离）
# ------------------------------------------------------------------------------
# 使用可重入锁 RLock，兼容原实现中同一线程多次进入临界区的写法。
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
    "guide_velocity": {},   # 每车最新制导速度，供监控画面叠加
    "guide_frequency": 20,  # Hz
    "guide_timeout": 0.3,   # 秒
    "target_distance": 5.0, # 5cm
    "current_image": None,      # 当前显示的图像
    "current_image_car": None,  # 当前图像对应的小车
    "image_timestamp": 0,       # 图像时间戳
    "current_error": None,      # 当前误差 (x, y, yaw)
    "current_error_car": None,
    "current_has_tag": False,
    "debug_logs": [],           # 调试日志
    "prep_waiting": {},
    "events": {},
    "step_runtime": {},
    "subphase": "idle",
}

# ------------------------------------------------------------------------------
# 位姿保存会话表：car_id -> PoseSaveSession
# ------------------------------------------------------------------------------
save_lock = threading.Lock()
active_pose_saves = {}

# ------------------------------------------------------------------------------
# 日志文件写入锁（持久化 jsonl / 可读日志共用）
# ------------------------------------------------------------------------------
file_log_lock = threading.Lock()

# ------------------------------------------------------------------------------
# 广播开关与参数（运行时可被 HTTP 接口重新赋值）
# ------------------------------------------------------------------------------
broadcast_enabled = False
broadcast_interval = 0.07
broadcast_group_size = 1

# ------------------------------------------------------------------------------
# 通信拓扑（运行时可被 HTTP 接口 / 重构流程重新赋值）
# ------------------------------------------------------------------------------
# 初始值取默认矩阵的深拷贝，避免与 config.DEFAULT_COMMUNICATION_TOPOLOGY 共享引用。
communication_topology = [row[:] for row in config.DEFAULT_COMMUNICATION_TOPOLOGY]
topology_enabled = False
topology_cache = {}

# ------------------------------------------------------------------------------
# 视觉状态
# ------------------------------------------------------------------------------
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
    # === 显示/检测解耦用的轻量 overlay 状态 ===
    # 检测线程只更新这些“数据”（不做重复 JPEG 编码），MJPEG 线程读取后在原始采集帧上
    # 轻量绘制（标签框 + 误差 + 制导速度）。这样显示帧率与检测帧率彻底独立，
    # 标签入画导致的检测耗时尖峰不再拖累画面流畅度。
    "overlay_cam_index": None,          # 当前 overlay 对应的物理相机索引
    "overlay_car_id": None,             # 当前 overlay 对应的小车
    "overlay_corners": None,            # 最近一次检测到的标签角点（全分辨率，np.ndarray 或 None）
    "overlay_error": (0.0, 0.0, 0.0),   # (error_x, error_y, error_yaw)
    "overlay_has_tag": False,
    "overlay_ts": 0,                    # 最近一次检测更新时间（用于判定新鲜度）
}

manual_bind_state = {
    "active": False,
    "cameras": [],
    "current_index": 0,
    "current_frame_b64": None,

    "bound_so_far": {},
    "finished": False,
}

# 低频快照节流状态：仅供 vision 模块的低频快照函数使用
snapshot_lowfreq_state = {"last_ts": 0.0}

# ------------------------------------------------------------------------------
# UDP 服务器单例
# ------------------------------------------------------------------------------
# 在 app 装配阶段创建并赋值；其它模块通过 state.udp_server 访问，
# 以打破 net 层与 reconstruct 层之间的循环依赖。
udp_server = None
