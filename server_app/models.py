# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.models
 模块职责: 纯数据模型定义（无业务逻辑、无外部依赖）
--------------------------------------------------------------------------------
 本模块定义两个纯数据类：

   Car               单辆小车的运行时状态快照（位置/朝向/电量/速度/连接状态等）
   PoseSaveSession   一次位姿数据保存会话（记录时间窗内的位姿采样）

 这两个类只承载数据与最基础的数据操作，不涉及网络、锁或全局状态，
 便于被其它模块自由实例化与引用，也便于单元测试。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import time


class Car:
    """单辆小车的运行时状态。"""

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


class PoseSaveSession:
    """一次位姿数据保存会话：在给定时长内累积位姿采样，到期后统一落盘。"""

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
            "yaw": yaw,
        })
