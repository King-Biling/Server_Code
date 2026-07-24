# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.guide
 模块职责: 末端制导控制器与航向误差计算
--------------------------------------------------------------------------------
 本模块负责“拼接阶段”对当前待拼接小车的末端视觉制导：

   GuideController          单车制导控制器：独立线程周期下发 [R,GUIDE,...]
   wrap_angle_deg           把角度归一化到 (-180, 180]
   get_front_heading_error  计算“前车朝向 - 本车朝向”的航向误差(度)

 控制原理:
   - 输入为视觉解算得到的位姿误差 (error_x 前进/cm, error_y 横向/cm, error_yaw)，
     以及由前车/本车朝向差得到的航向误差。
   - 采用简化 P 控制，配合漏斗解耦（横向/航向未对中时先整列、暂缓前进）、
     死区抑制与限幅，最终以子网广播方式下发制导速度。
   - 只有在“确有 AprilTag”且到达条件连续多帧稳定成立时，才判定 DONE=1，
     防止无 Tag 或单帧抖动误触发到达。

 依赖关系:
   - 依赖 config（增益/阈值/限幅）、state（重构状态与锁、udp_server 单例）。
   - 下发指令通过 state.udp_server.broadcast_global_command 完成。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import time
import threading

from . import config
from . import state


def wrap_angle_deg(angle_deg):
    """把角度归一化到 (-180, 180]。"""
    while angle_deg <= -180:
        angle_deg += 360
    while angle_deg > 180:
        angle_deg -= 360
    return angle_deg


def get_front_heading_error(car_id):
    """航向误差 = 前车朝向 - 本车朝向（度）。

    仅当本车正是当前待拼接车、且本车与前车均已连接时才有效，否则返回 0。
    """
    with state.reconstruct_lock:
        order = list(state.reconstruct_state.get("order", []))
        waiting = state.reconstruct_state.get("waiting_car_id")

    if not order or waiting != car_id:
        return 0.0

    try:
        idx = order.index(car_id)
    except ValueError:
        return 0.0

    if idx <= 0:
        return 0.0

    front_car_id = order[idx - 1]
    with state.car_lock:
        if car_id not in state.cars or front_car_id not in state.cars:
            return 0.0
        if not state.cars[car_id].connected or not state.cars[front_car_id].connected:
            return 0.0
        current_heading = state.cars[car_id].heading
        front_heading = state.cars[front_car_id].heading

    return wrap_angle_deg(front_heading - current_heading)


class GuideController:
    """末端制导控制器：为单辆待拼接小车独立运行制导线程。"""

    def __init__(self, car_id):
        self.car_id = car_id
        self.active = False
        self.last_guide_time = 0
        self.target_reached = False
        self.last_pose_error = None
        self.last_vision_time = 0
        self.last_has_tag = False          # 最新一帧是否真正检测到 AprilTag
        self.last_tag_time = 0             # 最近一次“检测到 Tag”的时间
        self.reached_hold_count = 0        # 到达条件连续成立的帧数（去抖，防止单帧误判 DONE）
        self.guide_thread = None

    def start_guidance(self):
        """开始制导（幂等：已激活则直接返回）。"""
        if self.active:
            return
        self.active = True
        self.target_reached = False
        self.guide_thread = threading.Thread(target=self._guidance_loop, daemon=True)
        self.guide_thread.start()

    def stop_guidance(self):
        """停止制导。"""
        self.active = False

    def _guidance_loop(self):
        """制导循环：按 guide_frequency 周期下发速度或保活指令。"""
        guide_interval = 1.0 / state.reconstruct_state["guide_frequency"]

        while self.active:
            try:
                if self.target_reached:
                    self._send_guide_command(0, 0, 0, done=1)
                    time.sleep(guide_interval)
                    continue

                # 没有可用位姿时仍保持刷新，避免车端超时
                if self.last_pose_error is None:
                    self._send_guide_command(0, 0, 0, done=0)
                    time.sleep(guide_interval)
                    continue

                # 计算制导速度
                vx, vy, vz, done = self._calculate_guide_velocity()

                # 发送制导指令
                self._send_guide_command(vx, vy, vz, done)

                # 目标已到达时继续保活发送 DONE=1，等待车端回 STEP_OK 再推进下一辆
                if done == 1:
                    self.target_reached = True
                    time.sleep(guide_interval)
                    continue

                time.sleep(guide_interval)

            except Exception as e:
                print(f" 制导循环错误 ({self.car_id}): {e}")
                time.sleep(guide_interval)

    def _calculate_guide_velocity(self):
        """根据最新位姿误差计算制导速度，返回 (vx, vy, vz, done)。"""
        if self.last_pose_error is None:
            self.reached_hold_count = 0
            return 0, 0, 0, 0

        # 没有实时视觉（>0.5s 无 Tag）时，绝不判定到达，只保活并停车，
        # 等待视觉重新锁定。此前的 bug 是无 Tag 时用 error_x=0 误判到达导致假 DONE。
        if (not self.last_has_tag) or (time.time() - self.last_tag_time > 0.5):
            self.reached_hold_count = 0
            return 0.0, 0.0, 0.0, 0

        error_x, error_y, _ = self.last_pose_error
        yaw_error_deg = get_front_heading_error(self.car_id)

        # 检查是否到达目标：必须在“确有 Tag”的前提下，且连续多帧稳定成立才判定 DONE，
        # 防止单帧抖动误触发。error_x 是到 Tag 的纵向距离(cm)，需大于 0 才是有效锁定。
        if (0.0 < error_x < 20.0 and abs(error_y) < 2.0 and abs(yaw_error_deg) < 2.0):
            self.reached_hold_count += 1
            if self.reached_hold_count >= 3:
                return 0.0, 0.0, 0.0, 1
            # 尚未去抖完成，先停车保持，不推进
            return 0.0, 0.0, 0.0, 0
        else:
            self.reached_hold_count = 0

        # PID控制（简化版，只有P项）
        vx = config.GUIDE_P_GAIN_X * error_x  # 前进误差直接乘以增益
        vy = config.GUIDE_P_GAIN_Y * error_y  # 横向误差直接乘以增益
        vz = config.GUIDE_P_GAIN_YAW * yaw_error_deg  # 角度误差直接乘以增益

        # 漏斗解耦：横向或航向未对中时先整列、暂缓前进（判定对象是横向 error_y / 航向 yaw，
        # 而非纵向距离 error_x——远处纵向大属于正常，应当前进而不是禁止前进）
        if abs(error_y) > config.GUIDE_FUNNEL_Y_THRESHOLD or abs(yaw_error_deg) > config.GUIDE_FUNNEL_YAW_THRESHOLD:
            vx = 0.0

        # 死区抑制
        if abs(error_x) < 1.5:
            vx = 0.0
        if abs(error_y) < 1.5:
            vy = 0.0
        if abs(yaw_error_deg) < 2.0:
            vz = 0.0

        # 线速度限幅
        speed_magnitude = (vx ** 2 + vy ** 2) ** 0.5
        if speed_magnitude > config.GUIDE_SPEED_LIMIT:
            scale = config.GUIDE_SPEED_LIMIT / speed_magnitude
            vx *= scale
            vy *= scale

        # 角速度独立限幅
        if vz > config.GUIDE_MAX_VZ:
            vz = config.GUIDE_MAX_VZ
        if vz < -config.GUIDE_MAX_VZ:
            vz = -config.GUIDE_MAX_VZ

        return vx, vy, vz, 0

    def _send_guide_command(self, vx, vy, vz, done):
        """发送制导指令（子网广播下发，车端按 target==自身 过滤）。"""
        # 命名字段格式（推荐）
        cmd = f"[R,GUIDE,{self.car_id},VX={vx:.3f},VY={vy:.3f},VZ={vz:.3f},DONE={done}]"
        # 关键修复：GUIDE 与 STEP 一样改用“子网广播”下发，走与 PREP/ORDER/ASM_START 相同的已验证链路。
        # 之前 GUIDE 走单播(send_to_car)，即便 STEP 握手成功，制导速度也发不到车端 —— 表现为“车报 ACK
        # 却一直不动”。GUIDE 为 20Hz 高频，单发即可（丢一帧下一帧立即补上），无需 burst。
        # 车端 handle_guide 已按 target==自身 过滤，广播安全。broadcast_global_command 内部已重发多次。
        success = state.udp_server.broadcast_global_command(cmd)

        if success:
            self.last_guide_time = time.time()
            # 记录最后一次下发 GUIDE 时间到全局事件表
            try:
                with state.reconstruct_lock:
                    ev = state.reconstruct_state.setdefault("events", {}).setdefault(self.car_id, {})
                    ev["last_guide_sent"] = time.time()
                    if done == 1:
                        ev["last_guide_done"] = time.time()
                        state.reconstruct_state["subphase"] = "WAIT_STEP_OK"
                    # 记录最新制导速度，供监控画面实时叠加显示
                    state.reconstruct_state.setdefault("guide_velocity", {})[self.car_id] = {
                        "vx": vx, "vy": vy, "vz": vz, "done": done, "ts": time.time()
                    }
            except Exception:
                pass
            if done == 0:
                print(f"制导指令 ({self.car_id}): vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}")
            else:
                print(f"制导完成 ({self.car_id})")

    def update_pose_error(self, error_x, error_y, error_yaw, has_tag=True):
        """更新位姿误差。has_tag=False 表示这一帧没有检测到 AprilTag（不能作为到达依据）。"""
        now = time.time()
        self.last_pose_error = (error_x, error_y, error_yaw)
        self.last_has_tag = bool(has_tag)
        if has_tag:
            self.last_tag_time = now
            self.last_vision_time = now
        # 注意：无 Tag 时不刷新 last_vision_time，让“视觉超时”保护能够真正生效
