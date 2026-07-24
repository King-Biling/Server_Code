# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.reconstruct
 模块职责: 重构（拼接 / 分离）状态机与报文处理
--------------------------------------------------------------------------------
 本模块是整个服务器端的“重构流程”核心，负责作为调度者驱动小车完成编队拼接
 与分离，包含：

   日志与事件:
     log_reconstruct_event      记录重构事件到内存/控制台/持久化日志
     log_reconstruct_request    记录来自 HTTP 的重构请求（附带来源信息）

   拓扑切换（与重构日志耦合，故放在本模块）:
     switch_to_prep_topology    切换到准备阶段中心拓扑
     switch_to_chain_topology   切换到拼接阶段链式拓扑
     restore_previous_topology  恢复重构前的拓扑

   状态管理:
     reset_reconstruct_state    复位重构状态机
     make_step_runtime          构造单车 STEP 运行时记录
     enter_error_state          进入错误态
     all_prep_ok_in_window      判断是否全员在有效窗口内 PREP_OK

   流程推进:
     send_reconstruct_step      下发 STEP / SEP_STEP 指令并计握手超时
     advance_reconstruct_assembly   拼接完成后推进到下一辆
     advance_reconstruct_separation 分离完成后推进到上一辆
     run_separation_sequence    顺序执行分离（后退-停车）
     is_anchor_car              判断是否为首车（锚点，不参与制导）
     ensure_guide_controller_started 确保待拼接车的制导控制器已启动
     update_guide_controller    用最新位姿误差更新制导控制器

   报文入口:
     handle_reconstruct_report  解析并分派所有 [R,...] 报文（含二进制图像分片）

 依赖关系:
   - 依赖 config / state / topology / guide。
   - 与 net / vision 存在相互调用，跨模块调用一律通过 state.udp_server 单例
     或函数内延迟导入完成，避免循环导入。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import time
import threading
from datetime import datetime

from . import config
from . import state
from . import topology
from .guide import GuideController


# ------------------------------------------------------------------------------
# 便捷转发：连发广播（复用 topology 的实现，保持单一来源）
# ------------------------------------------------------------------------------
def burst_broadcast_command(command, repeat=3, delay=0.05):
    """连发多次全局广播指令（转发到 topology.burst_broadcast_command）。"""
    topology.burst_broadcast_command(command, repeat=repeat, delay=delay)


# ------------------------------------------------------------------------------
# 日志与事件
# ------------------------------------------------------------------------------
def log_reconstruct_event(message):
    """记录重构事件日志（内存环形缓冲 + 控制台）。"""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    log_entry = f"[{timestamp}] {message}"

    with state.reconstruct_lock:
        state.reconstruct_state["debug_logs"].append(log_entry)
        # 保持日志数量在合理范围内
        if len(state.reconstruct_state["debug_logs"]) > 100:
            state.reconstruct_state["debug_logs"] = state.reconstruct_state["debug_logs"][-50:]

    # 控制台输出
    print(f" {log_entry}")


def log_reconstruct_request(action, order=None):
    """记录来自 HTTP 的重构请求（含来源地址与 UA）。"""
    # 延迟导入 flask.request，避免在非请求上下文中引用出错
    from flask import request
    remote_addr = request.headers.get("X-Forwarded-For") or request.remote_addr
    user_agent = request.headers.get("User-Agent", "unknown")
    detail = f"{action} from {remote_addr} ua={user_agent}"
    if order:
        detail += f" order={order}"
    log_reconstruct_event(detail)


# ------------------------------------------------------------------------------
# 拓扑切换（与重构日志耦合）
# ------------------------------------------------------------------------------
def switch_to_prep_topology(order):
    """切换到准备阶段中心拓扑：首车向其后所有车单向可见。"""
    if not order or len(order) < 2:
        return
    car_mapping = config.CAR_INDEX_MAP
    matrix = [[0 for _ in range(4)] for _ in range(4)]
    head = order[0]
    if head not in car_mapping:
        return
    head_index = car_mapping[head]
    for target in order[1:]:
        if target in car_mapping:
            matrix[head_index][car_mapping[target]] = 1
    log_reconstruct_event(f"切换到准备阶段中心拓扑: {order}")
    log_reconstruct_event(f"准备阶段拓扑矩阵: {matrix}")
    topology.apply_topology(matrix, enable=True)


def switch_to_chain_topology(order):
    """切换到拼接阶段链式拓扑：前车 -> 后车 单向可见。"""
    if not order or len(order) < 2:
        return
    chain_topology = topology.build_reconstruct_topology(order)
    log_reconstruct_event(f"切换到拼接阶段链式拓扑: {order}")
    log_reconstruct_event(f"拼接阶段拓扑矩阵: {chain_topology}")
    topology.apply_topology(chain_topology, enable=True)


def restore_previous_topology():
    """恢复重构前保存的拓扑（无保存则回退到默认矩阵并禁用）。"""
    prev_topology = state.reconstruct_state.get("prev_topology")
    prev_enabled = state.reconstruct_state.get("prev_topology_enabled")

    if prev_topology is None or prev_enabled is None:
        return

    if prev_enabled:
        topology.apply_topology(prev_topology, enable=True)
        return

    topology.apply_topology(config.DEFAULT_COMMUNICATION_TOPOLOGY, enable=False)


# ------------------------------------------------------------------------------
# 位置标签
# ------------------------------------------------------------------------------
def get_reconstruct_pos_label(index, total):
    """根据索引返回位置标签（4 车时用 HEAD/MID2/MID3/TAIL，否则 P{n}）。"""
    if total == 4 and 0 <= index < 4:
        return config.RECONSTRUCT_POS_LABELS[index]
    return f"P{index + 1}"


# ------------------------------------------------------------------------------
# 状态管理
# ------------------------------------------------------------------------------
def reset_reconstruct_state():
    """复位重构状态机：停止所有制导控制器并清空运行时状态。"""
    # 停止所有制导控制器
    for car_id, controller in state.reconstruct_state["guide_controllers"].items():
        controller.stop_guidance()

    state.reconstruct_state.update({
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
        "guide_velocity": {},
        "debug_logs": [],
    })


def all_prep_ok_in_window(order):
    """判断 order 中所有车是否都在有效窗口内完成了 PREP_OK。"""
    now = time.time()
    prepared_map = state.reconstruct_state.get("prepared", {})
    for car_id in order:
        ts = prepared_map.get(car_id)
        if ts is None:
            return False
        if now - ts > config.PREP_OK_VALID_WINDOW:
            return False
    return True


def make_step_runtime():
    """构造单车 STEP 运行时记录（握手/图像/重试等时间戳与标志）。"""
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


def enter_error_state(msg):
    """统一设置进入错误态：记录错误、停止激活并写日志。"""
    with state.reconstruct_lock:
        state.reconstruct_state["phase"] = "error"
        state.reconstruct_state["active"] = False
        state.reconstruct_state["subphase"] = "idle"
        state.reconstruct_state["last_error"] = msg
    log_reconstruct_event(f"进入错误态: {msg}")


# ------------------------------------------------------------------------------
# 流程推进
# ------------------------------------------------------------------------------
def send_reconstruct_step(car_id, index, total, is_separation=False):
    """下发 STEP / SEP_STEP 指令，登记监控目标并在发出后开始计握手超时。"""
    pos_label = get_reconstruct_pos_label(index, total)
    with state.reconstruct_lock:
        runtime = state.reconstruct_state.setdefault("step_runtime", {}).setdefault(car_id, make_step_runtime())
        # 注意：step_sent_ts 在“真正发出 STEP 之后”再刻，避免把摄像头切换/发送耗时算进握手超时窗口
        runtime["step_sent_ts"] = 0
        runtime["step_ack_ts"] = 0
        runtime["img_meta_ts"] = 0
        runtime["assembling_started"] = False
        runtime["image_started"] = False
        runtime["step_retry_count"] = runtime.get("step_retry_count", 0)
        state.reconstruct_state["subphase"] = "WAIT_STEP_ACK"

    if is_separation:
        cmd = f"[R,SEP_STEP,{car_id},POS={pos_label}]"
        log_reconstruct_event(f"发送分离指令: {cmd}")
    else:
        cmd = f"[R,STEP,{car_id},POS={pos_label}]"
        log_reconstruct_event(f"发送拼接指令: {cmd}")

    # 只做“切换目标”的登记，实际打开摄像头交给 vision_loop 异步完成；
    # 绝不能在 UDP 接收线程里同步 switch_to（会阻塞接收线程，连带 STEP_ACK 都收不到）。
    try:
        with state.vision_lock:
            state.vision_state['monitor_car_id'] = car_id
    except Exception:
        pass

    # 关键修复：STEP 改用“子网广播”下发，与 PREP/ORDER/ASM_START 完全同一条已被验证可达的链路。
    # 之前 STEP 走单播(send_to_car ->(car_ip,8081))，而本项目准备阶段小车只依赖广播指令+僚车状态
    # 报文即可就位，单播链路从未被真正验证过；实测现象正是“准备阶段一切正常，一进入拼接、首次需要
    # 单播 STEP 时全线失灵、STEP_ACK 永远收不到”。小车端 handle_step 已按 target==自身 过滤，广播安全。
    # 用 burst_broadcast_command 连发多次，进一步对抗 UDP 丢包。
    burst_broadcast_command(cmd, repeat=3, delay=0.05)
    success = True

    # STEP 已经真正发出，此刻才开始计握手超时
    with state.reconstruct_lock:
        runtime = state.reconstruct_state.setdefault("step_runtime", {}).setdefault(car_id, make_step_runtime())
        runtime["step_sent_ts"] = time.time()
    return success


def advance_reconstruct_assembly(car_id):
    """某车拼接完成后推进到下一辆；全部完成则广播 DONE 并恢复拓扑。"""
    with state.reconstruct_lock:
        if state.reconstruct_state["phase"] != "assembling":
            return
        if state.reconstruct_state["waiting_car_id"] != car_id:
            return

        order = state.reconstruct_state["order"]
        index = state.reconstruct_state["current_index"]
        if index is None:
            return

        next_index = index + 1
        if next_index >= len(order):
            state.reconstruct_state["phase"] = "assembled"
            state.reconstruct_state["current_index"] = None
            state.reconstruct_state["waiting_car_id"] = None
            log_reconstruct_event(f"所有小车拼接完成! 广播重构完成指令")
            state.udp_server.broadcast_global_command("[R,DONE]")
            restore_previous_topology()
            return

        state.reconstruct_state["current_index"] = next_index
        state.reconstruct_state["waiting_car_id"] = order[next_index]
        # 仅设置监控目标，摄像头切换交给 vision_loop 异步执行，
        # 避免在 UDP 接收线程中阻塞（switch_to 含冷却+重试，可能耗时秒级）
        with state.vision_lock:
            state.vision_state["monitor_car_id"] = order[next_index]
        log_reconstruct_event(f"小车 {car_id} 拼接完成，开始下一辆: {order[next_index]}")
        send_reconstruct_step(order[next_index], next_index, len(order), is_separation=False)


def is_anchor_car(car_id):
    """首车作为基准等待被拼接，不参与视觉制导。"""
    with state.reconstruct_lock:
        order = list(state.reconstruct_state.get("order", []))
    if not order:
        return False
    return car_id == order[0]


def ensure_guide_controller_started(car_id):
    """确保等待拼接车辆的制导控制器已启动，即便暂时未检测到Tag也持续下发保活GUIDE。"""
    if is_anchor_car(car_id):
        return
    with state.reconstruct_lock:
        if state.reconstruct_state.get("phase") != "assembling":
            return
        if state.reconstruct_state.get("waiting_car_id") != car_id:
            return
        runtime = state.reconstruct_state.setdefault("step_runtime", {}).setdefault(car_id, make_step_runtime())
        if not runtime.get("assembling_started", False):
            return
        if car_id not in state.reconstruct_state["guide_controllers"]:
            state.reconstruct_state["guide_controllers"][car_id] = GuideController(car_id)
        controller = state.reconstruct_state["guide_controllers"][car_id]
        if not controller.active:
            controller.start_guidance()
            state.reconstruct_state["subphase"] = "GUIDING_WAIT_VISION"
            log_reconstruct_event(f"启动制导控制器（等待视觉锁定）: {car_id}")


def advance_reconstruct_separation(car_id):
    """某车分离完成后推进到上一辆；全部完成则广播 SEP_DONE 并复位。"""
    with state.reconstruct_lock:
        if state.reconstruct_state["phase"] != "separating":
            return
        if state.reconstruct_state["waiting_car_id"] != car_id:
            return

        order = state.reconstruct_state["order"]
        index = state.reconstruct_state["current_index"]
        if index is None:
            return

        next_index = index - 1
        if next_index < 0:
            log_reconstruct_event(f"所有小车分离完成! 广播分离完成指令")
            state.udp_server.broadcast_global_command("[R,SEP_DONE]")
            reset_reconstruct_state()
            return

        state.reconstruct_state["current_index"] = next_index
        state.reconstruct_state["waiting_car_id"] = order[next_index]
        log_reconstruct_event(f"小车 {car_id} 分离完成，开始下一辆: {order[next_index]}")
        send_reconstruct_step(order[next_index], next_index, len(order), is_separation=True)


def run_separation_sequence(order):
    """顺序执行分离：从队尾到队首依次后退 5 秒再停车，最后复位并恢复拓扑。"""
    for idx in range(len(order) - 1, -1, -1):
        car_id = order[idx]
        with state.reconstruct_lock:
            state.reconstruct_state["waiting_car_id"] = car_id
            state.reconstruct_state["current_index"] = idx
            state.reconstruct_state["subphase"] = "SEPARATING"

        back_cmd = f"[M,{car_id},-0.15,0,0]"
        state.udp_server.send_to_car(car_id, back_cmd)
        log_reconstruct_event(f"分离后退: {car_id} 5s")

        time.sleep(5.0)

        stop_cmd = f"[M,{car_id},0,0,0]"
        state.udp_server.send_to_car(car_id, stop_cmd)
        log_reconstruct_event(f"分离停止: {car_id}")

    with state.reconstruct_lock:
        state.reconstruct_state["phase"] = "idle"
        state.reconstruct_state["subphase"] = "idle"
        state.reconstruct_state["active"] = False
        state.reconstruct_state["waiting_car_id"] = None
        state.reconstruct_state["current_index"] = None

    restore_previous_topology()


def update_guide_controller(car_id, pose_error, has_tag=True):
    """更新制导控制器。has_tag=False 表示这一帧未检测到 AprilTag，仅用于保活，不能触发到达判定。"""
    if is_anchor_car(car_id):
        return
    with state.reconstruct_lock:
        if state.reconstruct_state["phase"] != "assembling":
            return

        if state.reconstruct_state["waiting_car_id"] != car_id:
            return

        runtime = state.reconstruct_state.setdefault("step_runtime", {}).setdefault(car_id, make_step_runtime())
        if not runtime.get("assembling_started", False):
            return

        # 获取或创建制导控制器
        if car_id not in state.reconstruct_state["guide_controllers"]:
            state.reconstruct_state["guide_controllers"][car_id] = GuideController(car_id)

        controller = state.reconstruct_state["guide_controllers"][car_id]
        state.reconstruct_state["subphase"] = "GUIDING"

        # 如果控制器未激活，启动它
        if not controller.active:
            controller.start_guidance()
            print(f" 启动制导控制器 ({car_id})")

        # 更新位姿误差
        controller.update_pose_error(*pose_error, has_tag=has_tag)


# ------------------------------------------------------------------------------
# 报文入口
# ------------------------------------------------------------------------------
def handle_reconstruct_report(data):
    """处理重构相关报告，返回 True 表示已消费该报文。"""
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
        start_assembling = False
        start_order = None
        resend_asm_start = False
        with state.reconstruct_lock:
            phase = state.reconstruct_state["phase"]
            if phase == "assembling":
                resend_asm_start = True
            elif phase != "preparing":
                return True
            if not resend_asm_start:
                if car_id not in state.reconstruct_state["order"]:
                    return True
                state.reconstruct_state["prepared"][car_id] = time.time()
                # 记录 PREP_OK 时间
                entry = state.reconstruct_state.setdefault("prep_waiting", {}).setdefault(car_id, {"last_prep_ok": 0, "last_retry": 0, "retries": 0})
                now = time.time()
                entry["last_prep_ok"] = now
                # 记录事件时间用于诊断
                ev = state.reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
                ev["last_prep_ok"] = now
                # PREP_OK 支持重复上报，全员到齐后自动进入组装拓扑与流程
                order = state.reconstruct_state.get("order", [])
                if order and all(cid in state.reconstruct_state["prepared"] for cid in order):
                    switch_to_chain_topology(order)
                    state.reconstruct_state["phase"] = "assembling"
                    state.reconstruct_state["subphase"] = "SEND_STEP"
                    state.reconstruct_state["current_index"] = None
                    state.reconstruct_state["waiting_car_id"] = None
                    start_assembling = True
                    start_order = list(order)
        if resend_asm_start:
            burst_broadcast_command("[R,ASM_START]")
            log_reconstruct_event(f"补救 ASM_START 广播 (来自 {car_id} 的 PREP_OK 重传)")
            return True
        if start_assembling and start_order:
            burst_broadcast_command("[R,ASM_START]")
            log_reconstruct_event("全员准备就绪，自动开启顺序拼接流程...")
            if len(start_order) < 2:
                # 仅有首车时视为无需拼接，直接完成
                with state.reconstruct_lock:
                    state.reconstruct_state["phase"] = "assembled"
                    state.reconstruct_state["subphase"] = "idle"
                    state.reconstruct_state["current_index"] = None
                    state.reconstruct_state["waiting_car_id"] = None
                log_reconstruct_event("仅有首车，无需拼接，直接完成")
                state.udp_server.broadcast_global_command("[R,DONE]")
                restore_previous_topology()
                return True

            # 首车为锚点，不下发 STEP；从第2辆开始拼接到首车
            second_car = start_order[1]
            with state.reconstruct_lock:
                state.reconstruct_state["current_index"] = 1
                state.reconstruct_state["waiting_car_id"] = second_car
                state.reconstruct_state["subphase"] = "SEND_STEP"
            with state.vision_lock:
                state.vision_state["monitor_car_id"] = second_car
            send_reconstruct_step(second_car, 1, len(start_order), is_separation=False)
        return True

    if command == "STEP_ACK" and car_id:
        if is_anchor_car(car_id):
            log_reconstruct_event(f"忽略锚点车 STEP_ACK: {car_id}")
            return True
        with state.reconstruct_lock:
            if state.reconstruct_state.get("phase") != "assembling":
                return True
            if state.reconstruct_state.get("waiting_car_id") != car_id:
                return True
            runtime = state.reconstruct_state.setdefault("step_runtime", {}).setdefault(car_id, make_step_runtime())
            runtime["assembling_started"] = True
            runtime["step_ack_ts"] = time.time()
            state.reconstruct_state["subphase"] = "GUIDING" if runtime.get("image_started", False) else "WAIT_FIRST_IMAGE"
            ev = state.reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
            ev["last_step_ack"] = runtime["step_ack_ts"]
            log_reconstruct_event(f"收到 STEP_ACK: {car_id}")
        # 仅登记监控目标，摄像头切换交给 vision_loop 异步执行，
        # 避免在 UDP 接收线程内做阻塞式串行切换（会拖住后续回包处理）。
        with state.vision_lock:
            state.vision_state["monitor_car_id"] = car_id
        ensure_guide_controller_started(car_id)
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
        with state.reconstruct_lock:
            runtime = state.reconstruct_state.setdefault("step_runtime", {}).setdefault(car_id, make_step_runtime())
            runtime["last_diag"] = {"level": level, "code": code, "ts": diag_ts}
            runtime["last_diag_code"] = code
            if code == "ASM_STAGGER_WAIT":
                runtime["diag_hold_until_ts"] = max(runtime.get("diag_hold_until_ts", 0), diag_ts + config.ASM_STAGGER_WAIT_GRACE)
            elif code == "ASM_NO_FIRST_IMAGE":
                runtime["diag_hold_until_ts"] = 0
            ev = state.reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
            ev["last_diag_ts"] = diag_ts
        log_reconstruct_event(f"DIAG {car_id}: L={level}, C={code}")
        return True

    if command == "STEP_OK" and car_id:
        # 停止当前车的制导控制器
        with state.reconstruct_lock:
            if state.reconstruct_state.get("phase") != "assembling":
                return True
            if state.reconstruct_state.get("waiting_car_id") != car_id:
                return True
            if car_id in state.reconstruct_state["guide_controllers"]:
                state.reconstruct_state["guide_controllers"][car_id].stop_guidance()

        # 记录 STEP_OK 时间
        with state.reconstruct_lock:
            ev = state.reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
            ev["last_step_ok"] = time.time()
            state.reconstruct_state["subphase"] = "SEND_STEP"

        advance_reconstruct_assembly(car_id)
        return True

    if command == "SEP_OK" and car_id:
        advance_reconstruct_separation(car_id)
        return True

    if command == "STEP_FAIL" and car_id:
        with state.reconstruct_lock:
            ev = state.reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
            ev["last_step_fail"] = time.time()
            if car_id in state.reconstruct_state.get("guide_controllers", {}):
                state.reconstruct_state["guide_controllers"][car_id].stop_guidance()

            if state.reconstruct_state.get("phase") == "assembling":
                waiting_car = state.reconstruct_state.get("waiting_car_id")
                index = state.reconstruct_state.get("current_index")
                order = state.reconstruct_state.get("order", [])
                if waiting_car == car_id and index is not None:
                    log_reconstruct_event(f"收到 STEP_FAIL，重试 STEP: {car_id}")
                    state.reconstruct_state["subphase"] = "SEND_STEP"
                    send_reconstruct_step(car_id, index, len(order), is_separation=False)
                    return True

            enter_error_state(data)
        return True

    if command == "SEP_FAIL":
        with state.reconstruct_lock:
            enter_error_state(data)
            if car_id:
                ev = state.reconstruct_state.setdefault("events", {}).setdefault(car_id, {})
                ev["last_step_fail"] = time.time()
                if car_id in state.reconstruct_state.get("guide_controllers", {}):
                    state.reconstruct_state["guide_controllers"][car_id].stop_guidance()
        return True

    return False
