# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.routes.reconstruct
 模块职责: 重构流程控制与状态/参数/日志/图像查询接口
--------------------------------------------------------------------------------
 提供重构（拼接/分离）流程的全部 HTTP 接口:

   POST /api/reconstruct/start     启动重构流程（下发 ORDER + PREP）
   POST /api/reconstruct/separate  在拼接完成态启动分离流程
   POST /api/reconstruct/abort     中止重构流程并恢复拓扑
   GET  /api/reconstruct/status    查询重构状态机快照
   GET  /api/reconstruct/events    查询每车事件时间戳
   GET/POST /api/reconstruct/params  查询/设置重构可调参数
   GET  /api/reconstruct/image     获取当前重构监控图像（base64）
   GET  /api/reconstruct/logs      获取内存中的重构调试日志
   GET  /api/reconstruct/logfile   下载/查看按日持久化的 jsonl 日志

 说明:
   业务逻辑复用 server_app.reconstruct 模块；可调参数（PREP 重试、GUIDE 漏斗
   阈值等）存放于 config，本模块通过 config.xxx 直接读写，确保守护线程与制导
   线程能感知运行时变更。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import os
import json
import time
import threading
from datetime import datetime

from flask import Blueprint, request, jsonify

from .. import config
from .. import state
from .. import logging_utils
from .. import topology as topology_ops
from .. import reconstruct as reconstruct_ops

reconstruct_bp = Blueprint('reconstruct', __name__)


@reconstruct_bp.route('/api/reconstruct/start', methods=['POST'])
def start_reconstruct():
    """启动重构流程：切换准备拓扑并广播 ORDER 与 PREP。"""
    # 延迟导入编队信息查询，避免与 formation_controller 形成模块级耦合
    from formation_controller import get_formation_info

    formation_info = get_formation_info()
    if formation_info.get('enabled'):
        return jsonify({'success': False, 'error': '编队执行中，无法启动重构'}), 400

    data = request.json or {}
    order = data.get('order') or ["CAR1", "CAR2", "CAR3", "CAR4"]
    if not isinstance(order, list) or len(order) < 1:
        return jsonify({'success': False, 'error': '需要提供至少1辆小车的重构顺序'}), 400
    if len(set(order)) != len(order):
        return jsonify({'success': False, 'error': '重构顺序中包含重复小车'}), 400

    reconstruct_ops.log_reconstruct_request("reconstruct_start", order=order)

    with state.car_lock:
        missing = [
            car_id for car_id in order
            if car_id not in state.cars or not state.cars[car_id].connected
        ]
    if missing:
        return jsonify({'success': False, 'error': f'小车 {missing[0]} 未连接'}), 400

    with state.reconstruct_lock:
        # 仅在“进行中”的阶段拒绝重复启动；error/idle/assembled 均允许重新开始，
        # 避免上一轮出错后卡在 error 态无法再启动（前端会显示“进行中请先中止”的假象）。
        if state.reconstruct_state["phase"] in ("preparing", "assembling", "separating"):
            return jsonify({'success': False, 'error': '重构流程进行中，请先结束或中止'}), 400
        # 若上一轮遗留了制导控制器（例如异常/错误态），先全部停止再重置，防止旧线程继续下发 GUIDE
        for _cid, _ctrl in list(state.reconstruct_state.get("guide_controllers", {}).items()):
            try:
                _ctrl.stop_guidance()
            except Exception:
                pass
        state.reconstruct_state["active"] = True
        state.reconstruct_state["phase"] = "preparing"
        state.reconstruct_state["subphase"] = "WAIT_PREP_OK_ALL"
        state.reconstruct_state["order"] = order
        state.reconstruct_state["prepared"] = {}
        state.reconstruct_state["step_runtime"] = {}
        state.reconstruct_state["guide_controllers"] = {}
        state.reconstruct_state["events"] = {}
        state.reconstruct_state["current_index"] = None
        state.reconstruct_state["waiting_car_id"] = None
        state.reconstruct_state["last_error"] = None
        state.reconstruct_state["prev_topology"] = topology_ops.copy_topology(state.communication_topology)
        state.reconstruct_state["prev_topology_enabled"] = state.topology_enabled

        # 初始化 prep_waiting 表
        prep_map = {}
        now = time.time()
        for car_id in order:
            prep_map[car_id] = {"last_prep_ok": 0, "last_retry": 0, "retries": 0}
        state.reconstruct_state["prep_waiting"] = prep_map

    # 让视觉线程在准备阶段就把“下一辆要拼接的车”的画面提前调出来（多车时为 order[1]，
    # 单车时为 order[0]）。这样小车到达准备位置时画面已就绪，无需等到 assembling。
    with state.vision_lock:
        state.vision_state["monitor_car_id"] = order[1] if len(order) >= 2 else order[0]

    # 进入重构模式后切换为准备阶段中心拓扑
    reconstruct_ops.switch_to_prep_topology(order)

    reconstruct_ops.burst_broadcast_command(f"[R,ORDER,{','.join(order)}]")
    reconstruct_ops.burst_broadcast_command("[R,PREP]")

    return jsonify({
        'success': True,
        'message': '重构流程已启动，等待准备完成',
        'order': order,
        'waiting_car_id': None
    })


@reconstruct_bp.route('/api/reconstruct/separate', methods=['POST'])
def separate_reconstruct():
    """在拼接完成态启动分离流程（后台线程顺序执行）。"""
    with state.reconstruct_lock:
        if state.reconstruct_state["phase"] != "assembled":
            return jsonify({'success': False, 'error': '当前未处于拼接完成状态'}), 400
        order = state.reconstruct_state["order"]
        if not order:
            return jsonify({'success': False, 'error': '未找到重构顺序'}), 400

        state.reconstruct_state["phase"] = "separating"
        state.reconstruct_state["current_index"] = len(order) - 1
        state.reconstruct_state["waiting_car_id"] = order[-1]
        state.reconstruct_state["last_error"] = None

    threading.Thread(target=reconstruct_ops.run_separation_sequence, args=(list(order),), daemon=True).start()

    return jsonify({
        'success': True,
        'message': '分离流程已启动',
        'order': order,
        'waiting_car_id': order[-1]
    })


@reconstruct_bp.route('/api/reconstruct/abort', methods=['POST'])
def abort_reconstruct():
    """中止重构流程：广播 ABORT，恢复拓扑并复位状态。"""
    reconstruct_ops.log_reconstruct_request("reconstruct_abort")
    state.udp_server.broadcast_global_command("[R,ABORT]")
    with state.reconstruct_lock:
        reconstruct_ops.restore_previous_topology()
        reconstruct_ops.reset_reconstruct_state()
    return jsonify({
        'success': True,
        'message': '重构流程已中止'
    })


@reconstruct_bp.route('/api/reconstruct/status')
def get_reconstruct_status():
    """返回重构状态机快照，供前端轮询展示。"""
    with state.reconstruct_lock:
        prepared_valid = list(state.reconstruct_state["prepared"].keys())
        waiting_car = state.reconstruct_state["waiting_car_id"]
        waiting_runtime = {}
        if waiting_car:
            waiting_runtime = state.reconstruct_state.get("step_runtime", {}).get(waiting_car, {})

        state_snapshot = {
            'active': state.reconstruct_state["active"],
            'phase': state.reconstruct_state["phase"],
            'subphase': state.reconstruct_state.get("subphase", "idle"),
            'order': list(state.reconstruct_state["order"]),
            'prepared': prepared_valid,
            'waiting_car_id': state.reconstruct_state["waiting_car_id"],
            'waiting_runtime': waiting_runtime,
            'last_error': state.reconstruct_state["last_error"],
            'current_image_car': state.reconstruct_state["current_image_car"],
            'image_timestamp': state.reconstruct_state["image_timestamp"],
            'current_error': state.reconstruct_state.get("current_error"),
            'current_error_car': state.reconstruct_state.get("current_error_car"),
            'current_has_tag': state.reconstruct_state.get("current_has_tag"),
            'debug_logs': state.reconstruct_state["debug_logs"][-20:]  # 返回最近20条日志
        }

    # 当前应显示的视觉目标车：拼接阶段用 waiting_car_id，其它阶段用 monitor_car_id
    with state.vision_lock:
        monitor_car = state.vision_state.get("monitor_car_id")
    state_snapshot['vision_target_car'] = waiting_car or monitor_car

    return jsonify(state_snapshot)


@reconstruct_bp.route('/api/reconstruct/events')
def get_reconstruct_events():
    """获取每车事件时间戳。"""
    with state.reconstruct_lock:
        events_snapshot = state.reconstruct_state.get("events", {})
    return jsonify({'events': events_snapshot})


@reconstruct_bp.route('/api/reconstruct/params', methods=['GET', 'POST'])
def get_or_set_reconstruct_params():
    """获取或设置重构参数（超时、窗口、GUIDE 频率等）。

    可调参数存放于 config，通过 config.xxx 读写以保证守护/制导线程感知变更。
    """
    if request.method == 'GET':
        return jsonify({
            'prep_retry_timeout_s': config.PREP_RETRY_TIMEOUT,
            'prep_retry_prep_interval_s': config.PREP_RETRY_PREP_INTERVAL,
            'prep_retry_order_interval_s': config.PREP_RETRY_ORDER_INTERVAL,
            'guide_frequency_hz': state.reconstruct_state.get("guide_frequency"),
            'guide_timeout_s': state.reconstruct_state.get("guide_timeout"),
            'guide_funnel_y_threshold_cm': config.GUIDE_FUNNEL_Y_THRESHOLD,
            'guide_funnel_yaw_threshold_deg': config.GUIDE_FUNNEL_YAW_THRESHOLD
        })

    data = request.json or {}
    with state.reconstruct_lock:
        if 'prep_retry_timeout_s' in data:
            config.PREP_RETRY_TIMEOUT = float(data['prep_retry_timeout_s'])
        if 'prep_retry_prep_interval_s' in data:
            config.PREP_RETRY_PREP_INTERVAL = float(data['prep_retry_prep_interval_s'])
        if 'prep_retry_order_interval_s' in data:
            config.PREP_RETRY_ORDER_INTERVAL = float(data['prep_retry_order_interval_s'])
        if 'guide_frequency_hz' in data:
            state.reconstruct_state["guide_frequency"] = float(data['guide_frequency_hz'])
        if 'guide_timeout_s' in data:
            state.reconstruct_state["guide_timeout"] = float(data['guide_timeout_s'])
        if 'guide_funnel_y_threshold_cm' in data:
            config.GUIDE_FUNNEL_Y_THRESHOLD = float(data['guide_funnel_y_threshold_cm'])
        if 'guide_funnel_yaw_threshold_deg' in data:
            config.GUIDE_FUNNEL_YAW_THRESHOLD = float(data['guide_funnel_yaw_threshold_deg'])

    return jsonify({'success': True})


@reconstruct_bp.route('/api/reconstruct/image')
def get_reconstruct_image():
    """获取当前重构监控图像（base64）。"""
    with state.reconstruct_lock:
        if state.reconstruct_state["current_image"]:
            return jsonify({
                'success': True,
                'image': state.reconstruct_state["current_image"],
                'car_id': state.reconstruct_state["current_image_car"],
                'timestamp': state.reconstruct_state["image_timestamp"],
                'error': state.reconstruct_state.get("current_error"),
                'has_tag': state.reconstruct_state.get("current_has_tag", False)
            })
        else:
            return jsonify({
                'success': False,
                'message': '暂无图像数据'
            })


@reconstruct_bp.route('/api/reconstruct/logs')
def get_reconstruct_logs():
    """获取内存中的重构调试日志。"""
    with state.reconstruct_lock:
        return jsonify({
            'logs': state.reconstruct_state["debug_logs"],
            'count': len(state.reconstruct_state["debug_logs"])
        })


@reconstruct_bp.route('/api/reconstruct/logfile')
def get_reconstruct_logfile():
    """下载或查看按日持久化的重构日志。参数: date=YYYYMMDD (默认今天), lines=整数(返回尾部N行)。"""
    date = request.args.get('date')
    try:
        if not date:
            date = datetime.now().strftime('%Y%m%d')
        lines = int(request.args.get('lines', '200'))
    except Exception:
        return jsonify({'success': False, 'error': '无效参数'}), 400

    log_dir = logging_utils._ensure_log_dir()
    filename = os.path.join(log_dir, f"reconstruct-{date}.jsonl")
    if not os.path.exists(filename):
        return jsonify({'success': False, 'error': '日志文件不存在', 'file': filename}), 404

    try:
        with state.file_log_lock:
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
