# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.pose_save
 模块职责: 小车位姿数据的定时采样与落盘
--------------------------------------------------------------------------------
 本模块提供一套“按时长采样并保存位姿”的能力：

   record_pose_sample   在遥测回调中被调用，向活跃会话追加一条位姿采样
   finalize_pose_save   会话到期后把累积的位姿写入 CSV 文件
   start_pose_save      启动一次保存会话，并在后台线程中到期落盘

 数据落盘位置: <工程根>/pose_data/<car_id>_<时间戳>.csv

 依赖关系: config（目录）、state（会话表与锁）、models（PoseSaveSession）。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import os
import csv
import time
import threading
from datetime import datetime

from . import config
from . import state
from .models import PoseSaveSession


def record_pose_sample(car_id, timestamp, x, y, yaw):
    """向该车的活跃保存会话追加一条采样（超出时间窗的采样被忽略）。"""
    with state.save_lock:
        session = state.active_pose_saves.get(car_id)
        if not session:
            return
        if timestamp > session.end_time:
            return
        session.add_record(timestamp, x, y, yaw)


def finalize_pose_save(car_id):
    """结束该车的保存会话，把累积的位姿采样写入 CSV 文件。"""
    with state.save_lock:
        session = state.active_pose_saves.pop(car_id, None)
    if not session:
        return

    try:
        save_path = session.save_path or os.path.join(config.PROJECT_ROOT, session.filename)
        with open(save_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["timestamp", "elapsed_s", "x", "y", "yaw"])
            for item in session.records:
                ts = datetime.fromtimestamp(item["timestamp"]).isoformat(timespec="milliseconds")
                elapsed = item["timestamp"] - session.start_time
                writer.writerow([ts, round(elapsed, 3), item["x"], item["y"], item["yaw"]])
        print(f" 位姿数据保存完成: {save_path} (共 {len(session.records)} 条)")
    except Exception as e:
        print(f" 位姿数据保存失败: {e}")


def start_pose_save(car_id, duration_sec):
    """启动一次位姿保存会话，并在后台线程中到期自动落盘。

    返回 (success, result)：
      success=True 时 result 为 {"filename", "save_path"}；
      success=False 时 result 为错误说明字符串。
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{car_id}_{timestamp}.csv"
    save_dir = os.path.abspath(os.path.join(config.PROJECT_ROOT, config.POSE_SAVE_DIRNAME))
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, filename)

    with state.save_lock:
        if car_id in state.active_pose_saves:
            return False, "该小车正在保存中"
        session = PoseSaveSession(car_id, duration_sec, filename)
        session.save_path = save_path
        state.active_pose_saves[car_id] = session

    def _finalize_after_delay():
        time.sleep(duration_sec)
        finalize_pose_save(car_id)

    threading.Thread(target=_finalize_after_delay, daemon=True).start()
    return True, {"filename": filename, "save_path": save_path}
