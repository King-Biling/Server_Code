# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.logging_utils
 模块职责: 重构事件的持久化日志记录与自动归档
--------------------------------------------------------------------------------
 本模块负责把重构流程中的关键事件（心跳、PREP_OK、STEP_ACK、STEP_OK 等）
 以两种形式落盘，并提供后台自动归档能力：

   1. JSONL 结构化日志: logs/reconstruct-YYYYMMDD.jsonl
      每行一个 JSON 对象，便于程序化解析与回放。
   2. 纯文本可读日志: logs/reconstruct-readable-YYYYMMDD.log
      便于用记事本直接打开查看。
   3. 自动归档: 定时把当日 jsonl 压缩为 .jsonl.gz 存入 logs/auto_downloads。

 依赖关系: 仅依赖 config（路径/间隔常量）与 state（文件写入锁），无循环依赖。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import os
import json
import gzip
import time
from datetime import datetime

from . import config
from . import state


def _ensure_log_dir():
    """确保日志目录存在并返回其绝对路径。"""
    log_dir = os.path.abspath(os.path.join(config.PROJECT_ROOT, config.LOG_DIRNAME))
    try:
        os.makedirs(log_dir, exist_ok=True)
    except Exception:
        pass
    return log_dir


def write_persistent_event(event_type, car_id=None, payload=None):
    """将事件以 JSONL 形式追加到按日切分的日志文件中。"""
    try:
        log_dir = _ensure_log_dir()
        date_str = datetime.now().strftime("%Y%m%d")
        filename = os.path.join(log_dir, f"reconstruct-{date_str}.jsonl")
        entry = {
            "ts": datetime.now().isoformat(timespec='milliseconds'),
            "type": event_type,
            "car_id": car_id,
            "payload": payload or {},
        }
        line = json.dumps(entry, ensure_ascii=False)
        with state.file_log_lock:
            with open(filename, 'a', encoding='utf-8') as f:
                f.write(line + "\n")
    except Exception as e:
        # 持久化日志失败不阻塞主流程，仅在控制台记录
        print(f" 写持久化日志失败: {e}")


def write_readable_log(event_type, car_id=None, **kwargs):
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

        with state.file_log_lock:
            with open(filename, 'a', encoding='utf-8') as f:
                f.write(line + "\n")
    except Exception:
        pass


def _ensure_archive_dir():
    """确保自动归档目录存在并返回其绝对路径。"""
    archive_dir = os.path.abspath(os.path.join(config.PROJECT_ROOT, config.LOG_ARCHIVE_DIRNAME))
    try:
        os.makedirs(archive_dir, exist_ok=True)
    except Exception:
        pass
    return archive_dir


def archive_current_log():
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
        print(f" 自动归档日志: {dst}")
        write_persistent_event("archive", None, {"file": os.path.basename(dst)})
        return True, dst
    except Exception as e:
        print(f" 自动归档失败: {e}")
        return False, str(e)


def archive_worker_loop(interval_s=None):
    """后台守护线程循环：按固定间隔归档当日日志。"""
    if interval_s is None:
        interval_s = config.LOG_ARCHIVE_INTERVAL_S
    while True:
        try:
            archive_current_log()
        except Exception as e:
            print(f" 归档守护错误: {e}")
        time.sleep(interval_s)
