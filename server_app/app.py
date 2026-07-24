# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.app
 模块职责: Flask 应用装配与程序主入口
--------------------------------------------------------------------------------
 本模块把拆分后的各功能子模块装配为一个可运行的服务器程序，职责包括：

   _configure_utf8_console  统一 Windows 控制台与标准输出为 UTF-8
   create_app               创建 Flask 应用、启用 CORS、注册全部蓝图
   _init_vision             加载本地视觉引擎、交互选择部署车辆并完成摄像头绑定
   get_local_ip             获取本机对外 IP（用于打印访问地址）
   get_network_info         打印本机网络接口信息
   main                     程序主入口：初始化视觉 -> 启动 UDP -> 运行 Flask

 装配要点（保持行为不变的关键）:
   1. 在此创建 UDPServer 单例并赋值给 state.udp_server，供其它模块统一访问，
      从而打破 net 层与 reconstruct 层之间的循环依赖。
   2. 本地视觉引擎 (DeviceBinder / PoseEstimator / CONFIG) 在此按需导入，
      并把其 CONFIG 注入到 state.vision_state["vision_config"]，供 vision 模块使用。
   3. 蓝图注册顺序与原实现保持一致；编队蓝图 formation_bp 一并注册，URL 不变。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import os
import sys
import socket
import threading

# 兼容“直接运行本文件”（例如在 IDE 中对 app.py 按运行/调试）的场景。
# 直接以脚本方式运行时，本文件不属于任何包，相对导入 (from . import xxx)
# 会因“no known parent package”失败。这里在导入前做 PEP 366 引导：
# 把工程根目录加入 sys.path，并显式设置 __package__，使相对导入可用。
# 推荐的启动方式仍是根目录的 web_car_server.py 或 `python -m server_app`。
if __package__ in (None, ""):
    _PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _PROJECT_ROOT not in sys.path:
        sys.path.insert(0, _PROJECT_ROOT)
    __package__ = "server_app"

from flask import Flask
from flask_cors import CORS

from . import config
from . import state
from . import net
from . import vision
from .routes import register_all_blueprints


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


def create_app():
    """创建并配置 Flask 应用：启用 CORS，注册全部蓝图（含编队蓝图）。

    模板目录指向工程根目录下的 templates，保持与原实现一致。
    """
    template_dir = os.path.join(config.PROJECT_ROOT, "templates")
    app = Flask(__name__, template_folder=template_dir)
    CORS(app)

    # 注册按业务域拆分的蓝图（页面/小车/广播/控制/拓扑/重构/视觉/位姿）
    register_all_blueprints(app)

    # 注册编队蓝图（仍由工程根目录的 formation_controller 提供，URL 不变）
    from formation_controller import formation_bp
    app.register_blueprint(formation_bp)

    return app


def _init_vision():
    """加载本地视觉引擎，交互选择部署车辆并完成摄像头绑定，随后启动视觉线程。

    与原实现一致：视觉模块导入失败或绑定异常均不阻断服务器启动。
    """
    # 把视觉引擎目录加入 sys.path，便于导入 Car_vision_system
    if config.VISION_MODULE_DIR not in sys.path:
        sys.path.append(config.VISION_MODULE_DIR)

    try:
        from car_vision_system import DeviceBinder, PoseEstimator, CONFIG as VISION_CONFIG
    except Exception as e:
        print(f" 本地视觉模块导入失败: {e}")
        return

    # 把视觉配置注入共享状态，供 vision 模块的设备选择函数使用
    with state.vision_lock:
        state.vision_state["vision_config"] = VISION_CONFIG

    try:
        # 模板目录指向 Car_vision_system/templates，保持与原实现一致
        if "TEMPLATE_DIR" in VISION_CONFIG:
            VISION_CONFIG["TEMPLATE_DIR"] = os.path.join(
                config.VISION_MODULE_DIR, "templates"
            )
        binder = DeviceBinder(VISION_CONFIG)
        estimator = PoseEstimator(VISION_CONFIG)
        available_ids = vision.get_available_car_ids()
        selected_cars = vision.prompt_deployed_cars(available_ids)
        print(f" 本次部署车辆: {selected_cars}")
        try:
            if not vision.bind_vision_until_ready(binder, estimator, selected_cars):
                print(" 摄像头绑定未完成，系统退出")
                sys.exit(1)
        except KeyboardInterrupt:
            print("\n 已手动退出摄像头绑定，系统退出")
            sys.exit(1)

        threading.Thread(target=vision.vision_loop, daemon=True).start()
        print(" 本地视觉线程已启动")
    except Exception as e:
        print(f" 初始化本地视觉失败，继续运行: {e}")


def get_local_ip():
    """获取本机对外 IP 地址（失败返回提示串）。"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "无法获取IP"


def get_network_info():
    """打印本机网络接口信息（需要 netifaces，缺失则忽略）。"""
    try:
        import netifaces
        interfaces = netifaces.interfaces()
        print(" 网络接口信息:")
        for interface in interfaces:
            addrs = netifaces.ifaddresses(interface)
            if netifaces.AF_INET in addrs:
                for addr_info in addrs[netifaces.AF_INET]:
                    print(f"  {interface}: {addr_info['addr']} - 广播地址: {addr_info.get('broadcast', 'N/A')}")
    except ImportError:
        print(" 无法获取详细网络信息，请安装 netifaces")


def main():
    """程序主入口：初始化视觉、启动 UDP 服务器与 Flask 应用。"""
    _configure_utf8_console()

    get_network_info()

    # 创建 UDP 服务器单例并注入共享状态，供全体模块访问（打破循环依赖）
    state.udp_server = net.UDPServer(config.UDP_HOST, config.UDP_PORT)

    # 刷新拓扑缓存（与原实现启动顺序一致）
    from . import topology as topology_ops
    topology_ops.update_topology_cache()

    # 初始化本地视觉（失败不阻断）
    _init_vision()

    app = create_app()

    if state.udp_server.start():
        print(" UDP服务器启动成功")
        # 初始化编队控制器：注入共享小车表与 UDP 服务器
        from formation_controller import init_formation_controller
        init_formation_controller(state.cars, state.udp_server)
        print(f" 广播频率: {1 / state.broadcast_interval:.0f}Hz ({state.broadcast_interval * 1000:.0f}ms间隔)")
        print(f" 广播分组大小: 每组最多 {state.broadcast_group_size} 辆小车")
        print(f" 使用子网广播地址，端口: {config.BROADCAST_PORT}")
        local_ip = get_local_ip()
        print(f" 服务器本地IP地址: {local_ip}")
        print(f" 访问 http://{local_ip}:{config.WEB_PORT} 打开控制界面")
        # 启动自动归档守护线程（自动下载日志到 logs/auto_downloads）
        try:
            from . import logging_utils
            archive_thread = threading.Thread(
                target=logging_utils.archive_worker_loop,
                args=(config.LOG_ARCHIVE_INTERVAL_S,),
                daemon=True,
            )
            archive_thread.start()
            archive_dir = os.path.join(config.PROJECT_ROOT, config.LOG_ARCHIVE_DIRNAME)
            print(f" 自动归档守护已启动，间隔 {config.LOG_ARCHIVE_INTERVAL_S}s，目录: {archive_dir}")
        except Exception as e:
            print(f" 启动自动归档守护失败: {e}")
        app.run(host='0.0.0.0', port=config.WEB_PORT, debug=False, use_reloader=False, threaded=True)
    else:
        print(" UDP服务器启动失败，无法运行应用")


if __name__ == '__main__':
    # 允许直接运行本文件（配合顶部的 PEP 366 引导）；
    # 常规启动仍推荐使用根目录的 web_car_server.py。
    main()
