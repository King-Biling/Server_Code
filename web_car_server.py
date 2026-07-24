# -*- coding: utf-8 -*-
"""
================================================================================
 文件名称: web_car_server.py
 文件职责: 服务器端程序启动入口（薄封装）
--------------------------------------------------------------------------------
 本文件是整个服务器端的启动入口，保持与重构前完全一致的启动方式：

     python web_car_server.py

 重构说明:
   原单体文件（约 3000 行）已按“功能域”拆分到 server_app/ 子包中，
   各子模块职责见 server_app/__init__.py 顶部说明。本文件仅作为薄封装，
   转调 server_app.app.main() 完成全部装配与运行，不再包含任何业务逻辑。

   历史版本已备份为 web_car_server_legacy_backup.py，仅供对照参考，
   不参与运行。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import os
import sys

# 确保工程根目录（本文件所在目录）位于模块搜索路径，
# 以便无论从何处启动都能正确导入 server_app 包及
# formation_controller / car_vision_system 等根级模块。
_PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from server_app.app import main

if __name__ == '__main__':
    main()
