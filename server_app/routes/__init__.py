# -*- coding: utf-8 -*-
"""
================================================================================
 子包名称: server_app.routes
 子包职责: 按业务域拆分的 Flask 蓝图集合
--------------------------------------------------------------------------------
 原单体文件把约 35 个 HTTP 接口全部堆在一个文件里，多人协同时极易在同一处
 产生合并冲突。本子包按“业务域”将路由拆分为多个蓝图，各自独立成文件：

   cars_bp        小车列表查询（/api/cars）
   broadcast_bp   广播开关与参数（/api/broadcast*）
   control_bp     单车位置/速度控制（/api/control_*）
   reconstruct_bp 重构流程控制与状态/日志/图像（/api/reconstruct/*）
   vision_bp      视觉监控与 MJPEG 流（/api/vision/*、/stream/mjpeg/*）
   topology_bp    通信拓扑设置与查询（/api/topology*）
   pose_bp        位姿数据保存（/api/save_pose）
   page_bp        Web 首页（/）

 重要约定（保持行为不变的关键）:
   所有蓝图注册时均不加 url_prefix，因此每个接口的最终 URL 与原单体实现
   逐字一致，前端 templates/index.html 中的 fetch 路径无需任何改动。
   编队相关蓝图（formation_bp）仍由工程根目录的 formation_controller.py 提供，
   在 app 装配阶段一并注册。

 register_all_blueprints(app) 由 app 模块调用，集中完成全部蓝图注册。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

from .cars import cars_bp
from .broadcast import broadcast_bp
from .control import control_bp
from .reconstruct import reconstruct_bp
from .vision import vision_bp
from .topology import topology_bp
from .pose import pose_bp
from .page import page_bp


def register_all_blueprints(app):
    """把本子包内的全部蓝图注册到给定的 Flask 应用（不加 url_prefix）。"""
    app.register_blueprint(page_bp)
    app.register_blueprint(cars_bp)
    app.register_blueprint(broadcast_bp)
    app.register_blueprint(control_bp)
    app.register_blueprint(reconstruct_bp)
    app.register_blueprint(vision_bp)
    app.register_blueprint(topology_bp)
    app.register_blueprint(pose_bp)
