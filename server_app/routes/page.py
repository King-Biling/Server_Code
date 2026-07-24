# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.routes.page
 模块职责: Web 控制台首页路由
--------------------------------------------------------------------------------
 提供唯一的页面入口 GET /，渲染 templates/index.html。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

from flask import Blueprint, render_template

page_bp = Blueprint('page', __name__)


@page_bp.route('/')
def index():
    """渲染 Web 控制台首页。"""
    return render_template('index.html')
