# -*- coding: utf-8 -*-
"""
================================================================================
 模块名称: server_app.topology
 模块职责: 通信拓扑矩阵的构建、缓存与下发
--------------------------------------------------------------------------------
 本模块负责与“通信拓扑”相关的通用操作：

   update_topology_cache      根据当前矩阵刷新“每辆车可见哪些车”的缓存
   copy_topology              深拷贝矩阵（避免共享引用）
   flatten_topology           把 4x4 矩阵拉平为逗号分隔字符串（用于下发指令）
   apply_topology             应用新矩阵并向小车广播启用/矩阵/禁用指令
   burst_broadcast_command    连发多次广播指令，对抗 UDP 丢包
   build_reconstruct_topology 根据拼接顺序构建链式拓扑矩阵

 说明:
   与“重构日志”耦合的拓扑切换函数（准备阶段中心拓扑 / 拼接阶段链式拓扑）
   放在 reconstruct 模块中，本模块只保留与业务无关的纯矩阵与下发逻辑。

 依赖关系:
   通过 state.udp_server 单例访问广播能力，避免与 net 层形成循环导入。

 编码格式: UTF-8（无 BOM）
================================================================================
"""

import time

from . import config
from . import state


def burst_broadcast_command(command, repeat=3, delay=0.05):
    """连发多次全局广播指令，提升 UDP 丢包环境下的送达率。"""
    for i in range(repeat):
        state.udp_server.broadcast_global_command(command)
        if i < repeat - 1:
            time.sleep(delay)


def update_topology_cache():
    """根据当前通信拓扑矩阵刷新“每辆车能收到哪些车状态”的缓存。"""
    state.topology_cache = {}
    car_ids = config.ALL_CAR_IDS
    car_mapping = config.CAR_INDEX_MAP
    for target_car in car_ids:
        visible_cars = []
        target_index = car_mapping[target_car]
        for other_car in car_ids:
            if other_car != target_car:
                other_index = car_mapping[other_car]
                if state.communication_topology[other_index][target_index] == 1:
                    visible_cars.append(other_car)
        state.topology_cache[target_car] = visible_cars
    print(f" 拓扑缓存已更新: {state.topology_cache}")


def copy_topology(matrix):
    """返回矩阵的逐行深拷贝。"""
    return [row[:] for row in matrix]


def flatten_topology(matrix):
    """把二维矩阵拉平为逗号分隔的字符串（用于构造 [T,M,...] 指令）。"""
    return ','.join(str(cell) for row in matrix for cell in row)


def apply_topology(matrix, enable):
    """应用新的拓扑矩阵，并按需向小车广播启用/矩阵/禁用指令。

    行为与原实现保持一致：
      1. 覆盖 state.communication_topology 并刷新缓存；
      2. 若本次要启用且此前未启用，先广播 [T,E,1]；
      3. 广播矩阵指令 [T,M,...]；
      4. 若本次要禁用且此前已启用，广播 [T,E,0]；
      5. 更新 state.topology_enabled。
    """
    state.communication_topology = copy_topology(matrix)
    update_topology_cache()

    if enable and not state.topology_enabled:
        burst_broadcast_command("[T,E,1]")

    topology_cmd = f"[T,M,{flatten_topology(state.communication_topology)}]"
    burst_broadcast_command(topology_cmd)

    if not enable and state.topology_enabled:
        burst_broadcast_command("[T,E,0]")

    state.topology_enabled = enable


def build_reconstruct_topology(order):
    """根据拼接顺序构建链式拓扑矩阵：前车 -> 后车 单向可见。"""
    car_mapping = config.CAR_INDEX_MAP
    matrix = [[0 for _ in range(4)] for _ in range(4)]
    if not order or len(order) < 2:
        return matrix
    for i in range(1, len(order)):
        source = order[i - 1]
        target = order[i]
        if source in car_mapping and target in car_mapping:
            matrix[car_mapping[source]][car_mapping[target]] = 1
    return matrix
