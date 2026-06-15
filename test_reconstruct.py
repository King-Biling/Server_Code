#!/usr/bin/env python3
"""
重构功能测试脚本
测试服务器端的重构功能是否正常工作
"""

import sys
import os

# 添加当前目录到Python路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from web_car_server import (
    handle_reconstruct_report,
    reconstruct_state,
    reconstruct_lock,
    _reset_reconstruct_state,
    RECONSTRUCT_POS_LABELS
)

def test_reconstruct_commands():
    """测试重构相关命令处理"""
    print("? 开始测试重构功能...")
    
    # 重置重构状态
    _reset_reconstruct_state()
    
    # 测试1: 准备完成消息
    print("\n1. 测试 PREP_OK 消息处理")
    test_message = "[R,PREP_OK,CAR1]"
    result = handle_reconstruct_report(test_message)
    print(f"   PREP_OK 处理结果: {result}")
    
    # 测试2: 拼接完成消息
    print("\n2. 测试 STEP_OK 消息处理")
    test_message = "[R,STEP_OK,CAR1]"
    result = handle_reconstruct_report(test_message)
    print(f"   STEP_OK 处理结果: {result}")
    
    # 测试3: 分离完成消息
    print("\n3. 测试 SEP_OK 消息处理")
    test_message = "[R,SEP_OK,CAR1]"
    result = handle_reconstruct_report(test_message)
    print(f"   SEP_OK 处理结果: {result}")
    
    # 测试4: 图像元信息消息
    print("\n4. 测试 IMG_META 消息处理")
    test_message = "[R,IMG_META,CAR1,SEQ=123456,W=640,H=480,FMT=0,LEN=102400]"
    result = handle_reconstruct_report(test_message)
    print(f"   IMG_META 处理结果: {result}")
    
    # 测试5: 图像分片消息（模拟）
    print("\n5. 测试 IMG_CHUNK 消息处理")
    # 创建模拟的二进制数据
    header = "[R,IMG_CHUNK,CAR1,SEQ=123456,IDX=0,TOT=10,SZ=10240]"
    binary_data = b"x" * 10240  # 模拟图像数据
    test_message = header + "\n" + binary_data.decode('latin-1')
    result = handle_reconstruct_report(test_message)
    print(f"   IMG_CHUNK 处理结果: {result}")
    
    # 显示重构状态
    print("\n? 重构状态:")
    with reconstruct_lock:
        print(f"   活跃: {reconstruct_state['active']}")
        print(f"   阶段: {reconstruct_state['phase']}")
        print(f"   顺序: {reconstruct_state['order']}")
        print(f"   已准备: {list(reconstruct_state['prepared'].keys())}")
        print(f"   等待车辆: {reconstruct_state['waiting_car_id']}")
        print(f"   制导控制器: {list(reconstruct_state['guide_controllers'].keys())}")
    
    print("\n? 重构功能测试完成!")

def test_guide_controller():
    """测试制导控制器"""
    print("\n? 测试制导控制器...")
    
    from web_car_server import GuideController
    
    # 创建制导控制器
    controller = GuideController("CAR1")
    
    # 测试位姿误差更新
    controller.update_pose_error(0.1, 0.05, 0.02)
    print("? 位姿误差更新测试通过")
    
    # 测试速度计算
    vx, vy, vz, done = controller._calculate_guide_velocity()
    print(f"? 速度计算测试: vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}, done={done}")
    
    print("? 制导控制器测试完成!")

if __name__ == "__main__":
    try:
        test_reconstruct_commands()
        test_guide_controller()
        print("\n? 所有测试通过!")
    except Exception as e:
        print(f"? 测试失败: {e}")
        import traceback
        traceback.print_exc()