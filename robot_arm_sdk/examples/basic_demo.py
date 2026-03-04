"""
基础示例

展示机械臂的基本操作：连接、配置、运动控制、状态读取。
"""

import sys
import time

# 添加父目录到路径
sys.path.insert(0, '..')

from robot_arm import RobotArm
from robot_arm.constants import Direction, GripperMode


def get_port():
    """自动检测串口"""
    import os
    import glob

    # Windows
    if sys.platform == 'win32':
        ports = glob.glob('COM*')
        if ports:
            return ports[0]
    # Linux
    else:
        ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        if ports:
            return ports[0]

    return None


def main():
    # 获取串口
    port = get_port()
    if not port:
        print("未找到可用的串口")
        print("请手动指定串口，例如: arm = RobotArm('COM3')")
        return

    print(f"使用串口: {port}")

    # 创建机械臂对象（使用上下文管理器自动开关连接）
    with RobotArm(port) as arm:
        print("连接成功!")

        # 读取系统状态
        print("\n=== 读取系统状态 ===")
        status = arm.get_system_status()
        print(f"电压: {status['voltage'] / 1000:.1f}V")
        print(f"Flash: {'正常' if status['flash_ok'] else '异常'}")

        # 设置关节1减速比为10
        print("\n=== 设置减速比 ===")
        arm.set_gripper(GripperMode.HOLD)  # 关闭夹爪
        arm.set_gear_ratio(1, 10.0)
        print("关节1减速比设置为10:1")

        # 设置零位
        print("\n=== 设置零位 ===")
        print("将关节1移动到零位后，按回车继续...")
        input()
        arm.set_zero_position(1)
        print("关节1零位已设置")

        # 设置极限位置
        print("\n=== 设置极限位置 ===")
        print("将关节1移动到极限位置后，按回车继续...")
        input()
        arm.set_limit_position(1)
        print("关节1极限位置已设置")

        # 保存配置
        print("\n=== 保存配置 ===")
        arm.save_config()
        print("配置已保存到Flash")

        # 读取极限位置
        print("\n=== 读取极限位置 ===")
        limit = arm.get_limit_position(1)
        print(f"极限位置已设置: {limit['set']}")
        print(f"极限位置脉冲: {limit['position']}")

        # 移动到角度
        print("\n=== 运动测试 ===")
        arm.move_to(1, 30.0, 10.0)
        print("已发送移动命令: 关节1移动到30度")

        # 等待运动完成
        time.sleep(2)

        # 读取关节状态
        print("\n=== 读取关节状态 ===")
        status = arm.get_joint_status(1)
        print(f"当前角度: {status['angle']:.2f}°")
        print(f"目标角度: {status['target_angle']:.2f}°")
        print(f"速度: {status['speed']:.2f}°/s")
        print(f"电机电流: {status['current']}mA")
        print(f"堵转: {'是' if status['stall'] else '否'}")

        # 夹爪测试
        print("\n=== 夹爪测试 ===")
        arm.set_gripper(GripperMode.OPEN_CLOSE, 50, 100)
        print("夹爪打开50%")

        time.sleep(1)

        arm.set_gripper(GripperMode.CENTER, 0, 100)
        print("夹爪回中")

        # 急停测试
        print("\n=== 急停测试 ===")
        arm.move_to(1, 45.0, 10.0)
        time.sleep(0.5)
        arm.emergency_stop(1)
        print("已发送急停命令")

        print("\n测试完成!")


if __name__ == '__main__':
    main()
