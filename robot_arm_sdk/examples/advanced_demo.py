"""
高级示例

展示机械臂的高级功能：位置保护、多关节同步控制、状态监控。
"""

import sys
import time

sys.path.insert(0, '..')

from robot_arm import RobotArm
from robot_arm.constants import Direction, GripperMode


def example_position_protection(arm: RobotArm):
    """位置保护示例"""
    print("\n=== 位置保护示例 ===")

    # 先关闭保护进行测试
    arm.set_protection(False)
    print("已关闭位置保护")

    # 尝试移动到超出范围的位置（如果设置了零位和极限位置）
    try:
        arm.move_to(1, 1000.0, 10.0)  # 尝试移动到1000度
        print("警告: 移动成功，但超出了物理限制")
    except Exception as e:
        print(f"移动失败: {e}")

    # 开启保护
    arm.set_protection(True)
    print("已开启位置保护")

    # 尝试超出范围移动
    try:
        arm.move_to(1, 1000.0, 10.0)
        print("移动成功")
    except Exception as e:
        print(f"移动被阻止: {e}")


def example_multi_joint(arm: RobotArm):
    """多关节控制示例"""
    print("\n=== 多关节控制示例 ===")

    # 依次移动多个关节
    joints = [1, 2, 3]
    angles = [10.0, 20.0, 15.0]
    speed = 10.0

    # 依次发送命令（非同步）
    for joint, angle in zip(joints, angles):
        arm.move_to(joint, angle, speed)
        print(f"已发送关节{joint}移动到{angle}度")

    # 等待一段时间
    print("等待运动完成...")
    time.sleep(3)

    # 读取所有关节状态
    print("\n所有关节状态:")
    for i in range(1, 4):
        status = arm.get_joint_status(i)
        print(f"关节{i}: 角度={status['angle']:.2f}°  速度={status['speed']:.2f}°/s")


def example_continuous_monitoring(arm: RobotArm):
    """连续状态监控示例"""
    print("\n=== 连续状态监控示例 ===")

    # 启动一个运动
    arm.move_to(1, 45.0, 10.0)

    # 连续监控状态
    print("监控关节1运动状态 (按Ctrl+C停止):")
    try:
        while True:
            status = arm.get_joint_status(1)
            print(
                f"\r角度: {status['angle']:.2f}° -> "
                f"{status['target_angle']:.2f}° | "
                f"速度: {status['speed']:.2f}°/s | "
                f"电流: {status['current']}mA",
                end=''
            )

            # 检查是否到达目标
            if abs(status['angle'] - status['target_angle']) < 0.1:
                print("\n已到达目标位置")
                break

            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n监控已停止")
        arm.emergency_stop(1)


def example_calibration(arm: RobotArm):
    """完整标定流程示例"""
    print("\n=== 完整标定流程示例 ===")
    print("请按回车键开始标定...")

    # 重置所有设置
    input("步骤1: 按回车重置所有设置...")
    arm.reset_positions()
    print("已重置")

    # 为每个关节设置零位和极限位置
    for joint in range(1, 7):
        input(f"\n步骤2: 移动关节{joint}到零位，按回车继续...")
        arm.set_zero_position(joint)
        print(f"关节{joint}零位已设置")

        input(f"步骤3: 移动关节{joint}到极限位置，按回车继续...")
        arm.set_limit_position(joint)
        print(f"关节{joint}极限位置已设置")

    # 保存配置
    input("\n步骤4: 保存配置到Flash，按回车继续...")
    arm.save_config()
    print("配置已保存")

    # 开启位置保护
    arm.set_protection(True)
    print("位置保护已开启")

    print("\n标定完成!")


def example_gripper_control(arm: RobotArm):
    """夹爪控制示例"""
    print("\n=== 夹爪控制示例 ===")

    # 打开夹爪
    print("打开夹爪 (PWM=80)...")
    arm.set_gripper(GripperMode.OPEN_CLOSE, 80, 200)
    time.sleep(0.5)

    # 读取夹爪状态
    status = arm.get_gripper_status()
    print(f"夹爪状态: PWM={status['pwm']}, 状态={'张开' if status['state'] else '闭合'}")

    time.sleep(1)

    # 关闭夹爪
    print("关闭夹爪 (PWM=20)...")
    arm.set_gripper(GripperMode.OPEN_CLOSE, 20, 200)
    time.sleep(0.5)

    status = arm.get_gripper_status()
    print(f"夹爪状态: PWM={status['pwm']}, 状态={'张开' if status['state'] else '闭合'}")

    time.sleep(1)

    # 回中
    print("夹爪回中...")
    arm.set_gripper(GripperMode.CENTER, 0, 200)


def main():
    # 根据平台选择串口
    if len(sys.argv) > 1:
        port = sys.argv[1]
    elif sys.platform == 'win32':
        port = 'COM3'
    else:
        port = '/dev/ttyUSB0'

    print(f"连接到: {port}")

    try:
        with RobotArm(port) as arm:
            print("连接成功!")

            # 运行各示例
            example_position_protection(arm)
            # example_multi_joint(arm)  # 取消注释以运行
            # example_continuous_monitoring(arm)  # 取消注释以运行
            # example_calibration(arm)  # 取消注释以运行
            example_gripper_control(arm)

            print("\n所有示例完成!")

    except Exception as e:
        print(f"错误: {e}")


if __name__ == '__main__':
    main()
