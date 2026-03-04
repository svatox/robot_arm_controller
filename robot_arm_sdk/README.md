# Robot Arm SDK

Python SDK for communicating with the 6-DOF robot arm controller.

## 安装

```bash
pip install pyserial
```

## 使用示例

```python
from robot_arm import RobotArm

# 创建机械臂对象（Windows: 'COM3', Linux: '/dev/ttyUSB0'）
arm = RobotArm('/dev/ttyUSB0')

# 设置关节1的减速比为10
arm.set_gear_ratio(1, 10.0)

# 设置关节1的零位
arm.set_zero_position(1)

# 设置关节1的极限位置
arm.set_limit_position(1)

# 移动关节1到角度30度，速度10度/秒
arm.move_to(1, 30.0, 10.0)

# 读取关节状态
status = arm.get_joint_status(1)
print(f"角度: {status['angle']:.2f}°")

arm.close()
```

## 详细文档

见 [Wiki](https://github.com/yourrepo/robot_arm_sdk/wiki)
