# Robot Arm Controller

6-DOF机械臂下位机控制器固件

## 硬件平台

- **MCU**: STM32F103C8T6 (ARM Cortex-M3, 72MHz)
- **电机驱动**: 6x Emm_V5.0 闭环步进电机
- **通信**: USART1 (上位机) + USART3 (电机)

## 功能特性

- 6轴关节位置控制
- 关节限位保护
- 配置参数Flash存储
- ROS2兼容协议
- 支持位置模式3种运动模式
- 支持PID参数调整
- 支持速度缩放（0.1RPM精度）

## 通信协议

- 上位机通信: 自定义二进制协议 @ 115200bps
- 电机通信: Emm_V5.0 协议 @ 115200bps

## 目录结构

```
robot_arm_code/
└── Core/
    ├── Inc/          # 头文件
    └── Src/          # 源文件
```

## 编译工具

- STM32CubeMX
- STM32CubeIDE
- ARM GCC Compiler
