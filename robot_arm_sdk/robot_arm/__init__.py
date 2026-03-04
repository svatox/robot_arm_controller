"""
机械臂主类

封装所有机械臂操作，提供简洁的API接口。
"""

from typing import Dict, Any, Optional
from .commander import Commander
from .protocol import ProtocolHandler
from .constants import (
    Address, FunctionCode, StatusCode, GripperMode, Direction
)
from .exceptions import raise_from_status


class RobotArm:
    """
    机械臂控制器

    提供与6-DOF机械臂下位机通信的完整接口。
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 1.0
    ):
        """
        初始化机械臂控制器

        Args:
            port: 串口名称
                - Windows: 'COM3', 'COM4' 等
                - Linux: '/dev/ttyUSB0', '/dev/ttyACM0' 等
            baudrate: 波特率，默认115200
            timeout: 超时时间（秒），默认1秒
        """
        self.commander = Commander(port, baudrate, timeout)
        self.protocol = ProtocolHandler()

    def open(self) -> None:
        """打开连接"""
        self.commander.open()

    def close(self) -> None:
        """关闭连接"""
        self.commander.close()

    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self.commander.is_open()

    def __enter__(self):
        """上下文管理器入口"""
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.close()

    # ==================== 配置命令 ====================

    def set_gear_ratio(self, joint: int, ratio: float) -> None:
        """
        设置减速比

        Args:
            joint: 关节编号 (1-6)
            ratio: 减速比 (0.1-100.0)

        Raises:
            ParameterError: 参数错误
            ExecutionError: 执行失败
        """
        if not Address.is_valid_joint(joint):
            raise ValueError(f"无效的关节编号: {joint}")

        data = self.protocol.write_float32(ratio)
        status, _ = self.commander.send_command(joint, FunctionCode.SET_GEAR_RATIO, data)
        raise_from_status(status)

    def set_zero_position(self, joint: int) -> None:
        """
        设置零位

        将当前位置设置为关节的物理零位。

        Args:
            joint: 关节编号 (1-6)

        Raises:
            ParameterError: 参数错误
            ExecutionError: 执行失败
        """
        if not Address.is_valid_joint(joint):
            raise ValueError(f"无效的关节编号: {joint}")

        status, _ = self.commander.send_command(joint, FunctionCode.SET_ZERO_POS)
        raise_from_status(status)

    def set_limit_position(self, joint: int) -> None:
        """
        设置极限位置

        将当前位置设置为关节的极限位置（与零位共同构成运动范围）。

        Args:
            joint: 关节编号 (1-6)

        Raises:
            ParameterError: 参数错误
            ExecutionError: 执行失败
        """
        if not Address.is_valid_joint(joint):
            raise ValueError(f"无效的关节编号: {joint}")

        status, _ = self.commander.send_command(joint, FunctionCode.SET_LIMIT_POS)
        raise_from_status(status)

    def get_limit_position(self, joint: int) -> Dict[str, Any]:
        """
        读取极限位置

        Args:
            joint: 关节编号 (1-6)

        Returns:
            dict: {'set': bool, 'position': int}

        Raises:
            ParameterError: 参数错误
        """
        if not Address.is_valid_joint(joint):
            raise ValueError(f"无效的关节编号: {joint}")

        status, data = self.commander.send_command(joint, FunctionCode.READ_LIMIT_POS)
        raise_from_status(status)

        return {
            'set': data[0] != 0,
            'position': self.protocol.read_int32(data, 1)
        }

    def set_protection(self, enable: bool) -> None:
        """
        设置位置保护开关

        Args:
            enable: True=开启, False=关闭

        Raises:
            ParameterError: 参数错误
        """
        data = bytes([1 if enable else 0])
        status, _ = self.commander.send_command(
            Address.SYSTEM, FunctionCode.SET_PROTECTION, data
        )
        raise_from_status(status)

    def reset_positions(self) -> None:
        """
        重置位置

        清零所有零位和极限位置，关闭位置保护。

        Raises:
            ExecutionError: 执行失败
        """
        status, _ = self.commander.send_command(
            Address.SYSTEM, FunctionCode.RESET_POSITIONS
        )
        raise_from_status(status)

    def save_config(self) -> None:
        """
        保存配置

        将所有配置写入Flash。
        """
        status, _ = self.commander.send_command(
            Address.SYSTEM, FunctionCode.SAVE_CONFIG
        )
        raise_from_status(status)

    # ==================== 运动控制 ====================

    def jog(
        self,
        joint: int,
        direction: int,
        step: float,
        speed: float
    ) -> None:
        """
        关节微动

        Args:
            joint: 关节编号 (1-6)
            direction: 方向 (0=正向, 1=反向)
            step: 步长 (0.01-5.0度)
            speed: 速度 (0.1-10.0度/秒)

        Raises:
            ParameterError: 参数错误
            ExecutionError: 执行失败
        """
        if not Address.is_valid_joint(joint):
            raise ValueError(f"无效的关节编号: {joint}")

        # 速度需要乘以10转换为uint16
        speed_raw = int(speed * 10)

        data = bytes([direction]) + self.protocol.write_float32(step) + self.protocol.write_uint16(speed_raw)
        status, _ = self.commander.send_command(joint, FunctionCode.JOINT_JOG, data)
        raise_from_status(status)

    def move_to(self, joint: int, angle: float, speed: float) -> None:
        """
        移动到指定角度

        Args:
            joint: 关节编号 (1-6)
            angle: 目标角度 (度, ±360)
            speed: 运动速度 (度/秒, 0.1-50.0)

        Raises:
            ParameterError: 参数错误
            ExecutionError: 执行失败
        """
        if not Address.is_valid_joint(joint):
            raise ValueError(f"无效的关节编号: {joint}")

        data = self.protocol.write_float32(angle) + self.protocol.write_float32(speed)
        status, _ = self.commander.send_command(joint, FunctionCode.JOINT_POSITION, data)
        raise_from_status(status)

    def home(self, joint: int, speed: float) -> None:
        """
        回零

        Args:
            joint: 关节编号 (1-6)，或0表示所有关节
            speed: 回零速度 (度/秒, 0.5-20.0)

        Raises:
            ParameterError: 参数错误
            ExecutionError: 执行失败
        """
        if joint != 0 and not Address.is_valid_joint(joint):
            raise ValueError(f"无效的关节编号: {joint}")

        data = self.protocol.write_float32(speed)
        status, _ = self.commander.send_command(joint, FunctionCode.HOMING, data)
        raise_from_status(status)

    def emergency_stop(self, address: int = 0) -> None:
        """
        急停

        Args:
            address: 设备地址 (0=所有, 1-6=指定关节, 7=夹爪)
        """
        if address < 0 or address > 7:
            raise ValueError(f"无效的地址: {address}")

        status, _ = self.commander.send_command(address, FunctionCode.EMERGENCY_STOP)
        raise_from_status(status)

    # ==================== 夹爪控制 ====================

    def set_gripper(self, mode: int, pwm: int = 50, time_ms: int = 100) -> None:
        """
        控制夹爪

        Args:
            mode: 模式
                0 = 开合（按指定PWM）
                1 = 回中
                2 = 急停保持
            pwm: PWM值 (0-100)，mode=0时有效
            time_ms: 运动时间 (10-255ms)，mode=0或1时有效

        Raises:
            ParameterError: 参数错误
        """
        if mode not in [GripperMode.OPEN_CLOSE, GripperMode.CENTER, GripperMode.HOLD]:
            raise ValueError(f"无效的模式: {mode}")

        data = bytes([mode, pwm, time_ms])
        status, _ = self.commander.send_command(
            Address.GRIPPER, FunctionCode.GRIPPER_PWM, data
        )
        raise_from_status(status)

    def get_gripper_status(self) -> Dict[str, Any]:
        """
        读取夹爪状态

        Returns:
            dict: {
                'pwm': int,           # 当前PWM值
                'mode': int,           # 控制模式
                'state': int,         # 开合状态 (0=闭合, 1=张开)
                'moving': int,         # 运动状态 (0=停止, 1=运动中)
                'error': int           # 错误标志
            }
        """
        status, data = self.commander.send_command(
            Address.GRIPPER, FunctionCode.READ_GRIPPER_STATUS
        )
        raise_from_status(status)

        return {
            'pwm': data[0],
            'mode': data[1],
            'state': data[2],
            'moving': data[3],
            'error': data[4]
        }

    # ==================== 状态读取 ====================

    def get_joint_status(self, joint: int) -> Dict[str, Any]:
        """
        读取关节状态

        Args:
            joint: 关节编号 (1-6)

        Returns:
            dict: {
                'angle': float,          # 当前角度 (度)
                'target_angle': float,    # 目标角度 (度)
                'speed': float,           # 实时速度 (度/秒)
                'current': int,          # 电机相电流 (mA)
                'motor_speed': int,      # 电机转速 (RPM)
                'pos_error': float,     # 位置误差 (度)
                'enabled': bool,         # 使能状态
                'voltage': int,          # 总线电压 (mV)
                'stall': bool            # 堵转标志
            }

        Raises:
            ParameterError: 参数错误
            ExecutionError: 执行失败
        """
        if not Address.is_valid_joint(joint):
            raise ValueError(f"无效的关节编号: {joint}")

        status, data = self.commander.send_command(
            joint, FunctionCode.READ_SINGLE_STATUS
        )
        raise_from_status(status)

        # 解析24字节数据
        return {
            'angle': self.protocol.read_float32(data, 0),
            'target_angle': self.protocol.read_float32(data, 4),
            'speed': self.protocol.read_float32(data, 8),
            'current': self.protocol.read_uint16(data, 12),
            'motor_speed': self.protocol.read_int16(data, 14),
            'pos_error': self.protocol.read_float32(data, 16),
            'enabled': data[20] != 0,
            'voltage': self.protocol.read_uint16(data, 21),
            'stall': data[23] != 0
        }

    def get_system_status(self) -> Dict[str, Any]:
        """
        读取系统状态

        Returns:
            dict: {
                'voltage': int,    # 系统电压 (mV)
                'uart_ok': bool,   # UART状态
                'flash_ok': bool   # Flash状态
            }
        """
        status, data = self.commander.send_command(
            Address.SYSTEM, FunctionCode.READ_FULL_STATUS
        )
        raise_from_status(status)

        return {
            'voltage': self.protocol.read_uint16(data, 0),
            'uart_ok': data[2] == 0,
            'flash_ok': data[3] == 0
        }
