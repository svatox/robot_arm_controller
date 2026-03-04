"""
常量定义

定义通信协议中使用的各种常量和枚举值。
"""

# ==================== 地址定义 ====================

class Address:
    """设备地址"""
    BROADCAST = 0x00  # 广播地址
    JOINT1 = 0x01    # 关节1
    JOINT2 = 0x02    # 关节2
    JOINT3 = 0x03    # 关节3
    JOINT4 = 0x04    # 关节4
    JOINT5 = 0x05    # 关节5
    JOINT6 = 0x06    # 关节6
    GRIPPER = 0x07   # 夹爪
    SYSTEM = 0x08    # 系统

    @staticmethod
    def is_valid_joint(addr: int) -> bool:
        """检查是否为有效的关节地址"""
        return Address.JOINT1 <= addr <= Address.JOINT6


# ==================== 功能码 ====================

class FunctionCode:
    """功能码"""

    # 配置命令 (0x01-0x07)
    SET_GEAR_RATIO = 0x01     # 设置减速比
    SET_ZERO_POS = 0x02        # 设置零位
    SAVE_CONFIG = 0x03         # 保存配置
    SET_LIMIT_POS = 0x04       # 设置极限位置
    READ_LIMIT_POS = 0x05      # 读取极限位置
    SET_PROTECTION = 0x06      # 设置位置保护
    RESET_POSITIONS = 0x07      # 重置位置

    # 运动控制命令 (0x20-0x2F)
    JOINT_JOG = 0x20           # 关节微动
    HOMING = 0x21              # 一键回零
    JOINT_POSITION = 0x22      # 关节位置控制
    GRIPPER_PWM = 0x27         # 夹爪PWM控制
    EMERGENCY_STOP = 0x2F      # 急停

    # 状态读取命令 (0x41-0x47)
    READ_SINGLE_STATUS = 0x41  # 读单关节状态
    READ_FULL_STATUS = 0x42     # 读全量状态
    READ_JOINT_ANGLE = 0x43     # 读关节角度
    READ_PROTECTION = 0x44      # 读位置保护状态
    READ_GRIPPER_STATUS = 0x47 # 读夹爪状态

    # 电机透传命令 (0xF0-0xFF)
    MOTOR_PASSTHROUGH_BASE = 0xF0  # 电机透传起始码


# ==================== 状态码 ====================

class StatusCode:
    """命令执行状态码"""
    SUCCESS = 0x00        # 成功
    PARAM_ERROR = 0x01    # 参数错误
    EXEC_FAILED = 0x02   # 执行失败
    CRC_ERROR = 0x03      # CRC错误
    DEVICE_NOT_READY = 0x04  # 设备未就绪


# ==================== 夹爪模式 ====================

class GripperMode:
    """夹爪控制模式"""
    OPEN_CLOSE = 0x00    # 开合模式
    CENTER = 0x01       # 回中模式
    HOLD = 0x02         # 急停保持


# ==================== 运动方向 ====================

class Direction:
    """关节运动方向"""
    FORWARD = 0x00  # 正向
    REVERSE = 0x01  # 反向


# ==================== 协议常量 ====================

class Protocol:
    """协议常量"""
    FRAME_HEADER = b'\xAA\x55'     # 帧头
    FRAME_TAIL = b'\x55\xAA'       # 帧尾
    CRC_POLYNOMIAL = 0x31           # CRC多项式
    CRC_INITIAL = 0x00              # CRC初始值

    # 串口默认参数
    DEFAULT_BAUDRATE = 115200
    DEFAULT_TIMEOUT = 1.0           # 秒
