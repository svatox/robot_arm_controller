"""
通信协议模块 - 自定义二进制协议实现
帧格式: 0xAA 0x55 + 地址(1B) + 功能码(1B) + 数据长度(1B) + 数据(NB) + CRC8(1B) + 0x55 0xAA
"""

import struct
from typing import Optional, Tuple

# 帧定界符
FRAME_HEADER_1 = 0xAA
FRAME_HEADER_2 = 0x55
FRAME_TAIL_1 = 0x55
FRAME_TAIL_2 = 0xAA

# 地址定义
ADDR_BROADCAST = 0x00
ADDR_JOINT_1 = 0x01
ADDR_JOINT_2 = 0x02
ADDR_JOINT_3 = 0x03
ADDR_JOINT_4 = 0x04
ADDR_JOINT_5 = 0x05
ADDR_JOINT_6 = 0x06
ADDR_GRIPPER = 0x07
ADDR_SYSTEM = 0x08

# 功能码 - 配置
FUNC_SET_GEAR_RATIO = 0x01
FUNC_SET_ZERO_POS = 0x02
FUNC_SAVE_CONFIG = 0x03
FUNC_SET_LIMIT_POS = 0x04      # 设置极限位置
FUNC_READ_LIMIT_POS = 0x05     # 读取极限位置
FUNC_SET_PROTECTION = 0x06     # 设置位置保护开关
FUNC_RESET_POSITION = 0x07     # 重置位置

# 功能码 - 运动控制
FUNC_JOINT_JOG = 0x20
FUNC_HOMING = 0x21
FUNC_JOINT_POSITION = 0x22
FUNC_GRIPPER_PWM = 0x27
FUNC_EMERGENCY_STOP = 0x2F

# 功能码 - 状态读取
FUNC_READ_SINGLE_STATUS = 0x41
FUNC_READ_FULL_STATUS = 0x42
FUNC_READ_GRIPPER_STATUS = 0x47

# 功能码 - 异常
FUNC_EXCEPTION_REPORT = 0x60

# 功能码 - 电机透传
FUNC_MOTOR_PASSTHROUGH_BASE = 0xF0
FUNC_MOTOR_PASSTHROUGH_MAX = 0xFF

# 状态码
STATUS_SUCCESS = 0x00
STATUS_PARAM_ERROR = 0x01
STATUS_EXEC_FAILED = 0x02
STATUS_CRC_ERROR = 0x03
STATUS_NOT_READY = 0x04

# 异常类型
EXCEPT_MOTOR_STALL = 0x01
EXCEPT_MOTOR_UNDERVOLT = 0x02
EXCEPT_UART_ERROR = 0x03
EXCEPT_SYS_UNDERVOLT = 0x04
EXCEPT_JOINT_OUT_RANGE = 0x05

# 帧参数
FRAME_MIN_LENGTH = 7
FRAME_MAX_DATA_LENGTH = 54
FRAME_MAX_LENGTH = 64


class ProtocolFrame:
    """协议帧结构"""
    def __init__(self, address: int = 0, function_code: int = 0,
                 data: bytes = b'', status: int = 0):
        self.address = address
        self.function_code = function_code
        self.data = data
        self.status = status


def crc8_calculate(data: bytes) -> int:
    """CRC8计算 - 多项式0x31，初始值0x00"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc <<= 1
            crc &= 0xFF
    return crc


def build_frame(address: int, function_code: int, data: bytes = b'') -> bytes:
    """构建协议帧"""
    # 构建不含CRC和帧尾的数据
    payload = bytes([address, function_code, len(data)]) + data
    crc = crc8_calculate(payload)
    # 帧头 + 数据 + CRC + 帧尾
    return bytes([FRAME_HEADER_1, FRAME_HEADER_2]) + payload + bytes([crc, FRAME_TAIL_1, FRAME_TAIL_2])


def build_response_frame(address: int, function_code: int, status: int,
                         data: bytes = b'') -> bytes:
    """构建应答帧"""
    return build_frame(address, function_code, bytes([status]) + data)


def parse_frame(data: bytes) -> Optional[ProtocolFrame]:
    """解析协议帧"""
    if len(data) < FRAME_MIN_LENGTH:
        return None

    # 查找帧头
    start = 0
    for i in range(len(data) - 1):
        if data[i] == FRAME_HEADER_1 and data[i+1] == FRAME_HEADER_2:
            start = i
            break
    else:
        return None

    # 检查帧尾
    if len(data) < start + FRAME_MIN_LENGTH:
        return None

    # 提取数据长度
    data_len = data[start + 3]

    # 检查完整帧
    frame_len = start + 2 + 1 + 1 + 1 + data_len + 1 + 2  # header + addr + func + len + data + crc + tail
    if len(data) < frame_len:
        return None

    # 检查帧尾
    if data[start + frame_len - 2] != FRAME_TAIL_1 or data[start + frame_len - 1] != FRAME_TAIL_2:
        return None

    # 提取数据
    payload = data[start+2:start+2+1+1+1+data_len]
    address = payload[0]
    function_code = payload[1]
    frame_data = payload[3:3+data_len]
    received_crc = data[start + frame_len - 3]

    # 校验CRC
    calculated_crc = crc8_calculate(payload)
    if received_crc != calculated_crc:
        return None

    return ProtocolFrame(address, function_code, frame_data)


def build_response(address: int, function_code: int, status: int,
                   data: bytes = b'') -> bytes:
    """构建应答帧"""
    return build_frame(address, function_code, bytes([status]) + data)


# 数据转换函数 (小端模式)
def read_float(data: bytes, offset: int = 0) -> float:
    """读取float32"""
    if len(data) < offset + 4:
        return 0.0
    return struct.unpack('<f', data[offset:offset+4])[0]


def read_uint16(data: bytes, offset: int = 0) -> int:
    """读取uint16"""
    if len(data) < offset + 2:
        return 0
    return struct.unpack('<H', data[offset:offset+2])[0]


def read_int16(data: bytes, offset: int = 0) -> int:
    """读取int16"""
    if len(data) < offset + 2:
        return 0
    return struct.unpack('<h', data[offset:offset+2])[0]


def read_int32(data: bytes, offset: int = 0) -> int:
    """读取int32"""
    if len(data) < offset + 4:
        return 0
    return struct.unpack('<i', data[offset:offset+4])[0]


def write_float(value: float) -> bytes:
    """写入float32"""
    return struct.pack('<f', value)


def write_uint16(value: int) -> bytes:
    """写入uint16"""
    return struct.pack('<H', value)


def write_int16(value: int) -> bytes:
    """写入int16"""
    return struct.pack('<h', value)


def write_int32(value: int) -> bytes:
    """写入int32"""
    return struct.pack('<i', value)


# 指令构建函数
def cmd_set_gear_ratio(joint_addr: int, ratio: float) -> bytes:
    """设置减速比"""
    return build_frame(joint_addr, FUNC_SET_GEAR_RATIO, write_float(ratio))


def cmd_set_zero_pos(joint_addr: int) -> bytes:
    """设置零位"""
    return build_frame(joint_addr, FUNC_SET_ZERO_POS)


def cmd_save_config() -> bytes:
    """保存配置"""
    return build_frame(ADDR_SYSTEM, FUNC_SAVE_CONFIG)


def cmd_set_limit_pos(joint_addr: int) -> bytes:
    """设置极限位置"""
    return build_frame(joint_addr, FUNC_SET_LIMIT_POS)


def cmd_read_limit_pos(joint_addr: int) -> bytes:
    """读取极限位置"""
    return build_frame(joint_addr, FUNC_READ_LIMIT_POS)


def cmd_set_protection(enabled: bool) -> bytes:
    """设置位置保护开关"""
    return build_frame(ADDR_SYSTEM, FUNC_SET_PROTECTION, bytes([1 if enabled else 0]))


def cmd_reset_position() -> bytes:
    """重置位置"""
    return build_frame(ADDR_SYSTEM, FUNC_RESET_POSITION)


def cmd_joint_jog(joint_addr: int, direction: int, step: float, speed: float) -> bytes:
    """关节微动"""
    # direction(1B) + step(4B) + speed(2B)
    data = bytes([direction]) + write_float(step) + write_uint16(int(speed * 10))
    return build_frame(joint_addr, FUNC_JOINT_JOG, data)


def cmd_homing(joint_addr: int, speed: float) -> bytes:
    """回零"""
    return build_frame(joint_addr, FUNC_HOMING, write_float(speed))


def cmd_joint_position(joint_addr: int, target_angle: float, speed: float) -> bytes:
    """关节位置控制"""
    data = write_float(target_angle) + write_float(speed)
    return build_frame(joint_addr, FUNC_JOINT_POSITION, data)


def cmd_gripper_pwm(mode: int, pwm: int, time_ms: int) -> bytes:
    """夹爪PWM控制"""
    return build_frame(ADDR_GRIPPER, FUNC_GRIPPER_PWM, bytes([mode, pwm, time_ms]))


def cmd_emergency_stop(address: int = ADDR_BROADCAST) -> bytes:
    """急停"""
    return build_frame(address, FUNC_EMERGENCY_STOP)


def cmd_read_single_status(joint_addr: int) -> bytes:
    """读取单关节状态"""
    return build_frame(joint_addr, FUNC_READ_SINGLE_STATUS)


def cmd_read_full_status() -> bytes:
    """读取全量状态"""
    return build_frame(ADDR_SYSTEM, FUNC_READ_FULL_STATUS)


def cmd_read_gripper_status() -> bytes:
    """读取夹爪状态"""
    return build_frame(ADDR_GRIPPER, FUNC_READ_GRIPPER_STATUS)


def cmd_motor_passthrough(joint_addr: int, motor_data: bytes) -> bytes:
    """电机透传"""
    return build_frame(joint_addr, FUNC_MOTOR_PASSTHROUGH_BASE, motor_data)


# 状态解析函数
def parse_joint_status(data: bytes) -> dict:
    """解析单关节状态 (30字节)"""
    if len(data) < 24:
        return {}
    return {
        'current_angle': read_float(data, 0),
        'target_angle': read_float(data, 4),
        'current_speed': read_float(data, 8),
        'motor_current': read_uint16(data, 12),
        'motor_speed': read_int16(data, 14),
        'pos_error': read_float(data, 16),
        'enabled': data[20],
        'voltage': read_uint16(data, 21),
        'stall_flag': data[23]
    }


def parse_gripper_status(data: bytes) -> dict:
    """解析夹爪状态 (5字节)"""
    if len(data) < 5:
        return {}
    return {
        'current_pwm': data[0],
        'mode': data[1],
        'open_status': data[2],
        'moving': data[3],
        'error': data[4]
    }


def parse_system_status(data: bytes) -> dict:
    """解析系统状态"""
    if len(data) < 4:
        return {}
    return {
        'voltage': read_uint16(data, 0),
        'uart_status': data[2],
        'flash_status': data[3]
    }


def parse_limit_position(data: bytes) -> dict:
    """解析极限位置 (5字节)"""
    if len(data) < 5:
        return {}
    return {
        'is_set': data[0] != 0,
        'limit_pulse': read_int32(data, 1)
    }


# 功能码到名称映射
FUNCTION_NAMES = {
    FUNC_SET_GEAR_RATIO: "设置减速比",
    FUNC_SET_ZERO_POS: "设置零位",
    FUNC_SAVE_CONFIG: "保存配置",
    FUNC_SET_LIMIT_POS: "设置极限位置",
    FUNC_READ_LIMIT_POS: "读取极限位置",
    FUNC_SET_PROTECTION: "设置位置保护",
    FUNC_RESET_POSITION: "重置位置",
    FUNC_JOINT_JOG: "关节微动",
    FUNC_HOMING: "回零",
    FUNC_JOINT_POSITION: "关节位置",
    FUNC_GRIPPER_PWM: "夹爪控制",
    FUNC_EMERGENCY_STOP: "急停",
    FUNC_READ_SINGLE_STATUS: "读单关节状态",
    FUNC_READ_FULL_STATUS: "读全量状态",
    FUNC_READ_GRIPPER_STATUS: "读夹爪状态",
    FUNC_EXCEPTION_REPORT: "异常上报",
}

for code in range(FUNC_MOTOR_PASSTHROUGH_BASE, FUNC_MOTOR_PASSTHROUGH_MAX + 1):
    FUNCTION_NAMES[code] = f"电机透传(0x{code:02X})"


def get_status_text(status: int) -> str:
    """状态码转文本"""
    texts = {
        STATUS_SUCCESS: "成功",
        STATUS_PARAM_ERROR: "参数错误",
        STATUS_EXEC_FAILED: "执行失败",
        STATUS_CRC_ERROR: "CRC错误",
        STATUS_NOT_READY: "设备未就绪"
    }
    return texts.get(status, f"未知({status})")


def get_function_name(code: int) -> str:
    """功能码转文本"""
    return FUNCTION_NAMES.get(code, f"未知(0x{code:02X})")
