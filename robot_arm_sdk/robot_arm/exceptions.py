"""
异常定义

定义SDK中使用的各种异常类型。
"""


class RobotArmException(Exception):
    """基础异常类"""
    pass


class ConnectionError(RobotArmException):
    """连接错误"""
    pass


class CommunicationError(RobotArmException):
    """通信错误"""
    pass


class ProtocolError(RobotArmException):
    """协议错误"""
    pass


class ParameterError(RobotArmException):
    """参数错误"""
    pass


class ExecutionError(RobotArmException):
    """执行失败"""
    pass


class CRCError(RobotArmException):
    """CRC校验错误"""
    pass


class TimeoutError(RobotArmException):
    """超时错误"""
    pass


class DeviceNotReadyError(RobotArmException):
    """设备未就绪"""
    pass


def get_error_message(status_code: int) -> str:
    """根据状态码获取错误消息"""
    from .constants import StatusCode

    messages = {
        StatusCode.SUCCESS: "成功",
        StatusCode.PARAM_ERROR: "参数错误",
        StatusCode.EXEC_FAILED: "执行失败",
        StatusCode.CRC_ERROR: "CRC校验错误",
        StatusCode.DEVICE_NOT_READY: "设备未就绪",
    }
    return messages.get(status_code, f"未知错误(0x{status_code:02X})")


def raise_from_status(status_code: int) -> None:
    """根据状态码抛出异常"""
    from .constants import StatusCode

    if status_code == StatusCode.SUCCESS:
        return
    elif status_code == StatusCode.PARAM_ERROR:
        raise ParameterError(get_error_message(status_code))
    elif status_code == StatusCode.EXEC_FAILED:
        raise ExecutionError(get_error_message(status_code))
    elif status_code == StatusCode.CRC_ERROR:
        raise CRCError(get_error_message(status_code))
    elif status_code == StatusCode.DEVICE_NOT_READY:
        raise DeviceNotReadyError(get_error_message(status_code))
    else:
        raise RobotArmException(get_error_message(status_code))
