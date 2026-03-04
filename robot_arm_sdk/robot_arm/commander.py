"""
串口通信

实现与下位机的串口通信，包括发送命令和接收响应。
"""

import serial
import time
from typing import Optional, Tuple
from .protocol import ProtocolHandler
from .constants import Protocol
from .exceptions import ConnectionError, CommunicationError, TimeoutError


class Commander:
    """串口命令发送器"""

    def __init__(
        self,
        port: str,
        baudrate: int = Protocol.DEFAULT_BAUDRATE,
        timeout: float = Protocol.DEFAULT_TIMEOUT
    ):
        """
        初始化串口

        Args:
            port: 串口名称
                - Windows: 'COM3', 'COM4' 等
                - Linux: '/dev/ttyUSB0', '/dev/ttyACM0' 等
            baudrate: 波特率，默认115200
            timeout: 超时时间（秒），默认1秒
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial: Optional[serial.Serial] = None
        self._rx_buffer = b''
        self._protocol = ProtocolHandler()

    def open(self) -> None:
        """打开串口连接"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
        except serial.SerialException as e:
            raise ConnectionError(f"无法打开串口 {self.port}: {e}")

    def close(self) -> None:
        """关闭串口连接"""
        if self.serial and self.serial.is_open:
            self.serial.close()

    def is_open(self) -> bool:
        """检查串口是否打开"""
        return self.serial is not None and self.serial.is_open

    def send_command(
        self,
        address: int,
        function_code: int,
        data: bytes = b'',
        expect_response: bool = True
    ) -> Tuple[int, bytes]:
        """
        发送命令并接收响应

        Args:
            address: 设备地址
            function_code: 功能码
            data: 数据部分
            expect_response: 是否期待响应

        Returns:
            (status_code, response_data)

        Raises:
            ConnectionError: 连接错误
            TimeoutError: 超时
            CommunicationError: 通信错误
        """
        if not self.is_open():
            raise ConnectionError("串口未打开")

        # 构建帧
        frame = self._protocol.build_frame(address, function_code, data)

        # 发送
        try:
            self.serial.write(frame)
        except serial.SerialException as e:
            raise CommunicationError(f"发送失败: {e}")

        # 如果不期待响应，直接返回
        if not expect_response:
            return (0, b'')

        # 接收响应
        start_time = time.time()
        while time.time() - start_time < self.timeout:
            try:
                # 读取所有可用数据
                data = self.serial.read(self.serial.in_waiting or 1)
                if data:
                    self._rx_buffer += data

                    # 尝试解析响应
                    result = self._protocol.parse_response(self._rx_buffer)
                    if result is not None:
                        _, func_code, status, response_data = result
                        self._rx_buffer = b''  # 清空缓冲区

                        # 检查功能码是否匹配（有些响应会修改功能码）
                        return (status, response_data)

            except serial.SerialException as e:
                raise CommunicationError(f"接收失败: {e}")

        # 超时
        self._rx_buffer = b''
        raise TimeoutError(f"命令超时: 功能码 0x{function_code:02X}")

    def send_command_no_response(self, address: int, function_code: int, data: bytes = b'') -> None:
        """
        发送命令，不等待响应

        Args:
            address: 设备地址
            function_code: 功能码
            data: 数据部分
        """
        if not self.is_open():
            raise ConnectionError("串口未打开")

        # 构建帧
        frame = self._protocol.build_frame(address, function_code, data)

        # 发送
        try:
            self.serial.write(frame)
        except serial.SerialException as e:
            raise CommunicationError(f"发送失败: {e}")

    def __enter__(self):
        """上下文管理器入口"""
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.close()
