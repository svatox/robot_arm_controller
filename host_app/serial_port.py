"""
串口通信模块
"""

import serial
import serial.tools.list_ports
from typing import Optional, Callable
import threading
import time
from protocol import parse_frame, ProtocolFrame


class SerialPort:
    """串口通信类"""

    def __init__(self):
        self.serial: Optional[serial.Serial] = None
        self.is_connected = False
        self.port_name = ""
        self.baudrate = 115200
        self._receive_thread: Optional[threading.Thread] = None
        self._running = False
        self._callback: Optional[Callable[[ProtocolFrame], None]] = None

    @staticmethod
    def list_ports() -> list:
        """列出可用串口"""
        ports = serial.tools.list_ports.comports()
        return [{"name": p.name, "description": p.description}
                for p in ports]

    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """连接串口"""
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.port_name = port
            self.baudrate = baudrate
            self.is_connected = True
            self._running = True
            self._receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._receive_thread.start()
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self) -> None:
        """断开连接"""
        self._running = False
        if self._receive_thread and self._receive_thread.is_alive():
            self._receive_thread.join(timeout=1.0)
        if self.serial and self.is_connected:
            self.serial.close()
        self.is_connected = False
        self.serial = None

    def send(self, data: bytes) -> int:
        """发送数据"""
        if not self.is_connected or not self.serial:
            return 0
        try:
            return self.serial.write(data)
        except Exception as e:
            print(f"发送失败: {e}")
            return 0

    def set_receive_callback(self, callback: Callable[[ProtocolFrame], None]) -> None:
        """设置接收回调"""
        self._callback = callback

    def _receive_loop(self) -> None:
        """接收数据循环"""
        buffer = bytearray()
        while self._running:
            if not self.serial or not self.is_connected:
                break
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    buffer.extend(data)
                    # 尝试解析完整帧
                    while len(buffer) >= 7:
                        frame = parse_frame(bytes(buffer))
                        if frame:
                            # 完整帧解析成功
                            if self._callback:
                                self._callback(frame)
                            # 移除已解析的帧
                            buffer.clear()
                        else:
                            # 查找帧头
                            found = False
                            for i in range(len(buffer) - 1):
                                if buffer[i] == 0xAA and buffer[i+1] == 0x55:
                                    if i > 0:
                                        del buffer[:i]
                                    found = True
                                    break
                            if not found:
                                break
                time.sleep(0.001)
            except Exception as e:
                print(f"接收错误: {e}")
                time.sleep(0.1)
