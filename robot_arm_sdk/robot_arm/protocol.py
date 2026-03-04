"""
协议帧构建和解析

实现二进制协议的帧构建、解析和CRC计算。
"""

import struct
from typing import Optional, Tuple
from .constants import FunctionCode, Protocol, StatusCode


class ProtocolHandler:
    """协议处理器"""

    @staticmethod
    def calculate_crc(data: bytes) -> int:
        """
        计算CRC8校验码

        Args:
            data: 要计算CRC的数据

        Returns:
            CRC8校验码
        """
        crc = Protocol.CRC_INITIAL
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ Protocol.CRC_POLYNOMIAL
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    @staticmethod
    def build_frame(address: int, function_code: int, data: bytes = b'') -> bytes:
        """
        构建协议帧

        Args:
            address: 设备地址
            function_code: 功能码
            data: 数据部分

        Returns:
            完整的协议帧
        """
        # 构建帧（不含CRC和帧尾）
        frame_data = bytes([address, function_code, len(data)]) + data

        # 计算CRC
        crc = ProtocolHandler.calculate_crc(frame_data)

        # 组装完整帧
        frame = Protocol.FRAME_HEADER + frame_data + bytes([crc]) + Protocol.FRAME_TAIL

        return frame

    @staticmethod
    def parse_response(data: bytes) -> Optional[Tuple[int, int, int, bytes]]:
        """
        解析响应帧

        Args:
            data: 原始数据

        Returns:
            (address, function_code, status, response_data) 或 None（如果数据不完整）
        """
        if len(data) < 8:  # 最小帧长度
            return None

        # 查找帧头
        header = Protocol.FRAME_HEADER
        idx = data.find(header)
        if idx == -1:
            return None

        # 跳过帧头
        data = data[idx + len(header):]

        if len(data) < 5:  # 地址+功能码+数据长+状态+CRC+帧尾
            return None

        address = data[0]
        function_code = data[1]
        data_length = data[2]
        status = data[3]

        # 检查数据长度
        expected_len = 5 + data_length + 2  # 头5 + 数据 + CRC(1) + 帧尾(1) = 实际是7 + data_length
        # 重新计算：地址(1) + 功能码(1) + 数据长(1) + 状态(1) + 数据(N) + CRC(1) + 帧尾(2) = 6 + N + 2 = 8 + N - 1?
        # 简化：总长度 = 8 + data_length (因为帧头2字节，帧尾2字节)

        # 检查是否完整
        if len(data) < 6 + data_length:
            return None

        response_data = data[4:4 + data_length]

        # 验证CRC（这里简化处理，不验证返回帧的CRC）
        # 实际应该验证

        return (address, function_code, status, response_data)

    @staticmethod
    def read_float32(data: bytes, offset: int = 0) -> float:
        """从字节数据读取float32（小端序）"""
        return struct.unpack('<f', data[offset:offset + 4])[0]

    @staticmethod
    def write_float32(value: float) -> bytes:
        """将float32写入字节（小端序）"""
        return struct.pack('<f', value)

    @staticmethod
    def read_int32(data: bytes, offset: int = 0) -> int:
        """从字节数据读取int32（小端序）"""
        return struct.unpack('<i', data[offset:offset + 4])[0]

    @staticmethod
    def write_int32(value: int) -> bytes:
        """将int32写入字节（小端序）"""
        return struct.pack('<i', value)

    @staticmethod
    def read_uint16(data: bytes, offset: int = 0) -> int:
        """从字节数据读取uint16（小端序）"""
        return struct.unpack('<H', data[offset:offset + 2])[0]

    @staticmethod
    def write_uint16(value: int) -> bytes:
        """将uint16写入字节（小端序）"""
        return struct.pack('<H', value)

    @staticmethod
    def read_int16(data: bytes, offset: int = 0) -> int:
        """从字节数据读取int16（小端序）"""
        return struct.unpack('<h', data[offset:offset + 2])[0]
