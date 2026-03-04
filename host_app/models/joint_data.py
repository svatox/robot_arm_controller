"""
数据模型模块
"""

from dataclasses import dataclass, field
from typing import List
from datetime import datetime


@dataclass
class JointData:
    """关节数据"""
    joint_id: int = 0
    current_angle: float = 0.0
    target_angle: float = 0.0
    current_speed: float = 0.0
    motor_current: int = 0
    motor_speed: int = 0
    pos_error: float = 0.0
    enabled: bool = False
    voltage: int = 0
    stall_flag: bool = False
    gear_ratio: float = 1.0
    zero_position: int = 0


@dataclass
class GripperData:
    """夹爪数据"""
    current_pwm: int = 0
    mode: int = 0
    open_status: int = 0
    moving: bool = False
    error: bool = False


@dataclass
class SystemData:
    """系统数据"""
    voltage: int = 0
    uart_status: int = 0
    flash_status: int = 0


@dataclass
class TrajectoryRecord:
    """轨迹记录"""
    timestamp: str = ""
    joint_angles: List[float] = field(default_factory=lambda: [0.0] * 6)
    joint_speeds: List[float] = field(default_factory=lambda: [0.0] * 6)
    gripper_pwm: int = 0


class TrajectoryRecorder:
    """轨迹记录器"""

    def __init__(self):
        self.records: List[TrajectoryRecord] = []
        self.is_recording = False

    def start(self) -> None:
        """开始记录"""
        self.records.clear()
        self.is_recording = True

    def stop(self) -> None:
        """停止记录"""
        self.is_recording = False

    def record(self, joints: List[JointData], gripper: GripperData) -> None:
        """记录当前状态"""
        if not self.is_recording:
            return

        record = TrajectoryRecord(
            timestamp=datetime.now().strftime("%H:%M:%S.%f")[:-3],
            joint_angles=[j.current_angle for j in joints],
            joint_speeds=[j.current_speed for j in joints],
            gripper_pwm=gripper.current_pwm
        )
        self.records.append(record)

    def save_to_csv(self, filename: str) -> bool:
        """保存为CSV文件"""
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                # 写入表头
                headers = ["时间"]
                for i in range(6):
                    headers.extend([f"关节{i+1}角度", f"关节{i+1}速度"])
                headers.append("夹爪PWM")
                f.write(",".join(headers) + "\n")

                # 写入数据
                for record in self.records:
                    row = [record.timestamp]
                    row.extend(record.joint_angles)
                    row.extend(record.joint_speeds)
                    row.append(record.gripper_pwm)
                    f.write(",".join(map(str, row)) + "\n")
            return True
        except Exception as e:
            print(f"保存失败: {e}")
            return False
