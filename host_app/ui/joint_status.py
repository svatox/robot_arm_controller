"""
关节状态显示组件
"""

from PySide6.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QLabel,
                               QGroupBox, QGridLayout, QFrame)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from models.joint_data import JointData


class JointStatusWidget(QFrame):
    """关节状态显示组件"""

    def __init__(self, joint_id: int, parent=None):
        super().__init__(parent)
        self.joint_id = joint_id
        self._init_ui()

    def _init_ui(self):
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.setMinimumWidth(200)

        # 标题
        title = QLabel(f"关节 {self.joint_id}")
        title.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)

        # 状态标签
        self.lbl_angle = QLabel("角度: 0.00°")
        self.lbl_target = QLabel("目标: 0.00°")
        self.lbl_speed = QLabel("速度: 0.00°/s")
        self.lbl_current = QLabel("电流: 0 mA")
        self.lbl_motor_speed = QLabel("电机: 0 RPM")
        self.lbl_voltage = QLabel("电压: 0 V")
        self.lbl_enabled = QLabel("使能: 否")
        self.lbl_stall = QLabel("堵转: 否")

        # 布局
        layout = QVBoxLayout()
        layout.addWidget(title)
        layout.addWidget(self.lbl_angle)
        layout.addWidget(self.lbl_target)
        layout.addWidget(self.lbl_speed)
        layout.addWidget(self.lbl_current)
        layout.addWidget(self.lbl_motor_speed)
        layout.addWidget(self.lbl_voltage)
        layout.addWidget(self.lbl_enabled)
        layout.addWidget(self.lbl_stall)
        layout.addStretch()

        self.setLayout(layout)

    def update_status(self, data: JointData) -> None:
        """更新状态"""
        self.lbl_angle.setText(f"角度: {data.current_angle:.2f}°")
        self.lbl_target.setText(f"目标: {data.target_angle:.2f}°")
        self.lbl_speed.setText(f"速度: {data.current_speed:.2f}°/s")
        self.lbl_current.setText(f"电流: {data.motor_current} mA")
        self.lbl_motor_speed.setText(f"电机: {data.motor_speed} RPM")
        self.lbl_voltage.setText(f"电压: {data.voltage / 1000.0:.1f} V")
        self.lbl_enabled.setText(f"使能: {'是' if data.enabled else '否'}")

        # 堵转状态颜色
        if data.stall_flag:
            self.lbl_stall.setText("堵转: 是")
            self.lbl_stall.setStyleSheet("color: red; font-weight: bold;")
        else:
            self.lbl_stall.setText("堵转: 否")
            self.lbl_stall.setStyleSheet("")


class JointStatusPanel(QWidget):
    """关节状态面板"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self):
        layout = QHBoxLayout()

        # 6个关节状态
        self.joint_widgets: list[JointStatusWidget] = []
        for i in range(1, 7):
            widget = JointStatusWidget(i)
            self.joint_widgets.append(widget)
            layout.addWidget(widget)

        self.setLayout(layout)

    def update_joint(self, joint_id: int, data: JointData) -> None:
        """更新单个关节"""
        if 1 <= joint_id <= 6:
            self.joint_widgets[joint_id - 1].update_status(data)
