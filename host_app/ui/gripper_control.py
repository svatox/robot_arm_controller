"""
夹爪控制组件
"""

from PySide6.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QLabel,
                               QGroupBox, QSlider, QPushButton, QDoubleSpinBox,
                               QFrame)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont


class GripperControlWidget(QFrame):
    """夹爪控制组件"""

    pwm_requested = Signal(int, int, int)  # mode, pwm, time_ms
    status_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self):
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.setMaximumWidth(200)  # 限制最大宽度

        title = QLabel("夹爪控制")
        title.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)

        # PWM滑块
        lbl_pwm = QLabel("PWM占空比:")
        self.slider_pwm = QSlider(Qt.Horizontal)
        self.slider_pwm.setRange(0, 100)
        self.slider_pwm.setValue(50)
        self.lbl_pwm_value = QLabel("50%")
        self.slider_pwm.valueChanged.connect(
            lambda v: self.lbl_pwm_value.setText(f"{v}%")
        )

        # 运动时间
        lbl_time = QLabel("运动时间:")
        self.spin_time = QDoubleSpinBox()
        self.spin_time.setRange(10, 2550)
        self.spin_time.setValue(500)
        self.spin_time.setSuffix(" ms")

        # 按钮
        btn_open = QPushButton("张开")
        btn_close = QPushButton("闭合")
        btn_center = QPushButton("回中")
        btn_stop = QPushButton("保持/停止")

        # 设置按钮高度
        btn_height = 35
        for btn in [btn_open, btn_close, btn_center, btn_stop]:
            btn.setMinimumHeight(btn_height)

        # 按钮点击事件
        btn_open.clicked.connect(lambda: self._send_pwm(0, 0, int(self.spin_time.value())))
        btn_close.clicked.connect(lambda: self._send_pwm(0, 100, int(self.spin_time.value())))
        btn_center.clicked.connect(lambda: self._send_pwm(1, 50, int(self.spin_time.value())))
        btn_stop.clicked.connect(lambda: self._send_pwm(2, self.slider_pwm.value(), 0))

        # 快速滑块按钮
        btn_set = QPushButton("设置PWM")
        btn_set.setMinimumHeight(btn_height)
        btn_set.clicked.connect(lambda: self._send_pwm(0, self.slider_pwm.value(), int(self.spin_time.value())))

        # 状态显示
        self.lbl_status = QLabel("状态: 未知")
        self.lbl_mode = QLabel("模式: 未知")

        # 布局
        layout = QVBoxLayout()
        layout.addWidget(title)
        layout.addWidget(lbl_pwm)
        layout.addWidget(self.slider_pwm)
        layout.addWidget(self.lbl_pwm_value)

        # 第一行按钮：张开、闭合
        row1 = QHBoxLayout()
        row1.addWidget(btn_open)
        row1.addWidget(btn_close)
        layout.addLayout(row1)

        # 第二行按钮：回中、保持/停止
        row2 = QHBoxLayout()
        row2.addWidget(btn_center)
        row2.addWidget(btn_stop)
        layout.addLayout(row2)

        # 第三行：时间设置
        layout.addWidget(lbl_time)
        layout.addWidget(self.spin_time)

        # 第四行：设置PWM
        layout.addWidget(btn_set)

        layout.addSpacing(10)
        layout.addWidget(self.lbl_status)
        layout.addWidget(self.lbl_mode)
        layout.addStretch()

        self.setLayout(layout)

    def _send_pwm(self, mode: int, pwm: int, time_ms: int):
        self.pwm_requested.emit(mode, pwm, time_ms)

    def update_status(self, current_pwm: int, mode: int, open_status: int, moving: bool, error: bool):
        """更新夹爪状态显示"""
        self.lbl_status.setText(f"状态: {'运动中' if moving else '静止'}")
        self.lbl_mode.setText(f"模式: {mode}")
        self.lbl_pwm_value.setText(f"{current_pwm}%")
        self.slider_pwm.setValue(current_pwm)

        if error:
            self.lbl_status.setStyleSheet("color: red;")
        else:
            self.lbl_status.setStyleSheet("")


class SystemStatusWidget(QFrame):
    """系统状态组件"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self):
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)

        title = QLabel("系统状态")
        title.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)

        self.lbl_voltage = QLabel("电压: -- V")
        self.lbl_uart = QLabel("UART: --")
        self.lbl_flash = QLabel("Flash: --")

        layout = QVBoxLayout()
        layout.addWidget(title)
        layout.addWidget(self.lbl_voltage)
        layout.addWidget(self.lbl_uart)
        layout.addWidget(self.lbl_flash)
        layout.addStretch()

        self.setLayout(layout)

    def update_status(self, voltage: int, uart_status: int, flash_status: int):
        """更新系统状态"""
        self.lbl_voltage.setText(f"电压: {voltage / 1000.0:.1f} V")

        # UART状态
        if uart_status == 0:
            self.lbl_uart.setText("UART: 正常")
            self.lbl_uart.setStyleSheet("color: green;")
        else:
            self.lbl_uart.setText(f"UART: 错误({uart_status})")
            self.lbl_uart.setStyleSheet("color: red;")

        # Flash状态
        if flash_status == 0:
            self.lbl_flash.setText("Flash: 正常")
            self.lbl_flash.setStyleSheet("color: green;")
        else:
            self.lbl_flash.setText(f"Flash: 错误({flash_status})")
            self.lbl_flash.setStyleSheet("color: red;")
