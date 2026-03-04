"""
配置面板组件
"""

from PySide6.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QLabel,
                               QGroupBox, QGridLayout, QFormLayout, QPushButton, QDoubleSpinBox,
                               QTableWidget, QTableWidgetItem, QHeaderView, QFrame,
                               QLineEdit, QTextEdit, QComboBox, QCheckBox)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont


class ConfigPanelWidget(QFrame):
    """配置面板组件"""

    set_gear_ratio_requested = Signal(int, float)
    set_zero_pos_requested = Signal(int)
    save_config_requested = Signal()
    read_all_config_requested = Signal()
    set_limit_pos_requested = Signal(int)
    read_limit_pos_requested = Signal(int)
    set_protection_requested = Signal(bool)
    reset_position_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self):
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)

        title = QLabel("配置面板")
        title.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)

        # 减速比配置
        group_ratio = QGroupBox("减速比配置")

        self.combo_joint = QComboBox()
        self.combo_joint.addItems(["全部", "关节1", "关节2", "关节3", "关节4", "关节5", "关节6"])

        self.spin_ratio = QDoubleSpinBox()
        self.spin_ratio.setRange(0.1, 100.0)
        self.spin_ratio.setValue(1.0)
        self.spin_ratio.setDecimals(2)

        btn_set_ratio = QPushButton("设置")
        btn_set_ratio.clicked.connect(self._on_set_ratio)

        ratio_layout = QHBoxLayout()
        ratio_layout.addWidget(QLabel("关节:"))
        ratio_layout.addWidget(self.combo_joint)
        ratio_layout.addWidget(QLabel("比:"))
        ratio_layout.addWidget(self.spin_ratio)
        ratio_layout.addWidget(btn_set_ratio)
        ratio_layout.setSpacing(5)
        group_ratio.setLayout(ratio_layout)

        # 零位校准
        group_zero = QGroupBox("零位校准")

        self.combo_joint_zero = QComboBox()
        self.combo_joint_zero.addItems(["关节1", "关节2", "关节3", "关节4", "关节5", "关节6"])

        btn_set_zero = QPushButton("设零位")
        btn_set_zero.clicked.connect(self._on_set_zero)

        btn_all_zero = QPushButton("全部")
        btn_all_zero.clicked.connect(self._on_all_zero)

        zero_layout = QHBoxLayout()
        zero_layout.addWidget(QLabel("关节:"))
        zero_layout.addWidget(self.combo_joint_zero)
        zero_layout.addWidget(btn_set_zero)
        zero_layout.addWidget(btn_all_zero)
        zero_layout.setSpacing(5)
        group_zero.setLayout(zero_layout)

        # 极限位置设置
        group_limit = QGroupBox("极限位置")

        self.combo_joint_limit = QComboBox()
        self.combo_joint_limit.addItems(["关节1", "关节2", "关节3", "关节4", "关节5", "关节6"])

        btn_set_limit = QPushButton("设置")
        btn_set_limit.clicked.connect(self._on_set_limit)

        btn_read_limit = QPushButton("读取")
        btn_read_limit.clicked.connect(self._on_read_limit)

        self.lbl_limit_status = QLabel("未设置")

        limit_layout = QHBoxLayout()
        limit_layout.addWidget(QLabel("关节:"))
        limit_layout.addWidget(self.combo_joint_limit)
        limit_layout.addWidget(btn_set_limit)
        limit_layout.addWidget(btn_read_limit)
        limit_layout.addWidget(self.lbl_limit_status)
        limit_layout.setSpacing(5)
        group_limit.setLayout(limit_layout)

        # 位置保护开关
        group_protection = QGroupBox("位置保护")

        self.chk_protection = QCheckBox("开启")
        self.chk_protection.setChecked(True)
        self.chk_protection.stateChanged.connect(self._on_protection_changed)

        btn_reset = QPushButton("重置")
        btn_reset.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                padding: 4px 8px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        btn_reset.clicked.connect(self._on_reset)

        protection_layout = QHBoxLayout()
        protection_layout.addWidget(self.chk_protection)
        protection_layout.addWidget(btn_reset)
        protection_layout.setSpacing(10)
        group_protection.setLayout(protection_layout)

        # 保存配置
        group_save = QGroupBox("配置保存")

        btn_save = QPushButton("保存Flash")
        btn_save.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                padding: 4px 8px;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #1e8449;
            }
        """)
        btn_save.clicked.connect(self._on_save)

        btn_read = QPushButton("读取")
        btn_read.clicked.connect(self._on_read)

        save_layout = QHBoxLayout()
        save_layout.addWidget(btn_save)
        save_layout.addWidget(btn_read)
        save_layout.setSpacing(10)
        group_save.setLayout(save_layout)

        # 布局 - 2列网格布局
        main_grid = QGridLayout()
        main_grid.addWidget(group_ratio, 0, 0)
        main_grid.addWidget(group_zero, 0, 1)
        main_grid.addWidget(group_limit, 1, 0)
        main_grid.addWidget(group_protection, 1, 1)
        main_grid.addWidget(group_save, 2, 0, 1, 2)
        main_grid.setSpacing(10)

        # 主布局
        layout = QVBoxLayout()
        layout.addWidget(title)
        layout.addLayout(main_grid)
        layout.addStretch()

        self.setLayout(layout)

    def _on_set_ratio(self):
        joint = self.combo_joint.currentIndex()
        ratio = self.spin_ratio.value()
        if joint == 0:
            # 广播到所有关节
            for i in range(1, 7):
                self.set_gear_ratio_requested.emit(i, ratio)
        else:
            self.set_gear_ratio_requested.emit(joint, ratio)

    def _on_set_zero(self):
        joint = self.combo_joint_zero.currentIndex() + 1
        self.set_zero_pos_requested.emit(joint)

    def _on_all_zero(self):
        for i in range(1, 7):
            self.set_zero_pos_requested.emit(i)

    def _on_set_limit(self):
        joint = self.combo_joint_limit.currentIndex() + 1
        self.set_limit_pos_requested.emit(joint)

    def _on_read_limit(self):
        joint = self.combo_joint_limit.currentIndex() + 1
        self.read_limit_pos_requested.emit(joint)

    def _on_protection_changed(self, state):
        enabled = state == Qt.Checked
        self.set_protection_requested.emit(enabled)

    def _on_reset(self):
        self.reset_position_requested.emit()

    def _on_save(self):
        self.save_config_requested.emit()

    def _on_read(self):
        self.read_all_config_requested.emit()

    def update_limit_position(self, joint: int, is_set: bool, pulse: int):
        """更新极限位置显示"""
        status = f"已设置 (脉冲: {pulse})" if is_set else "未设置"
        self.lbl_limit_status.setText(status)
        # 更新选中的关节
        if 1 <= joint <= 6:
            self.combo_joint_limit.setCurrentIndex(joint - 1)


class MotorPassthroughWidget(QFrame):
    """电机透传调试组件"""

    send_motor_cmd = Signal(int, bytes)  # joint_id, raw_data

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self):
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)

        title = QLabel("电机透传调试")
        title.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)

        # 关节选择
        self.combo_joint = QComboBox()
        self.combo_joint.addItems(["关节1", "关节2", "关节3", "关节4", "关节5", "关节6"])

        # 原始数据输入
        lbl_data = QLabel("原始数据(Hex):")
        self.edit_data = QLineEdit()
        self.edit_data.setPlaceholderText("如: AA 01 03 00")

        # 发送按钮
        btn_send = QPushButton("发送")
        btn_send.clicked.connect(self._on_send)

        # 响应显示
        lbl_response = QLabel("响应:")
        self.edit_response = QTextEdit()
        self.edit_response.setReadOnly(True)
        self.edit_response.setMaximumHeight(80)

        # 布局
        layout = QVBoxLayout()
        layout.addWidget(title)
        layout.addWidget(self.combo_joint)
        layout.addWidget(lbl_data)
        layout.addWidget(self.edit_data)
        layout.addWidget(btn_send)
        layout.addWidget(lbl_response)
        layout.addWidget(self.edit_response)

        self.setLayout(layout)

    def _on_send(self):
        joint_id = self.combo_joint.currentIndex() + 1
        data_str = self.edit_data.text().replace(" ", "")

        try:
            data = bytes.fromhex(data_str)
            self.send_motor_cmd.emit(joint_id, data)
        except ValueError:
            self.edit_response.setPlainText("数据格式错误")

    def show_response(self, data: bytes):
        """显示响应"""
        hex_str = " ".join(f"{b:02X}" for b in data)
        self.edit_response.setPlainText(hex_str)
