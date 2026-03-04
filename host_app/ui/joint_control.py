"""
关节控制组件
"""

from PySide6.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QLabel,
                               QGroupBox, QGridLayout, QPushButton, QDoubleSpinBox,
                               QComboBox, QSlider, QFrame, QSpinBox, QScrollArea)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont


class JointControlWidget(QFrame):
    """单关节控制组件"""

    jog_requested = Signal(int, int, float, float)  # joint_id, direction, step, speed (angle mode)
    jog_pulse_requested = Signal(int, int, int, int, int)  # joint_id, direction, pulses, speed_rpm, acc
    position_requested = Signal(int, float, float)  # joint_id, target, speed
    homing_requested = Signal(int, float)  # joint_id, speed
    set_zero_requested = Signal(int)  # joint_id
    set_limit_requested = Signal(int)  # joint_id

    def __init__(self, joint_id: int, parent=None):
        super().__init__(parent)
        self.joint_id = joint_id
        self._init_ui()

    def _init_ui(self):
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.setMinimumWidth(280)
        self.setMinimumHeight(450)

        # 标题
        title = QLabel(f"关节 {self.joint_id}")
        title.setFont(QFont("Microsoft YaHei", 12, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("padding: 8px; background-color: #3498db; color: white; border-radius: 3px;")

        # === 位置控制区域 ===
        pos_group = QGroupBox("位置控制")
        pos_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        pos_layout = QVBoxLayout()
        pos_layout.setSpacing(10)

        # 目标角度
        target_row = QHBoxLayout()
        lbl_target = QLabel("目标角度:")
        lbl_target.setMinimumWidth(70)
        self.spin_target = QDoubleSpinBox()
        self.spin_target.setRange(-360, 360)
        self.spin_target.setValue(0)
        self.spin_target.setDecimals(1)
        target_row.addWidget(lbl_target)
        target_row.addWidget(self.spin_target)
        target_row.addWidget(QLabel("°"))
        pos_layout.addLayout(target_row)

        # 速度
        speed_row = QHBoxLayout()
        lbl_speed = QLabel("运动速度:")
        lbl_speed.setMinimumWidth(70)
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(0.1, 50.0)
        self.spin_speed.setValue(10.0)
        self.spin_speed.setDecimals(1)
        speed_row.addWidget(lbl_speed)
        speed_row.addWidget(self.spin_speed)
        speed_row.addWidget(QLabel("°/s"))
        pos_layout.addLayout(speed_row)

        # 移动按钮
        btn_move = QPushButton("移动到目标位置")
        btn_move.setMinimumHeight(30)
        btn_move.clicked.connect(self._on_move_clicked)
        pos_layout.addWidget(btn_move)

        pos_group.setLayout(pos_layout)

        # === 微动控制区域 ===
        jog_group = QGroupBox("微动控制")
        jog_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        jog_layout = QVBoxLayout()
        jog_layout.setSpacing(10)

        # 微动模式选择
        mode_row = QHBoxLayout()
        lbl_mode = QLabel("微动模式:")
        lbl_mode.setMinimumWidth(60)
        self.combo_jog_mode = QComboBox()
        self.combo_jog_mode.addItems(["角度模式", "脉冲模式"])
        self.combo_jog_mode.setCurrentIndex(0)
        self.combo_jog_mode.currentIndexChanged.connect(self._on_jog_mode_changed)
        mode_row.addWidget(lbl_mode)
        mode_row.addWidget(self.combo_jog_mode)
        jog_layout.addLayout(mode_row)

        # 角度模式参数
        self.angle_params_widget = QWidget()
        self.angle_params_widget.setMinimumHeight(80)  # 确保显示完整
        angle_params_layout = QVBoxLayout()
        angle_params_layout.setSpacing(8)

        # 步长
        step_row = QHBoxLayout()
        lbl_step = QLabel("步长:")
        lbl_step.setMinimumWidth(50)
        self.spin_jog_step = QDoubleSpinBox()
        self.spin_jog_step.setRange(0.01, 1000)
        self.spin_jog_step.setValue(1.0)
        self.spin_jog_step.setDecimals(2)
        step_row.addWidget(lbl_step)
        step_row.addWidget(self.spin_jog_step)
        step_row.addWidget(QLabel("°"))
        angle_params_layout.addLayout(step_row)

        # 速度
        jog_speed_row = QHBoxLayout()
        lbl_jog_speed = QLabel("速度:")
        lbl_jog_speed.setMinimumWidth(50)
        self.spin_jog_speed = QDoubleSpinBox()
        self.spin_jog_speed.setRange(0.1, 50.0)
        self.spin_jog_speed.setValue(5.0)
        self.spin_jog_speed.setDecimals(1)
        jog_speed_row.addWidget(lbl_jog_speed)
        jog_speed_row.addWidget(self.spin_jog_speed)
        jog_speed_row.addWidget(QLabel("°/s"))
        angle_params_layout.addLayout(jog_speed_row)

        self.angle_params_widget.setLayout(angle_params_layout)
        jog_layout.addWidget(self.angle_params_widget)

        # 脉冲模式参数（初始隐藏）
        self.pulse_params_widget = QWidget()
        self.pulse_params_widget.setVisible(False)
        pulse_params_layout = QVBoxLayout()
        pulse_params_layout.setSpacing(5)

        # 脉冲步长
        pulse_step_row = QHBoxLayout()
        lbl_pulse_step = QLabel("步长:")
        lbl_pulse_step.setMinimumWidth(50)
        self.spin_pulse_step = QSpinBox()
        self.spin_pulse_step.setRange(1, 100000)
        self.spin_pulse_step.setValue(100)
        self.spin_pulse_step.setButtonSymbols(QSpinBox.NoButtons)
        pulse_step_row.addWidget(lbl_pulse_step)
        pulse_step_row.addWidget(QLabel("pulse"))
        pulse_step_row.addWidget(self.spin_pulse_step)
        pulse_params_layout.addLayout(pulse_step_row)

        # 电机转速
        pulse_speed_row = QHBoxLayout()
        lbl_pulse_speed = QLabel("转速:")
        lbl_pulse_speed.setMinimumWidth(50)
        self.spin_pulse_speed = QSpinBox()
        self.spin_pulse_speed.setRange(1, 3000)
        self.spin_pulse_speed.setValue(100)
        self.spin_pulse_speed.setButtonSymbols(QSpinBox.NoButtons)
        pulse_speed_row.addWidget(lbl_pulse_speed)
        pulse_speed_row.addWidget(QLabel("RPM"))
        pulse_speed_row.addWidget(self.spin_pulse_speed)
        pulse_params_layout.addLayout(pulse_speed_row)

        # 加速度
        pulse_acc_row = QHBoxLayout()
        lbl_pulse_acc = QLabel("加速度:")
        lbl_pulse_acc.setMinimumWidth(50)
        self.spin_pulse_acc = QSpinBox()
        self.spin_pulse_acc.setRange(0, 255)
        self.spin_pulse_acc.setValue(50)
        self.spin_pulse_acc.setButtonSymbols(QSpinBox.NoButtons)
        pulse_acc_row.addWidget(lbl_pulse_acc)
        pulse_acc_row.addWidget(self.spin_pulse_acc)
        pulse_acc_row.addStretch()
        pulse_params_layout.addLayout(pulse_acc_row)

        self.pulse_params_widget.setLayout(pulse_params_layout)
        jog_layout.addWidget(self.pulse_params_widget)

        # 微动按钮
        jog_btn_layout = QHBoxLayout()
        jog_btn_layout.setSpacing(10)
        btn_jog_pos = QPushButton("+ 正向")
        btn_jog_pos.setMinimumHeight(30)
        btn_jog_neg = QPushButton("- 反向")
        btn_jog_neg.setMinimumHeight(30)
        btn_jog_pos.clicked.connect(lambda: self._on_jog(0))
        btn_jog_neg.clicked.connect(lambda: self._on_jog(1))
        jog_btn_layout.addWidget(btn_jog_pos)
        jog_btn_layout.addWidget(btn_jog_neg)
        jog_layout.addLayout(jog_btn_layout)

        jog_group.setLayout(jog_layout)

        # === 校准区域 ===
        calib_group = QGroupBox("校准")
        calib_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        calib_layout = QHBoxLayout()
        calib_layout.setSpacing(15)

        btn_set_zero = QPushButton("设零位")
        btn_set_zero.setMinimumHeight(30)
        btn_set_zero.clicked.connect(self._on_set_zero_clicked)

        btn_set_limit = QPushButton("设极限")
        btn_set_limit.setMinimumHeight(30)
        btn_set_limit.clicked.connect(self._on_set_limit_clicked)

        btn_home = QPushButton("回零位")
        btn_home.setMinimumHeight(30)
        btn_home.clicked.connect(self._on_home_clicked)

        calib_layout.addWidget(btn_set_zero)
        calib_layout.addWidget(btn_set_limit)
        calib_layout.addWidget(btn_home)
        calib_group.setLayout(calib_layout)

        # === 主布局 ===
        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)
        main_layout.addWidget(title)
        main_layout.addWidget(pos_group)
        main_layout.addWidget(jog_group)
        main_layout.addWidget(calib_group)
        main_layout.addStretch()

        self.setLayout(main_layout)

    def _on_jog_mode_changed(self, index: int):
        """微动模式切换"""
        if index == 0:  # 角度模式
            self.angle_params_widget.setVisible(True)
            self.pulse_params_widget.setVisible(False)
        else:  # 脉冲模式
            self.angle_params_widget.setVisible(False)
            self.pulse_params_widget.setVisible(True)

    def _on_move_clicked(self):
        target = self.spin_target.value()
        speed = self.spin_speed.value()
        self.position_requested.emit(self.joint_id, target, speed)

    def _on_jog(self, direction: int):
        if self.combo_jog_mode.currentIndex() == 0:  # 角度模式
            step = self.spin_jog_step.value()
            speed = self.spin_jog_speed.value()
            self.jog_requested.emit(self.joint_id, direction, step, speed)
        else:  # 脉冲模式
            step_pulses = self.spin_pulse_step.value()
            speed_rpm = self.spin_pulse_speed.value()
            acc = self.spin_pulse_acc.value()
            self.jog_pulse_requested.emit(self.joint_id, direction, step_pulses, speed_rpm, acc)

    def _on_home_clicked(self):
        speed = self.spin_speed.value()
        self.homing_requested.emit(self.joint_id, speed)

    def _on_set_zero_clicked(self):
        self.set_zero_requested.emit(self.joint_id)

    def _on_set_limit_clicked(self):
        self.set_limit_requested.emit(self.joint_id)


class JointControlPanel(QWidget):
    """关节控制面板"""

    jog_requested = Signal(int, int, float, float)  # angle mode
    jog_pulse_requested = Signal(int, int, int, int, int)  # pulse mode
    position_requested = Signal(int, float, float)
    homing_requested = Signal(int, float)
    set_zero_requested = Signal(int)
    set_limit_requested = Signal(int)
    set_protection_requested = Signal(bool)  # True=开启, False=关闭
    read_protection_requested = Signal()
    emergency_stop_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self):
        main_layout = QHBoxLayout()
        main_layout.setSpacing(15)

        # 创建滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)

        # 滚动区域的内容 widget
        scroll_content = QWidget()
        scroll_layout = QHBoxLayout()
        scroll_layout.setSpacing(15)

        # 2x3网格布局放置6个关节
        grid_layout = QGridLayout()
        grid_layout.setSpacing(15)

        self.joint_widgets: list[JointControlWidget] = []
        for i in range(1, 7):
            widget = JointControlWidget(i)
            widget.jog_requested.connect(self.jog_requested.emit)
            widget.jog_pulse_requested.connect(self.jog_pulse_requested.emit)
            widget.position_requested.connect(self.position_requested.emit)
            widget.homing_requested.connect(self.homing_requested.emit)
            widget.set_zero_requested.connect(self.set_zero_requested.emit)
            widget.set_limit_requested.connect(self.set_limit_requested.emit)
            self.joint_widgets.append(widget)
            # 2行3列
            row = 0 if i <= 3 else 1
            col = i - 1 if i <= 3 else i - 4
            grid_layout.addWidget(widget, row, col)

        scroll_layout.addLayout(grid_layout)
        scroll_content.setLayout(scroll_layout)
        scroll_area.setWidget(scroll_content)

        main_layout.addWidget(scroll_area, 5)

        # 右侧全局控制面板
        global_group = QGroupBox("全局控制")
        global_layout = QVBoxLayout()
        global_layout.setSpacing(12)

        # 回零
        btn_all_home = QPushButton("全部回零")
        btn_all_home.setFont(QFont("Microsoft YaHei", 10))
        btn_all_home.setMinimumHeight(40)
        btn_all_home.clicked.connect(lambda: self.homing_requested.emit(0, 5.0))

        # 位置保护控制
        prot_group = QGroupBox("位置保护")
        prot_layout = QVBoxLayout()
        prot_layout.setSpacing(8)

        btn_prot_on = QPushButton("开启保护")
        btn_prot_on.setMinimumHeight(30)
        btn_prot_on.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #1e8449;
            }
        """)
        btn_prot_on.clicked.connect(lambda: self.set_protection_requested.emit(True))

        btn_prot_off = QPushButton("关闭保护")
        btn_prot_off.setMinimumHeight(30)
        btn_prot_off.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                border-radius: 3px;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        btn_prot_off.clicked.connect(lambda: self.set_protection_requested.emit(False))

        btn_prot_read = QPushButton("读取状态")
        btn_prot_read.setMinimumHeight(30)
        btn_prot_read.clicked.connect(self.read_protection_requested.emit)

        prot_layout.addWidget(btn_prot_on)
        prot_layout.addWidget(btn_prot_off)
        prot_layout.addWidget(btn_prot_read)
        prot_group.setLayout(prot_layout)

        # 急停
        btn_estop = QPushButton("急停")
        btn_estop.setFont(QFont("Microsoft YaHei", 12, QFont.Bold))
        btn_estop.setMinimumHeight(50)
        btn_estop.setStyleSheet("""
            QPushButton {
                background-color: #c0392b;
                color: white;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #922b21;
            }
        """)
        btn_estop.clicked.connect(self.emergency_stop_requested.emit)

        global_layout.addWidget(btn_all_home)
        global_layout.addWidget(prot_group)
        global_layout.addWidget(btn_estop)
        global_layout.addStretch()

        global_group.setLayout(global_layout)
        main_layout.addWidget(global_group, 1)

        self.setLayout(main_layout)


class MultiJointControlWidget(QWidget):
    """多关节同步控制组件"""

    position_requested = Signal(list, list)  # [targets], [speeds]
    homing_requested = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self):
        group = QGroupBox("多关节同步控制")
        layout = QGridLayout()

        # 表头
        layout.addWidget(QLabel("关节"), 0, 0)
        layout.addWidget(QLabel("目标角度(°)"), 0, 1)
        layout.addWidget(QLabel("速度(°/s)"), 0, 2)

        self.spin_targets: list[QDoubleSpinBox] = []
        self.spin_speeds: list[QDoubleSpinBox] = []

        for i in range(6):
            layout.addWidget(QLabel(f"J{i+1}"), i+1, 0)

            spin_target = QDoubleSpinBox()
            spin_target.setRange(-360, 360)
            spin_target.setValue(0)
            spin_target.setDecimals(2)
            self.spin_targets.append(spin_target)
            layout.addWidget(spin_target, i+1, 1)

            spin_speed = QDoubleSpinBox()
            spin_speed.setRange(0.1, 50.0)
            spin_speed.setValue(10.0)
            spin_speed.setDecimals(1)
            self.spin_speeds.append(spin_speed)
            layout.addWidget(spin_speed, i+1, 2)

        # 按钮
        btn_sync = QPushButton("同步移动")
        btn_sync.clicked.connect(self._on_sync_clicked)

        btn_all_home = QPushButton("全部回零")
        btn_all_home.clicked.connect(lambda: self.homing_requested.emit(5.0))

        layout.addWidget(btn_sync, 7, 0, 1, 2)
        layout.addWidget(btn_all_home, 7, 2)

        group.setLayout(layout)
        main_layout = QVBoxLayout()
        main_layout.addWidget(group)
        self.setLayout(main_layout)

    def _on_sync_clicked(self):
        targets = [s.value() for s in self.spin_targets]
        speeds = [s.value() for s in self.spin_speeds]
        self.position_requested.emit(targets, speeds)
