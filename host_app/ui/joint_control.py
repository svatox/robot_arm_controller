"""
关节控制组件
"""

from PySide6.QtWidgets import (QWidget, QHBoxLayout, QVBoxLayout, QLabel,
                               QGroupBox, QGridLayout, QPushButton, QDoubleSpinBox,
                               QComboBox, QSlider, QFrame)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont


class JointControlWidget(QFrame):
    """单关节控制组件"""

    jog_requested = Signal(int, int, float, float)  # joint_id, direction, step, speed
    position_requested = Signal(int, float, float)  # joint_id, target, speed
    homing_requested = Signal(int, float)  # joint_id, speed

    def __init__(self, joint_id: int, parent=None):
        super().__init__(parent)
        self.joint_id = joint_id
        self._init_ui()

    def _init_ui(self):
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.setMinimumWidth(160)
        self.setMaximumWidth(200)

        # 标题
        title = QLabel(f"J{self.joint_id}")
        title.setFont(QFont("Microsoft YaHei", 9, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)

        # 目标角度
        lbl_target = QLabel("目标°:")
        self.spin_target = QDoubleSpinBox()
        self.spin_target.setRange(-360, 360)
        self.spin_target.setValue(0)
        self.spin_target.setDecimals(1)
        self.spin_target.setButtonSymbols(QDoubleSpinBox.NoButtons)

        # 速度
        lbl_speed = QLabel("速度°/s:")
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(0.1, 50.0)
        self.spin_speed.setValue(10.0)
        self.spin_speed.setDecimals(1)
        self.spin_speed.setButtonSymbols(QDoubleSpinBox.NoButtons)

        # 位置控制按钮
        btn_move = QPushButton("移动")
        btn_move.setMinimumHeight(25)
        btn_move.clicked.connect(self._on_move_clicked)

        # 微动按钮
        btn_jog_pos = QPushButton("+")
        btn_jog_pos.setFixedWidth(35)
        btn_jog_pos.setMinimumHeight(25)
        btn_jog_neg = QPushButton("-")
        btn_jog_neg.setFixedWidth(35)
        btn_jog_neg.setMinimumHeight(25)

        self.combo_step = QComboBox()
        self.combo_step.addItems(["0.1", "1", "5", "10"])
        self.combo_step.setCurrentIndex(1)

        btn_jog_pos.clicked.connect(lambda: self._on_jog(0))
        btn_jog_neg.clicked.connect(lambda: self._on_jog(1))

        # 回零按钮
        btn_home = QPushButton("回零")
        btn_home.setMinimumHeight(25)

        # 布局 - 紧凑垂直排列
        layout = QVBoxLayout()
        layout.setSpacing(3)
        layout.addWidget(title)

        # 目标角度行
        target_layout = QHBoxLayout()
        target_layout.addWidget(lbl_target, 1)
        target_layout.addWidget(self.spin_target, 2)
        layout.addLayout(target_layout)

        # 速度行
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(lbl_speed, 1)
        speed_layout.addWidget(self.spin_speed, 2)
        layout.addLayout(speed_layout)

        layout.addWidget(btn_move)

        # 微动行
        jog_layout = QHBoxLayout()
        jog_layout.addWidget(self.combo_step, 1)
        jog_layout.addWidget(btn_jog_pos)
        jog_layout.addWidget(btn_jog_neg)
        layout.addLayout(jog_layout)

        layout.addWidget(btn_home)
        layout.addStretch()

        self.setLayout(layout)

        # 连接信号
        btn_home.clicked.connect(self._on_home_clicked)

    def _on_move_clicked(self):
        target = self.spin_target.value()
        speed = self.spin_speed.value()
        self.position_requested.emit(self.joint_id, target, speed)

    def _on_jog(self, direction: int):
        step_text = self.combo_step.currentText()
        step = float(step_text)
        speed = self.spin_speed.value()
        self.jog_requested.emit(self.joint_id, direction, step, speed)

    def _on_home_clicked(self):
        speed = self.spin_speed.value()
        self.homing_requested.emit(self.joint_id, speed)


class JointControlPanel(QWidget):
    """关节控制面板"""

    jog_requested = Signal(int, int, float, float)
    position_requested = Signal(int, float, float)
    homing_requested = Signal(int, float)
    emergency_stop_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()

    def _init_ui(self):
        main_layout = QHBoxLayout()

        # 2x3网格布局放置6个关节
        grid_layout = QGridLayout()
        grid_layout.setSpacing(10)

        self.joint_widgets: list[JointControlWidget] = []
        for i in range(1, 7):
            widget = JointControlWidget(i)
            widget.jog_requested.connect(self.jog_requested.emit)
            widget.position_requested.connect(self.position_requested.emit)
            widget.homing_requested.connect(self.homing_requested.emit)
            self.joint_widgets.append(widget)
            # 2行3列：第0-2列放第1-3关节，第0-2列放第4-6关节
            row = 0 if i <= 3 else 1
            col = i - 1 if i <= 3 else i - 4
            grid_layout.addWidget(widget, row, col)

        main_layout.addLayout(grid_layout)

        # 右侧全局控制按钮
        global_layout = QVBoxLayout()
        global_layout.setSpacing(15)

        lbl_global = QLabel("全局控制")
        lbl_global.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))
        lbl_global.setAlignment(Qt.AlignCenter)

        btn_all_home = QPushButton("全部回零")
        btn_all_home.setFont(QFont("Microsoft YaHei", 9, QFont.Bold))
        btn_all_home.setMinimumHeight(40)
        btn_all_home.clicked.connect(lambda: self.homing_requested.emit(0, 5.0))

        btn_estop = QPushButton("急停")
        btn_estop.setFont(QFont("Microsoft YaHei", 11, QFont.Bold))
        btn_estop.setMinimumHeight(50)
        btn_estop.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        btn_estop.clicked.connect(self.emergency_stop_requested.emit)

        global_layout.addWidget(lbl_global)
        global_layout.addWidget(btn_all_home)
        global_layout.addWidget(btn_estop)
        global_layout.addStretch()

        main_layout.addLayout(global_layout)

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
