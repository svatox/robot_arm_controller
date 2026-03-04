"""
主窗口组件
"""

from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                               QLabel, QPushButton, QComboBox, QStatusBar,
                               QMenuBar, QMenu, QTabWidget, QGroupBox)
from PySide6.QtCore import Signal, Qt, QTimer
from PySide6.QtGui import QFont, QAction
import serial.tools.list_ports

from ui.joint_status import JointStatusWidget
from ui.joint_control import JointControlPanel, MultiJointControlWidget
from ui.gripper_control import GripperControlWidget, SystemStatusWidget
from ui.config_panel import ConfigPanelWidget
from ui.log_panel import LogPanel


class MainWindow(QMainWindow):
    """主窗口"""

    # 连接信号
    connect_requested = Signal(str, int)  # port, baudrate
    disconnect_requested = Signal()

    # 发送信号
    send_frame_requested = Signal(bytes)

    # 命令信号
    cmd_set_gear_ratio = Signal(int, float)
    cmd_set_zero_pos = Signal(int)
    cmd_save_config = Signal()
    cmd_set_limit_pos = Signal(int)
    cmd_read_limit_pos = Signal(int)
    cmd_set_protection = Signal(bool)
    cmd_reset_position = Signal()
    cmd_joint_jog = Signal(int, int, float, float)  # joint, direction, step, speed
    cmd_homing = Signal(int, float)  # joint(0=all), speed
    cmd_joint_position = Signal(int, float, float)  # joint, target, speed
    cmd_gripper_pwm = Signal(int, int, int)  # mode, pwm, time
    cmd_estop = Signal(int)  # address(0=all)
    cmd_read_single_status = Signal(int)
    cmd_read_full_status = Signal()
    cmd_read_gripper_status = Signal()
    cmd_motor_passthrough = Signal(int, bytes)

    # 记录信号
    start_recording = Signal()
    stop_recording = Signal()

    def __init__(self):
        super().__init__()
        self._init_ui()
        self._init_menus()
        self._init_timers()

    def _init_ui(self):
        self.setWindowTitle("机械臂调试上位机 v1.0")
        self.setMinimumSize(900, 700)
        self.resize(1100, 800)

        # 中央部件
        central = QWidget()
        self.setCentralWidget(central)

        main_layout = QVBoxLayout()

        # 顶部：串口设置
        toolbar = self._create_toolbar()
        main_layout.addLayout(toolbar)

        # 中间：标签页
        self.tabs = QTabWidget()

        # 状态显示页
        status_tab = QWidget()
        status_layout = QVBoxLayout()

        # 第一行：关节1-4
        row1_layout = QHBoxLayout()
        # 重新创建4个关节状态
        from ui.joint_status import JointStatusWidget
        self.joint_status_widgets = []
        for i in range(1, 5):
            widget = JointStatusWidget(i)
            self.joint_status_widgets.append(widget)
            row1_layout.addWidget(widget)
        status_layout.addLayout(row1_layout)

        # 第二行：关节5-6 + 系统状态 + 读取按钮
        row2_layout = QHBoxLayout()
        # 关节5
        self.joint5_widget = JointStatusWidget(5)
        row2_layout.addWidget(self.joint5_widget)
        # 关节6
        self.joint6_widget = JointStatusWidget(6)
        row2_layout.addWidget(self.joint6_widget)
        # 系统状态 + 读取按钮
        sys_layout = QVBoxLayout()
        self.system_status = SystemStatusWidget()
        btn_read_status = QPushButton("读取状态")
        btn_read_status.clicked.connect(self.cmd_read_full_status.emit)
        sys_layout.addWidget(self.system_status)
        sys_layout.addWidget(btn_read_status)
        row2_layout.addLayout(sys_layout)

        status_layout.addLayout(row2_layout)
        status_tab.setLayout(status_layout)
        self.tabs.addTab(status_tab, "状态显示")

        # 控制页
        control_tab = QWidget()
        control_layout = QHBoxLayout()

        # 左侧：关节控制面板
        self.joint_control_panel = JointControlPanel()
        self.joint_control_panel.jog_requested.connect(self.cmd_joint_jog.emit)
        self.joint_control_panel.position_requested.connect(self.cmd_joint_position.emit)
        self.joint_control_panel.homing_requested.connect(self.cmd_homing.emit)
        self.joint_control_panel.emergency_stop_requested.connect(lambda: self.cmd_estop.emit(0))
        control_layout.addWidget(self.joint_control_panel, 4)

        # 右侧：夹爪控制
        self.gripper_control = GripperControlWidget()
        self.gripper_control.pwm_requested.connect(self.cmd_gripper_pwm.emit)
        self.gripper_control.status_requested.connect(self.cmd_read_gripper_status.emit)
        control_layout.addWidget(self.gripper_control, 1)

        control_tab.setLayout(control_layout)
        self.tabs.addTab(control_tab, "关节控制")

        # 同步控制页
        sync_tab = QWidget()
        sync_layout = QVBoxLayout()
        sync_layout.addWidget(MultiJointControlWidget())
        sync_tab.setLayout(sync_layout)
        self.tabs.addTab(sync_tab, "同步控制")

        # 配置页
        config_tab = QWidget()
        config_layout = QVBoxLayout()
        self.config_widget = ConfigPanelWidget()
        config_layout.addWidget(self.config_widget)
        config_tab.setLayout(config_layout)
        self.tabs.addTab(config_tab, "配置")

        # 连接配置面板信号
        self.config_widget.set_gear_ratio_requested.connect(self.cmd_set_gear_ratio)
        self.config_widget.set_zero_pos_requested.connect(self.cmd_set_zero_pos)
        self.config_widget.save_config_requested.connect(self.cmd_save_config)
        self.config_widget.set_limit_pos_requested.connect(self.cmd_set_limit_pos)
        self.config_widget.read_limit_pos_requested.connect(self.cmd_read_limit_pos)
        self.config_widget.set_protection_requested.connect(self.cmd_set_protection)
        self.config_widget.reset_position_requested.connect(self.cmd_reset_position)

        # 轨迹记录页
        record_tab = self._create_record_tab()
        self.tabs.addTab(record_tab, "轨迹记录")

        main_layout.addWidget(self.tabs)

        # 底部：日志
        self.log_panel = LogPanel()
        main_layout.addWidget(self.log_panel)

        central.setLayout(main_layout)

        # 状态栏
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("未连接")

    def _create_toolbar(self) -> QHBoxLayout:
        """创建工具栏"""
        layout = QHBoxLayout()

        # 串口选择
        lbl_port = QLabel("串口:")
        self.combo_port = QComboBox()
        self.combo_port.setMinimumWidth(120)
        self._refresh_ports()

        btn_refresh = QPushButton("刷新")
        btn_refresh.clicked.connect(self._refresh_ports)

        # 波特率
        lbl_baud = QLabel("波特率:")
        self.combo_baud = QComboBox()
        self.combo_baud.setMinimumWidth(100)
        self.combo_baud.addItems(["115200", "57600", "38400", "19200", "9600"])
        self.combo_baud.setCurrentText("115200")

        # 连接按钮
        self.btn_connect = QPushButton("连接")
        self.btn_connect.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                padding: 8px 20px;
                border-radius: 4px;
            }
            QPushButton:pressed {
                background-color: #1e8449;
            }
            QPushButton:disabled {
                background-color: #95a5a6;
            }
        """)
        self.btn_connect.clicked.connect(self._on_connect_clicked)

        # 断开按钮
        self.btn_disconnect = QPushButton("断开")
        self.btn_disconnect.setEnabled(False)
        self.btn_disconnect.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                padding: 8px 20px;
                border-radius: 4px;
            }
            QPushButton:pressed {
                background-color: #c0392b;
            }
            QPushButton:disabled {
                background-color: #95a5a6;
            }
        """)
        self.btn_disconnect.clicked.connect(self._on_disconnect_clicked)

        layout.addWidget(lbl_port)
        layout.addWidget(self.combo_port)
        layout.addWidget(btn_refresh)
        layout.addSpacing(20)
        layout.addWidget(lbl_baud)
        layout.addWidget(self.combo_baud)
        layout.addSpacing(20)
        layout.addWidget(self.btn_connect)
        layout.addWidget(self.btn_disconnect)
        layout.addStretch()

        return layout

    def _create_record_tab(self) -> QWidget:
        """创建轨迹记录页"""
        from PySide6.QtWidgets import QCheckBox, QPushButton, QLineEdit, QFileDialog

        tab = QWidget()
        layout = QVBoxLayout()

        # 记录控制
        control_layout = QHBoxLayout()

        self.chk_recording = QCheckBox("开始记录轨迹")
        self.chk_recording.toggled.connect(self._on_recording_toggled)

        btn_save_csv = QPushButton("保存为CSV")
        btn_save_csv.clicked.connect(self._on_save_csv)

        self.lbl_record_count = QLabel("记录点数: 0")

        control_layout.addWidget(self.chk_recording)
        control_layout.addWidget(btn_save_csv)
        control_layout.addWidget(self.lbl_record_count)
        control_layout.addStretch()

        layout.addLayout(control_layout)

        # 说明
        info = QLabel("说明: 记录模式下会自动保存所有关节角度和夹爪状态数据")
        info.setStyleSheet("color: #7f8c8d;")
        layout.addWidget(info)

        tab.setLayout(layout)
        return tab

    def _init_menus(self):
        """初始化菜单"""
        menubar = self.menuBar()

        # 文件菜单
        file_menu = menubar.addMenu("文件")

        action_exit = QAction("退出", self)
        action_exit.triggered.connect(self.close)
        file_menu.addAction(action_exit)

        # 视图菜单
        view_menu = menubar.addMenu("视图")

        # 主题子菜单
        theme_menu = view_menu.addMenu("主题")

        action_light = QAction("浅色", self)
        action_light.triggered.connect(lambda: self.set_theme("light"))

        action_dark = QAction("深色", self)
        action_dark.triggered.connect(lambda: self.set_theme("dark"))

        theme_menu.addAction(action_light)
        theme_menu.addAction(action_dark)

    def _init_timers(self):
        """初始化定时器"""
        # 端口刷新定时器
        self.port_refresh_timer = QTimer()
        self.port_refresh_timer.timeout.connect(self._refresh_ports)
        self.port_refresh_timer.start(5000)  # 每5秒刷新一次

    def _refresh_ports(self):
        """刷新串口列表"""
        current = self.combo_port.currentText()
        self.combo_port.clear()

        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_port.addItem(p.name)

        if current and current in [self.combo_port.itemText(i) for i in range(self.combo_port.count())]:
            self.combo_port.setCurrentText(current)

    def _on_connect_clicked(self):
        """连接按钮点击"""
        port = self.combo_port.currentText()
        baudrate = int(self.combo_baud.currentText())
        if port:
            self.connect_requested.emit(port, baudrate)

    def _on_disconnect_clicked(self):
        """断开按钮点击"""
        self.disconnect_requested.emit()

    def _on_recording_toggled(self, checked: bool):
        """记录状态切换"""
        if checked:
            self.start_recording.emit()
        else:
            self.stop_recording.emit()

    def _on_save_csv(self):
        """保存CSV"""
        from PySide6.QtWidgets import QFileDialog
        from datetime import datetime

        filename, _ = QFileDialog.getSaveFileName(
            self, "保存轨迹", f"trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            "CSV文件 (*.csv);;所有文件 (*)"
        )
        if filename:
            # TODO: 实现保存
            self.status_bar.showMessage(f"已保存到: {filename}")

    def set_connected(self, connected: bool):
        """设置连接状态"""
        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        self.combo_port.setEnabled(not connected)
        self.combo_baud.setEnabled(not connected)

        if connected:
            self.status_bar.showMessage(f"已连接: {self.combo_port.currentText()}")
        else:
            self.status_bar.showMessage("未连接")

    def set_theme(self, theme: str):
        """设置主题"""
        if theme == "dark":
            self.setStyleSheet("""
                QMainWindow {
                    background-color: #2b2b2b;
                }
                QWidget {
                    color: #e0e0e0;
                    background-color: #2b2b2b;
                }
                QPushButton {
                    background-color: #3c3c3c;
                    border: 1px solid #555;
                    padding: 5px 10px;
                    border-radius: 3px;
                }
                QPushButton:hover {
                    background-color: #4a4a4a;
                }
                QGroupBox {
                    border: 1px solid #555;
                    border-radius: 5px;
                    margin-top: 10px;
                    padding-top: 10px;
                }
                QGroupBox::title {
                    subcontrol-origin: margin;
                    left: 10px;
                }
                QTextEdit, QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
                    background-color: #3c3c3c;
                    border: 1px solid #555;
                    border-radius: 3px;
                    padding: 3px;
                }
                QTabWidget::pane {
                    border: 1px solid #555;
                }
            """)
        else:
            self.setStyleSheet("")

    def log_tx(self, data: bytes):
        """记录发送"""
        self.log_panel.log_tx(data)

    def log_rx(self, data: bytes):
        """记录接收"""
        self.log_panel.log_rx(data)

    def log_error(self, msg: str):
        """记录错误"""
        self.log_panel.log_error(msg)

    def log_info(self, msg: str):
        """记录信息"""
        self.log_panel.log_info(msg)

    def update_record_count(self, count: int):
        """更新记录点数"""
        self.lbl_record_count.setText(f"记录点数: {count}")
