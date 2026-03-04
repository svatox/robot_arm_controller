"""
机械臂调试上位机 - 主程序入口
"""

import sys
from PySide6.QtWidgets import QApplication, QMessageBox
from PySide6.QtCore import QTimer

from ui.main_window import MainWindow
from serial_port import SerialPort
from protocol import (build_frame, parse_frame,
                      FUNC_SET_GEAR_RATIO, FUNC_SET_ZERO_POS, FUNC_SAVE_CONFIG,
                      FUNC_SET_LIMIT_POS, FUNC_READ_LIMIT_POS, FUNC_SET_PROTECTION, FUNC_RESET_POSITION,
                      FUNC_JOINT_JOG, FUNC_HOMING, FUNC_JOINT_POSITION,
                      FUNC_GRIPPER_PWM, FUNC_EMERGENCY_STOP,
                      FUNC_READ_SINGLE_STATUS, FUNC_READ_FULL_STATUS, FUNC_READ_GRIPPER_STATUS,
                      FUNC_MOTOR_PASSTHROUGH_BASE,
                      parse_joint_status, parse_gripper_status, parse_system_status, parse_limit_position,
                      get_function_name, get_status_text, STATUS_SUCCESS)
from models.joint_data import JointData, GripperData, SystemData, TrajectoryRecorder


class RobotArmApp:
    """机械臂上位机应用"""

    def __init__(self):
        self.serial = SerialPort()
        self.window = MainWindow()
        self.recorder = TrajectoryRecorder()

        # 关节数据
        self.joints = [JointData(joint_id=i+1) for i in range(6)]
        self.gripper = GripperData()
        self.system = SystemData()

        # 状态轮询定时器
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._poll_status)

        self._connect_signals()

    def _connect_signals(self):
        """连接信号"""
        # 窗口信号
        self.window.connect_requested.connect(self._on_connect)
        self.window.disconnect_requested.connect(self._on_disconnect)

        # 命令信号
        self.window.cmd_set_gear_ratio.connect(self._send_set_gear_ratio)
        self.window.cmd_set_zero_pos.connect(self._send_set_zero_pos)
        self.window.cmd_save_config.connect(self._send_save_config)
        self.window.cmd_set_limit_pos.connect(self._send_set_limit_pos)
        self.window.cmd_read_limit_pos.connect(self._send_read_limit_pos)
        self.window.cmd_set_protection.connect(self._send_set_protection)
        self.window.cmd_reset_position.connect(self._send_reset_position)
        self.window.cmd_joint_jog.connect(self._send_joint_jog)
        self.window.cmd_homing.connect(self._send_homing)
        self.window.cmd_joint_position.connect(self._send_joint_position)
        self.window.cmd_gripper_pwm.connect(self._send_gripper_pwm)
        self.window.cmd_estop.connect(self._send_estop)
        self.window.cmd_read_full_status.connect(self._send_read_full_status)
        self.window.cmd_read_single_status.connect(self._send_read_single_status)
        self.window.cmd_read_gripper_status.connect(self._send_read_gripper_status)
        self.window.cmd_motor_passthrough.connect(self._send_motor_passthrough)

        # 串口回调
        self.serial.set_receive_callback(self._on_frame_received)

        # 记录信号
        self.window.start_recording.connect(self._on_start_recording)
        self.window.stop_recording.connect(self._on_stop_recording)

    def run(self):
        """运行应用"""
        self.window.show()
        return self.window

    def _on_connect(self, port: str, baudrate: int):
        """连接串口"""
        if self.serial.connect(port, baudrate):
            self.window.set_connected(True)
            self.window.log_info(f"已连接到 {port} @ {baudrate}")
            # 启动状态轮询
            self.status_timer.start(100)  # 10Hz
        else:
            QMessageBox.warning(self.window, "连接失败", f"无法打开串口 {port}")

    def _on_disconnect(self):
        """断开连接"""
        self.serial.disconnect()
        self.window.set_connected(False)
        self.status_timer.stop()
        self.window.log_info("已断开连接")

    def _send_frame(self, data: bytes):
        """发送帧"""
        self.serial.send(data)
        self.window.log_tx(data)

    def _log_tx_with_info(self, data: bytes, cmd_name: str, params: str):
        """发送帧并记录详细信息"""
        self.serial.send(data)
        # 打印详细信息 - 使用与原始帧一致的箭头
        if params:
            self.window.log_panel.add_log("TX", f"发送 [{cmd_name}]: {params}", "#3498db")
        else:
            self.window.log_panel.add_log("TX", f"发送 [{cmd_name}]", "#3498db")
        # 打印原始帧
        self.window.log_tx(data)

    def _poll_status(self):
        """轮询状态"""
        if not self.serial.is_connected:
            return

        # 记录轨迹
        if self.recorder.is_recording:
            self.recorder.record(self.joints, self.gripper)
            self.window.update_record_count(len(self.recorder.records))

    # 命令发送函数
    def _send_set_gear_ratio(self, joint: int, ratio: float):
        import struct
        data = build_frame(joint, FUNC_SET_GEAR_RATIO, struct.pack('<f', ratio))
        self._log_tx_with_info(data, "设置减速比", f"关节={joint if joint>0 else '全部'}, 减速比={ratio}")

    def _send_set_zero_pos(self, joint: int):
        data = build_frame(joint, FUNC_SET_ZERO_POS)
        self._log_tx_with_info(data, "设置零位", f"关节={joint}")

    def _send_save_config(self):
        data = build_frame(0x08, FUNC_SAVE_CONFIG)
        self._log_tx_with_info(data, "保存配置", "")

    def _send_set_limit_pos(self, joint: int):
        data = build_frame(joint, FUNC_SET_LIMIT_POS)
        self._log_tx_with_info(data, "设置极限位置", f"关节={joint}")

    def _send_read_limit_pos(self, joint: int):
        data = build_frame(joint, FUNC_READ_LIMIT_POS)
        self._log_tx_with_info(data, "读取极限位置", f"关节={joint}")

    def _send_set_protection(self, enabled: bool):
        data = build_frame(0x08, FUNC_SET_PROTECTION, bytes([1 if enabled else 0]))
        self._log_tx_with_info(data, "设置位置保护", f"开启={enabled}")

    def _send_reset_position(self):
        data = build_frame(0x08, FUNC_RESET_POSITION)
        self._log_tx_with_info(data, "重置位置", "")

    def _send_joint_jog(self, joint: int, direction: int, step: float, speed: float):
        import struct
        data = bytes([direction]) + struct.pack('<f', step) + struct.pack('<H', int(speed * 10))
        frame = build_frame(joint, FUNC_JOINT_JOG, data)
        dir_str = "正向" if direction == 0 else "反向"
        self._log_tx_with_info(frame, "关节微动", f"关节={joint}, 方向={dir_str}, 步长={step}°, 速度={speed}°/s")

    def _send_homing(self, joint: int, speed: float):
        import struct
        data = struct.pack('<f', speed)
        frame = build_frame(joint, FUNC_HOMING, data)
        self._log_tx_with_info(frame, "回零", f"关节={joint if joint > 0 else '全部'}, 速度={speed}°/s")

    def _send_joint_position(self, joint: int, target: float, speed: float):
        import struct
        data = struct.pack('<ff', target, speed)
        frame = build_frame(joint, FUNC_JOINT_POSITION, data)
        self._log_tx_with_info(frame, "关节位置控制", f"关节={joint}, 目标角度={target}°, 速度={speed}°/s")

    def _send_gripper_pwm(self, mode: int, pwm: int, time_ms: int):
        data = bytes([mode, pwm, time_ms])
        frame = build_frame(0x07, FUNC_GRIPPER_PWM, data)
        mode_str = {0: "开合", 1: "回中", 2: "急停保持"}.get(mode, f"未知({mode})")
        self._log_tx_with_info(frame, "夹爪PWM控制", f"模式={mode_str}, PWM={pwm}%, 时间={time_ms}ms")

    def _send_estop(self, address: int = 0):
        frame = build_frame(address, FUNC_EMERGENCY_STOP)
        self._log_tx_with_info(frame, "急停", f"地址={'全部' if address == 0 else f'关节{address}'}")

    def _send_read_full_status(self):
        frame = build_frame(0x08, FUNC_READ_FULL_STATUS)
        self._log_tx_with_info(frame, "读取全量状态", "")

    def _send_read_single_status(self, joint: int):
        frame = build_frame(joint, FUNC_READ_SINGLE_STATUS)
        self._log_tx_with_info(frame, "读取单关节状态", f"关节={joint}")

    def _send_read_gripper_status(self):
        frame = build_frame(0x07, FUNC_READ_GRIPPER_STATUS)
        self._log_tx_with_info(frame, "读取夹爪状态", "")

    def _send_motor_passthrough(self, joint: int, motor_data: bytes):
        frame = build_frame(joint, FUNC_MOTOR_PASSTHROUGH_BASE, motor_data)
        hex_data = " ".join(f"{b:02X}" for b in motor_data)
        self._log_tx_with_info(frame, "电机透传", f"关节={joint}, 数据={hex_data}")

    def _on_frame_received(self, frame):
        """接收帧回调"""
        # 构建原始帧用于显示
        from protocol import build_frame as bf
        raw = bf(frame.address, frame.function_code, frame.data)
        self.window.log_rx(raw)

        # 处理响应
        if len(frame.data) < 1:
            return

        status = frame.data[0]
        data = frame.data[1:] if len(frame.data) > 1 else b''

        # 解析状态
        if frame.function_code == FUNC_READ_SINGLE_STATUS:
            if status == STATUS_SUCCESS and frame.address <= 6:
                joint_data = parse_joint_status(data)
                joint = self.joints[frame.address - 1]
                joint.current_angle = joint_data.get('current_angle', 0)
                joint.target_angle = joint_data.get('target_angle', 0)
                joint.current_speed = joint_data.get('current_speed', 0)
                joint.motor_current = joint_data.get('motor_current', 0)
                joint.motor_speed = joint_data.get('motor_speed', 0)
                joint.pos_error = joint_data.get('pos_error', 0)
                joint.enabled = joint_data.get('enabled', 0) != 0
                joint.voltage = joint_data.get('voltage', 0)
                joint.stall_flag = joint_data.get('stall_flag', 0) != 0
                # 更新UI中的关节状态显示
                if frame.address <= 4:
                    self.window.joint_status_widgets[frame.address - 1].update_status(joint)
                elif frame.address == 5:
                    self.window.joint5_widget.update_status(joint)
                elif frame.address == 6:
                    self.window.joint6_widget.update_status(joint)
                self.window.log_info(f"关节{frame.address}状态已更新")

        elif frame.function_code == FUNC_READ_FULL_STATUS:
            if status == STATUS_SUCCESS:
                sys_data = parse_system_status(data)
                self.system.voltage = sys_data.get('voltage', 0)
                self.system.uart_status = sys_data.get('uart_status', 0)
                self.system.flash_status = sys_data.get('flash_status', 0)
                self.window.system_status.update_status(
                    self.system.voltage, self.system.uart_status, self.system.flash_status
                )
                self.window.log_info("系统状态已更新")

        elif frame.function_code == FUNC_READ_GRIPPER_STATUS:
            if status == STATUS_SUCCESS:
                grip_data = parse_gripper_status(data)
                self.gripper.current_pwm = grip_data.get('current_pwm', 0)
                self.gripper.mode = grip_data.get('mode', 0)
                self.gripper.open_status = grip_data.get('open_status', 0)
                self.gripper.moving = grip_data.get('moving', 0) != 0
                self.gripper.error = grip_data.get('error', 0) != 0
                self.window.gripper_control.update_status(
                    self.gripper.current_pwm, self.gripper.mode,
                    self.gripper.open_status, self.gripper.moving, self.gripper.error
                )

        elif frame.function_code == FUNC_READ_LIMIT_POS:
            if status == STATUS_SUCCESS and frame.address <= 6:
                limit_data = parse_limit_position(data)
                is_set = limit_data.get('is_set', False)
                limit_pulse = limit_data.get('limit_pulse', 0)
                self.window.config_widget.update_limit_position(
                    frame.address, is_set, limit_pulse
                )
                self.window.log_info(f"关节{frame.address}极限位置: {'已设置' if is_set else '未设置'}, 脉冲={limit_pulse}")

        # 通用响应处理
        if status != STATUS_SUCCESS:
            func_name = get_function_name(frame.function_code)
            status_text = get_status_text(status)
            self.window.log_error(f"{func_name} 失败: {status_text}")

    def _on_start_recording(self):
        """开始记录"""
        self.recorder.start()
        self.window.log_info("开始记录轨迹")

    def _on_stop_recording(self):
        """停止记录"""
        self.recorder.stop()
        self.window.log_info(f"停止记录，共 {len(self.recorder.records)} 个数据点")


def main():
    app = QApplication(sys.argv)

    # 创建应用
    robot_app = RobotArmApp()
    window = robot_app.run()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
