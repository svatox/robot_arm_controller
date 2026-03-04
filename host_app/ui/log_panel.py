"""
日志面板组件
"""

from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                               QPushButton, QTextEdit, QFrame, QCheckBox)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QFont, QTextCursor
from datetime import datetime
import os


class LogPanel(QFrame):
    """日志面板"""

    log_signal = Signal(str, str)  # direction, message

    def __init__(self, parent=None):
        super().__init__(parent)
        self._init_ui()
        self.max_lines = 1000

    def _init_ui(self):
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)

        title = QLabel("通信日志")
        title.setFont(QFont("Microsoft YaHei", 10, QFont.Bold))

        # 日志显示区域
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        self.log_display.setFont(QFont("Consolas", 9))

        # 按钮
        btn_clear = QPushButton("清除")
        btn_clear.clicked.connect(self._on_clear)

        btn_save = QPushButton("保存日志")
        btn_save.clicked.connect(self._on_save)

        # 自动滚动checkbox
        self.chk_auto_scroll = QCheckBox("自动滚动")
        self.chk_auto_scroll.setChecked(True)

        btn_layout = QHBoxLayout()
        btn_layout.addWidget(btn_clear)
        btn_layout.addWidget(btn_save)
        btn_layout.addWidget(self.chk_auto_scroll)
        btn_layout.addStretch()

        layout = QVBoxLayout()
        layout.addWidget(title)
        layout.addWidget(self.log_display, 1)
        layout.addLayout(btn_layout)

        self.setLayout(layout)

    def add_log(self, direction: str, message: str, color: str = ""):
        """添加日志"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        prefix = "→" if direction == "TX" else "←"

        if color:
            html = f'<span style="color: {color};">[{timestamp}] {prefix} {message}</span>'
        else:
            html = f'[{timestamp}] {prefix} {message}'

        self.log_display.append(html)

        # 限制行数
        if self.max_lines > 0:
            cursor = self.log_display.textCursor()
            cursor.movePosition(QTextCursor.Start)
            while self.log_display.document().blockCount() > self.max_lines:
                cursor.selectBlockFormat()
                cursor.removeSelectedText()
                cursor.deleteChar()

        # 自动滚动
        if self.chk_auto_scroll.isChecked():
            self.log_display.moveCursor(QTextCursor.End)

    def log_tx(self, data: bytes):
        """记录发送数据"""
        hex_str = " ".join(f"{b:02X}" for b in data)
        self.add_log("TX", f"发送: {hex_str}", "#3498db")

    def log_rx(self, data: bytes):
        """记录接收数据"""
        hex_str = " ".join(f"{b:02X}" for b in data)
        self.add_log("RX", f"接收: {hex_str}", "#27ae60")

    def log_error(self, message: str):
        """记录错误"""
        self.add_log("ERR", message, "#e74c3c")

    def log_info(self, message: str):
        """记录信息"""
        self.add_log("INFO", message, "#95a5a6")

    def _on_clear(self):
        """清除日志"""
        self.log_display.clear()

    def _on_save(self):
        """保存日志"""
        from PySide6.QtWidgets import QFileDialog
        filename, _ = QFileDialog.getSaveFileName(
            self, "保存日志", f"robot_arm_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
            "文本文件 (*.txt);;所有文件 (*)"
        )
        if filename:
            try:
                with open(filename, 'w', encoding='utf-8') as f:
                    f.write(self.log_display.toPlainText())
            except Exception as e:
                self.log_error(f"保存失败: {e}")
