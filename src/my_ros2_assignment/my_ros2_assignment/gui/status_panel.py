"""glim_my_ros2_assignment - Status Panel"""

from datetime import datetime
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QGridLayout,
    QLabel, QTableWidget, QTableWidgetItem, QTextEdit, QPushButton,
    QHeaderView, QAbstractItemView
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QColor


class StatusPanel(QWidget):
    """Robot status display panel"""

    target_removed = pyqtSignal(int)
    targets_cleared = pyqtSignal()
    target_order_changed = pyqtSignal(int, int)

    SPINNER_FRAMES = ['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧', '⠇', '⠏']

    def __init__(self):
        super().__init__()
        self._targets = []
        self._moving_indices = set()
        self._spinner_index = 0
        self._setup_ui()
        self._connect_signals()
        self._setup_animation_timer()

    def _setup_ui(self):
        layout = QVBoxLayout(self)

        layout.addWidget(self._create_connection_status_group())
        layout.addWidget(self._create_joint_status_group())
        layout.addWidget(self._create_tcp_status_group())
        layout.addWidget(self._create_target_list_group())

        self.error_label = QLabel()
        self.error_label.setStyleSheet("""
            QLabel {
                background-color: #ffcccc; color: #cc0000;
                padding: 10px; border-radius: 5px; font-weight: bold;
            }
        """)
        self.error_label.setWordWrap(True)
        self.error_label.setVisible(False)
        layout.addWidget(self.error_label)

        layout.addWidget(self._create_log_group())

    def _create_connection_status_group(self) -> QGroupBox:
        group = QGroupBox("Connection / Robot Status")
        layout = QHBoxLayout()

        self.connection_indicator = QLabel("O")
        self.connection_indicator.setStyleSheet("color: gray; font-size: 20px;")
        self.connection_text = QLabel("Waiting for connection")
        self.robot_state_label = QLabel("State: -")

        layout.addWidget(self.connection_indicator)
        layout.addWidget(self.connection_text)
        layout.addStretch()
        layout.addWidget(self.robot_state_label)

        group.setLayout(layout)
        return group

    def _create_joint_status_group(self) -> QGroupBox:
        group = QGroupBox("Current Joint Angles (deg)")
        layout = QGridLayout()

        self.joint_labels = []
        for i in range(6):
            label = QLabel(f"J{i+1}: ---")
            label.setStyleSheet("font-family: monospace;")
            self.joint_labels.append(label)
            layout.addWidget(label, i // 3, i % 3)

        group.setLayout(layout)
        return group

    def _create_tcp_status_group(self) -> QGroupBox:
        group = QGroupBox("Current TCP Position (Base Reference)")
        layout = QGridLayout()

        self.tcp_labels = {}
        labels = [('X', 'mm'), ('Y', 'mm'), ('Z', 'mm'),
                  ('RX', 'deg'), ('RY', 'deg'), ('RZ', 'deg')]

        for i, (name, unit) in enumerate(labels):
            label = QLabel(f"{name}: --- {unit}")
            label.setStyleSheet("font-family: monospace;")
            self.tcp_labels[name] = label
            layout.addWidget(label, i // 2, i % 2)

        group.setLayout(layout)
        return group

    def _create_target_list_group(self) -> QGroupBox:
        group = QGroupBox("Target Position List")
        layout = QVBoxLayout()

        self.target_table = QTableWidget()
        self.target_table.setColumnCount(5)
        self.target_table.setHorizontalHeaderLabels(['#', 'Coordinates (X,Y,Z,RX,RY,RZ)', 'Vel/Acc', 'Mode', 'Status'])

        header = self.target_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.Fixed)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.Fixed)
        header.setSectionResizeMode(3, QHeaderView.Fixed)
        header.setSectionResizeMode(4, QHeaderView.Fixed)
        self.target_table.setColumnWidth(0, 30)
        self.target_table.setColumnWidth(2, 80)
        self.target_table.setColumnWidth(3, 70)
        self.target_table.setColumnWidth(4, 120)

        self.target_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.target_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.target_table.setMinimumHeight(150)
        layout.addWidget(self.target_table)

        btn_layout = QHBoxLayout()
        self.remove_target_btn = QPushButton("Remove")
        self.clear_targets_btn = QPushButton("Remove All")
        self.move_up_btn = QPushButton("Move Up")
        self.move_down_btn = QPushButton("Move Down")

        btn_layout.addWidget(self.remove_target_btn)
        btn_layout.addWidget(self.clear_targets_btn)
        btn_layout.addWidget(self.move_up_btn)
        btn_layout.addWidget(self.move_down_btn)
        layout.addLayout(btn_layout)

        group.setLayout(layout)
        return group

    def _create_log_group(self) -> QGroupBox:
        group = QGroupBox("Real-time Log")
        layout = QVBoxLayout()

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("font-family: monospace; font-size: 11px;")
        self.log_text.setMinimumHeight(100)
        layout.addWidget(self.log_text)

        self.clear_log_btn = QPushButton("Clear Log")
        layout.addWidget(self.clear_log_btn)

        group.setLayout(layout)
        return group

    def _connect_signals(self):
        self.remove_target_btn.clicked.connect(self._on_remove_target)
        self.clear_targets_btn.clicked.connect(self._on_clear_targets)
        self.move_up_btn.clicked.connect(self._on_move_up)
        self.move_down_btn.clicked.connect(self._on_move_down)
        self.clear_log_btn.clicked.connect(self.log_text.clear)

    def _setup_animation_timer(self):
        self._animation_timer = QTimer(self)
        self._animation_timer.timeout.connect(self._update_spinner)

    def _update_spinner(self):
        self._spinner_index = (self._spinner_index + 1) % len(self.SPINNER_FRAMES)
        spinner = self.SPINNER_FRAMES[self._spinner_index]

        for index in self._moving_indices:
            if 0 <= index < self.target_table.rowCount():
                item = self.target_table.item(index, 4)
                if item:
                    item.setText(f'{spinner} Moving...')

    def _on_remove_target(self):
        row = self.target_table.currentRow()
        if row >= 0:
            self.target_table.removeRow(row)
            if row < len(self._targets):
                self._targets.pop(row)
            self._update_row_numbers()
            self.target_removed.emit(row)

    def _on_clear_targets(self):
        self.target_table.setRowCount(0)
        self._targets.clear()
        self.targets_cleared.emit()

    def _on_move_up(self):
        row = self.target_table.currentRow()
        if row > 0:
            self._swap_rows(row, row - 1)
            self.target_table.selectRow(row - 1)
            self.target_order_changed.emit(row, row - 1)

    def _on_move_down(self):
        row = self.target_table.currentRow()
        if row >= 0 and row < self.target_table.rowCount() - 1:
            self._swap_rows(row, row + 1)
            self.target_table.selectRow(row + 1)
            self.target_order_changed.emit(row, row + 1)

    def _swap_rows(self, row1: int, row2: int):
        for col in range(self.target_table.columnCount()):
            item1 = self.target_table.takeItem(row1, col)
            item2 = self.target_table.takeItem(row2, col)
            self.target_table.setItem(row1, col, item2)
            self.target_table.setItem(row2, col, item1)

        if row1 < len(self._targets) and row2 < len(self._targets):
            self._targets[row1], self._targets[row2] = self._targets[row2], self._targets[row1]

        self._update_row_numbers()

    def _update_row_numbers(self):
        for row in range(self.target_table.rowCount()):
            self.target_table.setItem(row, 0, QTableWidgetItem(str(row + 1)))

    def update_connection_status(self, connected: bool):
        if connected:
            self.connection_indicator.setText("*")
            self.connection_indicator.setStyleSheet("color: #00aa00; font-size: 20px;")
            self.connection_text.setText("Connected")
        else:
            self.connection_indicator.setText("O")
            self.connection_indicator.setStyleSheet("color: gray; font-size: 20px;")
            self.connection_text.setText("Disconnected")

    def update_robot_state(self, state: int, state_str: str):
        self.robot_state_label.setText(f"State: {state_str}")
        if state == 2:
            self.robot_state_label.setStyleSheet("color: blue; font-weight: bold;")
        elif state == 1:
            self.robot_state_label.setStyleSheet("color: green;")
        else:
            self.robot_state_label.setStyleSheet("color: orange;")

    def update_joint_positions(self, positions: list):
        if positions is None:
            for i in range(6):
                self.joint_labels[i].setText(f"J{i+1}: ---")
            return

        for i, pos in enumerate(positions[:6]):
            self.joint_labels[i].setText(f"J{i+1}: {pos:.1f}")

    def update_tcp_position(self, position: list):
        names = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']
        units = ['mm', 'mm', 'mm', 'deg', 'deg', 'deg']

        if position is None:
            for name, unit in zip(names, units):
                self.tcp_labels[name].setText(f"{name}: --- {unit}")
            return

        for i, (name, unit) in enumerate(zip(names, units)):
            if i < len(position):
                self.tcp_labels[name].setText(f"{name}: {position[i]:.1f} {unit}")

    def add_target_to_list(self, position: list, mode: int, vel: float = 100.0, acc: float = 100.0):
        row = self.target_table.rowCount()
        self.target_table.insertRow(row)
        self._targets.append((position, mode, vel, acc))

        self.target_table.setItem(row, 0, QTableWidgetItem(str(row + 1)))

        coord_str = f"({position[0]:.1f}, {position[1]:.1f}, {position[2]:.1f}, " \
                    f"{position[3]:.1f}, {position[4]:.1f}, {position[5]:.1f})"
        self.target_table.setItem(row, 1, QTableWidgetItem(coord_str))

        self.target_table.setItem(row, 2, QTableWidgetItem(f"{vel:.0f}/{acc:.0f}"))
        self.target_table.setItem(row, 3, QTableWidgetItem("Relative" if mode == 1 else "Absolute"))
        self.target_table.setItem(row, 4, QTableWidgetItem("Pending"))

    def update_target_status(self, index: int, status: str, error_brief: str = None):
        if 0 <= index < self.target_table.rowCount():
            item = self.target_table.item(index, 4)
            if item:
                if status == 'Moving':
                    self._moving_indices.add(index)
                    item.setText(f'{self.SPINNER_FRAMES[0]} Moving...')
                    if not self._animation_timer.isActive():
                        self._animation_timer.start(100)
                elif status == 'Error':
                    self._moving_indices.discard(index)
                    item.setText(f'Error: {error_brief}' if error_brief else 'Error')
                    if not self._moving_indices and self._animation_timer.isActive():
                        self._animation_timer.stop()
                elif status == 'Stopped':
                    self._moving_indices.discard(index)
                    item.setText('Stopped')
                    if not self._moving_indices and self._animation_timer.isActive():
                        self._animation_timer.stop()
                else:
                    self._moving_indices.discard(index)
                    item.setText(status)
                    if not self._moving_indices and self._animation_timer.isActive():
                        self._animation_timer.stop()

            colors = {
                'Pending': QColor(255, 255, 255),
                'Moving': QColor(200, 200, 255),
                'Completed': QColor(200, 255, 200),
                'Failed': QColor(255, 200, 200),
                'Error': QColor(255, 180, 180),
                'Stopped': QColor(255, 220, 180)
            }
            bg_color = colors.get(status, QColor(255, 255, 255))
            for col in range(5):
                cell = self.target_table.item(index, col)
                if cell:
                    cell.setBackground(bg_color)

    def get_targets(self) -> list:
        return self._targets.copy()

    def get_selected_target(self) -> tuple:
        row = self.target_table.currentRow()
        if row >= 0 and row < len(self._targets):
            pos, mode, vel, acc = self._targets[row]
            return row, pos, mode, vel, acc
        return None, None, None, None, None

    def clear_target_statuses(self):
        for row in range(self.target_table.rowCount()):
            self.update_target_status(row, 'Pending')

    def show_error(self, message: str):
        self.error_label.setText(f"Warning: {message}")
        self.error_label.setVisible(True)
        self.append_log(f"[ERROR] {message}")

    def clear_error(self):
        self.error_label.setVisible(False)

    def append_log(self, message: str):
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
