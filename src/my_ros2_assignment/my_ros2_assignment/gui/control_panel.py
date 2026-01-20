"""glim_my_ros2_assignment - Control Panel"""

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QGridLayout,
    QLabel, QDoubleSpinBox, QRadioButton, QButtonGroup, QPushButton
)
from PyQt5.QtCore import pyqtSignal

from ..utils.constants import VELOCITY, ACCELERATION, COORDINATE_LIMITS, COORDINATE_MODE, HOME_TCP_POSITION


class CoordinateInputWidget(QWidget):
    """Single axis coordinate input with increment buttons"""

    def __init__(self, label: str, unit: str, min_val: float, max_val: float):
        super().__init__()
        self.unit = unit

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        self.label = QLabel(f"{label}:")
        self.label.setFixedWidth(25)
        layout.addWidget(self.label)

        self.spinbox = QDoubleSpinBox()
        self.spinbox.setRange(min_val, max_val)
        self.spinbox.setDecimals(2)
        self.spinbox.setSuffix(f" {unit}")
        self.spinbox.setFixedWidth(110)
        self.spinbox.setKeyboardTracking(False)
        layout.addWidget(self.spinbox)

        increments = [(-100, '-100'), (-10, '-10'), (-1, '-1'), (-0.1, '-0.1'),
                      (0.1, '+0.1'), (1, '+1'), (10, '+10'), (100, '+100')]

        for delta, text in increments:
            btn = QPushButton(text)
            btn.setFixedWidth(38)
            btn.setFixedHeight(24)
            btn.clicked.connect(lambda checked, d=delta: self._adjust_value(d))
            layout.addWidget(btn)

        layout.addStretch()

    def _adjust_value(self, delta: float):
        new_value = self.spinbox.value() + delta
        new_value = max(self.spinbox.minimum(), min(self.spinbox.maximum(), new_value))
        self.spinbox.setValue(new_value)

    def value(self) -> float:
        return self.spinbox.value()

    def setValue(self, value: float):
        self.spinbox.setValue(value)


class CoordinateInputGroup(QGroupBox):
    """Full coordinate input (X, Y, Z, RX, RY, RZ)"""

    def __init__(self):
        super().__init__("Target Coordinate Input")
        layout = QVBoxLayout(self)

        pos_limit = (COORDINATE_LIMITS['position_min'], COORDINATE_LIMITS['position_max'])
        rot_limit = (COORDINATE_LIMITS['rotation_min'], COORDINATE_LIMITS['rotation_max'])

        self.x_input = CoordinateInputWidget("X", "mm", *pos_limit)
        self.y_input = CoordinateInputWidget("Y", "mm", *pos_limit)
        self.z_input = CoordinateInputWidget("Z", "mm", *pos_limit)
        self.rx_input = CoordinateInputWidget("RX", "deg", *rot_limit)
        self.ry_input = CoordinateInputWidget("RY", "deg", *rot_limit)
        self.rz_input = CoordinateInputWidget("RZ", "deg", *rot_limit)

        layout.addWidget(self.x_input)
        layout.addWidget(self.y_input)
        layout.addWidget(self.z_input)
        layout.addWidget(self.rx_input)
        layout.addWidget(self.ry_input)
        layout.addWidget(self.rz_input)

        self._all_inputs = [self.x_input, self.y_input, self.z_input,
                           self.rx_input, self.ry_input, self.rz_input]

    def setEnabled(self, enabled: bool):
        for inp in self._all_inputs:
            inp.setEnabled(enabled)

    def get_values(self) -> list:
        return [
            self.x_input.value(), self.y_input.value(), self.z_input.value(),
            self.rx_input.value(), self.ry_input.value(), self.rz_input.value()
        ]

    def set_values(self, values: list):
        if len(values) >= 6:
            self.x_input.setValue(values[0])
            self.y_input.setValue(values[1])
            self.z_input.setValue(values[2])
            self.rx_input.setValue(values[3])
            self.ry_input.setValue(values[4])
            self.rz_input.setValue(values[5])


class ControlPanel(QWidget):
    """User input and control panel"""

    # Signals
    target_added = pyqtSignal(list, int, float, float)
    execute_all_requested = pyqtSignal()
    execute_single_requested = pyqtSignal(list, int, float, float)
    stop_requested = pyqtSignal()
    pause_requested = pyqtSignal()
    resume_requested = pyqtSignal()
    home_requested = pyqtSignal()
    mode_changed = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self._current_tcp = HOME_TCP_POSITION.copy()
        self._initial_tcp_set = False
        self._setup_ui()
        self._connect_signals()

    def _setup_ui(self):
        layout = QVBoxLayout(self)

        self.coord_input = CoordinateInputGroup()
        layout.addWidget(self.coord_input)

        self.coord_mode_group = self._create_coordinate_mode_group()
        layout.addWidget(self.coord_mode_group)

        self.velocity_group = self._create_velocity_group()
        layout.addWidget(self.velocity_group)

        self.target_mgmt_group = self._create_target_management_group()
        layout.addWidget(self.target_mgmt_group)

        layout.addWidget(self._create_execution_group())
        layout.addStretch()

        self._set_inputs_enabled(False)

    def _create_coordinate_mode_group(self) -> QGroupBox:
        group = QGroupBox("Coordinate Mode")
        layout = QVBoxLayout()

        self.mode_button_group = QButtonGroup(self)

        self.absolute_radio = QRadioButton("Absolute - Base origin reference")
        self.relative_radio = QRadioButton("Relative - Offset from current TCP")

        self.mode_button_group.addButton(self.absolute_radio, COORDINATE_MODE['ABSOLUTE'])
        self.mode_button_group.addButton(self.relative_radio, COORDINATE_MODE['RELATIVE'])

        self.absolute_radio.setChecked(True)

        layout.addWidget(self.absolute_radio)
        layout.addWidget(self.relative_radio)
        group.setLayout(layout)
        return group

    def _create_velocity_group(self) -> QGroupBox:
        group = QGroupBox("Max Velocity / Acceleration")
        layout = QGridLayout()

        layout.addWidget(QLabel("Max Velocity:"), 0, 0)
        self.max_velocity_input = QDoubleSpinBox()
        self.max_velocity_input.setRange(VELOCITY['task_trans_min'], VELOCITY['task_trans_max'])
        self.max_velocity_input.setValue(VELOCITY['task_trans_default'])
        self.max_velocity_input.setSuffix(" mm/s")
        self.max_velocity_input.setDecimals(1)
        layout.addWidget(self.max_velocity_input, 0, 1)

        layout.addWidget(QLabel("Max Acceleration:"), 1, 0)
        self.max_acceleration_input = QDoubleSpinBox()
        self.max_acceleration_input.setRange(ACCELERATION['task_trans_min'], ACCELERATION['task_trans_max'])
        self.max_acceleration_input.setValue(ACCELERATION['task_trans_default'])
        self.max_acceleration_input.setSuffix(" mm/s2")
        self.max_acceleration_input.setDecimals(1)
        layout.addWidget(self.max_acceleration_input, 1, 1)

        group.setLayout(layout)
        return group

    def _create_target_management_group(self) -> QGroupBox:
        group = QGroupBox("Target Management")
        layout = QHBoxLayout()

        self.add_target_btn = QPushButton("+ Add Target")
        self.add_target_btn.setStyleSheet("font-weight: bold;")
        layout.addWidget(self.add_target_btn)

        group.setLayout(layout)
        return group

    def _create_execution_group(self) -> QGroupBox:
        group = QGroupBox("Execution Control")

        btn_style = """
            QPushButton {{
                background-color: {bg};
                color: white;
                font-weight: bold;
                border: none;
                border-radius: 6px;
                padding: 10px 5px;
                font-size: 12px;
            }}
            QPushButton:hover {{ background-color: {hover}; }}
            QPushButton:pressed {{ background-color: {pressed}; }}
            QPushButton:disabled {{ background-color: #555555; color: #888888; }}
        """

        main_layout = QHBoxLayout()
        main_layout.setSpacing(10)

        # Column 1: Run buttons
        col1 = QVBoxLayout()
        col1.setSpacing(8)

        self.execute_all_btn = QPushButton("Run All")
        self.execute_all_btn.setStyleSheet(btn_style.format(bg='#10B981', hover='#059669', pressed='#047857'))
        self.execute_all_btn.setMinimumHeight(40)

        self.execute_single_btn = QPushButton("Run Single")
        self.execute_single_btn.setStyleSheet(btn_style.format(bg='#3B82F6', hover='#2563EB', pressed='#1D4ED8'))
        self.execute_single_btn.setMinimumHeight(40)

        col1.addWidget(self.execute_all_btn)
        col1.addWidget(self.execute_single_btn)

        # Column 2: Control buttons
        col2 = QVBoxLayout()
        col2.setSpacing(8)

        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setStyleSheet(btn_style.format(bg='#EF4444', hover='#DC2626', pressed='#B91C1C'))
        self.stop_btn.setMinimumHeight(40)

        self.pause_btn = QPushButton("Pause")
        self.pause_btn.setStyleSheet(btn_style.format(bg='#F59E0B', hover='#D97706', pressed='#B45309'))
        self.pause_btn.setMinimumHeight(40)

        self.resume_btn = QPushButton("Resume")
        self.resume_btn.setStyleSheet(btn_style.format(bg='#8B5CF6', hover='#7C3AED', pressed='#6D28D9'))
        self.resume_btn.setMinimumHeight(40)

        col2.addWidget(self.stop_btn)
        col2.addWidget(self.pause_btn)
        col2.addWidget(self.resume_btn)

        # Column 3: Home button
        col3 = QVBoxLayout()
        col3.setSpacing(8)

        self.home_btn = QPushButton("Home\nPosition")
        self.home_btn.setStyleSheet(btn_style.format(bg='#6366F1', hover='#4F46E5', pressed='#4338CA'))
        self.home_btn.setMinimumHeight(88)

        col3.addWidget(self.home_btn)
        col3.addStretch()

        main_layout.addLayout(col1, 1)
        main_layout.addLayout(col2, 1)
        main_layout.addLayout(col3, 1)

        group.setLayout(main_layout)
        return group

    def _connect_signals(self):
        self.add_target_btn.clicked.connect(self._on_add_target)
        self.execute_all_btn.clicked.connect(self.execute_all_requested.emit)
        self.execute_single_btn.clicked.connect(self._on_execute_single)
        self.stop_btn.clicked.connect(self.stop_requested.emit)
        self.pause_btn.clicked.connect(self.pause_requested.emit)
        self.resume_btn.clicked.connect(self.resume_requested.emit)
        self.home_btn.clicked.connect(self.home_requested.emit)
        self.mode_button_group.idClicked.connect(self._on_mode_changed)

    def _on_add_target(self):
        pos = self.coord_input.get_values()
        mode = self.get_coordinate_mode()
        vel = self.get_max_velocity()
        acc = self.get_max_acceleration()
        self.target_added.emit(pos, mode, vel, acc)

    def _on_execute_single(self):
        pos = self.coord_input.get_values()
        mode = self.get_coordinate_mode()
        vel = self.get_max_velocity()
        acc = self.get_max_acceleration()
        self.execute_single_requested.emit(pos, mode, vel, acc)

    def get_coordinate_mode(self) -> int:
        return self.mode_button_group.checkedId()

    def get_max_velocity(self) -> float:
        return self.max_velocity_input.value()

    def get_max_acceleration(self) -> float:
        return self.max_acceleration_input.value()

    def set_buttons_enabled(self, executing: bool):
        self.execute_all_btn.setEnabled(not executing)
        self.execute_single_btn.setEnabled(not executing)
        self.home_btn.setEnabled(not executing)
        self.add_target_btn.setEnabled(not executing)
        self.stop_btn.setEnabled(executing)
        self.pause_btn.setEnabled(executing)
        self.resume_btn.setEnabled(executing)

    def _on_mode_changed(self, mode_id: int):
        self.mode_changed.emit(mode_id)
        if mode_id == 0:  # ABSOLUTE
            self.coord_input.set_values(self._current_tcp)
        else:  # RELATIVE
            self.coord_input.set_values([0.0] * 6)

    def _set_inputs_enabled(self, enabled: bool):
        self.coord_input.setEnabled(enabled)
        self.coord_mode_group.setEnabled(enabled)
        self.velocity_group.setEnabled(enabled)
        self.target_mgmt_group.setEnabled(enabled)

    def set_current_tcp(self, tcp: list):
        if tcp is None or len(tcp) < 6:
            return
        self._current_tcp = list(tcp[:6])

        if not self._initial_tcp_set:
            self._initial_tcp_set = True
            self._set_inputs_enabled(True)
            if self.get_coordinate_mode() == 0:
                self.coord_input.set_values(self._current_tcp)
