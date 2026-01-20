"""glim_my_ros2_assignment - Main Window"""

from PyQt5.QtWidgets import QMainWindow, QWidget, QHBoxLayout, QSplitter, QMessageBox
from PyQt5.QtCore import Qt

from .control_panel import ControlPanel
from .status_panel import StatusPanel
from .worker_threads import MoveWorkerThread, SingleMoveWorkerThread, HomeMoveWorkerThread, StateMonitorThread


class MainWindow(QMainWindow):
    """Main application window"""

    def __init__(self, robot_controller):
        super().__init__()
        self.robot_controller = robot_controller
        self.move_worker = None
        self.state_monitor = None
        self._current_single_target_index = None

        self._setup_ui()
        self._connect_signals()
        self._start_state_monitor()

    def _setup_ui(self):
        self.setWindowTitle("my_ros2_assignment")
        self.setMinimumSize(1200, 800)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QHBoxLayout(central_widget)

        splitter = QSplitter(Qt.Horizontal)

        self.control_panel = ControlPanel()
        splitter.addWidget(self.control_panel)

        self.status_panel = StatusPanel()
        splitter.addWidget(self.status_panel)

        splitter.setSizes([480, 720])
        layout.addWidget(splitter)

    def _connect_signals(self):
        self.control_panel.target_added.connect(self._on_target_added)
        self.control_panel.execute_all_requested.connect(self._on_execute_all)
        self.control_panel.execute_single_requested.connect(self._on_execute_single)
        self.control_panel.stop_requested.connect(self._on_stop)
        self.control_panel.pause_requested.connect(self._on_pause)
        self.control_panel.resume_requested.connect(self._on_resume)
        self.control_panel.home_requested.connect(self._on_home)

    def _start_state_monitor(self):
        self.state_monitor = StateMonitorThread(self.robot_controller)
        self.state_monitor.state_updated.connect(self._on_state_updated)
        self.state_monitor.start()

    def _on_state_updated(self, state: dict):
        self.status_panel.update_connection_status(state['connected'])
        self.status_panel.update_robot_state(state['robot_state'], state['robot_state_str'])
        self.status_panel.update_joint_positions(state['current_posj'])
        self.status_panel.update_tcp_position(state['current_posx'])
        self.control_panel.set_current_tcp(state['current_posx'])

    def _on_target_added(self, pos: list, mode: int, vel: float, acc: float):
        self.status_panel.add_target_to_list(pos, mode, vel, acc)
        mode_str = "Relative" if mode == 1 else "Absolute"
        self.status_panel.append_log(f"Target added [{mode_str}]: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

    def _on_execute_all(self):
        targets = self.status_panel.get_targets()
        if not targets:
            QMessageBox.warning(self, "Warning", "No targets in the list")
            return

        if self.move_worker is not None:
            if self.move_worker.isRunning():
                self.move_worker.request_stop()
                self.move_worker.wait(1000)
            self.move_worker = None

        self.status_panel.clear_error()
        self.status_panel.clear_target_statuses()

        self.move_worker = MoveWorkerThread(self.robot_controller, targets)
        self._connect_move_worker_signals(self.move_worker)
        self.move_worker.start()

        self.control_panel.set_buttons_enabled(True)
        self.status_panel.append_log(f"Executing {len(targets)} targets...")

    def _on_execute_single(self, pos: list, mode: int, vel: float, acc: float):
        if self.move_worker is not None:
            if self.move_worker.isRunning():
                self.move_worker.request_stop()
                self.move_worker.wait(1000)
            self.move_worker = None

        self.status_panel.clear_error()

        selected_index, selected_pos, selected_mode, selected_vel, selected_acc = self.status_panel.get_selected_target()

        if selected_index is not None:
            pos, mode, vel, acc = selected_pos, selected_mode, selected_vel, selected_acc
            self._current_single_target_index = selected_index
            self.status_panel.update_target_status(selected_index, 'Moving')
        else:
            self._current_single_target_index = None

        mode_str = "Relative" if mode == 1 else "Absolute"
        self.status_panel.append_log(f"Executing [{mode_str}] vel={vel:.0f}")

        self.move_worker = SingleMoveWorkerThread(self.robot_controller, pos, mode, vel, acc)
        self._connect_single_move_worker_signals(self.move_worker)
        self.move_worker.start()

        self.control_panel.set_buttons_enabled(True)

    def _connect_move_worker_signals(self, worker: MoveWorkerThread):
        worker.started_signal.connect(lambda: self.status_panel.append_log("Motion started"))
        worker.target_started.connect(lambda i: self.status_panel.update_target_status(i, 'Moving'))
        worker.target_reached.connect(lambda i: self.status_panel.update_target_status(i, 'Completed'))
        worker.target_error.connect(lambda i, brief: self.status_panel.update_target_status(i, 'Error', brief))
        worker.target_stopped.connect(lambda i: self.status_panel.update_target_status(i, 'Stopped'))
        worker.progress.connect(lambda c, t: self.status_panel.append_log(f"Progress: {c}/{t}"))
        worker.error.connect(self._on_motion_error)
        worker.log.connect(self.status_panel.append_log)
        worker.finished_signal.connect(self._on_motion_finished)

    def _connect_single_move_worker_signals(self, worker: SingleMoveWorkerThread):
        worker.started_signal.connect(lambda: self.status_panel.append_log("Single motion started"))
        worker.stopped_signal.connect(self._on_single_motion_stopped)
        worker.error_brief.connect(self._on_single_motion_error_brief)
        worker.error.connect(self._on_single_motion_error)
        worker.log.connect(self.status_panel.append_log)
        worker.finished_signal.connect(self._on_single_motion_finished)

    def _on_stop(self):
        if self.move_worker and self.move_worker.isRunning():
            self.move_worker.request_stop()
        else:
            self.robot_controller.move_stop(stop_mode=2)
        self.status_panel.append_log("Stop requested")

    def _on_pause(self):
        if self.move_worker and self.move_worker.isRunning():
            if isinstance(self.move_worker, MoveWorkerThread):
                self.move_worker.request_pause()
        else:
            self.robot_controller.move_pause()
        self.status_panel.append_log("Pause requested")

    def _on_resume(self):
        if self.move_worker and self.move_worker.isRunning():
            if isinstance(self.move_worker, MoveWorkerThread):
                self.move_worker.request_resume()
        else:
            self.robot_controller.move_resume()
        self.status_panel.append_log("Resume requested")

    def _on_home(self):
        self.status_panel.clear_error()
        self.move_worker = HomeMoveWorkerThread(self.robot_controller)
        self.move_worker.started_signal.connect(lambda: self.status_panel.append_log("Moving to home..."))
        self.move_worker.error.connect(self._on_motion_error)
        self.move_worker.log.connect(self.status_panel.append_log)
        self.move_worker.finished_signal.connect(self._on_motion_finished)
        self.move_worker.start()
        self.control_panel.set_buttons_enabled(True)

    def _on_motion_error(self, message: str):
        self.status_panel.show_error(message)

    def _on_motion_finished(self, success: bool):
        self.control_panel.set_buttons_enabled(False)
        msg = "Motion completed successfully" if success else "Motion stopped or failed"
        self.status_panel.append_log(msg)

    def _on_single_motion_stopped(self):
        if self._current_single_target_index is not None:
            self.status_panel.update_target_status(self._current_single_target_index, 'Stopped')

    def _on_single_motion_error_brief(self, brief: str):
        if self._current_single_target_index is not None:
            self.status_panel.update_target_status(self._current_single_target_index, 'Error', brief)

    def _on_single_motion_error(self, message: str):
        self.status_panel.show_error(message)

    def _on_single_motion_finished(self, success: bool):
        self.control_panel.set_buttons_enabled(False)

        if self._current_single_target_index is not None:
            if success:
                self.status_panel.update_target_status(self._current_single_target_index, 'Completed')
            self._current_single_target_index = None

        msg = "Single motion completed" if success else "Single motion stopped or failed"
        self.status_panel.append_log(msg)

    def closeEvent(self, event):
        if self.state_monitor:
            self.state_monitor.stop()
            self.state_monitor.wait(1000)

        if self.move_worker and self.move_worker.isRunning():
            if isinstance(self.move_worker, MoveWorkerThread):
                self.move_worker.request_stop()
            self.move_worker.wait(1000)

        event.accept()
