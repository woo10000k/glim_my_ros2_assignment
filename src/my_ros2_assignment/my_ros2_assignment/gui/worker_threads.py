"""Worker Threads for background ROS2 operations"""

import time
import rclpy
from PyQt5.QtCore import QThread, pyqtSignal

from ..utils.constants import GUI_UPDATE_INTERVAL


class MoveWorkerThread(QThread):
    """Worker thread for sequential target execution"""

    started_signal = pyqtSignal()
    progress = pyqtSignal(int, int)
    target_started = pyqtSignal(int)
    target_reached = pyqtSignal(int)
    target_error = pyqtSignal(int, str)
    target_stopped = pyqtSignal(int)
    finished_signal = pyqtSignal(bool)
    error = pyqtSignal(str)
    log = pyqtSignal(str)

    def __init__(self, robot_controller, targets):
        super().__init__()
        self.robot_controller = robot_controller
        self.targets = targets
        self._stop_requested = False
        self._pause_requested = False
        self._current_target_index = -1

    def run(self):
        self.started_signal.emit()

        for i, target_data in enumerate(self.targets):
            pos, mode, vel, acc = target_data
            self._current_target_index = i

            if self._stop_requested:
                self.target_stopped.emit(i)
                self.log.emit("Stop requested - cancelling remaining targets")
                self.finished_signal.emit(False)
                return

            while self._pause_requested:
                time.sleep(0.1)
                if self._stop_requested:
                    self.target_stopped.emit(i)
                    self.finished_signal.emit(False)
                    return

            mode_str = "Relative" if mode == 1 else "Absolute"
            self.log.emit(f"Target {i+1}/{len(self.targets)} [{mode_str}] vel={vel:.0f}")
            self.target_started.emit(i)

            success = self.robot_controller.move_line(pos=pos, vel=vel, acc=acc, mode=mode, sync_type=0)

            if not success:
                if self._stop_requested:
                    self.target_stopped.emit(i)
                    self.finished_signal.emit(False)
                    return

                error_type, error_brief, error_detail = self.robot_controller.get_last_motion_error()
                if error_type:
                    self.target_error.emit(i, error_brief)
                    self.error.emit(f"Target {i+1} failed: {error_detail}")
                else:
                    alarm = self.robot_controller.get_last_alarm()
                    error_msg = self.robot_controller.parse_alarm(alarm)
                    self.target_error.emit(i, "Error")
                    self.error.emit(f"Target {i+1} failed: {error_msg}")
                self.finished_signal.emit(False)
                return

            self.target_reached.emit(i)
            self.progress.emit(i + 1, len(self.targets))

        self.log.emit("All targets completed")
        self.finished_signal.emit(True)

    def request_stop(self):
        self._stop_requested = True
        self._pause_requested = False
        self.robot_controller.move_stop(stop_mode=2)

    def request_pause(self):
        self._pause_requested = True
        self.robot_controller.move_pause()

    def request_resume(self):
        self._pause_requested = False
        self.robot_controller.move_resume()


class SingleMoveWorkerThread(QThread):
    """Worker thread for single target execution"""

    started_signal = pyqtSignal()
    finished_signal = pyqtSignal(bool)
    stopped_signal = pyqtSignal()
    error = pyqtSignal(str)
    error_brief = pyqtSignal(str)
    log = pyqtSignal(str)

    def __init__(self, robot_controller, pos, mode, max_velocity, max_acceleration):
        super().__init__()
        self.robot_controller = robot_controller
        self.pos = pos
        self.mode = mode
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self._stop_requested = False

    def run(self):
        self.started_signal.emit()

        mode_str = "Relative" if self.mode == 1 else "Absolute"
        self.log.emit(f"Single move [{mode_str}] vel={self.max_velocity:.0f}")

        success = self.robot_controller.move_line(
            pos=self.pos, vel=self.max_velocity, acc=self.max_acceleration,
            mode=self.mode, sync_type=0
        )

        if not success:
            if self._stop_requested:
                self.stopped_signal.emit()
                self.finished_signal.emit(False)
                return

            error_type, error_brief, error_detail = self.robot_controller.get_last_motion_error()
            if error_type:
                self.error_brief.emit(error_brief)
                self.error.emit(f"Move failed: {error_detail}")
            else:
                alarm = self.robot_controller.get_last_alarm()
                error_msg = self.robot_controller.parse_alarm(alarm)
                self.error_brief.emit("Error")
                self.error.emit(f"Move failed: {error_msg}")
            self.finished_signal.emit(False)
            return

        self.log.emit("Single move completed")
        self.finished_signal.emit(True)

    def request_stop(self):
        self._stop_requested = True
        self.robot_controller.move_stop(stop_mode=2)


class HomeMoveWorkerThread(QThread):
    """Worker thread for home position movement"""

    started_signal = pyqtSignal()
    finished_signal = pyqtSignal(bool)
    error = pyqtSignal(str)
    log = pyqtSignal(str)

    def __init__(self, robot_controller):
        super().__init__()
        self.robot_controller = robot_controller

    def run(self):
        self.started_signal.emit()
        self.log.emit("Moving to home position...")

        success = self.robot_controller.move_home()

        if not success:
            alarm = self.robot_controller.get_last_alarm()
            error_msg = self.robot_controller.parse_alarm(alarm)
            self.error.emit(f"Home move failed: {error_msg}")
            self.finished_signal.emit(False)
            return

        self.log.emit("Home position reached")
        self.finished_signal.emit(True)


class StateMonitorThread(QThread):
    """Thread for monitoring robot state via ROS2"""

    state_updated = pyqtSignal(dict)

    def __init__(self, robot_controller):
        super().__init__()
        self.robot_controller = robot_controller
        self._running = True

    def run(self):
        while self._running:
            rclpy.spin_once(self.robot_controller, timeout_sec=0.05)

            state = {
                'robot_state': self.robot_controller.robot_state,
                'robot_state_str': self.robot_controller.robot_state_str,
                'current_posj': self.robot_controller.current_posj,
                'current_posx': self.robot_controller.current_posx,
                'connected': self.robot_controller.is_connected
            }
            self.state_updated.emit(state)

            time.sleep(GUI_UPDATE_INTERVAL / 1000.0)

    def stop(self):
        self._running = False
