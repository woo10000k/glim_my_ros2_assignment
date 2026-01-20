"""
glim_my_ros2_assignment - Robot Controller
ROS2 Node with service clients for motion control
"""

import math
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import (
    MoveLine, MoveJoint, MoveJointx, MoveHome, MoveStop, MovePause, MoveResume, MoveWait,
    GetCurrentPosj, GetCurrentPosx, GetRobotState, GetLastAlarm, Fkin
)
from dsr_msgs2.msg import RobotState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .utils.constants import ROBOT_STATE, ALARM_MESSAGES, DEFAULT_NAMESPACE, HOME_TCP_POSITION


class MotionError:
    """Motion error types"""
    UNREACHABLE = "unreachable"
    IKIN_FAILED = "ikin_failed"
    SERVICE_UNAVAILABLE = "service_unavailable"
    UNKNOWN = "unknown"

    @staticmethod
    def get_brief(error_type: str) -> str:
        briefs = {
            MotionError.UNREACHABLE: "Unreachable",
            MotionError.IKIN_FAILED: "IKin Failed",
            MotionError.SERVICE_UNAVAILABLE: "No Service",
            MotionError.UNKNOWN: "Error"
        }
        return briefs.get(error_type, "Error")

    @staticmethod
    def get_detail(error_type: str, **kwargs) -> str:
        if error_type == MotionError.UNREACHABLE:
            pos_error = kwargs.get('pos_error', 0)
            requested = kwargs.get('requested', [])
            reachable = kwargs.get('reachable', [])
            return (f"Target position unreachable (error: {pos_error:.1f}mm)\n"
                    f"Requested: {[f'{p:.1f}' for p in requested[:3]]}\n"
                    f"Nearest reachable: {[f'{p:.1f}' for p in reachable[:3]]}")
        elif error_type == MotionError.IKIN_FAILED:
            pos = kwargs.get('pos', [])
            return f"Inverse kinematics failed for position {[f'{p:.1f}' for p in pos[:3]]}"
        elif error_type == MotionError.SERVICE_UNAVAILABLE:
            service = kwargs.get('service', 'Unknown')
            return f"Service not available: {service}"
        return "Unknown error occurred"


class RobotController(Node):
    """Doosan Robot Controller ROS2 Node"""

    def __init__(self, namespace=DEFAULT_NAMESPACE, use_gazebo=True):
        super().__init__('robot_controller')
        self.namespace = namespace
        self.use_gazebo = use_gazebo

        # State variables
        self.robot_state = 0
        self.robot_state_str = 'DISCONNECTED'
        self.current_posj = None
        self.current_posx = None
        self.is_connected = False

        # Error tracking
        self.last_error_type = None
        self.last_error_brief = None
        self.last_error_detail = None

        # Initialize ROS2 interfaces
        self._create_service_clients()
        self._create_subscribers()

        if self.use_gazebo:
            self._create_gazebo_publisher()

        # TCP update control
        self._tcp_logged = False
        self._fkin_pending = False
        self._last_posj_for_fkin = None
        self._tcp_update_timer = self.create_timer(0.5, self._check_and_update_tcp)

        self.get_logger().info(f'RobotController initialized: ns={namespace}, gazebo={use_gazebo}')

    def _check_and_update_tcp(self):
        """Update TCP position using Fkin (only when joints change)"""
        if not self.is_connected or self.current_posj is None or self._fkin_pending:
            return

        # Check for significant joint change (threshold: 0.1 deg)
        if self._last_posj_for_fkin is not None:
            max_diff = max(abs(a - b) for a, b in zip(self.current_posj, self._last_posj_for_fkin))
            if max_diff < 0.1:
                return

        if not self.fkin_client.service_is_ready():
            return

        self._fkin_pending = True
        self._last_posj_for_fkin = list(self.current_posj)

        request = Fkin.Request()
        request.pos = list(self.current_posj)
        request.ref = 0

        future = self.fkin_client.call_async(request)
        future.add_done_callback(self._on_fkin_response)

    def _on_fkin_response(self, future):
        """Handle Fkin response"""
        self._fkin_pending = False
        try:
            result = future.result()
            if result and result.success:
                self.current_posx = list(result.conv_posx)
                if not self._tcp_logged:
                    self._tcp_logged = True
                    self.get_logger().info(f'Initial TCP: {[f"{p:.1f}" for p in self.current_posx]}')
        except Exception:
            pass

    def request_tcp_update(self):
        """Force TCP update on next timer tick"""
        self._last_posj_for_fkin = None

    def _create_service_clients(self):
        """Create ROS2 service clients"""
        # Motion services
        self.move_line_client = self.create_client(MoveLine, f'/{self.namespace}/motion/move_line')
        self.move_joint_client = self.create_client(MoveJoint, f'/{self.namespace}/motion/move_joint')
        self.move_jointx_client = self.create_client(MoveJointx, f'/{self.namespace}/motion/move_jointx')
        self.move_home_client = self.create_client(MoveHome, f'/{self.namespace}/motion/move_home')
        self.move_stop_client = self.create_client(MoveStop, f'/{self.namespace}/motion/move_stop')
        self.move_pause_client = self.create_client(MovePause, f'/{self.namespace}/motion/move_pause')
        self.move_resume_client = self.create_client(MoveResume, f'/{self.namespace}/motion/move_resume')
        self.move_wait_client = self.create_client(MoveWait, f'/{self.namespace}/motion/move_wait')

        # Kinematics service
        self.fkin_client = self.create_client(Fkin, f'/{self.namespace}/motion/fkin')

        # State query services
        self.get_posj_client = self.create_client(GetCurrentPosj, f'/{self.namespace}/aux_control/get_current_posj')
        self.get_posx_client = self.create_client(GetCurrentPosx, f'/{self.namespace}/aux_control/get_current_posx')
        self.get_state_client = self.create_client(GetRobotState, f'/{self.namespace}/system/get_robot_state')
        self.get_alarm_client = self.create_client(GetLastAlarm, f'/{self.namespace}/system/get_last_alarm')

    def _create_subscribers(self):
        """Create topic subscribers"""
        # RobotState (real robot)
        self.state_subscription = self.create_subscription(
            RobotState, f'/{self.namespace}/state', self._state_callback, 10)

        # JointState (Gazebo)
        self.joint_state_subscription = self.create_subscription(
            JointState, f'/{self.namespace}/joint_states', self._joint_state_callback, 10)

        self._joint_name_map = {
            'joint_1': 0, 'joint_2': 1, 'joint_3': 2,
            'joint_4': 3, 'joint_5': 4, 'joint_6': 5
        }

    def _create_gazebo_publisher(self):
        """Create Gazebo position controller publisher"""
        self.gazebo_cmd_pub = self.create_publisher(
            Float64MultiArray, f'/{self.namespace}/gz/dsr_position_controller/commands', 10)

    def _state_callback(self, msg: RobotState):
        """Robot state callback (real robot)"""
        self.robot_state = msg.robot_state
        self.robot_state_str = ROBOT_STATE.get(msg.robot_state, 'UNKNOWN')
        self.current_posj = list(msg.current_posj)
        self.current_posx = list(msg.current_posx)
        self.is_connected = True

    def _joint_state_callback(self, msg: JointState):
        """Joint state callback (Gazebo)"""
        posj = [0.0] * 6
        for i, name in enumerate(msg.name):
            if name in self._joint_name_map:
                idx = self._joint_name_map[name]
                posj[idx] = math.degrees(msg.position[i])

        self.current_posj = posj

        if not self.is_connected:
            self.is_connected = True
            self.robot_state = 1
            self.robot_state_str = 'STATE_STANDBY'
            self.get_logger().info(f'Connected via joint_states')

    def move_line(self, pos, vel, acc, mode=0, sync_type=0) -> bool:
        """
        Move to TCP position
        Gazebo: uses MoveJointx (avoids singularity)
        Real robot: uses MoveLine
        """
        if self.use_gazebo:
            joint_vel = vel[0] if isinstance(vel, (list, tuple)) else vel
            joint_acc = acc[0] if isinstance(acc, (list, tuple)) else acc
            return self._move_line_gazebo(pos, mode, vel=joint_vel, acc=joint_acc)

        if not self.move_line_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('MoveLine service not available')
            return False

        request = MoveLine.Request()
        request.pos = list(pos) if len(pos) == 6 else list(pos) + [0.0] * (6 - len(pos))
        request.vel = [float(vel), 30.0] if isinstance(vel, (int, float)) else list(vel)
        request.acc = [float(acc), 30.0] if isinstance(acc, (int, float)) else list(acc)
        request.time = 0.0
        request.radius = 0.0
        request.mode = mode
        request.blend_type = 0
        request.sync_type = sync_type

        future = self.move_line_client.call_async(request)

        if sync_type == 0:
            rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
            return future.result().success if future.result() else False
        return True

    def _move_line_gazebo(self, pos, mode=0, vel=100.0, acc=100.0) -> bool:
        """Gazebo TCP movement using MoveJointx (internal IKin)"""
        self.last_error_type = None
        self.last_error_brief = None
        self.last_error_detail = None

        tcp_before = self._get_current_posx_gazebo() if self.current_posj else None
        mode_str = "RELATIVE" if mode == 1 else "ABSOLUTE"
        self.get_logger().info(f'[MoveJointx] mode={mode_str}, pos={[f"{p:.1f}" for p in pos]}, vel={vel}')

        if not self.move_jointx_client.wait_for_service(timeout_sec=2.0):
            self.last_error_type = MotionError.SERVICE_UNAVAILABLE
            self.last_error_brief = MotionError.get_brief(MotionError.SERVICE_UNAVAILABLE)
            self.last_error_detail = MotionError.get_detail(MotionError.SERVICE_UNAVAILABLE, service='MoveJointx')
            return False

        request = MoveJointx.Request()
        request.pos = list(pos)
        request.vel = float(vel)
        request.acc = float(acc)
        request.time = 0.0
        request.radius = 0.0
        request.ref = 0
        request.mode = mode
        request.blend_type = 0
        request.sol = 0
        request.sync_type = 0

        future = self.move_jointx_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        if future.result() is None:
            self.last_error_type = MotionError.UNKNOWN
            self.last_error_brief = "Timeout"
            self.last_error_detail = "MoveJointx service call timed out"
            return False

        import time
        time.sleep(0.05)

        # Verify motion
        motion_failed = False
        current_tcp = self._get_current_posx_gazebo()

        if current_tcp and tcp_before:
            if mode == 0:  # ABSOLUTE
                distance_to_target = sum((a - b) ** 2 for a, b in zip(pos[:3], current_tcp[:3])) ** 0.5
                if distance_to_target > 3.0:
                    motion_failed = True
            else:  # RELATIVE
                moved_distance = sum((a - b) ** 2 for a, b in zip(current_tcp[:3], tcp_before[:3])) ** 0.5
                expected_distance = sum(p ** 2 for p in pos[:3]) ** 0.5
                if expected_distance > 1.0 and moved_distance < 1.0:
                    motion_failed = True

        if future.result().success and not motion_failed:
            self.get_logger().info('Motion completed')
            self.request_tcp_update()
            return True

        # Get error info
        alarm = self.get_last_alarm()
        if alarm:
            alarm_code = alarm['index']
            if alarm_code == 1206:
                self.last_error_brief = "Unreachable"
                self.last_error_detail = f"Position unreachable: {[f'{p:.1f}' for p in pos[:3]]}"
            elif alarm_code == 1207:
                self.last_error_brief = "Joint Limit"
                self.last_error_detail = "Joint position limit exceeded"
            elif alarm_code == 3509:
                self.last_error_brief = "Singularity"
                self.last_error_detail = f"Singularity error at position {[f'{p:.1f}' for p in pos[:3]]}"
            else:
                self.last_error_brief = f"Alarm {alarm_code}"
                self.last_error_detail = self.parse_alarm(alarm)
            self.last_error_type = MotionError.UNREACHABLE
        else:
            self.last_error_type = MotionError.UNKNOWN
            self.last_error_brief = "Move Failed"
            self.last_error_detail = "Robot did not reach target position"

        return False

    def move_joint(self, pos, vel, acc, mode=0, sync_type=0) -> bool:
        """Move to joint positions"""
        if self.use_gazebo:
            return self._move_joint_gazebo(pos, mode)

        if not self.move_joint_client.wait_for_service(timeout_sec=1.0):
            return False

        request = MoveJoint.Request()
        request.pos = list(pos)
        request.vel = float(vel)
        request.acc = float(acc)
        request.time = 0.0
        request.radius = 0.0
        request.mode = mode
        request.blend_type = 0
        request.sync_type = sync_type

        future = self.move_joint_client.call_async(request)

        if sync_type == 0:
            rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
            return future.result().success if future.result() else False
        return True

    def _move_joint_gazebo(self, pos, mode=0, vel=30.0, acc=30.0) -> bool:
        """Gazebo joint movement using MoveJoint"""
        if not self.move_joint_client.wait_for_service(timeout_sec=2.0):
            return False

        target_pos = list(pos)
        if mode == 1 and self.current_posj:
            target_pos = [c + p for c, p in zip(self.current_posj, pos)]

        self.get_logger().info(f'[MoveJoint] joints={[f"{d:.2f}" for d in target_pos]}')

        request = MoveJoint.Request()
        request.pos = target_pos
        request.vel = float(vel)
        request.acc = float(acc)
        request.time = 0.0
        request.radius = 0.0
        request.mode = 0
        request.blend_type = 0
        request.sync_type = 0

        future = self.move_joint_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        if future.result() and future.result().success:
            self.current_posj = target_pos
            self.request_tcp_update()
            return True
        return False

    def move_home(self, vel=30.0, acc=30.0) -> bool:
        """Move to home position (all joints = 0)"""
        if self.use_gazebo:
            self.get_logger().info(f'[MoveHome] Moving to home (vel={vel})')
            return self._move_joint_gazebo([0.0] * 6, mode=0, vel=vel, acc=acc)

        if not self.move_home_client.wait_for_service(timeout_sec=1.0):
            return False

        request = MoveHome.Request()
        future = self.move_home_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)
        return future.result().success if future.result() else False

    def move_stop(self, stop_mode=2) -> bool:
        """Stop robot motion (2=SOFT stop)"""
        if not self.move_stop_client.wait_for_service(timeout_sec=1.0):
            return False

        request = MoveStop.Request()
        request.stop_mode = stop_mode
        future = self.move_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result().success if future.result() else False

    def move_pause(self) -> bool:
        """Pause robot motion"""
        if not self.move_pause_client.wait_for_service(timeout_sec=1.0):
            return False

        request = MovePause.Request()
        future = self.move_pause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result().success if future.result() else False

    def move_resume(self) -> bool:
        """Resume robot motion"""
        if not self.move_resume_client.wait_for_service(timeout_sec=1.0):
            return False

        request = MoveResume.Request()
        future = self.move_resume_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result().success if future.result() else False

    def get_current_posj(self) -> list:
        """Get current joint positions"""
        if not self.get_posj_client.wait_for_service(timeout_sec=1.0):
            return self.current_posj

        request = GetCurrentPosj.Request()
        future = self.get_posj_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.result():
            return list(future.result().pos)
        return self.current_posj

    def get_current_posx(self, ref=0) -> list:
        """Get current TCP position"""
        if self.use_gazebo:
            return self._get_current_posx_gazebo()

        if not self.get_posx_client.wait_for_service(timeout_sec=1.0):
            return self.current_posx

        request = GetCurrentPosx.Request()
        request.ref = ref
        future = self.get_posx_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.result() and future.result().success:
            return list(future.result().task_pos_info[0].data[:6])
        return self.current_posx

    def _get_current_posx_gazebo(self) -> list:
        """Get TCP using Fkin (Gazebo)"""
        if self.current_posj is None:
            return HOME_TCP_POSITION.copy()

        if not self.fkin_client.wait_for_service(timeout_sec=1.0):
            return self.current_posx if self.current_posx else HOME_TCP_POSITION.copy()

        request = Fkin.Request()
        request.pos = self.current_posj
        request.ref = 0

        future = self.fkin_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.result() and future.result().success:
            tcp_pos = list(future.result().conv_posx)
            self.current_posx = tcp_pos
            return tcp_pos

        return self.current_posx if self.current_posx else HOME_TCP_POSITION.copy()

    def get_last_alarm(self) -> dict:
        """Get last alarm information"""
        if not self.get_alarm_client.wait_for_service(timeout_sec=1.0):
            return None

        request = GetLastAlarm.Request()
        future = self.get_alarm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.result() and future.result().success:
            alarm = future.result().log_alarm
            return {
                'level': alarm.level,
                'group': alarm.group,
                'index': alarm.index,
                'param': list(alarm.param)
            }
        return None

    def parse_alarm(self, alarm: dict) -> str:
        """Convert alarm to human readable message"""
        if alarm is None:
            return "Unknown error"

        key = (alarm['group'], alarm['index'])
        if key in ALARM_MESSAGES:
            return ALARM_MESSAGES[key]

        return f"Alarm - Group:{alarm['group']}, Index:{alarm['index']}"

    def get_last_motion_error(self) -> tuple:
        """Get last motion error info"""
        return self.last_error_type, self.last_error_brief, self.last_error_detail
