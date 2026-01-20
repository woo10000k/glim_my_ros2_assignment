"""glim_my_ros2_assignment - Constants"""

# Coordinate Mode
COORDINATE_MODE = {
    'ABSOLUTE': 0,  # Base origin reference
    'RELATIVE': 1   # Offset from current TCP
}

# Sync Type
SYNC_TYPE = {
    'SYNC': 0,   # Wait for motion completion
    'ASYNC': 1   # Return immediately
}

# Stop Mode
STOP_MODE = {
    'QUICK_STO': 0,
    'QUICK': 1,
    'SOFT': 2,
    'HOLD': 3
}

# Robot State
ROBOT_STATE = {
    0: 'STATE_INITIALIZING',
    1: 'STATE_STANDBY',
    2: 'STATE_MOVING',
    3: 'STATE_SAFE_OFF',
    4: 'STATE_TEACHING',
    5: 'STATE_SAFE_STOP',
    6: 'STATE_EMERGENCY_STOP',
    7: 'STATE_HOMMING',
    8: 'STATE_RECOVERY',
    9: 'STATE_SAFE_STOP2',
    10: 'STATE_SAFE_OFF2'
}

# Velocity Limits (E0509)
VELOCITY = {
    'joint_min': 1.0,
    'joint_max': 225.0,
    'joint_default': 30.0,
    'task_trans_min': 1.0,
    'task_trans_max': 1000.0,
    'task_trans_default': 100.0,
    'task_rot_min': 1.0,
    'task_rot_max': 225.0,
    'task_rot_default': 30.0
}

# Acceleration Limits (E0509)
ACCELERATION = {
    'joint_min': 1.0,
    'joint_max': 225.0,
    'joint_default': 30.0,
    'task_trans_min': 1.0,
    'task_trans_max': 1000.0,
    'task_trans_default': 100.0,
    'task_rot_min': 1.0,
    'task_rot_max': 225.0,
    'task_rot_default': 30.0
}

# Coordinate Limits (E0509)
COORDINATE_LIMITS = {
    'position_min': -1500.0,
    'position_max': 1500.0,
    'rotation_min': -180.0,
    'rotation_max': 180.0
}

# Reference Frame
REFERENCE_FRAME = {
    'DR_BASE': 0,
    'DR_TOOL': 1,
    'DR_WORLD': 2
}

# Alarm Messages
ALARM_MESSAGES = {
    (1, 1): "Workspace limit exceeded",
    (1, 2): "Singularity - cannot move near singular point",
    (1, 3): "Joint limit exceeded",
    (1, 4): "Velocity limit exceeded",
    (1, 5): "Acceleration limit exceeded",
    (2, 1): "Self collision detected",
    (2, 2): "External collision detected",
    (3, 1): "Servo error",
    (3, 2): "Communication error",
}

# Default namespace
DEFAULT_NAMESPACE = 'dsr01'

# E0509 Home Position TCP (all joints = 0)
HOME_TCP_POSITION = [0.0, 0.0, 1123.0, 0.0, 0.0, 0.0]

# GUI Update Interval (ms)
GUI_UPDATE_INTERVAL = 100
