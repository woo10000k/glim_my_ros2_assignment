#!/usr/bin/env python3
"""
glim_my_ros2_assignment - Main Entry Point
ROS2 + PyQt5 GUI Application
"""

import sys
import rclpy
from PyQt5.QtWidgets import QApplication

from my_ros2_assignment.robot_controller import RobotController
from my_ros2_assignment.gui.main_window import MainWindow
from my_ros2_assignment.utils.constants import DEFAULT_NAMESPACE


def main(args=None):
    """Main entry point"""
    # Initialize ROS2
    rclpy.init(args=args)

    # Create PyQt5 Application
    app = QApplication(sys.argv)

    # Create RobotController (ROS2 Node)
    robot_controller = RobotController(
        namespace=DEFAULT_NAMESPACE,
        use_gazebo=True
    )

    # Create and show GUI
    window = MainWindow(robot_controller)
    window.show()

    # Run event loop
    exit_code = app.exec_()

    # Cleanup
    robot_controller.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
