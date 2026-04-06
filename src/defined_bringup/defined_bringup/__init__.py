"""defined_bringup — ROS2 adapter package.

Re-exports the platform interface so callers can do:
  from defined_bringup import RobotCommand, RobotState, NavigationGoal
"""
from defined_bringup.platform_interface import RobotCommand, RobotState, NavigationGoal

__all__ = ["RobotCommand", "RobotState", "NavigationGoal"]
