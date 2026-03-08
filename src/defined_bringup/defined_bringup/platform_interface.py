"""
platform_interface.py
Defined Robotics — Middleware-agnostic platform boundary types.

PURPOSE
-------
This module defines the canonical types that flow across the boundary between:
  - The middleware-agnostic layers (defined_rdf, defined_compiler, dashboard)
  - The middleware-specific layer (this ROS2 workspace, defined_platform)

All types here are plain Python dataclasses.  No rclpy, no ROS2 message types,
no Nav2 imports.  If you find yourself importing from geometry_msgs, nav_msgs,
or any ros2 package in this file, you are in the wrong place — put that
translation in the appropriate ROS2 adapter node instead.

MIDDLEWARE SWAP CONTRACT
------------------------
A future "defined_platform_zenoh" or "defined_platform_dds" workspace would:
  1. Import these same types.
  2. Provide its own adapter nodes that translate to/from its wire format.
  3. Require zero changes to defined_rdf, defined_compiler, or the dashboard.

TYPES
-----
  RobotCommand      — instruction sent TO the robot (from compiler or dashboard)
  RobotState        — current robot state reported FROM the platform
  NavigationGoal    — a point in 2D space + optional heading

TRANSLATION POINTS (ROS2-specific, NOT in this file)
------------------------------------------------------
  RobotCommand  → geometry_msgs/Twist on /cmd_vel       (motor_controller_node)
  RobotState    ← nav_msgs/Odometry from /odom          (state_bridge_node, TBD)
  NavigationGoal → nav2_msgs/NavigateToPose action       (bt_executor_node)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional


# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------

class CommandType(Enum):
    """High-level intent of a RobotCommand."""
    NAVIGATE_TO   = auto()   # go_to verb: move to a 2D pose
    STOP          = auto()   # halt all motion immediately
    WAIT          = auto()   # pause for a duration
    CAPTURE_IMAGE = auto()   # trigger camera capture (future)
    REPORT        = auto()   # emit a status report (future)


class ExecutionStatus(Enum):
    """Execution outcome — mirrors the three-layer result model (DR-003)."""
    SUCCESS   = "SUCCESS"
    DEGRADED  = "DEGRADED"
    FAILED    = "FAILED"
    REJECTED  = "REJECTED"   # capability check failed at compile time


# ---------------------------------------------------------------------------
# Core types
# ---------------------------------------------------------------------------

@dataclass
class NavigationGoal:
    """
    A 2D navigation goal in the map frame.

    Coordinates are in metres, yaw in radians.
    This is middleware-agnostic — the ROS2 adapter translates it to
    geometry_msgs/PoseStamped when sending to Nav2's action server.
    """
    x:   float
    y:   float
    yaw: float = 0.0
    frame_id: str = "map"   # logical frame name — adapter maps to ROS2 frame


@dataclass
class RobotCommand:
    """
    A command issued to the robot platform.

    Produced by:
      - The BT executor (running compiled verb YAML)
      - The dashboard (manual override or waypoint selection)

    Consumed by:
      - The ROS2 adapter layer, which translates to the appropriate
        ROS2 topic or action call.

    This type intentionally knows nothing about ROS2 message formats.
    """
    command_type: CommandType
    goal: Optional[NavigationGoal] = None    # set when command_type == NAVIGATE_TO
    wait_duration_s: float = 0.0             # set when command_type == WAIT
    task_id: str = ""                        # correlates command to a compiled task


@dataclass
class RobotState:
    """
    Current robot state, reported from the platform to consumers.

    Produced by:
      - The ROS2 adapter layer, which translates from /odom and TF.

    Consumed by:
      - The dashboard (live position display)
      - The BT executor (pose feedback during navigation)

    All values are in SI units (metres, radians, m/s, rad/s).
    stamp_s is a UNIX timestamp (float), not a ROS2 Time — the adapter
    converts from rclpy.time.Time when building this struct.
    """
    x:        float = 0.0
    y:        float = 0.0
    yaw:      float = 0.0
    vx:       float = 0.0    # linear velocity (m/s)
    vyaw:     float = 0.0    # angular velocity (rad/s)
    stamp_s:  float = 0.0    # seconds since epoch
    frame_id: str   = "map"  # logical frame of the pose


@dataclass
class TaskResult:
    """
    Final outcome of a compiled task execution.

    Produced by the BT executor once the top-level BT node completes.
    Consumed by the dashboard to update task status in the UI.
    """
    task_id: str
    status:  ExecutionStatus
    message: str = ""        # human-readable detail (shown in dashboard)
