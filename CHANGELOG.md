# Changelog

All notable changes to `defined-platform` are documented here.

Contributors: add entries under `[Unreleased]` as part of your PR.
Release PRs (`release/vX.Y.Z`) promote `[Unreleased]` → the versioned heading.

---

## [0.0.1] - 2026-04-06

### Added
- Initial ROS2 Jazzy + Gz Sim Ionic workspace with five packages: `defined_bringup`, `defined_description`, `defined_gazebo`, `defined_navigation`, `defined_runtime`
- `defined_description`: TurtleBot3 Burger-based URDF/xacro (`defined_mvp.urdf.xacro`) with differential drive, 2D lidar, and caster wheel
- `defined_gazebo`: 10x10 maze world (`maze_10x10.sdf`), Gz–ROS2 bridge config, odometry TF broadcaster
- `defined_navigation`: Nav2 stack (navfn planner, DWB controller, recovery behaviours) and SLAM Toolbox (`async_slam_toolbox_node`) with tuned params for Gz Sim Ionic
- `defined_runtime`: BT.CPP executor lifecycle node with `GoTo`, `Wait`, and `Report` action nodes; `/task_command` action server; `/task_status` IDLE heartbeat
- `defined_bringup`: top-level `simulation.launch.py` composing the full stack; launch args for world, spawn pose, SLAM toggle, BT executor, and auto-run task path
- Docker Compose setup (`docker/docker-compose.yml`, `Dockerfile.ros2`): headless `sim`, `sim-gui`, `rviz`, and `teleop` services; named install-cache volume
- Foxglove WebSocket bridge on port 8765 with pre-built layout (`foxglove/default_layout.json`)
- 13 pre-built Nav2 BT trees in `bt_trees/` (replanning, recovery, and odometry-calibration variants)
- `docs/simulation.md`: TF tree, bridge topic list, known quirks

### Fixed
- URDF joint and sensor plugin namespacing corrected for Gz Sim Ionic compatibility
- Physics instability: wheel joints moved to x=−0.08 m, caster radius 0.010 → 0.020 m, kp/kd damping added
- Nav2 planner staying inactive on startup: resolved by enforcing `use_slam:=true`
- `defined_runtime` missing from initial Docker image build
