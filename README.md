# defined-platform

ROS2 Jazzy + Gz Sim (Ionic) simulation and runtime workspace for Defined Robotics. This is the middleware adapter layer — everything outside this repo is middleware-agnostic.

## Packages

| Package | Description |
|---------|-------------|
| `defined_bringup` | Top-level launch file (`simulation.launch.py`) — composes Gz Sim, Nav2, rosbridge, foxglove, BT executor |
| `defined_gazebo` | Gz Sim worlds, bridge config, odom TF broadcaster |
| `defined_description` | Robot URDF/xacro (`defined_mvp.urdf.xacro`) |
| `defined_navigation` | Nav2 params, SLAM config, navigation launch files |
| `defined_runtime` | BT.CPP executor node with goto, wait, report action nodes |

## Quick Start (Docker)

```bash
# From work/defined_platform/
docker compose -f docker/docker-compose.yml up --build
```

Wait for `Managed nodes are active` in the logs (2-4 min first run).

### Visualize with Foxglove

Connect [Foxglove Studio](https://studio.foxglove.dev/) to `ws://localhost:8765` and import `foxglove/default_layout.json`.

### Teleop

```bash
docker compose -f docker/docker-compose.yml --profile teleop run teleop
```

### Send a Nav2 goal

```bash
docker exec -it defined_sim bash
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## Running Locally (no Docker)

### Gz Sim world only (no robot)

```bash
# Linux:
gz sim -r src/defined_gazebo/worlds/maze_10x10.sdf

# macOS (server + GUI must be separate processes):
gz sim -r -s src/defined_gazebo/worlds/maze_10x10.sdf   # Terminal 1
gz sim -g                                                 # Terminal 2
```

### Full stack (requires ROS2 Jazzy)

```bash
colcon build
source install/setup.bash
ros2 launch defined_bringup simulation.launch.py x:=-3.0 y:=-3.0
```

## GUI on macOS (Docker + XQuartz)

```bash
# Once per session:
socat TCP-LISTEN:6001,reuseaddr,fork UNIX-CLIENT:/tmp/.X11-unix/X0 &
xhost +127.0.0.1

# Gz Sim GUI:
DISPLAY=host.docker.internal:1 docker compose -f docker/docker-compose.yml --profile gui up sim-gui

# RViz:
DISPLAY=host.docker.internal:1 docker compose -f docker/docker-compose.yml --profile viz up rviz
```

## Key Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `maze_10x10.sdf` | Gz Sim world file |
| `use_gui` | `true` | Launch Gz GUI (false for headless/Docker) |
| `use_slam` | `true` | SLAM toolbox vs static map + AMCL |
| `use_sim_time` | `true` | Use `/clock` for simulation time |
| `x`, `y`, `z`, `yaw` | `0, 0, 0.05, 0` | Robot spawn pose |
| `run_bt` | `false` | Launch BT executor |
| `run_task` | `` | Path to BT XML to auto-execute |

## Troubleshooting

- **Stale builds**: `docker compose -f docker/docker-compose.yml down -v` to clear named volumes
- **sim and sim-gui conflict**: Both bind port 9090 — stop one before starting the other
- **Nav2 planner stays inactive**: Ensure `use_slam:=true` is set (no static map provided by default)

## Architecture

See [docs/simulation.md](docs/simulation.md) for detailed simulation setup. This workspace implements the adapter pattern from [DR-010](../../knowledge_base/decision_records/DR-010.md) — ROS2 is replaceable without affecting defined-rdf, defined-compiler, or the dashboard.
