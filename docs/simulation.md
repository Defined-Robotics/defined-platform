# How to Run the Simulation

All commands run from `work/defined_platform/`.

## 1. Start the simulation stack

```bash
docker compose -f docker/docker-compose.yml up --build
```

Wait until you see `Managed nodes are active` in the logs (2-4 min on first run).

## 2. Teleop (keyboard control)

In a separate terminal:

```bash
docker compose -f docker/docker-compose.yml --profile teleop run teleop
```

Use `i/j/k/l` keys to move the robot, `q/z` to increase/decrease speed.

## 3. Foxglove Studio

1. Open [Foxglove Studio](https://studio.foxglove.dev/) (web or desktop app)
2. Connect via Foxglove WebSocket: `ws://localhost:8765`
3. Import the layout: **Layout menu > Import** and select `foxglove/default_layout.json`
4. You should see the 3D view (robot + map + costmaps), laser scan overlay, and cmd_vel / odom plots

## 4. Send a Nav2 goal via CLI

Open a shell in the running sim container:

```bash
docker exec -it defined_sim bash
```

Then send a navigation goal:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

The robot should plan a path and navigate to (2.0, 1.0).

## 5. GUI options (requires X11)

### macOS (XQuartz)

```bash
# Once per session:
socat TCP-LISTEN:6001,reuseaddr,fork UNIX-CLIENT:/tmp/.X11-unix/X0 &
xhost +127.0.0.1

# RViz:
DISPLAY=host.docker.internal:1 docker compose -f docker/docker-compose.yml --profile viz up rviz

# Gz Sim GUI:
DISPLAY=host.docker.internal:1 docker compose -f docker/docker-compose.yml --profile gui up sim-gui
```

## 6. Run the BT Executor

The BT executor loads a compiled BehaviorTree XML and drives the robot through it.

### Launch alongside the sim

Open a shell in the running sim container:

```bash
docker exec -it defined_sim bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
```

Then launch the executor with a task file:

```bash
ros2 launch defined_runtime bt_executor.launch.py \
  bt_xml_path:=/ros2_ws/src/defined_runtime/test/fixtures/test_patrol.xml \
  use_sim_time:=true
```

### Monitor task progress

In another terminal inside the container:

```bash
ros2 topic echo /task_status
```

You will see JSON progress updates:

```json
{"step": "go_to_1_0", "status": "RUNNING", "current": 1, "total": 5, "progress": 20}
```

### Load a new task at runtime (no restart needed)

```bash
ros2 topic pub /task_command std_msgs/String \
  "data: '/path/to/new_task.bt.xml'" --once
```

### Compile a task with defined-compiler

From the host machine (requires `defined-compiler` installed):

```bash
defined-compile examples/patrol_task.yaml \
  --rdf examples/defined_mvp.rdf.yaml \
  --verbs-dir verb_library/ \
  --output /tmp/patrol.bt.xml
```

Then copy into the container and load via `/task_command`.

### Visualise the live tree in Groot2

With `enable_groot:=true` (default), connect Groot2 to `localhost:1667`.

## Troubleshooting

- **Named volumes cache stale builds**: `docker compose -f docker/docker-compose.yml down -v` to clear
- **sim and sim-gui conflict**: Both bind port 9090. Stop one before starting the other
