# defined_runtime

ROS2 package that executes compiled BehaviorTree.CPP task programs on the Defined Robotics platform.

## What it does

Loads a BT XML file produced by `defined-compiler`, ticks the behavior tree at a configurable rate, and publishes per-step execution status to `/task_status`. New tasks can be loaded at runtime without restarting the node.

### Action nodes

| Node | Type | Description |
|------|------|-------------|
| `GoTo` | Stateful (async) | Sends a `NavigateToPose` goal to Nav2 and tracks completion |
| `Wait` | Stateful | Pauses execution for a configurable duration (sim-time-aware) |
| `Report` | Sync | Publishes a string message to a ROS2 topic and logs it |

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/task_status` | `std_msgs/String` | Published | JSON execution progress |
| `/task_command` | `std_msgs/String` | Subscribed | Path to a BT XML file to load |
| `/task_reports` | `std_msgs/String` | Published | Messages from `Report` nodes |

### `/task_status` JSON format

```json
{"step": "go_to_1_0", "status": "RUNNING", "current": 2, "total": 5, "progress": 40}
```

Status values: `IDLE`, `RUNNING`, `SUCCESS`, `FAILURE`

### Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `bt_xml_path` | string | `""` | BT XML to load on startup (empty = wait for `/task_command`) |
| `tick_rate` | double | `100.0` | Tree tick frequency in Hz |
| `enable_groot` | bool | `true` | Enable Groot2 ZMQ publisher on port 1667 |
| `use_sim_time` | bool | `true` | Use simulation clock |

## Build

Build inside the Docker sim stack (ROS Jazzy):

```bash
cd work/defined_platform
docker compose -f docker/docker-compose.yml up --build
```

Or build locally if you have ROS Jazzy installed:

```bash
cd work/defined_platform
colcon build --packages-select defined_runtime
```

## Run

### Launch with a BT XML file

```bash
ros2 launch defined_runtime bt_executor.launch.py \
  bt_xml_path:=/path/to/task.bt.xml
```

### Load a task at runtime

```bash
ros2 topic pub /task_command std_msgs/String \
  "data: '/path/to/task.bt.xml'" --once
```

### Monitor execution

```bash
ros2 topic echo /task_status
```

### Full demo (patrol task)

First compile a task with `defined-compiler`:

```bash
defined-compile examples/patrol_task.yaml \
  --rdf examples/defined_mvp.rdf.yaml \
  --verbs-dir verb_library/ \
  --output /tmp/patrol.bt.xml
```

Then launch the executor:

```bash
ros2 launch defined_runtime bt_executor.launch.py \
  bt_xml_path:=/tmp/patrol.bt.xml \
  use_sim_time:=true
```

Watch `/task_status` and the Foxglove 3D view simultaneously.

## Visualise with Groot2

When `enable_groot:=true` (default), connect Groot2 to `localhost:1667` to see
the live tree state while execution is running.

## Test

Unit tests (no sim required):

```bash
colcon test --packages-select defined_runtime
colcon test-result --verbose
```

Integration tests require the full sim stack and are tagged for CI.

## BT XML authoring

The tree uses BT.CPP format 4. A minimal patrol tree:

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="Patrol">
    <Sequence>
      <Action ID="GoTo" x="1.0" y="0.5" theta="0.0" timeout="30.0"/>
      <Action ID="Report" message="reached_waypoint_1"/>
      <Action ID="Wait" duration="2.0"/>
      <Action ID="GoTo" x="0.0" y="0.0" theta="0.0" timeout="30.0"/>
      <Action ID="Report" message="patrol_complete"/>
    </Sequence>
  </BehaviorTree>
</root>
```

See `test/fixtures/` for more examples.

## Current limitations

- Frame ID defaults to `map`; only 2D navigation (x, y, theta) is supported
- No goal restart on `FAILURE` — send a new `/task_command` to retry
- `defined-tune` (Layer 2 nav tuning) is Milestone 2 scope
