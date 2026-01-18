# atwork_commander_msgs & interpreter (ROS 2 Humble Port)

This repository contains the ROS 2 Humble port of the original **atwork_commander_msgs** package from ROS 1,
along with the **interpreter** package, which converts referee task messages into a planner-ready format.

---

## Packages in This Repository

### 1. atwork_commander_msgs

Provides custom message and service definitions used by the AtWork Commander system.

This package is a ROS 2 port of the original ROS 1 messages and is built using:

* `ament_cmake`
* `rosidl`

It is fully compatible with **ROS 2 Humble Hawksbill**.

---

### 2. interpreter

The `interpreter` package is a ROS 2 Python node that acts as a bridge between the referee system and the robot task planner.

It subscribes to referee task messages and converts them into a simplified, planner-ready 2D task table.

#### Function

Subscribes to:

```
/atwork_commander/object_task   (atwork_commander_msgs/ObjectTask)
```

Publishes:

```
/parsed_object_tasks            (std_msgs/String, JSON encoded)
```

#### Output Format

The published message contains a 2D string array encoded as JSON:

```json
[
  ["11", "WS01", "WS04"],
  ["13", "WS01", "WS04"],
  ["12", "WS02", "WS04"],
  ["16", "WS04", "WS02"]
]
```

Each row represents:

```
[ object_type, source_workspace, destination_workspace ]
```

Duplicate referee messages are automatically filtered so that the planner is only triggered on real task changes.

---

## Converting ROS 1 Bags to ROS 2 (Humble)

To use recorded ROS 1 bag files with ROS 2 Humble, convert the bag format:

```bash
rosbags-convert --src path/to/ros1_bag.bag --dst path/to/ros2_bag
```

---

## Playing the Converted Bag in ROS 2

After conversion, play the bag using:

```bash
ros2 bag play path/to/ros2_bag
```

---

## Playing at a Slower Rate

To slow down playback:

```bash
ros2 bag play ~/bags/atwork_run_ros2 --rate 0.5
```

To loop continuously:

```bash
ros2 bag play ~/bags/atwork_run_ros2 --loop
```

---

## Running the Interpreter Node

After building the workspace:
Run the interpreter node:

```bash
ros2 run interpreter atwork_task_parser
```

Verify output:

```bash
ros2 topic echo /parsed_object_tasks
```

---
