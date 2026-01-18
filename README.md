# atwork_commander_msgs (ROS 2 Humble Port)

This repository contains the ROS 2 Humble port of the original **atwork_commander_msgs** package from ROS 1.
It provides custom message and service definitions used by the AtWork Commander system.

The package has been migrated to ROS 2 using `ament_cmake` and `rosidl` and is fully compatible with **ROS 2 Humble Hawksbill**.

---

## Converting ROS 1 Bags to ROS 2 (Humble)

To use recorded ros bag file from ROS1 with ROS 2 Humble, you must convert the bag format.
Use the following command:

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
