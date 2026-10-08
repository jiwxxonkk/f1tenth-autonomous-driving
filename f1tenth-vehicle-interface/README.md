# Vehicle Interface

## Purpose and architecture

This directory preserves vehicle startup configuration and joystick-to-command logic. It connects the ROS planning/control system to the VESC motor/steering interface, LiDAR, and joystick. The archived files use ROS 1 XML launch configuration and Python; the referenced VESC executables come from external C++ packages.

## Files

| File | Role |
|---|---|
| `vesc_driver/launch/f1_start.launch` | Starts VESC, VESC odometry, joystick input, and Ethernet LiDAR for driving |
| `vesc_driver/launch/vesc_driver_node.launch` | Starts the VESC driver for mapping |
| `vesc_driver/scripts/joy.py` | Converts `/joy` input to `/ackermann_cmd` for mapping teleoperation |
| `vesc_ackermann/launch/ackermann_to_vesc_node.launch` | Starts command conversion with vehicle calibration parameters |

The launch files preserve `/dev/ttyACM0`, LiDAR IP `192.168.0.10`, speed-to-ERPM gain `3850`, steering gain `-1.0`, steering offset `0.5`, and wheelbase `0.36`. These are archived vehicle settings, not universal defaults.

## ROS connections

```text
/joy -> controller (driving) or joy.py (mapping) -> /ackermann_cmd
/ackermann_cmd -> Ackermann conversion -> VESC command topics -> driver
VESC state -> odometry conversion -> /odom
VESC IMU -> /sensors/imu/raw
LiDAR -> /scan
```

`/joy` uses `sensor_msgs/Joy`; `/ackermann_cmd` uses `ackermann_msgs/AckermannDriveStamped`. The inspected original driver publishes `sensors/core`, `sensors/imu`, and `sensors/imu/raw`. Command conversion publishes `commands/motor/speed` and `commands/servo/position`. Relative names resolve at the root namespace in the supplied launches.

## Original execution

Driving, in separate sourced terminals:

```bash
cd ~/vesc_ws
source devel/setup.bash
roslaunch vesc_driver f1_start.launch
```

```bash
cd ~/vesc_ws
source devel/setup.bash
roslaunch vesc_ackermann ackermann_to_vesc_node.launch
```

Mapping uses `vesc_driver_node.launch`, separate LiDAR and joystick nodes, and `rosrun vesc_driver joy.py`. See the [workflow reference](../docs/README.md) for the original sequence. The original alias additionally set device permissions before opening `/dev/ttyACM0`.

The two folders here contain selected launch/script files, not complete buildable driver packages. Driver sources, headers, package metadata, and `vesc_msgs` have not been copied into this module. Restore the original packages or prepare an explicit integration before using `roslaunch`/`rosrun`. Do not launch the driving controller and mapping `joy.py` as simultaneous command sources without an intentional command-selection setup.
