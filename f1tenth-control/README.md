# Control

## Purpose and architecture

This module tracks global or avoidance waypoints and converts the selected reference into speed and steering commands. The ROS 1 Python MAP controller uses NumPy and a steering lookup table; the vehicle interface subsequently converts Ackermann commands to VESC commands.

```text
AMCL pose + global waypoints + avoidance waypoints + joystick
                         -> MAP controller -> /ackermann_cmd -> vehicle interface
```

## Contents

- `map_controller/src/MAP_controller_real1.py`: path tracking, reference selection, and joystick mode handling.
- `map_controller/src/time_error_tracker.py`: separate lap/time and path-error recording logic.
- `map_controller/launch/sim_MAP.launch`: starts both scripts. Despite its name, the selected controller is the real-vehicle script.
- `map_controller/cfg/map_params_real.yaml`: lookahead, recovery, speed-rate limits, and lookup-table selection.
- `steering_lookup/`: Python package with `lookup_steer_angle.py` and `cfg/SIM_linear_lookup_table.csv`.

## ROS connections

| Direction | Topic | Message |
|---|---|---|
| Input | `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| Input | `/global_path/optimal_trajectory_wpnt` | `f110_msgs/WpntArray` |
| Input | `/planner/avoidance/otwpnts` | `f110_msgs/OTWpntArray` |
| Input | `/planner/avoidance/mode` | `std_msgs/String` |
| Input | `/joy` | `sensor_msgs/Joy` |
| Output | `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` |
| Visualization | `lookahead_point` | `visualization_msgs/Marker` |

The controller starts in manual mode. It selects a fresh avoidance path when available and otherwise uses the global reference. The configured lookup name `SIM_linear` resolves to the archived CSV.

The separate `time_error_tracker.py` expects `/car_state/pose` (`PoseStamped`) and `/global_waypoints` (`WpntArray`). These differ from the controller inputs; the pose message type also differs. Its connection must be resolved before assuming the recorder works with this workflow.

## Original execution

After building and sourcing the original workspace and providing localization, waypoints, and the vehicle interface:

```bash
cd ~/MAP_ws
source devel/setup.bash
roslaunch map_controller sim_MAP.launch
```

The archive preserves source packages rather than a built workspace. `steering_lookup` and shared messages must be discoverable in the sourced environment. The supplied build configuration has not been validated here. See [workflow reference](../docs/README.md) and [source notices](../docs/source-notices/MAP-Controller/LICENSE).
