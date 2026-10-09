# Control

## Purpose and architecture

This module contains the archived ROS 1 MAP path-tracking controller, its steering lookup package, and a separate lap-time/path-error evaluation node. The controller selects a global or recent avoidance reference and generates Ackermann steering and speed commands for the downstream vehicle interface.

```text
/amcl_pose + global waypoints + avoidance waypoints + planner mode + /joy
                               |
                               v
                    MAP_controller_real1.py
                               |
                               v
                         /ackermann_cmd
                               |
                               v
                    Ackermann-to-VESC interface
```

## Archived files

| Path | Actual role |
|---|---|
| `map_controller/src/MAP_controller_real1.py` | Path tracking, avoidance-reference selection, joystick handling, and `/ackermann_cmd` publication. |
| `map_controller/src/time_error_tracker.py` | Separate node that calculates approximate lap duration and RMS distance to the nearest reference waypoint and **prints** those values to the console. The archived script does not save a CSV or a persistent log file itself. |
| `map_controller/launch/sim_MAP.launch` | Launches the real-vehicle MAP controller **and** `time_error_tracker.py`, despite the `sim_` filename. |
| `map_controller/cfg/map_params_real.yaml` | Controller parameters including lookahead, acceleration/deceleration limits, recovery behavior and steering lookup selection. |
| `steering_lookup/` | Steering lookup implementation and an archived `SIM_linear` CSV table. |

## Main controller interfaces

| Direction | Topic | Message type |
|---|---|---|
| Input | `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| Input | `/global_path/optimal_trajectory_wpnt` | `f110_msgs/WpntArray` |
| Input | `/planner/avoidance/otwpnts` | `f110_msgs/OTWpntArray` |
| Input | `/planner/avoidance/mode` | `std_msgs/String` |
| Input | `/joy` | `sensor_msgs/Joy` |
| Output | `/ackermann_cmd` | `ackermann_msgs/AckermannDriveStamped` |
| Visualization | `lookahead_point` | `visualization_msgs/Marker` |

The controller initializes in manual mode. It can use a recent avoidance reference and otherwise tracks the global path. This does not mean that an avoidance planner is automatically started by the control launch file.

## Evaluation node: separate interface

`time_error_tracker.py` subscribes to:

- `/car_state/pose` (`geometry_msgs/PoseStamped`)
- `/global_waypoints` (`f110_msgs/WpntArray`)

These are **not** the main controller's input topics. In particular, `/amcl_pose` uses `PoseWithCovarianceStamped`, not `PoseStamped`; a topic remap alone cannot convert those message types. The evaluation node therefore requires a compatible pose publisher/converter and a matching waypoint topic before it can be considered integrated with this driving stack.

The script reports lap duration and an RMS value computed from distances to the nearest reference waypoint. It should not be described as writing driving telemetry files, measuring true signed cross-track error, or providing verified race timing without further implementation and validation.

## Historical launch

```bash
cd ~/MAP_ws
source devel/setup.bash
roslaunch map_controller sim_MAP.launch
```

This command assumes the original ROS 1 catkin workspace and its dependencies. The repository is a source archive, not a verified ready-to-build workspace. Required messages, the steering lookup package, external vehicle drivers and working input publishers must be available.

## Pending integration

- Add and verify the pose and waypoint interfaces needed by `time_error_tracker.py`, or adapt its subscribers and message handling to the deployed topics.
- Add persistent results export if recorded lap-time and tracking-error data are required for later analysis.
- Validate joystick mode switching, controller reference transitions and Ackermann-to-VESC calibration on the vehicle.

See the [workflow reference](../docs/README.md) and [preserved MAP Controller source notice](../docs/source-notices/MAP-Controller/LICENSE).
