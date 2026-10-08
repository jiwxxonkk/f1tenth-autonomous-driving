# Perception

## Purpose and role

This module detects obstacles from LiDAR scans and publishes obstacle geometry for local avoidance. It uses ROS 1 Python nodes, TF, NumPy, SciPy-based Frenet conversion, and dynamic_reconfigure.

## Files and processing flow

`perception/opponent_tracker/launch/opponent_track.launch` loads `perception/cfg/opponent_tracker_params221.yaml` and starts:

- `detect_4.py`: laser processing, obstacle extraction, and obstacle/marker publication.
- `dynamic_tracker_server.py`: dynamic parameter server; this is not a velocity-tracking algorithm.

`frenet_utils.py` provides coordinate conversion. `cfg/dyn_tracker_tuner.cfg` generates `perception.cfg.dyn_tracker_tunerConfig` through the package's CMake configuration.

The detector combines scans, global waypoints, the `map` to `laser` transform. Detection feeds the local planner directly or the separate velocity estimator archived under [Local Planning](../f1tenth-planning/local-planning/README.md).

## ROS connections

| Direction | Topic | Message |
|---|---|---|
| Input | `/scan` | `sensor_msgs/LaserScan` |
| Input | `/global_path/optimal_trajectory_wpnt` | `f110_msgs/WpntArray` |
| Input, unless `from_bag` is enabled | `/dynamic_tracker_server/parameter_updates` | `dynamic_reconfigure/Config` |
| Output | `/perception/detection/raw_obstacles` | `f110_msgs/ObstacleArray` |
| Visualization | `/perception/breakpoints_markers`, `/perception/obstacles_markers_new` | `visualization_msgs/MarkerArray` |
| Visualization | `/perception/detect_bound` | `visualization_msgs/Marker` |

The detector waits for global waypoints.

## Original execution and configuration

After building and sourcing the original workspace and starting the required input publishers:

```bash
cd ~/perception_ws
source devel/setup.bash
roslaunch perception opponent_track.launch
```

The archive's launch uses `$(find perception)/cfg/opponent_tracker_params221.yaml`. The original launch referenced an unavailable `opponent_tracker_params2.yaml`; this previously recorded substitution is documented in [local-changes.md](../docs/local-changes.md).

The detector reads `/detect/rate`, `/detect/lambda`, `/detect/sigma`, and `/detect/min_2_points_dist` from the YAML. Several tunable values are read under `dynamic_tracker_server`, so similarly named values under `detect` should not be assumed to control every setting. Build the dynamic configuration and shared messages before running. External package installation has not been verified.
