# Local Planning

## Purpose and scope

This directory archives alternative ROS 1 Python local-planning implementations for the F1TENTH vehicle. The scripts are not a single combined planning pipeline and have not been packaged here as a standalone catkin package. They consume global waypoints, vehicle state, and obstacle or LiDAR data to produce local avoidance references.

The preserved code uses `f110_msgs`, NumPy/SciPy and Frenet-coordinate conversion (`frenet_utils.py`). The original driving workflow launches the spline planner; other files are alternative or experimental implementations rather than confirmed simultaneous runtime components.

## Files and algorithms

| File | Description |
|---|---|
| `static_obs_spline_planner.py` | Spline-based obstacle avoidance; the planner launched by the recorded driving alias (`7`); uses raw obstacle detections. |
| `follow_the_gap.py` | LiDAR gap-based reactive avoidance alternative. |
| `lattice_planner3.py` | Lattice-based candidate generation, collision checking, path selection, and optional dynamic-obstacle handling. Its default obstacle topic is `/perception/detection/tracked_obstacles`, but `~obstacle_topic` is configurable. |
| `lattice_planner_follow111.py` | Experimental follow/avoidance variant; the presence of overtaking helper functions alone does not prove that a full overtaking state sequence executes. |
| `lattice_planner3_dynamic_overtake.py` | Experimental variant with an explicit dynamic-overtaking sequence; its default obstacle input is raw detections. |
| `obstacle_velocity_estimator.py` | Separate estimator that transforms raw detections into tracked obstacles with velocity estimates; stored here alongside the planning experiments. |
| `frenet_utils.py` | Shared Cartesian/Frenet coordinate conversion helper. |

## ROS interfaces

| Topic | Message type | Usage |
|---|---|---|
| `/global_path/optimal_trajectory_wpnt` | `f110_msgs/WpntArray` | Global reference input |
| `/perception/detection/raw_obstacles` | `f110_msgs/ObstacleArray` | Detection input for raw-obstacle planners and the external velocity estimator |
| `/perception/detection/tracked_obstacles` | `f110_msgs/ObstacleArray` | Default obstacle input to `lattice_planner3.py` |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Vehicle pose for selected planners |
| `/odom` | `nav_msgs/Odometry` | Vehicle speed input for selected planners |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR-based alternatives; optional internal LiDAR processing in `lattice_planner3.py` |
| `/map` | `nav_msgs/OccupancyGrid` | Map-based checks in selected planners |
| `/planner/avoidance/otwpnts` | `f110_msgs/OTWpntArray` | Avoidance waypoint output to the controller |
| `/planner/avoidance/mode` | `std_msgs/String` | Mode output from selected follow/overtaking variants |

## Lattice planner integration details

`lattice_planner3.py` includes its own obstacle motion-history logic and an optional internal LiDAR processing mode (`~use_internal_lidar`, disabled by default). Thus, the presence of an internal motion tracker does **not** imply that the separate `obstacle_velocity_estimator.py` is always mandatory. However, with the archived default `~obstacle_topic=/perception/detection/tracked_obstacles`, some compatible publisher must supply that topic. A separate estimator is one way to provide it; changing the obstacle topic requires validation of message content and the planner's expected behavior.

The planner also exposes `~dynamic_avoidance_enabled` (default `False`). Dynamic following/overtaking behavior should not be documented as active in the original driving workflow without the corresponding runtime configuration and test results.

**Known parameter-name issue:** the archived source reads `~dynamic_trigger_s_msax` and `~dynamic_blsock_d_margin`, although the Python variables are named `dynamic_trigger_s_max` and `dynamic_block_d_margin`. Correcting these typos in the Python source and updating any parameter files is a pending code change; setting the expected spellings alone will not override the present defaults.

## Original execution

The original driving alias ran the spline planner from the original `vesc_driver` package:

```bash
cd ~/vesc_ws
source devel/setup.bash
rosrun vesc_driver static_obs_spline_planner.py
```

This command documents the historical workspace, not an executable command for this rearranged directory. After restoring dependencies and the ROS environment, individual scripts may be run as experiments, but their topic publishers, `f110_msgs` definitions, configuration and TF dependencies must be checked first. Do not start multiple alternatives publishing the same avoidance topic unless command/path arbitration is explicitly implemented.

## Pending development and validation

- Confirm input topics and message definitions for the selected planner.
- Validate the raw-to-tracked obstacle pipeline where tracked detections are used.
- Fix the two misspelled ROS parameter keys in `lattice_planner3.py` and test parameter overrides.
- Validate dynamic-obstacle classification, following/overtaking transitions, map-based rejection and collision checks on the target vehicle.
- Verify output compatibility with the MAP controller and test only one active avoidance publisher at a time.

For historical selection notes, see [local planning selection](../../docs/local-planning-selection.md).
