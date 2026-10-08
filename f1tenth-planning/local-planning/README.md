# Local Planning

## Purpose and technologies

This directory preserves ROS 1 Python alternatives for generating obstacle-avoidance waypoints on the Nano. They use shared `f110_msgs` messages, NumPy/SciPy calculations, and `frenet_utils.py` coordinate conversion.

## Algorithms and roles

| File | Role and default obstacle input |
|---|---|
| `static_obs_spline_planner.py` | Historical alias 7: spline avoidance using `/perception/detection/raw_obstacles` |
| `follow_the_gap.py` | LiDAR gap-based avoidance; perception obstacle integration is optional |
| `lattice_planner3.py` | Lattice planning using `/perception/detection/tracked_obstacles` |
| `lattice_planner_follow111.py` | Following/avoidance alternative using raw obstacles; defined overtaking helpers do not establish execution of an overtaking sequence |
| `lattice_planner3_dynamic_overtake.py` | Alternative with an active overtaking state sequence, using raw obstacles by default |
| `obstacle_velocity_estimator.py` | Converts raw obstacles to tracked obstacles with velocity estimates; functionally a perception component retained here with the selected experiments |
| `frenet_utils.py` | Shared Cartesian/Frenet conversion helper |

These planners are alternatives. Select one avoidance publisher for the controller rather than starting every script together.

## ROS connections

| Topic | Message | Usage |
|---|---|---|
| `/global_path/optimal_trajectory_wpnt` | `f110_msgs/WpntArray` | Global reference input |
| `/perception/detection/raw_obstacles` | `f110_msgs/ObstacleArray` | Spline/raw-obstacle planners and velocity estimator input |
| `/perception/detection/tracked_obstacles` | `f110_msgs/ObstacleArray` | Estimator output; default input to `lattice_planner3.py` |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Lattice and gap planner pose input |
| `/odom` | `nav_msgs/Odometry` | Default lattice planner speed input |
| `/scan`, `/map` | `sensor_msgs/LaserScan`, `nav_msgs/OccupancyGrid` | Inputs used by selected alternatives |
| `/planner/avoidance/otwpnts` | `f110_msgs/OTWpntArray` | Avoidance output to MAP control |
| `/planner/avoidance/mode` | `std_msgs/String` | Output of the follow111 and dynamic-overtake variants |

## Execution

The recorded spline command in the original workspace is:

```bash
cd ~/vesc_ws
source devel/setup.bash
rosrun vesc_driver static_obs_spline_planner.py
```

There is no local-planning ROS package or launch file in this directory. After providing built ROS dependencies and active input publishers, a script may be invoked directly from this folder, for example:

```bash
cd ~/f1tenth-portfolio/f1tenth-planning/local-planning
python3 static_obs_spline_planner.py
```

This is a conditional invocation, not a verified full startup procedure. `lattice_planner3.py` requires the separate velocity estimator with its default topic selection. The co-located `frenet_utils.py` resolves the local helper import; external imports and ROS environment setup still need to be available.

See [selection notes](../../docs/local-planning-selection.md) for the earlier static comparison of alternatives. Those notes predate the later spline/helper copy.
