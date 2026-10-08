# Common Interfaces

## Purpose and technologies

This directory contains shared ROS 1 message definitions used to connect perception, planning, and control. `f110_msgs` is a catkin message package. It defines data contracts and does not run a publisher or subscriber by itself.

## Messages

| Definition | Purpose |
|---|---|
| `Wpnt.msg` | Map/Frenet coordinates, track bounds, heading, curvature, speed, and acceleration |
| `WpntArray.msg` | Header and global waypoint array |
| `OTWpntArray.msg` | Avoidance waypoint array with switching metadata |
| `Obstacle.msg` | Obstacle extent, center, velocity, uncertainty, and status fields |
| `ObstacleArray.msg` | Header and obstacle array |
| `FrenetState.msg` | Longitudinal/lateral position and velocity |

The six message definition files matched across the inspected Nano VESC, control, and perception workspaces. One source package is preserved here. `vesc_msgs` is not included. Coordinate-conversion utilities remain beside their Python consumers rather than inside this message package.

## Build and usage

Place or link `f110_msgs` into the `src` directory of the intended ROS 1 catkin workspace, alongside the relevant packages. With its declared dependencies available, build that workspace and source the generated setup file, for example from the workspace root:

```bash
catkin_make
source devel/setup.bash
rosmsg show f110_msgs/WpntArray
```

This is a build outline, not confirmation that a workspace or dependency environment has been prepared. Do not run `catkin_make` at the portfolio root expecting the archive to be a complete workspace.

## ROS connections

- `/global_path/optimal_trajectory_wpnt`: `WpntArray`, from the CSV publisher to perception/planning/control.
- `/planner/avoidance/otwpnts`: `OTWpntArray`, from the selected local planner to the controller.
- `/perception/detection/raw_obstacles` and `/perception/detection/tracked_obstacles`: `ObstacleArray` between detection, tracking, and planning.

