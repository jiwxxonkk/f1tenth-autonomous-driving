# Planning

## Purpose and architecture

Planning connects the saved map to the controller's reference path. The offline stage generates a global trajectory on a laptop. A ROS publisher on the vehicle reads a trajectory CSV and publishes waypoints. A selected local planner produces an avoidance path when needed.

```text
Map -> offline trajectory generation -> CSV -> global waypoint publisher
                                                  |
LiDAR / obstacles / vehicle pose ----------------> local planner
                                                  |
                                      avoidance waypoints -> MAP controller
```

## Directory roles

| Directory | Technologies and role |
|---|---|
| [global-planning](global-planning/README.md) | Python, numerical optimization, image processing, CSV generation, and ROS waypoint publication |
| [local-planning](local-planning/README.md) | ROS 1 Python, Frenet coordinates, spline/lattice/gap-based avoidance, and obstacle velocity estimation |

## Execution and ROS connections

See each directory for commands and prerequisites. In the recorded Nano workflow, alias 4 runs `lane_visualize1.py` and alias 7 runs `static_obs_spline_planner.py`, both originally inside `vesc_driver`.

The global publisher supplies `/global_path/optimal_trajectory_wpnt` (`f110_msgs/WpntArray`). Avoidance planners publish `/planner/avoidance/otwpnts` (`f110_msgs/OTWpntArray`) for the MAP controller. Obstacle inputs use `f110_msgs/ObstacleArray`.

The archived local planners are alternatives, not one combined launch configuration. Moving scripts into this functional directory has not created new ROS packages or changed the original `rosrun` commands.
