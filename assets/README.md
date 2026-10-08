# Assets

## Purpose and role

This directory preserves map and trajectory data used by localization and planning. It contains YAML/image map pairs and CSV/NumPy trajectory artifacts; it does not contain executable ROS nodes.

## Contents

| Path | Role |
|---|---|
| `maps/0824.yaml`, `maps/0824.png` | Nano localization map, with 0.03 m resolution |
| `trajectories/nano/0824/traj_race_cl.csv` | Default input selected by the archived `lane_visualize1.py` |
| `trajectories/nano/0824_1/traj_race_cl.csv` | Additional Nano trajectory; not selected by the supplied alias defaults |
| Other files under `trajectories/` | Previously preserved path, boundary, profile, and debug artifacts; their existence alone does not establish a particular execution history |

Offline generator maps and outputs remain under [Global Planning](../f1tenth-planning/global-planning/README.md). Nano CSVs were kept separately because their bytes differ from existing same-name trajectory files.

## Loading and ROS connections

`map_server.launch` currently expects `/home/nvidia/maps/0824.yaml`. The archived YAML correctly references its adjacent `0824.png`. Update the YAML path in `map_server.launch` to match your map directory before running. The map server supplies `/map` (`nav_msgs/OccupancyGrid`) to localization and selected planners.

`lane_visualize1.py` currently expects `/home/nvidia/maps/<map_name>/<traj_file>.csv`, defaulting to `0824/traj_race_cl.csv`. It reads semicolon-separated rows after one header line and publishes `/global_path/optimal_trajectory_wpnt` (`f110_msgs/WpntArray`). Before running, update the base path in `lane_visualize1.py` to the chosen Nano trajectory directory.

See [Global Planning execution](../f1tenth-planning/global-planning/README.md) for the publisher command. Preserve the distinction between generated outputs and the separately archived Nano inputs when selecting files.
