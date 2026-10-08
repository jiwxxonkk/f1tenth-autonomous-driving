# Global Planning

## Purpose and technologies

This directory contains offline reference-path generation and the ROS node that publishes a saved trajectory on the vehicle. The generator uses Python, NumPy, OpenCV, Matplotlib, YAML configuration, and numerical trajectory-planning helpers. The selected optimization mode in `main_globaltraj.py` is `mincurv_iqp`.

## Contents and data flow

- `f1tenth-racing-stack-ICRA22/maps/`: source map images and metadata.
- `config/params.yaml` inside that project: map and planning parameters.
- `trajectory_generator/lane_generator.py`: map processing and track/boundary extraction.
- `trajectory_generator/main_globaltraj.py`: trajectory optimization and export.
- `trajectory_generator/params/racecar.ini` and `inputs/veh_dyn_info/`: vehicle parameters and dynamic limits.
- `helper_funcs_glob/` and `opt_mintime_traj/`: internal generator modules.
- `trajectory_generator/outputs/<map>/`: available track and trajectory artifacts; not every directory contains every output type.
- `ros-publisher/lane_visualize1.py`: reads CSV and publishes ROS waypoints and a path marker.

## Offline execution

Once dependencies from the archived requirements and imports are available, and the map/configuration paths are updated for your environment, the intended order is:

```bash
cd ~/f1tenth-portfolio/f1tenth-planning/global-planning/f1tenth-racing-stack-ICRA22/trajectory_generator
python3 lane_generator.py
python3 main_globaltraj.py
```

Before running, update the `/home/kimseokjin/...` paths in both Python scripts to your own project location. Set the map YAML/image paths in `lane_generator.py` and `map_name` in `config/params.yaml` to the same available map. The archived values are `0816` and `0824_final`, respectively. Update each selected map YAML image reference to the actual image filename and extension. Generation writes output files.

## Original ROS execution

In the original Nano workspace:

```bash
cd ~/vesc_ws
source devel/setup.bash
rosrun vesc_driver lane_visualize1.py
```

The script defaults to `map_name=0824` and `traj_file=traj_race_cl`, reading `/home/nvidia/maps/0824/traj_race_cl.csv`. Update the base path in `lane_visualize1.py` to your trajectory directory before running. It expects semicolon-separated data after one header row, with columns `s, x, y, heading, curvature, speed, acceleration`.

| Output topic | Message | Consumers |
|---|---|---|
| `/global_path/optimal_trajectory_wpnt` | `f110_msgs/WpntArray` | Detector, local planners, controller |
| `/global_path/optimal_trajectory_marker` | `visualization_msgs/Marker` | Visualization |

The source is now stored by function; it has not been repackaged as a new ROS package. The Nano CSV is preserved separately in [assets/trajectories/nano](../../assets/trajectories/nano/). Its bytes differ from the existing `outputs/0824/traj_race_cl.csv`; these files must not be treated as interchangeable.
