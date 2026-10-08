# Project Documentation

## Purpose and role

This directory stores operational reference material, backup records, and source notices for the ROS 1 F1TENTH archive. It explains how the original workspaces were connected; it is not an executable package.

## Document index

| Path | Contents |
|---|---|
| `copy-manifest.csv` | Historical source paths and copy hashes |
| `copy-status.md` | Earlier copy scope and unresolved items |
| `local-changes.md` | Previously recorded changes to archived copies |
| `local-planning-selection.md` | Earlier static comparison of planning alternatives |
| `source-notices/` | Preserved MAP-Controller, VESC, and trajectory-generator license texts |

Historical records predate some later copies. For example, the old statement that `assets/trajectories` is empty is no longer current. They are retained as records and are not a complete current inventory. Dedicated `driving.md`, `mapping.md`, and `code-origins.md` have not yet been created; operational notes are consolidated below.

## Execution prerequisites

The commands below transcribe the supplied Nano workflow. Each row is intended for a separate terminal and sources the relevant original workspace. The backup directory is organized by function and does not recreate those home-directory workspaces. Prepare the required packages, device access, message builds, and topic connections. Update the absolute paths in commands, launch files, and scripts to match your environment before running. No ROS execution was performed when writing these documents.

## Driving workflow

| Original alias | Command |
|---|---|
| `1` | `cd ~/vesc_ws && sudo chmod 777 /dev/ttyACM0 && source devel/setup.bash && roslaunch vesc_driver f1_start.launch` |
| `2` | `cd ~/vesc_ws && source devel/setup.bash && roslaunch vesc_ackermann ackermann_to_vesc_node.launch` |
| `3` | `cd ~/loc_ws && source devel/setup.bash && roslaunch /home/nvidia/loc_ws/amcl_ws/src/navigation/amcl/examples/f1tenth_localization.launch` |
| `4` | `cd ~/vesc_ws && source devel/setup.bash && rosrun vesc_driver lane_visualize1.py` |
| `5` | `cd ~/MAP_ws && source devel/setup.bash && roslaunch map_controller sim_MAP.launch` |
| `6` | `cd ~/perception_ws && source devel/setup.bash && roslaunch perception opponent_track.launch` |
| `7` | `cd ~/vesc_ws && source devel/setup.bash && rosrun vesc_driver static_obs_spline_planner.py` |

This records the supplied ordering, not a validated startup guarantee. The waypoint CSV defaults to `/home/nvidia/maps/0824/traj_race_cl.csv`. The archive's Perception launch has a previously documented YAML-path substitution.

## Mapping workflow

| Original alias | Command |
|---|---|
| `m1` | `cd ~/vesc_ws && sudo chmod 777 /dev/ttyACM0 && source devel/setup.bash && roslaunch vesc_driver vesc_driver_node.launch` |
| `m2` | `cd ~/vesc_ws && source devel/setup.bash && roslaunch vesc_ackermann ackermann_to_vesc_node.launch` |
| `m3` | `source /opt/ros/noetic/setup.bash && rosrun urg_node urg_node _ip_address:=192.168.0.10` |
| `m4` | `source /opt/ros/noetic/setup.bash && rosrun joy joy_node` |
| `m5` | `cd ~/vesc_ws && source devel/setup.bash && rosrun vesc_driver joy.py` |
| `m6` | `cd ~/loc_ws && source devel/setup.bash && roslaunch amcl_ws/src/navigation/amcl/examples/tf_carto.launch` |
| `carto` | `source /opt/ros/noetic/setup.bash && source ~/cartographer_ws/install_isolated/setup.bash && source ~/f1carto_ws/devel/setup.bash --extend && roslaunch slam backpack_2d.launch` |

The device permission commands are preserved historical commands, not a new device-access configuration. The mapping settings disable Cartographer odometry and IMU input. The separately launched EKF still names those inputs; the supplied mapping aliases do not start the driving odometry node. Refer to the [Mapping README](../f1tenth-mapping/README.md) for TF and configuration details.

## ROS connection reference

The [root README](../README.md) presents the system architecture and primary topics. Each module README lists its own inputs, outputs, execution command, and known integration gaps. Topic names describe the root-namespace configuration in the archived code; changed namespaces or remaps require corresponding documentation updates.
