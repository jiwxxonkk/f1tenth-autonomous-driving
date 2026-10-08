# Mapping

## Purpose and technologies

This directory preserves the Cartographer configuration for creating a 2D occupancy map from LiDAR scans. It contains ROS 1 launch files, Lua settings, and the mapping-specific TF/EKF configuration. Saved maps later support localization and offline path generation.

## Contents and configuration

- `f1carto_ws/src/slam/launch/backpack_2d.launch`: starts Cartographer and its occupancy-grid node.
- `f1carto_ws/src/slam/lua/backpack_2d_test.lua`: 2D trajectory settings with 0.03 m submap resolution.
- `loc_ws/amcl_ws/src/navigation/amcl/examples/tf_carto.launch`: includes mapping TF and EKF launch files.
- `loc_ws/localization_ws/src/robot_localization/launch/`: `tf_carto.launch` and `forzaekf.launch`.

The Lua configuration uses `map` as the map frame and `base_link` as both the tracking and published frame. `provide_odom_frame`, `use_odometry`, and `use_imu_data` are disabled. Although the launch remaps odometry to `/state_estimation/odom`, the current Lua settings do not enable odometry input.

## ROS connections

| Connection | Role |
|---|---|
| `/scan` (`sensor_msgs/LaserScan`) | LiDAR input to Cartographer |
| Sensor TF | Resolves LiDAR data into the tracking frame |
| `/map` (`nav_msgs/OccupancyGrid`) | Occupancy grid produced by the mapping stack |
| `/odom`, `/sensors/imu/raw` | Inputs configured for the separately launched EKF |
| `/state_estimation/odom` | EKF output |

Mapping TF includes a static `odom -> base_footprint -> base_link` chain, while the EKF also has TF publication enabled. Frame ownership needs review before reproducing the combined configuration. The supplied mapping aliases do not launch the `vesc_to_odom_node` present in the driving startup, so an EKF odometry source is not established by those aliases alone.

## Original execution

The original workflow first starts VESC, Ackermann conversion, LiDAR, joystick input, and `joy.py`; see [mapping commands](../docs/README.md#mapping-workflow). It then runs:

```bash
cd ~/loc_ws
source devel/setup.bash
roslaunch amcl_ws/src/navigation/amcl/examples/tf_carto.launch
```

In another terminal:

```bash
cd ~/f1carto_ws
source /opt/ros/noetic/setup.bash
source ~/cartographer_ws/install_isolated/setup.bash
source ~/f1carto_ws/devel/setup.bash --extend
roslaunch slam backpack_2d.launch
```

These commands describe the original Nano environment. Before running, update `/home/nvidia/...` launch include paths and workspace setup paths to match your environment. Cartographer's installation and its standard Lua includes are external to this archive. A map-export command was not supplied with the aliases, so export is not represented as a verified step here.
