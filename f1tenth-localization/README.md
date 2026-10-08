# Localization

## Purpose and role

This directory preserves the ROS 1 launch configuration for locating the vehicle on a saved map. AMCL provides map-based localization, robot_localization runs an EKF, and static TF publishers describe the LiDAR and IMU mounting transforms. It supplies vehicle pose information to planning and control.

## Files and architecture

- `loc_ws/amcl_ws/src/navigation/amcl/examples/f1tenth_localization.launch`: includes the TF, EKF, AMCL, and map-server launch files.
- `amcl3.launch` in the same directory: AMCL parameters.
- `loc_ws/localization_ws/src/robot_localization/launch/tf_broadcaster.launch`: `base_link` to `imu` and `laser` transforms.
- `forzaekf.launch` in the same directory: EKF parameters, with `odom` as the world frame and TF publication enabled.
- `loc_ws/src/launch/map_server.launch`: loads `/home/nvidia/maps/0824.yaml`.
- The archived map is in [assets/maps](../assets/maps/).

## ROS connections

| Input/output | Topic or transform | Role |
|---|---|---|
| EKF input | `/odom` | VESC odometry; configured to use planar velocity components |
| EKF input | `/sensors/imu/raw` | IMU; the supplied selection enables yaw |
| EKF output | `/state_estimation/odom` | Filtered `nav_msgs/Odometry` |
| Localization input | `/scan`, `/map`, TF | Laser scan, occupancy grid, and frame transforms |
| Localization output | `/amcl_pose` | Pose consumed by controller and selected planners |
| TF configuration | `odom`, `base_link`, `imu`, `laser` | Vehicle and sensor frame relationships |

`amcl3.launch` contains an `odom` remap to `/odom/filtered`, while the EKF output is `/state_estimation/odom`. This textual mismatch should be reviewed alongside AMCL's TF inputs; the remap alone does not establish that AMCL consumes an odometry topic.

## Original execution

In the original Nano workspace, with the required packages built and hardware inputs running:

```bash
cd ~/loc_ws
source devel/setup.bash
roslaunch /home/nvidia/loc_ws/amcl_ws/src/navigation/amcl/examples/f1tenth_localization.launch
```

Before running, replace `/home/nvidia/...` in the launch include paths and the command above with your own workspace paths. Update the map path in `map_server.launch` to the location of your `0824.yaml`. AMCL, robot_localization, and map_server implementations are not provided by these launch-only folders. See the [workflow reference](../docs/README.md).
