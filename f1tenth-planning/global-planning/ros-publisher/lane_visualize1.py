#!/usr/bin/env python3
import rospy
import numpy as np
import os
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from f110_msgs.msg import WpntArray, Wpnt

"""
Constant Definition
"""
WIDTH = 0.2032  # (m)
WHEEL_LENGTH = 0.0381  # (m)
MAX_STEER = 0.36  # (rad)


class LaneVisualize:
    """
    Class for lane visualization
    """

    def __init__(self):
        # Initialize the node
        rospy.init_node("lane_visualize_node")

        # ROS Params
        self.real_test = rospy.get_param("~real_test", False)
        self.map_name = rospy.get_param("~map_name", "0824")
        self.traj_file = rospy.get_param("~traj_file", "traj_race_cl")

        # Optimal Trajectory
        traj_csv_loc = os.path.join(
            "/home/nvidia/maps",
            self.map_name,
            self.traj_file + ".csv",
        )
        traj_data = np.loadtxt(traj_csv_loc, delimiter=";", skiprows=1)

        self.num_traj_pts = len(traj_data)
        self.traj_s = traj_data[:, 0]  # s_m
        self.traj_x = traj_data[:, 1]  # x_m
        self.traj_y = traj_data[:, 2]  # y_m
        self.traj_psi = traj_data[:, 3]  # psi_rad
        self.traj_kappa = traj_data[:, 4]  # kappa_radpm
        self.traj_vx = traj_data[:, 5]  # vx_mps
        self.traj_ax = traj_data[:, 6]  # ax_mps2

        # Publishers
        self.waypoint_pub_ = rospy.Publisher(
            "/global_path/optimal_trajectory_wpnt", WpntArray, queue_size=10
        )
        self.marker_pub_ = rospy.Publisher(
            "/global_path/optimal_trajectory_marker", Marker, queue_size=10
        )

        # Timer for publishing
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, event):
        self.publish_optimal_trajectory()
        self.publish_marker()

    def publish_optimal_trajectory(self):
        # Publish trajectory waypoints in WpntArray format
        wpnt_array = WpntArray()
        for i in range(self.num_traj_pts):
            wpnt = Wpnt()
            wpnt.id = i
            wpnt.s_m = self.traj_s[i]
            wpnt.x_m = self.traj_x[i]
            wpnt.y_m = self.traj_y[i]
            wpnt.psi_rad = self.traj_psi[i]
            wpnt.kappa_radpm = self.traj_kappa[i]
            wpnt.vx_mps = self.traj_vx[i]
            wpnt.ax_mps2 = self.traj_ax[i]

            wpnt_array.wpnts.append(wpnt)

        # Publish the waypoint array
        self.waypoint_pub_.publish(wpnt_array)

    def publish_marker(self):
        # Publish trajectory waypoints as Marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 0
        marker.ns = "global_planner"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = []
        marker.colors = []

        v_min = np.nanmin(self.traj_vx)
        v_max = np.nanmax(self.traj_vx)
        den = v_max - v_min

        for i in range(self.num_traj_pts):
            # Create a point for each trajectory point
            this_point = Point()
            this_point.x = self.traj_x[i]
            this_point.y = self.traj_y[i]
            this_point.z = 0.0
            marker.points.append(this_point)

            # Assign color based on speed
            this_color = ColorRGBA()
            
            if np.isnan(self.traj_vx[i]) or den <= 1e-6:
                speed_ratio = 0.5
            else:
                speed_ratio = (self.traj_vx[i] - v_min) / den
                
            # speed_ratio = (self.traj_vx[i] - np.min(self.traj_vx)) / 
            
            this_color.a = 1.0
            this_color.r = 1 - speed_ratio  # Red for low speed
            this_color.g = speed_ratio      # Green for high speed
            this_color.b = 0.0
            marker.colors.append(this_color)

        # Marker properties
        marker.scale.x = 0.04  # Line width
        marker.pose.orientation.w = 1.0

        # Publish the marker
        self.marker_pub_.publish(marker)


if __name__ == "__main__":
    print("Lane Visualize Initialized")
    LaneVisualize()
    rospy.spin()

