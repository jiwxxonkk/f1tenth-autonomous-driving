#!/usr/bin/env python3

import rospy
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from f110_msgs.msg import WpntArray, OTWpntArray
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from steering_lookup.lookup_steer_angle import LookupSteerAngle


class Controller:
    def __init__(self):
        rospy.init_node('control_node', anonymous=True)

        # Get parameters for the MAP controller
        self.param_q_map = rospy.get_param('/control_node/q_map')
        self.param_m_map = rospy.get_param('/control_node/m_map')
        self.param_t_clip_min = rospy.get_param('/control_node/t_clip_min')
        self.param_t_clip_max = rospy.get_param('/control_node/t_clip_max')

        # Avoidance path 전용 추종 파라미터
        self.avoidance_q_map = rospy.get_param('/control_node/avoidance_q_map')
        self.avoidance_m_map = rospy.get_param('/control_node/avoidance_m_map')
        if rospy.has_param('/control_node/avoidance_lookahead_min'):
            self.avoidance_t_clip_min = rospy.get_param('/control_node/avoidance_lookahead_min')
        else:
            self.avoidance_t_clip_min = rospy.get_param('/control_node/avoidance_t_clip_min')

        if rospy.has_param('/control_node/avoidance_lookahead_max'):
            self.avoidance_t_clip_max = rospy.get_param('/control_node/avoidance_lookahead_max')
        else:
            self.avoidance_t_clip_max = rospy.get_param('/control_node/avoidance_t_clip_max')

        self.speed_accel_limit = float(
            rospy.get_param('/control_node/speed_accel_limit')
        )
        self.speed_decel_limit = float(
            rospy.get_param('/control_node/speed_decel_limit')
        )
        self.lookahead_recovery_time = float(
            rospy.get_param('/control_node/lookahead_recovery_time')
        )

        # Load lookup table to calculate steering angle
        LUT_name = rospy.get_param('/control_node/LU_table')
        self.steer_lookup = LookupSteerAngle(LUT_name)

        # Set loop rate in hertz
        self.loop_rate = 40

        # Initialize variables
        self.position = None  # current position in map frame [x, y, theta]
        self.waypoints = None  # waypoints in map frame [x, y, speed]
        self.avoidance_waypoints = None  # avoidance waypoints in map frame [x, y, speed]
        self.last_avoidance_time = rospy.Time(0)
        self.avoidance_timeout = rospy.Duration(0.5)
        self.planner_mode = "GLOBAL"
        self.ros_time = rospy.Time()

        self.last_cmd_speed = None
        self.last_control_time = rospy.Time.now()
        self.prev_active_source = "GLOBAL"
        self.global_recovery_start = None

        # -------------------------- REAL CAR / joystick -------------------------
        # MAP_controller_real과 동일하게 실차는 수동 모드로 시작한다.
        # A(0): 자동 / B(1): 수동
        self.joy_msg = Joy()
        self.joy_mode = False
        self.manual_speed = 0.0
        self.manual_steer = 0.0

        # -------------------------- publisher ---------------------------------
        # Publisher to publish lookahead point and steering angle
        self.lookahead_pub = rospy.Publisher('lookahead_point', Marker, queue_size=10)
        # Publisher for steering and speed command
        # 실차 ackermann_to_vesc가 받는 명령 topic
        self.drive_pub = rospy.Publisher(
            '/ackermann_cmd',
            AckermannDriveStamped,
            queue_size=10
        )
        # -------------------------- Subscriber (REAL) ----------------------------
        rospy.Subscriber(
            '/amcl_pose',
            PoseWithCovarianceStamped,
            self.car_state_cb
        )
        rospy.Subscriber(
            '/global_path/optimal_trajectory_wpnt',
            WpntArray,
            self.waypoints_cb
        )
        rospy.Subscriber(
            '/planner/avoidance/otwpnts',
            OTWpntArray,
            self.avoidance_waypoints_cb
        )
        rospy.Subscriber(
            '/planner/avoidance/mode',
            String,
            self.planner_mode_cb
        )
        rospy.Subscriber('/joy', Joy, self.joy_cb)

        rospy.logwarn(
            "[REAL MAP Controller] "
            "pose=/amcl_pose, "
            "global=/global_path/optimal_trajectory_wpnt, "
            "avoidance=/planner/avoidance/otwpnts(OTWpntArray), "
            "drive=/ackermann_cmd"
        )

    def car_state_cb(self, data):
        """실차 AMCL PoseWithCovarianceStamped -> [x, y, yaw]."""
        pose = data.pose.pose

        x = pose.position.x
        y = pose.position.y
        theta = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])[2]

        self.position = [x, y, theta]

    def joy_cb(self, msg):
        """MAP_controller_real과 동일한 A/B 수동-자동 전환."""
        self.joy_msg = msg

        if len(msg.axes) > 1:
            self.manual_speed = 3.0 * float(msg.axes[1])
        if len(msg.axes) > 3:
            self.manual_steer = float(msg.axes[3]) / 2.0

        # A -> AUTO
        if len(msg.buttons) > 0 and msg.buttons[0] == 1:
            if not self.joy_mode:
                rospy.logwarn("[REAL Controller] AUTO mode ON")
            self.joy_mode = True

            # 수동에서 자동으로 바뀔 때 rate limiter 시작점을 현재 속도 근처로 둔다.
            self.last_cmd_speed = max(self.manual_speed, 0.0)
            self.last_control_time = rospy.Time.now()

        # B -> MANUAL
        if len(msg.buttons) > 1 and msg.buttons[1] == 1:
            if self.joy_mode:
                rospy.logwarn("[REAL Controller] MANUAL mode ON")
            self.joy_mode = False

    def waypoints_cb(self, data):
        """
        The callback function for the '/global_waypoints' subscriber.

        Args:
            data (WpntArray): The message containing the global waypoints.
        """
        self.waypoints = np.empty((len(data.wpnts), 3), dtype=np.float32)
        for i, waypoint in enumerate(data.wpnts):
            speed = waypoint.vx_mps
            self.waypoints[i] = [waypoint.x_m, waypoint.y_m, speed]

    def avoidance_waypoints_cb(self, data):
        """
        The callback function for the '/planner/avoidance/otwpnts' subscriber.

        Args:
            data (WpntArray): The message containing avoidance waypoints.
        """
        if len(data.wpnts) == 0:
            self.avoidance_waypoints = None
            return

        self.avoidance_waypoints = np.empty((len(data.wpnts), 3), dtype=np.float32)
        for i, waypoint in enumerate(data.wpnts):
            speed = waypoint.vx_mps
            self.avoidance_waypoints[i] = [waypoint.x_m, waypoint.y_m, speed]

        self.last_avoidance_time = rospy.Time.now()

    def planner_mode_cb(self, data):
        self.planner_mode = str(data.data)

    def limit_speed_rate(self, desired_speed):
        now = rospy.Time.now()
        dt = (now - self.last_control_time).to_sec()
        self.last_control_time = now

        if dt <= 0.0 or dt > 0.2:
            dt = 1.0 / float(self.loop_rate)

        desired_speed = max(float(desired_speed), 0.0)

        if self.last_cmd_speed is None:
            self.last_cmd_speed = desired_speed
            return desired_speed

        delta = desired_speed - self.last_cmd_speed

        if delta >= 0.0:
            delta = min(delta, self.speed_accel_limit * dt)
        else:
            delta = max(delta, -self.speed_decel_limit * dt)

        self.last_cmd_speed = max(self.last_cmd_speed + delta, 0.0)
        return self.last_cmd_speed

    def control_loop(self):
        """
        실차 control loop.

        B/manual:
            joystick -> /ackermann_cmd

        A/auto:
            최신 시뮬에서 검증한 GLOBAL / AVOIDANCE 추종 로직 사용
        """
        rate = rospy.Rate(self.loop_rate)

        while not rospy.is_shutdown():
            # ======================================================
            # MANUAL
            # ======================================================
            if not self.joy_mode:
                ack_msg = AckermannDriveStamped()
                ack_msg.header.stamp = rospy.Time.now()
                ack_msg.header.frame_id = 'base_link'
                ack_msg.drive.speed = self.manual_speed
                ack_msg.drive.steering_angle = self.manual_steer
                self.drive_pub.publish(ack_msg)

                rate.sleep()
                continue

            # ======================================================
            # AUTO
            # ======================================================
            avoidance_is_fresh = (
                self.avoidance_waypoints is not None
                and (rospy.Time.now() - self.last_avoidance_time)
                    < self.avoidance_timeout
                and self.avoidance_waypoints.shape[0] > 2
            )

            if avoidance_is_fresh:
                active_waypoints = self.avoidance_waypoints
                wrap_waypoints = False
                active_source = "AVOIDANCE"
                active_mode = self.planner_mode
            else:
                active_waypoints = self.waypoints
                wrap_waypoints = True
                active_source = "GLOBAL"
                active_mode = "GLOBAL"

            # avoidance 종료 -> GLOBAL 전환 시 lookahead 점진 복구
            if (
                self.prev_active_source == "AVOIDANCE"
                and active_source == "GLOBAL"
            ):
                self.global_recovery_start = rospy.Time.now()

            self.prev_active_source = active_source

            if self.position is None or active_waypoints is None:
                rate.sleep()
                continue

            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = rospy.Time.now()
            ack_msg.header.frame_id = 'base_link'

            if active_waypoints.shape[0] > 2:
                idx_nearest_waypoint = self.nearest_waypoint(
                    self.position[:2],
                    active_waypoints[:, :2]
                )

                target_speed = float(
                    active_waypoints[idx_nearest_waypoint, 2]
                )

                # 동적 추월은 planner가 보낸 기존 속도를 그대로 사용한다.
                # 그 외 회피는 기존과 동일하게 60% 속도를 사용한다.
                if (
                    active_source == "AVOIDANCE"
                    and active_mode != "DYNAMIC_OVERTAKE"
                ):
                    target_speed *= 0.5 # 정적 회피 속도 퍼센티지 (정적 0.35, 동적 0.65)

                # planner BLOCKED_STOP은 실차에서도 즉시 정지
                if (
                    active_source == "AVOIDANCE"
                    and active_mode == "BLOCKED_STOP"
                ):
                    ack_msg.drive.speed = 0.0
                    ack_msg.drive.steering_angle = 0.0
                    self.last_cmd_speed = 0.0
                    self.drive_pub.publish(ack_msg)

                    rospy.logwarn_throttle(
                        0.5,
                        "[REAL MAP Controller] BLOCKED_STOP -> speed=0"
                    )

                    rate.sleep()
                    continue

                # AVOIDANCE lookahead
                if active_source == "AVOIDANCE":
                    lookahead_distance = (
                        self.avoidance_q_map
                        + target_speed * self.avoidance_m_map
                    )
                    lookahead_distance = np.clip(
                        lookahead_distance,
                        self.avoidance_t_clip_min,
                        self.avoidance_t_clip_max
                    )

                # GLOBAL lookahead
                else:
                    global_lookahead = (
                        self.param_q_map
                        + target_speed * self.param_m_map
                    )
                    global_lookahead = np.clip(
                        global_lookahead,
                        self.param_t_clip_min,
                        self.param_t_clip_max
                    )

                    lookahead_distance = global_lookahead

                    if self.global_recovery_start is not None:
                        elapsed = (
                            rospy.Time.now()
                            - self.global_recovery_start
                        ).to_sec()

                        if elapsed < self.lookahead_recovery_time:
                            alpha = np.clip(
                                elapsed
                                / max(
                                    self.lookahead_recovery_time,
                                    1e-3
                                ),
                                0.0,
                                1.0
                            )

                            lookahead_distance = (
                                (1.0 - alpha)
                                * self.avoidance_t_clip_max
                                + alpha * global_lookahead
                            )
                        else:
                            self.global_recovery_start = None

                lookahead_point = self.waypoint_at_distance_infront_car(
                    lookahead_distance,
                    active_waypoints[:, :2],
                    idx_nearest_waypoint,
                    wrap_waypoints
                )

                if (
                    lookahead_point is not None
                    and np.size(lookahead_point) >= 2
                ):
                    position_la_vector = np.array([
                        lookahead_point[0] - self.position[0],
                        lookahead_point[1] - self.position[1]
                    ], dtype=float)

                    la_norm = np.linalg.norm(position_la_vector)

                    if la_norm < 1e-6:
                        ack_msg.drive.speed = 0.0
                        ack_msg.drive.steering_angle = 0.0

                        rospy.logwarn_throttle(
                            0.5,
                            "[REAL MAP Controller] "
                            "lookahead vector too small -> STOP"
                        )
                    else:
                        yaw = self.position[2]

                        sin_eta = np.dot(
                            [-np.sin(yaw), np.cos(yaw)],
                            position_la_vector
                        ) / la_norm

                        sin_eta = np.clip(
                            sin_eta,
                            -1.0,
                            1.0
                        )
                        eta = np.arcsin(sin_eta)

                        # 현재 시뮬 controller의 가감속 limiter 유지
                        command_speed = self.limit_speed_rate(
                            target_speed
                        )

                        lat_acc_cmd = (
                            2.0 * command_speed**2
                            / max(lookahead_distance, 1e-3)
                            * np.sin(eta)
                        )

                        steering_angle = (
                            self.steer_lookup.lookup_steer_angle(
                                lat_acc_cmd,
                                max(command_speed, 0.01)
                            )
                        )

                        ack_msg.drive.steering_angle = steering_angle
                        ack_msg.drive.speed = command_speed

                        self.visualize_lookahead(
                            lookahead_point
                        )
                        self.visualize_steering(
                            steering_angle
                        )

                        rospy.loginfo_throttle(
                            0.5,
                            f"[REAL MAP Controller] "
                            f"source={active_source} "
                            f"mode={active_mode} "
                            f"target={target_speed:.2f} "
                            f"cmd={command_speed:.2f} "
                            f"lookahead={lookahead_distance:.2f}"
                        )

            else:
                ack_msg.drive.speed = 0.0
                ack_msg.drive.steering_angle = 0.0

                rospy.logerr_throttle(
                    0.5,
                    "[REAL MAP Controller] no waypoints -> STOP"
                )

            self.drive_pub.publish(ack_msg)
            rate.sleep()

    @staticmethod
    def distance(point1, point2):
        """
        Calculates the Euclidean distance between two points.

        Args:
            point1 (tuple): A tuple containing the x and y coordinates of the first point.
            point2 (tuple): A tuple containing the x and y coordinates of the second point.

        Returns:
            float: The Euclidean distance between the two points.
        """
        return (((point2[0] - point1[0]) ** 2) + ((point2[1] - point1[1]) ** 2))**0.5

    @staticmethod
    def nearest_waypoint(position, waypoints):
        """
        Finds the index of the nearest waypoint to a given position.

        Args:
            position (tuple): A tuple containing the x and y coordinates of the position.
            waypoints (numpy.ndarray): An array of tuples representing the x and y coordinates of waypoints.

        Returns:
            int: The index of the nearest waypoint to the position.
        """
        position_array = np.array([position]*len(waypoints))
        distances_to_position = np.linalg.norm(abs(position_array - waypoints), axis=1)
        return np.argmin(distances_to_position)

    def waypoint_at_distance_infront_car(self, distance, waypoints, idx_waypoint_behind_car, wrap=True):
        """
        Finds the waypoint a given distance in front of a given waypoint.

        Args:
            distance (float): The distance to travel from the given waypoint.
            waypoints (numpy.ndarray): An array of tuples representing the x and y coordinates of waypoints.
            idx_waypoint_behind_car (int): The index of the waypoint behind the car.

        Returns:
            numpy.ndarray: A tuple containing the x and y coordinates of the waypoint a given distance in front of the given waypoint.
        """
        dist = 0
        i = idx_waypoint_behind_car
        count = 0

        while dist < distance and count < len(waypoints) - 1:
            if wrap:
                j = (i + 1) % len(waypoints)
            else:
                j = i + 1
                if j >= len(waypoints):
                    break
            dist += self.distance(waypoints[i], waypoints[j])
            i = j
            count += 1

        return np.array(waypoints[i])

    def visualize_steering(self, theta):
        """
        Publishes a visualization of the steering direction as an arrow.

        Args:
            theta (float): The steering angle in radians.
        """
        quaternions = quaternion_from_euler(0, 0, theta)

        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "base_link"
        lookahead_marker.header.stamp = self.ros_time.now()
        lookahead_marker.type = Marker.ARROW
        lookahead_marker.id = 2
        lookahead_marker.scale.x = 0.6
        lookahead_marker.scale.y = 0.05
        lookahead_marker.scale.z = 0
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.lifetime = rospy.Duration()
        lookahead_marker.pose.position.x = 0
        lookahead_marker.pose.position.y = 0
        lookahead_marker.pose.position.z = 0
        lookahead_marker.pose.orientation.x = quaternions[0]
        lookahead_marker.pose.orientation.y = quaternions[1]
        lookahead_marker.pose.orientation.z = quaternions[2]
        lookahead_marker.pose.orientation.w = quaternions[3]
        self.lookahead_pub.publish(lookahead_marker)

    def visualize_lookahead(self, lookahead_point):
        """
        Publishes a marker indicating the lookahead point on the map.

        Args:
            lookahead_point (tuple): A tuple of two floats representing the x and y coordinates of the lookahead point.
        """
        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "map"
        lookahead_marker.header.stamp = self.ros_time.now()
        lookahead_marker.type = 2
        lookahead_marker.id = 1
        lookahead_marker.scale.x = 0.15
        lookahead_marker.scale.y = 0.15
        lookahead_marker.scale.z = 0.15
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.pose.position.x = lookahead_point[0]
        lookahead_marker.pose.position.y = lookahead_point[1]
        lookahead_marker.pose.position.z = 0
        lookahead_marker.pose.orientation.x = 0
        lookahead_marker.pose.orientation.y = 0
        lookahead_marker.pose.orientation.z = 0
        lookahead_marker.pose.orientation.w = 1
        self.lookahead_pub.publish(lookahead_marker)


if __name__ == "__main__":
    controller = Controller()
    controller.control_loop()
