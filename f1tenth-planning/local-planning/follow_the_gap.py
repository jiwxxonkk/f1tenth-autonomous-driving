#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import numpy as np
import rospy
from f110_msgs.msg import ObstacleArray, OTWpntArray, Wpnt, WpntArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from frenet_utils import FrenetConverter


class FollowTheGap:
    def __init__(self):
        rospy.init_node("follow_the_gap")

        self.scan_topic = rospy.get_param("~scan_topic", "/scan")  # 바꾸면 FTG가 보는 LiDAR 토픽이 바뀜
        self.pose_topic = rospy.get_param("~pose_topic", "/amcl_pose")  # 3번 localization과 5번 controller가 함께 쓰는 실차 pose
        self.use_perception_obstacles = rospy.get_param("~use_perception_obstacles", False)  # 켜면 6번 장애물 검출 결과도 FTG 발동 조건으로 사용함
        self.obstacle_topic = rospy.get_param("~obstacle_topic", "/perception/detection/raw_obstacles")  # 6번 연동 모드에서만 구독함
        self.output_topic = rospy.get_param("~output_topic", "/planner/avoidance/otwpnts")  # 바꾸면 MAP controller에 줄 회피 경로 토픽이 바뀜
        
        self.global_waypoints_topic = rospy.get_param("~global_waypoints_topic", "/global_path/optimal_trajectory_wpnt")  # 4번 lane_visualize1.py의 실제 출력
        self.map_topic = rospy.get_param("~map_topic", "/map")  # 바꾸면 벽 필터에 쓰는 map 토픽이 바뀜

        # ---- rviz 시각화용 파라미터 ----
        self.marker_topic = rospy.get_param("~marker_topic", "/planner/avoidance/ftg_marker")  # 바꾸면 RViz ftg 글자 토픽이 바뀜
        self.path_marker_topic = rospy.get_param("~path_marker_topic", "/planner/avoidance/ftg_path_marker")
        self.obstacle_marker_topic = rospy.get_param("~obstacle_marker_topic", "/planner/avoidance/ftg_obstacle_markers")
        
        # 5번 MAP_controller.py의 subscriber와 반드시 같은 타입이어야 연결된다.
        self.path_pub = rospy.Publisher(self.output_topic, OTWpntArray, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=1)
        self.path_marker_pub = rospy.Publisher(self.path_marker_topic, MarkerArray, queue_size=1)
        self.obstacle_marker_pub = rospy.Publisher(self.obstacle_marker_topic, MarkerArray, queue_size=1)

        self.vehicle_x = None
        self.vehicle_y = None
        self.cur_yaw = None
        self.cur_s = None
        self.cur_d = None
        self.gb_max_s = None
        self.converter = None
        self.map_data = None
        self.map_resolution = None
        self.map_width = 0
        self.map_height = 0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_yaw = 0.0
        self.obstacle_count = 0
        self.last_obstacle_time = rospy.Time(0)
        self.last_clear_publish_time = rospy.Time(0)
        self.ftg_active = False

        # RViz 시각화를 위한 현재 선택 Gap 정보
        self.selected_gap_start_angle = None
        self.selected_gap_end_angle = None
        self.selected_target_angle = None

        self.selected_gap_start_distance = None
        self.selected_gap_end_distance = None
        self.selected_target_distance = None
        self.locked_target_angle = None

        # 현재 LiDAR에서 실제 장애물로 판정된 cluster
        self.current_lidar_obstacle_clusters = []

        self.last_target_update_time = rospy.Time(0)

        self.fov_deg = rospy.get_param("~fov_deg", 180.0)  # 올리면 더 넓게 보고, 내리면 정면만 봄
        self.max_range = rospy.get_param("~max_range", 6.0)  # 올리면 먼 공간까지 보고, 내리면 가까운 공간만 봄
        self.safe_distance = rospy.get_param("~safe_distance", 1.5)  # 올리면 장애물 주변을 더 크게 피하고, 내리면 더 가까이 지나감
        self.min_gap_len = int(rospy.get_param("~min_gap_len", 8))  # 올리면 좁은 틈을 무시하고, 내리면 작은 틈도 선택함
        self.max_steer = rospy.get_param("~max_steer", 0.45)  # 올리면 더 크게 꺾고, 내리면 회피 각도가 제한됨
        self.angle_penalty = rospy.get_param("~angle_penalty", 1.4)  # 올리면 정면을 더 선호하고, 내리면 옆 gap도 잘 선택함
        self.target_angle_hold_time = rospy.get_param("~target_angle_hold_time", 1.0)  # 올리면 같은 회피 방향을 오래 물고, 내리면 새 gap으로 빨리 바뀜
        self.target_angle_update_threshold = rospy.get_param("~target_angle_update_threshold", 0.35)  # 올리면 좌우 흔들림을 덜 받고, 내리면 방향 전환이 민감해짐

        self.path_length = rospy.get_param("~path_length", 2.0)  # 올리면 MAP lookahead가 경로 끝까지 덜 밀리고, 내리면 짧게 회피함
        self.path_ds = rospy.get_param("~path_ds", 0.1)  # 올리면 waypoint 수가 줄고, 내리면 더 촘촘하게 따라감
        self.avoidance_speed = rospy.get_param("~avoidance_speed", 1.0)  # 올리면 회피 waypoint 목표속도가 올라가지만, simulator max_speed가 최종 상한임
        self.obstacle_timeout = rospy.get_param("~obstacle_timeout", 0.2)  # 올리면 감지 끊김에 덜 민감하고, 내리면 빨리 FTG OFF됨
        self.avoidance_hold_time = rospy.get_param("~avoidance_hold_time", 0.5)  # 올리면 FTG 복귀가 늦고, 내리면 빨리 글로벌로 복귀함
        self.trigger_s_min = rospy.get_param("~trigger_s_min", 0.0)  # 올리면 아주 가까운 장애물을 덜 보고, 내리면 더 가까운 것도 봄
        self.trigger_s_max = rospy.get_param("~trigger_s_max", 3.0)  # 올리면 더 먼 장애물도 FTG를 켜고, 내리면 가까운 것만 켬
        self.trigger_d_margin = rospy.get_param("~trigger_d_margin", 1.0)  # 올리면 옆 장애물에도 켜지고, 내리면 경로 중앙 장애물만 켜짐
        self.lidar_trigger_distance = rospy.get_param("~lidar_trigger_distance", 3.0)  # perception 출력이 없어도 정면 장애물로 FTG를 켬
        self.lidar_trigger_half_angle_deg = rospy.get_param("~lidar_trigger_half_angle_deg", 18.0)  # 정면 회피 발동 폭
        self.lidar_cluster_min_points = int(rospy.get_param("~lidar_cluster_min_points", 3))  # 단일 반사 노이즈를 장애물로 보지 않기 위한 최소 연속점 수
        self.lidar_cluster_max_gap = rospy.get_param("~lidar_cluster_max_gap", 0.12)  # 인접 LiDAR 점 사이가 이 거리보다 멀면 서로 다른 물체로 분리함
        self.obstacle_max_size = rospy.get_param("~obstacle_max_size", 0.8)  # 규정상 장애물 한 변의 최대 크기(m)
        self.obstacle_size_tolerance = rospy.get_param("~obstacle_size_tolerance", 0.10)  # LiDAR/map 오차를 고려한 군집 크기 여유(m)
        self.wall_filter_enabled = rospy.get_param("~wall_filter_enabled", True)  # 켜면 map 벽은 무시하고, 끄면 벽 감지도 FTG를 켤 수 있음
        self.map_occupied_threshold = rospy.get_param("~map_occupied_threshold", 80)  # 올리면 벽 판정이 까다롭고, 내리면 더 쉽게 벽으로 거름
        self.wall_filter_radius = rospy.get_param("~wall_filter_radius", 0.10)  # 올리면 실제 장애물도 벽으로 오판할 수 있고, 내리면 벽 필터가 약해짐
        self.treat_unknown_as_wall = rospy.get_param("~treat_unknown_as_wall", False)  # 켜면 unknown도 벽처럼 무시하고, 끄면 unknown은 장애물 후보로 둠
        self.clear_publish_period = rospy.Duration(rospy.get_param("~clear_publish_period", 0.1))  # 올리면 빈 경로 publish가 줄고, 내리면 글로벌 복귀 신호가 자주 나감
        self.wall_avoidance_distance = rospy.get_param("~wall_avoidance_distance", 0.35) # 지도상의 벽에서 최소한으로 떨어지도록 벽 주변을 통행 불가 영역으로 확장
        rospy.loginfo(
            "[FTG] mode=%s scan_topic=%s pose_topic=%s output_topic=%s",
            "lidar+perception" if self.use_perception_obstacles else "lidar-only",
            self.scan_topic,
            self.pose_topic,
            self.output_topic,
        )

        rospy.Subscriber(self.pose_topic, PoseWithCovarianceStamped, self.pose_callback, queue_size=1)
        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        # map 기반 벽 필터는 perception obstacle(6번) 사용 여부와 무관하게 필요하다.
        # 따라서 lidar-only 모드에서도 항상 map을 구독한다.
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        if self.use_perception_obstacles:
            rospy.Subscriber(self.global_waypoints_topic, WpntArray, self.global_waypoints_callback, queue_size=1)
            rospy.Subscriber(self.obstacle_topic, ObstacleArray, self.obstacle_callback, queue_size=1)

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def pose_callback(self, msg):
        pose = msg.pose.pose
        self.vehicle_x = pose.position.x
        self.vehicle_y = pose.position.y
        _, _, self.cur_yaw = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])

        if self.converter is not None:
            frenet = self.converter.get_frenet(self.vehicle_x, self.vehicle_y)
            self.cur_s = self.to_scalar(frenet[0])
            self.cur_d = self.to_scalar(frenet[1])

    def global_waypoints_callback(self, msg):
        if self.converter is not None or len(msg.wpnts) == 0:
            return

        waypoints = np.array([[wpnt.x_m, wpnt.y_m, wpnt.s_m] for wpnt in msg.wpnts])
        self.converter = FrenetConverter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2])
        self.gb_max_s = float(self.converter.raceline_length)
        rospy.loginfo("[FTG] FrenetConverter initialized from %s.", self.global_waypoints_topic)

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        _, _, self.map_origin_yaw = euler_from_quaternion([
            msg.info.origin.orientation.x,
            msg.info.origin.orientation.y,
            msg.info.origin.orientation.z,
            msg.info.origin.orientation.w
        ])
        self.map_data = np.array(msg.data, dtype=np.int16).reshape(
            (self.map_height, self.map_width)
        )

    def to_scalar(self, value):
        return float(np.asarray(value).reshape(-1)[0])

    def has_map(self):
        return (
            self.map_data is not None and
            self.map_resolution is not None and
            self.map_resolution > 0.0 and
            self.map_width > 0 and
            self.map_height > 0
        )

    def world_to_map(self, x, y):
        dx = x - self.map_origin_x
        dy = y - self.map_origin_y
        c = math.cos(-self.map_origin_yaw)
        s = math.sin(-self.map_origin_yaw)
        mx = (c * dx - s * dy) / self.map_resolution
        my = (s * dx + c * dy) / self.map_resolution
        return int(math.floor(mx)), int(math.floor(my))

    def is_wall_cell(self, mx, my):
        if mx < 0 or my < 0 or mx >= self.map_width or my >= self.map_height:
            return True

        value = self.map_data[my, mx]
        if value < 0:
            return self.treat_unknown_as_wall
        return value >= self.map_occupied_threshold

    def is_near_map_wall(self, x, y):
        if not self.has_map():
            return False

        center_x, center_y = self.world_to_map(x, y)
        radius_cells = int(math.ceil(self.wall_filter_radius / self.map_resolution))

        for my in range(center_y - radius_cells, center_y + radius_cells + 1):
            for mx in range(center_x - radius_cells, center_x + radius_cells + 1):
                if (mx - center_x) ** 2 + (my - center_y) ** 2 > radius_cells ** 2:
                    continue
                if self.is_wall_cell(mx, my):
                    return True

        return False

    def is_map_wall_obstacle(self, obs):
        if not self.wall_filter_enabled or not self.has_map() or self.converter is None:
            return False

        obs_s = self.to_scalar(obs.s_center)
        d_values = [
            self.to_scalar(obs.d_center),
            self.to_scalar(obs.d_left),
            self.to_scalar(obs.d_right),
        ]

        for obs_d in d_values:
            xy = self.converter.get_cartesian(obs_s, obs_d)
            x = self.to_scalar(xy[0])
            y = self.to_scalar(xy[1])
            if self.is_near_map_wall(x, y):
                return True

        return False

    def is_lidar_point_near_map_wall(self, distance, angle):
        """Return True when a LiDAR return belongs to a static wall in /map.

        The scan frame is assumed to have the same planar origin and heading as
        the pose frame. This matches the current pose-based waypoint generation.
        """
        if (
            not self.wall_filter_enabled or
            not self.has_map() or
            self.vehicle_x is None or
            self.vehicle_y is None or
            self.cur_yaw is None
        ):
            return False

        point_heading = self.cur_yaw + angle
        world_x = self.vehicle_x + distance * math.cos(point_heading)
        world_y = self.vehicle_y + distance * math.sin(point_heading)
        return self.is_near_map_wall(world_x, world_y)

    def forward_s_distance(self, from_s, target_s):
        if self.gb_max_s is None or self.gb_max_s <= 0.0:
            return target_s - from_s
        return (target_s - from_s + self.gb_max_s) % self.gb_max_s

    def is_trigger_obstacle(self, obs):
        if self.cur_s is None or self.cur_d is None:
            return False

        if self.is_map_wall_obstacle(obs):
            return False

        obs_s = self.to_scalar(obs.s_center)
        obs_rel_s = self.forward_s_distance(self.cur_s, obs_s)
        if not (self.trigger_s_min < obs_rel_s < self.trigger_s_max):
            return False

        obs_d_low = min(self.to_scalar(obs.d_left), self.to_scalar(obs.d_right))
        obs_d_high = max(self.to_scalar(obs.d_left), self.to_scalar(obs.d_right))
        return (
            obs_d_low - self.trigger_d_margin
            <= self.cur_d
            <= obs_d_high + self.trigger_d_margin
        )

    def obstacle_callback(self, msg):
        self.obstacle_count = sum(1 for obs in msg.obstacles if self.is_trigger_obstacle(obs))
        if self.obstacle_count > 0:
            self.last_obstacle_time = rospy.Time.now()

    def has_active_obstacle(self):
        if not self.use_perception_obstacles:
            return False
        if self.obstacle_count <= 0:
            return False
        age = (rospy.Time.now() - self.last_obstacle_time).to_sec()
        return age <= self.obstacle_timeout + self.avoidance_hold_time

    def publish_ftg_marker(self, active):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "follow_the_gap"
        marker.id = 0

        if not active:
            marker.action = Marker.DELETE
            self.marker_pub.publish(marker)
            return

        if self.vehicle_x is None or self.vehicle_y is None:
            return

        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self.vehicle_x
        marker.pose.position.y = self.vehicle_y
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.60
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.text = "ftg"
        self.marker_pub.publish(marker)

    def set_ftg_active(self, active):
        if self.ftg_active != active or active:
            self.publish_ftg_marker(active)

        if not active:
            self.locked_target_angle = None

            self.selected_gap_start_angle = None
            self.selected_gap_end_angle = None
            self.selected_target_angle = None

            self.selected_gap_start_distance = None
            self.selected_gap_end_distance = None
            self.selected_target_distance = None

            self.clear_ftg_path_markers()

        self.ftg_active = active

    def publish_empty_avoidance(self):
        now = rospy.Time.now()
        if now - self.last_clear_publish_time < self.clear_publish_period:
            return
        self.path_pub.publish(OTWpntArray())
        self.last_clear_publish_time = now

    def restrict_fov(self, scan, ranges):
        half_fov = math.radians(self.fov_deg) * 0.5
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        mask = np.abs(angles) <= half_fov
        return ranges[mask], angles[mask]

    def apply_bubble(self, ranges, angles):
        valid = ranges > 0.0
        if not np.any(valid):
            return ranges

        closest_idx = int(np.argmin(np.where(valid, ranges, np.inf)))
        closest_dist = max(float(ranges[closest_idx]), 1e-3)
        angle_step = abs(angles[1] - angles[0]) if len(angles) > 1 else 1e-3
        bubble_angle = math.atan2(self.safe_distance, closest_dist)
        bubble_cells = int(math.ceil(bubble_angle / max(angle_step, 1e-3)))

        start = max(0, closest_idx - bubble_cells)
        end = min(len(ranges), closest_idx + bubble_cells + 1)
        ranges[start:end] = 0.0
        return ranges
    
    def apply_detected_obstacle_bubbles(self, ranges, angles):
        """
        has_front_lidar_obstacle()에서 실제 장애물로 판정된
        모든 LiDAR cluster를 FTG의 blocked 영역으로 만든다.

        장애물 자체뿐만 아니라 safe_distance만큼 좌우 영역도
        추가로 막아서 차량이 장애물 바로 옆으로 붙지 않게 한다.
        """

        if not self.current_lidar_obstacle_clusters:
            return ranges

        if len(angles) > 1:
            angle_step = abs(float(angles[1] - angles[0]))
        else:
            angle_step = 1e-3

        for cluster in self.current_lidar_obstacle_clusters:

            if not cluster:
                continue

            # cluster가 차지하는 LiDAR index
            indices = [item[0] for item in cluster]

            cluster_start = min(indices)
            cluster_end = max(indices)

            # 장애물에서 차량까지 가장 가까운 거리
            cluster_distances = [
                float(np.linalg.norm(item[1]))
                for item in cluster
            ]

            closest_dist = max(
                min(cluster_distances),
                1e-3
            )

            # safe_distance를 각도 크기로 변환
            bubble_angle = math.atan2(
                self.safe_distance,
                closest_dist
            )

            bubble_cells = int(
                math.ceil(
                    bubble_angle /
                    max(angle_step, 1e-3)
                )
            )

            # 장애물 양옆으로 bubble 확장
            start = max(
                0,
                cluster_start - bubble_cells
            )

            end = min(
                len(ranges),
                cluster_end + bubble_cells + 1
            )

            # FTG에서 갈 수 없는 영역
            ranges[start:end] = 0.0

        return ranges

    def find_max_gap(self, free):
        best_start = best_end = -1
        cur_start = None

        for i, ok in enumerate(free):
            if ok and cur_start is None:
                cur_start = i
            if (not ok or i == len(free) - 1) and cur_start is not None:
                cur_end = i if ok and i == len(free) - 1 else i - 1
                if best_start == -1 or (cur_end - cur_start) > (best_end - best_start):
                    best_start, best_end = cur_start, cur_end
                cur_start = None

        return best_start, best_end

    def choose_gap_angle(self, ranges, angles):

        processed_ranges = ranges.copy()

        # 1. map 벽 주변 금지
        processed_ranges = self.apply_map_wall_mask(
            processed_ranges,
            angles
        )

        # 2. 실제 검출 장애물 주변 금지
        processed_ranges = self.apply_detected_obstacle_bubbles(
            processed_ranges,
            angles
        )

        # 3. 남아 있는 가장 가까운 위험점 추가 bubble
        processed_ranges = self.apply_bubble(
            processed_ranges,
            angles
        )

        # 4. 안전한 공간에서만 Gap 탐색
        gap_start, gap_end = self.find_max_gap(
            processed_ranges > 0.0
        )

        if gap_start < 0 or (gap_end - gap_start + 1) < self.min_gap_len:

            self.selected_gap_start_angle = None
            self.selected_gap_end_angle = None
            self.selected_target_angle = None

            self.selected_gap_start_distance = None
            self.selected_gap_end_distance = None
            self.selected_target_distance = None

            return None

        gap_ranges = processed_ranges[
            gap_start:gap_end + 1
        ]

        gap_angles = angles[
            gap_start:gap_end + 1
        ]

        scores = (
            gap_ranges
            - self.angle_penalty * np.abs(gap_angles)
        )

        target_local_idx = int(
            np.argmax(scores)
        )

        target_idx = (
            gap_start + target_local_idx
        )

        target_angle = float(
            angles[target_idx]
        )

        # RViz용 Gap 정보
        self.selected_gap_start_angle = float(
            angles[gap_start]
        )

        self.selected_gap_end_angle = float(
            angles[gap_end]
        )

        self.selected_gap_start_distance = float(
            processed_ranges[gap_start]
        )

        self.selected_gap_end_distance = float(
            processed_ranges[gap_end]
        )

        self.selected_target_angle = target_angle

        self.selected_target_distance = float(
            processed_ranges[target_idx]
        )

        return target_angle
    
    def apply_map_wall_mask(self, ranges, angles):

        if (
            not self.wall_filter_enabled or
            not self.has_map() or
            self.vehicle_x is None or
            self.vehicle_y is None or
            self.cur_yaw is None
        ):
            return ranges

        if len(angles) > 1:
            angle_step = abs(
                float(angles[1] - angles[0])
            )
        else:
            angle_step = 1e-3

        wall_mask = np.zeros(
            len(ranges),
            dtype=bool
        )

        # =====================================================
        # 1. map 벽에 해당하는 LiDAR point 탐색
        # =====================================================

        for i in range(len(ranges)):

            distance = float(ranges[i])

            if distance <= 0.0:
                continue

            angle = float(angles[i])

            if self.is_lidar_point_near_map_wall(
                distance,
                angle
            ):
                wall_mask[i] = True

        wall_indices = np.flatnonzero(
            wall_mask
        )

        # =====================================================
        # 2. 각 벽 point 주변에 safety margin 적용
        # =====================================================

        for index in wall_indices:

            wall_distance = max(
                float(ranges[index]),
                1e-3
            )

            bubble_angle = math.atan2(
                self.wall_avoidance_distance,
                wall_distance
            )

            bubble_cells = int(
                math.ceil(
                    bubble_angle /
                    max(angle_step, 1e-3)
                )
            )

            start = max(
                0,
                index - bubble_cells
            )

            end = min(
                len(ranges),
                index + bubble_cells + 1
            )

            ranges[start:end] = 0.0

        return ranges

    def publish_obstacle_markers(self, obstacle_clusters):

        # ---------------------------------------------------------
        # 이전 frame의 장애물 box 제거
        # ---------------------------------------------------------

        clear_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = rospy.Time.now()
        clear_marker.action = Marker.DELETEALL

        clear_array.markers.append(clear_marker)

        self.obstacle_marker_pub.publish(clear_array)

        if not obstacle_clusters:
            return

        # ---------------------------------------------------------
        # 현재 frame 장애물 box 생성
        # ---------------------------------------------------------

        marker_array = MarkerArray()

        now = rospy.Time.now()

        cos_yaw = math.cos(self.cur_yaw)
        sin_yaw = math.sin(self.cur_yaw)

        for obstacle_id, cluster in enumerate(obstacle_clusters):

            world_points = []

            for _, local_point in cluster:

                local_x = float(local_point[0])
                local_y = float(local_point[1])

                # LiDAR local 좌표 → map/world 좌표
                world_x = (
                    self.vehicle_x +
                    cos_yaw * local_x -
                    sin_yaw * local_y
                )

                world_y = (
                    self.vehicle_y +
                    sin_yaw * local_x +
                    cos_yaw * local_y
                )

                world_points.append([
                    world_x,
                    world_y
                ])

            world_points = np.asarray(world_points)

            min_x = float(np.min(world_points[:, 0]))
            max_x = float(np.max(world_points[:, 0]))

            min_y = float(np.min(world_points[:, 1]))
            max_y = float(np.max(world_points[:, 1]))

            center_x = (min_x + max_x) * 0.5
            center_y = (min_y + max_y) * 0.5

            size_x = max(max_x - min_x, 0.15)
            size_y = max(max_y - min_y, 0.15)

            marker = Marker()

            marker.header.frame_id = "map"
            marker.header.stamp = now

            marker.ns = "ftg_obstacles"
            marker.id = obstacle_id

            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = 0.15

            marker.pose.orientation.w = 1.0

            marker.scale.x = size_x
            marker.scale.y = size_y
            marker.scale.z = 0.30

            # 하늘색 / Cyan
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0

            # 반투명
            marker.color.a = 0.45

            marker.lifetime = rospy.Duration(0.15)

            marker_array.markers.append(marker)

        self.obstacle_marker_pub.publish(marker_array)

    def has_front_lidar_obstacle(self, ranges, angles):
        self.current_lidar_obstacle_clusters = []
        # 벽 필터가 켜진 경우 map과 pose가 준비되기 전에 LiDAR 점을
        # 장애물로 판정하면 트랙 벽 때문에 시작 즉시 FTG가 켜질 수 있다.
        if self.wall_filter_enabled and (
            not self.has_map() or
            self.vehicle_x is None or
            self.vehicle_y is None or
            self.cur_yaw is None
        ):
            rospy.logwarn_throttle(2.0, "[FTG] Waiting for /map and vehicle pose.")
            return False

        half_angle = math.radians(self.lidar_trigger_half_angle_deg)
        front_mask = (
            (np.abs(angles) <= half_angle) &
            (ranges > 0.0) &
            (ranges <= self.lidar_trigger_distance)
        )

        candidate_indices = np.flatnonzero(front_mask)
        clusters = []
        current_cluster = []

        for index in candidate_indices:
            distance = float(ranges[index])
            angle = float(angles[index])

            # /map의 점유 영역과 겹치는 LiDAR 점은 트랙의 고정 벽이므로
            # 새로운 장애물로 간주해 FTG를 발동하지 않는다.
            if self.is_lidar_point_near_map_wall(distance, angle):
                if current_cluster:
                    clusters.append(current_cluster)
                    current_cluster = []
                continue

            point = np.array([
                distance * math.cos(angle),
                distance * math.sin(angle),
            ])

            # 스캔 배열에서 연속되지 않거나 실제 점 사이 거리가 멀면
            # 이전 군집을 닫고 새로운 물체 후보를 시작한다.
            if current_cluster:
                previous_index, previous_point = current_cluster[-1]
                point_gap = float(np.linalg.norm(point - previous_point))
                if (
                    index != previous_index + 1 or
                    point_gap > self.lidar_cluster_max_gap
                ):
                    clusters.append(current_cluster)
                    current_cluster = []

            current_cluster.append((int(index), point))

        if current_cluster:
            clusters.append(current_cluster)

        # 0.5 x 0.5 m 정사각형은 관측 방향에 따라 최대 대각선 길이
        # sqrt(2) * 0.5 m로 보일 수 있으므로 이를 군집 폭 상한으로 쓴다.
        max_visible_width = (
            math.sqrt(2.0) * self.obstacle_max_size +
            self.obstacle_size_tolerance
        )

        valid_obstacle_clusters = []

        for cluster in clusters:

            if len(cluster) < self.lidar_cluster_min_points:
                continue

            points = np.array([
                item[1]
                for item in cluster
            ])

            cluster_width = float(
                np.linalg.norm(
                    points[-1] - points[0]
                )
            )

            if cluster_width <= max_visible_width:

                valid_obstacle_clusters.append(cluster)

                rospy.loginfo_throttle(
                    1.0,
                    "[FTG] LiDAR obstacle: points=%d width=%.2fm",
                    len(cluster),
                    cluster_width,
                )


        # 현재 scan에서 실제 장애물로 인정된 cluster 저장
        self.current_lidar_obstacle_clusters = valid_obstacle_clusters

        # RViz 장애물 Box 표시
        self.publish_obstacle_markers(
            valid_obstacle_clusters
        )

        return len(valid_obstacle_clusters) > 0

    def hold_target_angle(self, target_angle):
        now = rospy.Time.now()

        if self.locked_target_angle is None:
            self.locked_target_angle = target_angle
            self.last_target_update_time = now
            return self.locked_target_angle

        age = (now - self.last_target_update_time).to_sec()
        angle_delta = abs(target_angle - self.locked_target_angle)

        if (
            age >= self.target_angle_hold_time or
            angle_delta >= self.target_angle_update_threshold
        ):
            self.locked_target_angle = target_angle
            self.last_target_update_time = now

        return self.locked_target_angle

    def publish_ftg_path_markers(self, waypoint_msg, target_angle):
        if (
            self.vehicle_x is None or
            self.vehicle_y is None or
            self.cur_yaw is None
        ):
            return

        marker_array = MarkerArray()

        now = rospy.Time.now()

        # =========================================================
        # 1. 실제 MAP controller로 보내는 회피 경로
        # =========================================================

        path_marker = Marker()

        path_marker.header.frame_id = "map"
        path_marker.header.stamp = now

        path_marker.ns = "ftg_path"
        path_marker.id = 0

        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD

        path_marker.pose.orientation.w = 1.0

        # 선 굵기
        path_marker.scale.x = 0.05

        # 노란색
        path_marker.color.r = 1.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0

        # 차량 현재 위치부터 시작
        start_point = Point()
        start_point.x = self.vehicle_x
        start_point.y = self.vehicle_y
        start_point.z = 0.08

        path_marker.points.append(start_point)

        # 실제 publish되는 waypoint를 그대로 사용
        for wpnt in waypoint_msg.wpnts:
            point = Point()

            point.x = wpnt.x_m
            point.y = wpnt.y_m
            point.z = 0.08

            path_marker.points.append(point)

        marker_array.markers.append(path_marker)

        # =========================================================
        # 2. FTG가 최종적으로 선택한 Target point
        # =========================================================

        if self.selected_target_distance is not None:

            target_heading = self.cur_yaw + target_angle

            target_distance = min(
                self.selected_target_distance,
                self.max_range
            )

            target_x = (
                self.vehicle_x +
                target_distance * math.cos(target_heading)
            )

            target_y = (
                self.vehicle_y +
                target_distance * math.sin(target_heading)
            )

            target_marker = Marker()

            target_marker.header.frame_id = "map"
            target_marker.header.stamp = now

            target_marker.ns = "ftg_target"
            target_marker.id = 1

            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD

            target_marker.pose.position.x = target_x
            target_marker.pose.position.y = target_y
            target_marker.pose.position.z = 0.15

            target_marker.pose.orientation.w = 1.0

            target_marker.scale.x = 0.18
            target_marker.scale.y = 0.18
            target_marker.scale.z = 0.18

            # 빨간색 target
            target_marker.color.r = 1.0
            target_marker.color.g = 0.0
            target_marker.color.b = 0.0
            target_marker.color.a = 1.0

            marker_array.markers.append(target_marker)

        # =========================================================
        # 3. 선택된 Gap의 양쪽 경계
        # =========================================================

        if (
            self.selected_gap_start_angle is not None and
            self.selected_gap_end_angle is not None
        ):

            gap_marker = Marker()

            gap_marker.header.frame_id = "map"
            gap_marker.header.stamp = now

            gap_marker.ns = "ftg_gap"
            gap_marker.id = 2

            gap_marker.type = Marker.LINE_LIST
            gap_marker.action = Marker.ADD

            gap_marker.pose.orientation.w = 1.0

            gap_marker.scale.x = 0.035

            # 초록색 gap 경계
            gap_marker.color.r = 0.0
            gap_marker.color.g = 1.0
            gap_marker.color.b = 0.0
            gap_marker.color.a = 1.0

            # 차량 시작점
            car_point_1 = Point()
            car_point_1.x = self.vehicle_x
            car_point_1.y = self.vehicle_y
            car_point_1.z = 0.05

            car_point_2 = Point()
            car_point_2.x = self.vehicle_x
            car_point_2.y = self.vehicle_y
            car_point_2.z = 0.05

            # Gap 시작 방향
            start_heading = (
                self.cur_yaw +
                self.selected_gap_start_angle
            )

            start_distance = min(
                self.selected_gap_start_distance,
                self.max_range
            )

            gap_start_point = Point()

            gap_start_point.x = (
                self.vehicle_x +
                start_distance * math.cos(start_heading)
            )

            gap_start_point.y = (
                self.vehicle_y +
                start_distance * math.sin(start_heading)
            )

            gap_start_point.z = 0.05

            # Gap 끝 방향
            end_heading = (
                self.cur_yaw +
                self.selected_gap_end_angle
            )

            end_distance = min(
                self.selected_gap_end_distance,
                self.max_range
            )

            gap_end_point = Point()

            gap_end_point.x = (
                self.vehicle_x +
                end_distance * math.cos(end_heading)
            )

            gap_end_point.y = (
                self.vehicle_y +
                end_distance * math.sin(end_heading)
            )

            gap_end_point.z = 0.05

            gap_marker.points.append(car_point_1)
            gap_marker.points.append(gap_start_point)

            gap_marker.points.append(car_point_2)
            gap_marker.points.append(gap_end_point)

            marker_array.markers.append(gap_marker)

        self.path_marker_pub.publish(marker_array)

    def clear_ftg_path_markers(self):
        marker_array = MarkerArray()

        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        marker.action = Marker.DELETEALL

        marker_array.markers.append(marker)

        self.path_marker_pub.publish(marker_array)

    def publish_avoidance_waypoints(self, target_angle, speed=None):
        if self.vehicle_x is None or self.vehicle_y is None or self.cur_yaw is None:
            rospy.logwarn_throttle(
                1.0,
                "[FTG] Waiting for pose before publishing avoidance waypoints."
            )
            return

        target_angle = self.clamp(
            target_angle,
            -self.max_steer,
            self.max_steer
        )

        if speed is None:
            speed = self.avoidance_speed

        distances = np.arange(
            self.path_ds,
            self.path_length + self.path_ds * 0.5,
            self.path_ds
        )

        msg = OTWpntArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        # 현재 차량 위치
        x = self.vehicle_x
        y = self.vehicle_y

        previous_dist = 0.0

        for i, dist in enumerate(distances):

            # 0.0 → 1.0
            progress = dist / self.path_length

            # 처음에는 현재 차량 방향,
            # 뒤로 갈수록 target_angle 방향으로 부드럽게 변화
            smooth_progress = (
                3.0 * progress ** 2
                - 2.0 * progress ** 3
            )

            local_angle = target_angle * smooth_progress

            heading = self.cur_yaw + local_angle

            ds = dist - previous_dist

            x += ds * math.cos(heading)
            y += ds * math.sin(heading)

            previous_dist = dist

            wpnt = Wpnt()
            wpnt.id = i

            wpnt.x_m = x
            wpnt.y_m = y

            wpnt.s_m = dist
            wpnt.d_m = 0.0
            wpnt.vx_mps = speed

            msg.wpnts.append(wpnt)

        self.path_pub.publish(msg)

        self.publish_ftg_path_markers(
            msg,
            target_angle
        )

    def scan_callback(self, scan):
        ranges = np.asarray(scan.ranges, dtype=float)
        # 대부분의 LiDAR는 반사체가 없으면 +inf를 준다. 이를 0(막힘)으로
        # 바꾸면 열린 gap이 전부 사라지므로 max_range의 자유 공간으로 취급한다.
        ranges = np.where(np.isposinf(ranges), self.max_range, ranges)
        ranges = np.where(np.isnan(ranges), 0.0, ranges)
        ranges = np.where(
            (ranges >= max(scan.range_min, 1e-3)),
            np.minimum(ranges, min(self.max_range, scan.range_max)),
            0.0,
        )

        ranges, angles = self.restrict_fov(scan, ranges)
        if not (
            self.has_active_obstacle() or
            self.has_front_lidar_obstacle(ranges, angles)
        ):
            self.set_ftg_active(False)
            self.publish_empty_avoidance()
            return

        target_angle = self.choose_gap_angle(ranges, angles)

        if target_angle is None:
            rospy.logerr_throttle(1.0, "[FTG] No safe gap: publishing stop path.")
            self.set_ftg_active(True)
            self.publish_avoidance_waypoints(0.0, speed=0.0)
            return
        else:
            target_angle = self.hold_target_angle(target_angle)

        self.set_ftg_active(True)
        self.publish_avoidance_waypoints(target_angle)


if __name__ == "__main__":
    try:
        FollowTheGap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
