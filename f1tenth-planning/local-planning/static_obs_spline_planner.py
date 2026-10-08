#!/usr/bin/env python3
import time
import sys
from typing import List, Tuple

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from scipy.interpolate import InterpolatedUnivariateSpline as Spline

from f110_msgs.msg import ObstacleArray, OTWpntArray, Wpnt, WpntArray
from frenet_utils import FrenetConverter

class StaticObstacleSpliner:
    """
    A ROS node for generating spline paths around static obstacles.
    """

    def __init__(self):
        rospy.init_node("static_obs_spliner_node")

        # Variables
        self.obs = ObstacleArray()
        self.gb_wpnts = WpntArray()
        self.gb_vmax = None
        self.gb_max_idx = None
        self.gb_max_s = None
        self.cur_s = 0
        self.cur_d = 0
        self.cur_vs = 0
        self.gb_scaled_wpnts = WpntArray()
        self.last_wpnts = OTWpntArray()  # 마지막 유효 메시지 저장

        # Parameters
        self.pre_apex_0 = -8
        self.pre_apex_1 = -6
        self.pre_apex_2 = -3
        self.post_apex_0 = 1.5
        self.post_apex_1 = 3
        self.post_apex_2 = 6
        
        self.evasion_dist = 0.6
        self.obs_traj_tresh = 0.8
        self.spline_bound_mindist = 0.5

        # --- [추가/수정된 파트] ---
        # 1. 곡률 판단을 위해 몇 개의 웨이포인트 앞을 볼 것인가 (5~10 추천)
        self.curve_lookahead = 8 
        
        # 2. 회피 경로가 생성될 수 있는 최대 d값 (트랙 폭)
        # 기존 0.7(evasion_dist)로 clip하면 바깥으로 크게 돌지 못해 벽에 박습니다.
        # 트랙 폭이 3m라면 여유 있게 2.0m 정도로 설정
        self.max_d_limit = 1.5 
        # ------------------------

        # Subscribers
        rospy.Subscriber("/global_path/optimal_trajectory_wpnt", WpntArray, self.gb_cb)
        rospy.Subscriber("/perception/detection/raw_obstacles", ObstacleArray, self.obs_cb)
        rospy.Subscriber("/odom_frenet", Odometry, self.state_cb)

        # Publishers
        self.mrks_pub = rospy.Publisher("/planner/avoidance/markers", MarkerArray, queue_size=10)
        self.evasion_pub = rospy.Publisher("/planner/avoidance/otwpnts",
                                          OTWpntArray, queue_size=10)

        # Initialize Frenet Converter
        self.converter = self.initialize_converter()

        # Loop rate
        self.rate = rospy.Rate(20)  # Hz

    # Callbacks
    def obs_cb(self, data: ObstacleArray):
        self.obs = data

    def state_cb(self, data: Odometry):
        self.cur_vs = data.twist.twist.linear.x
        self.cur_s, self.cur_d = data.pose.pose.position.x, data.pose.pose.position.y

    def gb_cb(self, data: WpntArray):
        # [수정 후] (detect_j1.py와 일치시킴)
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m, wpnt.s_m] for wpnt in data.wpnts])
        self.gb_wpnts = data
        if self.gb_vmax is None:
            self.gb_vmax = np.max(np.array([wpnt.vx_mps for wpnt in data.wpnts]))
            self.gb_max_idx = data.wpnts[-1].id
            self.gb_max_s = data.wpnts[-1].s_m

    # Main loop
    def loop(self):
        rospy.loginfo("[static_obs_spliner_node] Waiting for messages...")
        rospy.wait_for_message("/global_path/optimal_trajectory_wpnt", WpntArray)
        rospy.wait_for_message("/perception/detection/raw_obstacles", ObstacleArray)
        rospy.loginfo("[static_obs_spliner_node] Ready!")

        while not rospy.is_shutdown():
            obs = self.obs
            gb_wpnts = self.gb_wpnts.wpnts

            wpnts, mrks = OTWpntArray(), MarkerArray()

            if len(obs.obstacles) > 0:
                wpnts, mrks = self.do_spline(obstacles=obs, gb_wpnts=gb_wpnts)
                self.last_wpnts = wpnts  # 마지막 유효 메시지 저장
            else:
                del_mrk = Marker()
                del_mrk.action = Marker.DELETEALL
                mrks.markers.append(del_mrk)

            # MUX용 토픽으로 발행
            self.evasion_pub.publish(wpnts)
            self.mrks_pub.publish(mrks)
            self.rate.sleep()

    def do_spline(self, obstacles: ObstacleArray, gb_wpnts: WpntArray) -> Tuple[OTWpntArray, MarkerArray]:
        wpnts = OTWpntArray()
        mrks = MarkerArray()

        # [수정된 코드] 2차 필터링을 제거하고 모든 장애물을 사용
        all_obs = obstacles.obstacles

        threshold_low = 0.2
        threshold_high = 0.5

        if len(all_obs) > 0:
            closest_obs = min(all_obs, key=lambda obs: (obs.s_center - self.cur_s) % self.gb_max_s)
            s_apex = (closest_obs.s_end + closest_obs.s_start) / 2
            d_center = closest_obs.d_center

            # ------------------------------------------------------------------
            # [Step 1] 곡률(회전 방향) 계산: S자 코너의 안쪽 벽 충돌 방지
            # ------------------------------------------------------------------
            # 현재 장애물 위치의 Global Path 인덱스 찾기
            total_wpnts = len(self.gb_wpnts.wpnts)
            obs_idx = int((closest_obs.s_center / self.gb_max_s) * total_wpnts) % total_wpnts

            # 현재 벡터 (p1 -> p1_next)
            p1 = self.gb_wpnts.wpnts[obs_idx]
            p1_next = self.gb_wpnts.wpnts[(obs_idx + 1) % total_wpnts]
            vec_current_x = p1_next.x_m - p1.x_m
            vec_current_y = p1_next.y_m - p1.y_m

            # 미래 벡터 (lookahead 지점 -> lookahead_next)
            future_idx = (obs_idx + self.curve_lookahead) % total_wpnts
            p2 = self.gb_wpnts.wpnts[future_idx]
            p2_next = self.gb_wpnts.wpnts[(future_idx + 1) % total_wpnts]
            vec_future_x = p2_next.x_m - p2.x_m
            vec_future_y = p2_next.y_m - p2.y_m

            # 외적(Cross Product)을 통한 회전 방향 판별 (2D)
            # Cross > 0: 좌회전 (Left Turn)
            # Cross < 0: 우회전 (Right Turn)
            cross_prod = (vec_current_x * vec_future_y) - (vec_current_y * vec_future_x)
            
            # 외적 값이 너무 작으면 직선으로 간주 (노이즈 방지)
            is_turning_left = cross_prod > 0.05
            is_turning_right = cross_prod < -0.05

            # ------------------------------------------------------------------
            # [Step 2] 회피 방향(d_apex) 결정 로직
            # ------------------------------------------------------------------
            target_d = self.evasion_dist  # 0.7m

            # 기본 로직: 장애물 중심이 양수면 음수 쪽으로, 음수면 양수 쪽으로 피함
            # 하지만 코너에서는 "무조건 바깥쪽"으로 피하도록 강제함
            
            if is_turning_left: 
                # [좌회전 구간]
                # 경로(d=0) 기준 왼쪽(+)은 안쪽 벽(Inside), 오른쪽(-)은 바깥쪽 공간(Outside)
                # 장애물이 경로 중앙 근처(threshold_low)거나, 이미 안쪽(+)에 있다면 -> 무조건 오른쪽(-) 회피
                if d_center > -threshold_low: 
                    d_apex = -target_d 
                else:
                    # 장애물이 이미 오른쪽에 있다면 더 오른쪽으로
                    d_apex = d_center - target_d

            elif is_turning_right:
                # [우회전 구간]
                # 경로(d=0) 기준 오른쪽(-)은 안쪽 벽(Inside), 왼쪽(+)은 바깥쪽 공간(Outside)
                # 장애물이 경로 중앙 근처거나, 이미 안쪽(-)에 있다면 -> 무조건 왼쪽(+) 회피
                if d_center < threshold_low:
                    d_apex = target_d
                else:
                    # 장애물이 이미 왼쪽에 있다면 더 왼쪽으로
                    d_apex = d_center + target_d
            
            else:
                # [직선 구간] 기존 로직 유지
                if abs(d_center) < threshold_low:
                    # 완전 중앙이면 + 방향으로 (혹은 상황따라)
                    d_apex = target_d 
                elif d_center > 0:
                    d_apex = -(d_center + target_d) if d_center < threshold_high else -target_d
                else:
                    d_apex = -(d_center - target_d) if d_center > -threshold_high else target_d


            # ------------------------------------------------------------------
            # [Step 3] Spline 생성 (기존 코드 유지)
            # ------------------------------------------------------------------  

            evasion_points = []
            spline_params = [
                self.pre_apex_0, self.pre_apex_1, self.pre_apex_2,
                0,
                self.post_apex_0, self.post_apex_1, self.post_apex_2
            ]
            for dst in spline_params:
                si = s_apex + dst
                di = d_apex if dst == 0 else 0
                evasion_points.append([si, di])

            evasion_points = np.array(evasion_points)
            spatial_spline = Spline(x=evasion_points[:,0], y=evasion_points[:,1])
            evasion_s = np.arange(evasion_points[0,0], evasion_points[-1,0], 0.1)
            evasion_d = spatial_spline(evasion_s)
            evasion_s = evasion_s % self.gb_max_s
            evasion_d = np.clip(evasion_d, -self.evasion_dist, self.evasion_dist)

            resp = self.converter.get_cartesian(evasion_s, evasion_d)
            line_strip_points = []

            for i in range(evasion_s.shape[0]):
                gb_wpnt_i = int((evasion_s[i] / (self.gb_wpnts.wpnts[1].s_m - self.gb_wpnts.wpnts[0].s_m)) % self.gb_max_idx)

                vi = self.gb_wpnts.wpnts[gb_wpnt_i].vx_mps

                wpnts.wpnts.append(Wpnt(
                    id=i,
                    x_m=resp[0,i],
                    y_m=resp[1,i],
                    s_m=evasion_s[i],
                    d_m=evasion_d[i],
                    vx_mps=vi
                ))
                line_strip_points.append([resp[0,i], resp[1,i]])

            marker_id = 0
            line_strip_marker = self.create_marker(line_strip_points, marker_id)
            mrks.markers.append(line_strip_marker)

            wpnts.header.stamp = rospy.Time.now()
            wpnts.header.frame_id = "map"

        return wpnts, mrks

    def create_marker(self, waypoints: list, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        for waypoint in waypoints:
            point = Point()
            point.x = waypoint[0]
            point.y = waypoint[1]
            point.z = 0.0
            marker.points.append(point)
        marker.id = marker_id
        return marker

    def initialize_converter(self) -> FrenetConverter:
        rospy.wait_for_message("/global_path/optimal_trajectory_wpnt", WpntArray)
        converter = FrenetConverter(self.waypoints[:,0], self.waypoints[:,1], self.waypoints[:,2])
        rospy.loginfo("[static_obs_spliner_node] FrenetConverter initialized.")
        return converter

if __name__ == "__main__":
    spliner = StaticObstacleSpliner()
    spliner.loop()
