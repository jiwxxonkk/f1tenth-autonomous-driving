#!/usr/bin/env python3 
import rospy 
import math
import numpy as np 
from typing import List, Tuple 

from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Point, PoseWithCovarianceStamped 
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray 
from scipy.interpolate import InterpolatedUnivariateSpline as Spline 
from scipy.ndimage import distance_transform_edt
from tf.transformations import euler_from_quaternion
from collections import deque
from copy import deepcopy

from f110_msgs.msg import Obstacle, ObstacleArray, WpntArray, OTWpntArray 
import roslib.message
from frenet_utils import FrenetConverter

class StaticObstacleLatticePlanner: 
    def __init__(self): 
        rospy.init_node("static_obs_lattice_node") 

        # --- Variables & Data Storage --- 
        self.obs = ObstacleArray() 
        self.gb_wpnts = WpntArray() 
        self.gb_max_s = None 
        self.cur_s = 0 
        self.cur_d = 0 
        self.cur_vs = 0  
        self.cur_yaw = 0.0 

        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.converter = None
        self.waypoints = None
        self.pose_received = False

        # /scan에서 트랙 벽을 제외하기 위한 /map 정보
        self.map_data = None
        self.map_resolution = None
        self.map_width = 0
        self.map_height = 0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_origin_yaw = 0.0
        self.map_wall_clearance = None

        # detect.py는 속도를 계산하지 않으므로 raw obstacle의 Frenet 위치를
        # 시간축으로 연결해 동적 차량만 별도로 확정한다.
        self.motion_tracks = {}
        self.current_obstacle_track_ids = {}
        self.current_dynamic_object_ids = set()
        self.next_motion_track_id = 1
        self.follow_target_track_id = None
        self.follow_target_cache = None
        self.follow_target_last_seen = rospy.Time(0)
        self.follow_speed_command = None
        self.follow_speed_update_time = rospy.Time(0)
        self.follow_filtered_gap = None
        self.follow_filtered_opponent_speed = None
        self.follow_off_path_since = rospy.Time(0)
        self.follow_off_path_since = rospy.Time(0)
        self.static_latch_track_id = None

        # 동적 추월 상태. 정적 Lattice의 생성/평가/latch 코드는 건드리지
        # 않고, FOLLOW 위에 별도의 상태 머신으로만 동작한다.
        self.overtake_phase = "FOLLOW"
        self.overtake_target_track_id = None
        self.overtake_target_cache = None
        self.overtake_target_last_seen = rospy.Time(0)
        self.overtake_locked_side = 0
        self.overtake_locked_d = None
        self.overtake_started_at = rospy.Time(0)
        self.overtake_follow_since = rospy.Time(0)
        self.overtake_gap_since = rospy.Time(0)
        self.overtake_ready_track_id = None
        self.overtake_candidate_side = 0
        self.overtake_candidate_d = None
        self.overtake_candidate_path = None
        self.overtake_reject_reason = "waiting_for_dynamic_target"
        self.overtake_shift_start_s = None
        self.overtake_shift_start_d = None
        self.overtake_shift_length = None
        self.overtake_return_start_s = None
        self.overtake_return_start_d = None
        self.overtake_return_length_locked = None
        self.overtake_locked_rollout = None

        # detect_2.py가 발행하는 ObstacleArray를 기본 장애물 입력으로 쓴다.
        # 기존 플래너 내부 LiDAR 처리는 필요할 때 파라미터로 다시 켤 수 있다.
        self.obstacle_topic = rospy.get_param(
            "~obstacle_topic",
            "/perception/detection/raw_obstacles"
        )
        self.use_internal_lidar = rospy.get_param("~use_internal_lidar", False)

        # ==========================================================
        # REAL CAR / NANO I/O
        # 시뮬 알고리즘은 그대로 두고 ROS 인터페이스만 실차에 맞춘다.
        # ==========================================================
        self.global_topic = rospy.get_param(
            "~global_topic",
            "/global_path/optimal_trajectory_wpnt"
        )
        self.pose_topic = rospy.get_param(
            "~pose_topic",
            "/amcl_pose"
        )
        self.odom_topic = rospy.get_param(
            "~odom_topic",
            "/odom"
        )
        self.avoidance_output_topic = rospy.get_param(
            "~avoidance_output_topic",
            "/planner/avoidance/otwpnts"
        )

        # 내부 LiDAR 모드에서 사용할 입력 토픽이다.
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.map_topic = rospy.get_param("~map_topic", "/map")

        self.speed_received = False

        # Nano의 f110_msgs 버전에 따라 OTWpntArray.wpnts의
        # 실제 element type이 다를 수 있으므로 런타임에 찾는다.
        self.ot_wpnt_cls = self.resolve_array_element_class(
            OTWpntArray,
            "wpnts"
        )

        # --- Parameters ---
        # 1. Lookahead 관련 파라미터
        self.m_map = rospy.get_param("~m_map", 1.4) # 속도에 따라 lookahead 거리를 늘리는 비율
        self.q_map = rospy.get_param("~q_map", 0.7)  # 기본 lookahead 거리
        self.t_clip_min = rospy.get_param("~t_clip_min", 0.5) # lookahead 최소 제한값
        self.t_clip_max = rospy.get_param("~t_clip_max", 3.0) # lookahead 최대 제한값

        # 2. 후보 경로 생성 관련 파라미터
        self.num_samples = rospy.get_param("~num_samples", 13) # 생성할 후보 경로 개수
        self.max_offset = rospy.get_param("~max_offset", 1.4) # global path 기준 좌우 최대 회피 폭
        self.visual_start_offset = rospy.get_param("~visual_start_offset", 0.0) # 후보 경로 시각화 시작 s offset

        # 3. 장애물 충돌 판정 관련 파라미터
        self.collision_margin = rospy.get_param("~collision_margin", 0.2) # 장애물 d 범위를 좌우로 부풀리는 안전 margin
        self.front_s_buffer = rospy.get_param("~front_s_buffer", 0.8) # 정적 장애물 앞쪽 s 위험 구간 buffer
        self.rear_s_buffer = rospy.get_param("~rear_s_buffer", 0.8) # 정적 장애물 뒤쪽 s 위험 구간 buffer
        self.dynamic_front_s_buffer = rospy.get_param("~dynamic_front_s_buffer", 0.5) # 동적 장애물 앞쪽 s 위험 구간 buffer
        self.dynamic_rear_s_buffer = rospy.get_param("~dynamic_rear_s_buffer", 0.5)  # 동적 장애물 뒤쪽 s 위험 구간 buffer
        self.dynamic_time_buffer = rospy.get_param("~dynamic_time_buffer", 0.15) # 동적 장애물 미래 위치 예측 시 추가 시간 buffer

        # 4. 동적 모드 진입 조건 파라미터
        self.dynamic_avoidance_enabled = rospy.get_param("~dynamic_avoidance_enabled", True)  # True: 동적 장애물 회피/추종 사용, False: 모든 장애물을 정적 장애물처럼 처리

        self.dynamic_trigger_s_min = rospy.get_param("~dynamic_trigger_s_min", 0.3) # 동적 모드 판단에서 차량 바로 앞 제외 거리
        self.dynamic_trigger_s_max = rospy.get_param("~dynamic_trigger_s_max", 7.0) # 약 7 m 앞부터 동적 FOLLOW 진입
        self.dynamic_release_s_max = rospy.get_param("~dynamic_release_s_max", 9.0) # FOLLOW 진입 후 검출 흔들림으로 바로 GLOBAL 복귀하지 않는 해제 거리
        self.dynamic_speed_threshold = rospy.get_param("~dynamic_speed_threshold", 0.15) # 이 속도 이상이면 동적 장애물로 판단
        self.dynamic_block_d_margin = rospy.get_param("~dynamic_block_d_margin", 0.45) # 동적 장애물이 현재 차선/global path를 막는지 판단할 d margin

        # 고속용 동적 판정: 순간 속도 한 번이 아니라 일정 시간의 순이동량,
        # 평균/중앙 속도, 진행 방향 일관성, map 벽 여부를 모두 확인한다.
        self.motion_track_timeout = rospy.get_param("~motion_track_timeout", 1.00)
        self.motion_history_time = rospy.get_param("~motion_history_time", 0.90)
        self.motion_match_s_max = rospy.get_param("~motion_match_s_max", 2.00)
        self.motion_match_d = rospy.get_param("~motion_match_d", 0.55)
        self.motion_confirm_time = rospy.get_param("~motion_confirm_time", 0.40)
        self.motion_confirm_distance = rospy.get_param("~motion_confirm_distance", 0.15)
        self.motion_direction_consistency = rospy.get_param("~motion_direction_consistency", 0.80)
        self.motion_max_abs_vd = rospy.get_param("~motion_max_abs_vd", 0.80)
        self.motion_max_abs_vs = rospy.get_param("~motion_max_abs_vs", 12.0)
        self.dynamic_wall_reject_clearance = rospy.get_param(
            "~dynamic_wall_reject_clearance", 0.12
        )

        # 4-1. 동적 장애물 추종 fallback 파라미터
        self.follow_min_distance = rospy.get_param("~follow_min_distance", 1.0) # 추종 시 앞 장애물과 최소로 유지할 거리
        self.follow_slow_distance = rospy.get_param("~follow_slow_distance", 2.5) # 이 거리 안으로 들어오면 속도를 낮추기 시작하는 기준 거리
        self.follow_path_length = rospy.get_param("~follow_path_length", 12.0) # GLOBAL 형상 추종 경로 길이
        self.follow_path_ds = rospy.get_param("~follow_path_ds", 0.15) # 추종 fallback 경로의 waypoint 간격
        self.follow_min_speed = rospy.get_param("~follow_min_speed", 0.0) # 앞차가 정지하면 같이 정지 가능
        self.follow_max_speed = rospy.get_param("~follow_max_speed", 7.0) # 고속 추종 상한
        self.follow_speed_margin = rospy.get_param("~follow_speed_margin", 0.0) # 1 m에서 앞차 속도를 그대로 사용
        self.follow_gap_deadband = rospy.get_param("~follow_gap_deadband", 0.15)
        self.follow_gap_kp = rospy.get_param("~follow_gap_kp", 0.60)
        self.follow_speed_tau = rospy.get_param("~follow_speed_tau", 0.35)
        self.follow_gap_filter_tau = rospy.get_param("~follow_gap_filter_tau", 0.20)
        self.follow_opponent_speed_filter_tau = rospy.get_param(
            "~follow_opponent_speed_filter_tau", 0.35
        )
        self.follow_max_speed_correction = rospy.get_param(
            "~follow_max_speed_correction", 1.20
        )
        self.follow_accel_limit = rospy.get_param("~follow_accel_limit", 4.00)
        self.follow_decel_limit = rospy.get_param("~follow_decel_limit", 4.00)
        self.follow_close_gap = rospy.get_param("~follow_close_gap", 0.55)
        self.follow_close_speed_reduction = rospy.get_param(
            "~follow_close_speed_reduction", 0.80
        )
        self.follow_ego_front_offset = rospy.get_param("~follow_ego_front_offset", 0.25)
        self.follow_target_lost_timeout = rospy.get_param("~follow_target_lost_timeout", 1.00)
        self.follow_global_merge_length = rospy.get_param("~follow_global_merge_length", 2.5)
        self.follow_reacquire_s_max = rospy.get_param("~follow_reacquire_s_max", 1.50)
        self.follow_reacquire_d_max = rospy.get_param("~follow_reacquire_d_max", 0.50)
        self.follow_release_d_margin = rospy.get_param(
            "~follow_release_d_margin", 0.20
        )
        self.follow_release_persist_time = rospy.get_param(
            "~follow_release_persist_time", 0.25
        )
        self.dynamic_overtake_min_gap = rospy.get_param("~dynamic_overtake_min_gap", 0.3) # 동적 장애물 추월을 허용할 최소 빈 공간 폭. 이보다 좁으면 추월하지 않고 추종
        self.dynamic_overtake_s_window = rospy.get_param("~dynamic_overtake_s_window", 0.8) # 동적 장애물 좌우 추월 공간 계산 시 같은 s 구간으로 볼 앞뒤 거리

        # 4-2. FOLLOW 우선 동적 추월 파라미터
        # FOLLOW 전용 실차본: GLOBAL 형상과 앞차 속도만 따라가며 추월
        # 상태 머신은 기본적으로 완전히 비활성화한다.
        self.dynamic_overtake_enabled = rospy.get_param(
            "~dynamic_overtake_enabled", False
        )
        self.overtake_follow_min_time = rospy.get_param(
            "~overtake_follow_min_time", 1.00
        )
        self.overtake_gap_persist_time = rospy.get_param(
            "~overtake_gap_persist_time", 0.80
        )
        self.overtake_min_start_gap = rospy.get_param(
            "~overtake_min_start_gap", 0.90
        )
        self.overtake_max_start_gap = rospy.get_param(
            "~overtake_max_start_gap", 2.50
        )
        self.overtake_min_lateral_clearance = rospy.get_param(
            "~overtake_min_lateral_clearance", 0.48
        )
        self.overtake_target_extra_clearance = rospy.get_param(
            "~overtake_target_extra_clearance", 0.08
        )
        self.overtake_entry_min_length = rospy.get_param(
            "~overtake_entry_min_length", 3.50
        )
        self.overtake_entry_time = rospy.get_param(
            "~overtake_entry_time", 0.75
        )
        self.overtake_return_min_length = rospy.get_param(
            "~overtake_return_min_length", 3.00
        )
        self.overtake_return_time = rospy.get_param(
            "~overtake_return_time", 0.50
        )
        self.overtake_path_min_length = rospy.get_param(
            "~overtake_path_min_length", 12.0
        )
        self.overtake_path_max_length = rospy.get_param(
            "~overtake_path_max_length", 22.0
        )
        self.overtake_path_ds = rospy.get_param("~overtake_path_ds", 0.12)
        self.overtake_speed_gain = rospy.get_param(
            "~overtake_speed_gain", 1.80
        )
        self.overtake_global_speed_fraction = rospy.get_param(
            "~overtake_global_speed_fraction", 0.95
        )
        self.overtake_min_speed_advantage = rospy.get_param(
            "~overtake_min_speed_advantage", 0.80
        )
        self.overtake_max_speed = rospy.get_param(
            "~overtake_max_speed", 7.0
        )
        self.overtake_pass_margin = rospy.get_param(
            "~overtake_pass_margin", 1.20
        )
        self.overtake_timeout = rospy.get_param("~overtake_timeout", 7.0)
        self.overtake_target_lost_timeout = rospy.get_param(
            "~overtake_target_lost_timeout", 1.00
        )
        self.overtake_min_straight_length = rospy.get_param(
            "~overtake_min_straight_length", 10.0
        )
        self.overtake_max_curvature = rospy.get_param(
            "~overtake_max_curvature", 0.28
        )
        self.overtake_candidate_switch_tolerance = rospy.get_param(
            "~overtake_candidate_switch_tolerance", 0.25
        )
        self.overtake_prediction_accel = rospy.get_param(
            "~overtake_prediction_accel", 2.0
        )
        self.overtake_prediction_decel = rospy.get_param(
            "~overtake_prediction_decel", 3.5
        )
        self.overtake_prediction_dt = rospy.get_param(
            "~overtake_prediction_dt", 0.05
        )
        self.overtake_prediction_max_time = rospy.get_param(
            "~overtake_prediction_max_time", 7.0
        )
        self.overtake_prediction_position_uncertainty = rospy.get_param(
            "~overtake_prediction_position_uncertainty", 0.10
        )
        self.overtake_prediction_speed_uncertainty = rospy.get_param(
            "~overtake_prediction_speed_uncertainty", 0.12
        )
        self.overtake_opponent_accel_bound = rospy.get_param(
            "~overtake_opponent_accel_bound", 1.0
        )
        self.overtake_prediction_uncertainty_max = rospy.get_param(
            "~overtake_prediction_uncertainty_max", 0.70
        )
        self.overtake_return_observed_margin = rospy.get_param(
            "~overtake_return_observed_margin", 0.10
        )
        self.overtake_full_plan_margin = rospy.get_param(
            "~overtake_full_plan_margin", 1.0
        )
        self.overtake_require_full_straight = rospy.get_param(
            "~overtake_require_full_straight", True
        )
        self.overtake_reacquire_s_max = rospy.get_param(
            "~overtake_reacquire_s_max", 1.50
        )
        self.overtake_reacquire_d_max = rospy.get_param(
            "~overtake_reacquire_d_max", 0.60
        )
        self.overtake_reacquire_speed_max = rospy.get_param(
            "~overtake_reacquire_speed_max", 2.0
        )

        # 5. Gate, 즉 빈 공간 선택 관련 파라미터
        self.gate_s_min = rospy.get_param("~gate_s_min", 0.2) # gate 계산에서 차량 바로 앞 제외 거리
        self.gate_s_max = rospy.get_param("~gate_s_max", 8.0) # gate 계산에 사용할 앞쪽 최대 거리
        self.gate_margin = rospy.get_param("~gate_margin", 0.2)  # gate 계산 시 장애물 d 범위를 부풀리는 margin

        # 6. Track / wall safety 관련 파라미터
        self.track_half_width = rospy.get_param("~track_half_width", 1.2) # global path 기준 트랙 반폭
        self.wall_margin = rospy.get_param("~wall_margin", 0.2) # 벽에서 떨어져야 하는 최소 안전거리. 
        self.map_wall_margin = rospy.get_param("~map_wall_margin", 0.2) # /map 벽에서 확보할 최소 실제 거리
        self.fallback_collision_margin = rospy.get_param("~fallback_collision_margin", 0.10) # fallback 경로 선택 시 완화해서 사용하는 충돌 margin

        # 7. 회피 latch 관련 파라미터
        # latch: 회피가 시작되면 장애물 인식이 순간적으로 끊겨도 바로 global path로 복귀하지 않게 유지하는 상태
        self.avoidance_active = False # 현재 회피 latch가 켜져 있는지 저장
        self.locked_target_d = None # 회피 중 유지할 목표 lateral offset
        self.avoidance_start_s = None # 회피 latch가 시작된 s 위치
        self.avoidance_release_s = None # 이 s 위치를 지나면 회피 latch 해제

        self.avoid_trigger_s_min = rospy.get_param("~avoid_trigger_s_min", 0.1) # 회피 시작 판단에서 차량 바로 앞 제외 거리
        self.avoid_trigger_s_max = rospy.get_param("~avoid_trigger_s_max", 8.0) # 회피 시작 판단에서 앞쪽 최대 감지 거리
        self.avoid_release_buffer = rospy.get_param("~avoid_release_buffer", 0.90) # 장애물을 지난 뒤 latch를 더 유지할 거리
        self.locked_target_weight = rospy.get_param("~locked_target_weight", 90.0) # 기존 회피 방향을 유지하도록 만드는 cost 가중치
        self.global_block_d_margin = rospy.get_param("~global_block_d_margin", 0.50)  # global path를 막는 장애물인지 판단할 d margin
        self.avoidance_off_target_d_threshold = rospy.get_param("~avoidance_off_target_d_threshold", 0.70) # 이 값보다 target_d가 작으면 회피 경로가 아니라고 판단
        self.lock_replan_target_threshold = rospy.get_param("~lock_replan_target_threshold", 1.2) # latch 중에도 더 안전한 방향이 있으면 lock 방향을 바꾸는 기준. 값을 키우면: 기존 회피 방향을 더 강하게 유지, 방향 전환이 둔해짐

        # LiDAR 직접 장애물 생성 파라미터 (follow_the_gap.py와 같은 입력 구조)
        self.lidar_fov_deg = rospy.get_param("~lidar_fov_deg", 180.0)
        self.lidar_max_range = rospy.get_param("~lidar_max_range", 8.0)
        self.lidar_cluster_min_points = int(rospy.get_param("~lidar_cluster_min_points", 3))
        self.lidar_cluster_max_gap = rospy.get_param("~lidar_cluster_max_gap", 0.12)
        self.lidar_obstacle_max_size = rospy.get_param("~lidar_obstacle_max_size", 0.5)
        self.lidar_obstacle_size_tolerance = rospy.get_param("~lidar_obstacle_size_tolerance", 0.10)
        self.wall_filter_enabled = rospy.get_param("~wall_filter_enabled", True)
        self.map_occupied_threshold = rospy.get_param("~map_occupied_threshold", 45)
        self.wall_filter_radius = rospy.get_param("~wall_filter_radius", 0.20)
        self.treat_unknown_as_wall = rospy.get_param("~treat_unknown_as_wall", False)

        # --- Subscribers & Publishers (REAL/NANO) ---
        rospy.Subscriber(
            self.global_topic,
            WpntArray,
            self.gb_cb
        )
        rospy.Subscriber(
            self.obstacle_topic,
            ObstacleArray,
            self.obstacle_callback,
            queue_size=1
        )
        if self.use_internal_lidar:
            rospy.Subscriber(self.scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber(
            self.pose_topic,
            PoseWithCovarianceStamped,
            self.pose_callback
        )
        rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.speed_callback
        )

        self.mrks_pub = rospy.Publisher(
            "/planner/avoidance/markers",
            MarkerArray,
            queue_size=10
        )
        self.evasion_pub = rospy.Publisher(
            self.avoidance_output_topic,
            OTWpntArray,
            queue_size=10
        )
        self.dynamic_obstacle_marker_pub = rospy.Publisher(
            "/planner/dynamic_obstacle_markers",
            MarkerArray,
            queue_size=10
        )
        self.mode_pub = rospy.Publisher(
            "/planner/avoidance/mode",
            String,
            queue_size=10
        )
        self.overtake_state_pub = rospy.Publisher(
            "/planner/avoidance/overtake_state",
            String,
            queue_size=10
        )
        self.overtake_marker_pub = rospy.Publisher(
            "/planner/avoidance/overtake_markers",
            MarkerArray,
            queue_size=10
        )
        # detect_2.py가 장애물의 전방 거리를 판정할 수 있도록
        # 현재 차량의 Frenet s/d를 전달한다.
        self.odom_frenet_pub = rospy.Publisher(
            "/odom_frenet",
            Odometry,
            queue_size=10
        )

        self.converter = self.initialize_converter()
        self.rate = rospy.Rate(20)

        rospy.logwarn(
            f"[REAL Lattice Planner] "
            f"global={self.global_topic}, "
            f"pose={self.pose_topic}, "
            f"odom={self.odom_topic}, "
            f"obstacle_source={'lidar:' + self.scan_topic if self.use_internal_lidar else self.obstacle_topic}, "
            f"avoidance_out={self.avoidance_output_topic}(OTWpntArray)"
        )

    @staticmethod
    def resolve_array_element_class(array_cls, slot_name):
        """OTWpntArray.wpnts의 실제 message class를 런타임에 찾는다."""
        slots = list(array_cls.__slots__)
        slot_types = list(array_cls._slot_types)

        idx = slots.index(slot_name)
        type_name = slot_types[idx]

        if type_name.endswith("[]"):
            type_name = type_name[:-2]

        cls = roslib.message.get_message_class(type_name)

        if cls is None:
            raise RuntimeError(
                f"message class not found: {type_name}"
            )

        return cls

    def make_ot_wpnt(self, idx, x, y, s, d, vx):
        """실차 controller가 받는 OT waypoint를 생성한다."""
        msg = self.ot_wpnt_cls()

        values = {
            "id": int(idx),
            "x_m": float(x),
            "y_m": float(y),
            "s_m": float(s),
            "d_m": float(d),
            "vx_mps": float(vx),
        }

        for name, value in values.items():
            if hasattr(msg, name):
                setattr(msg, name, value)

        return msg

    def new_avoidance_array(self):
        msg = OTWpntArray()

        if hasattr(msg, "header"):
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"

        return msg

    def map_callback(self, msg):
        """1~5번 구성의 /map을 저장해 LiDAR의 트랙 벽 점을 제거한다."""
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
        self.map_data = np.asarray(msg.data, dtype=np.int16).reshape(
            (self.map_height, self.map_width)
        )
        wall_mask = self.map_data >= self.map_occupied_threshold
        if self.treat_unknown_as_wall:
            wall_mask |= self.map_data < 0
        self.map_wall_clearance = distance_transform_edt(
            ~wall_mask
        ) * self.map_resolution

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
        return (
            int(math.floor((c * dx - s * dy) / self.map_resolution)),
            int(math.floor((s * dx + c * dy) / self.map_resolution))
        )

    def is_wall_cell(self, mx, my):
        if mx < 0 or my < 0 or mx >= self.map_width or my >= self.map_height:
            return True
        value = self.map_data[my, mx]
        return self.treat_unknown_as_wall if value < 0 else value >= self.map_occupied_threshold

    def is_near_map_wall(self, x, y):
        if not self.has_map():
            return False
        center_x, center_y = self.world_to_map(x, y)
        radius_cells = int(math.ceil(self.wall_filter_radius / self.map_resolution))
        for my in range(center_y - radius_cells, center_y + radius_cells + 1):
            for mx in range(center_x - radius_cells, center_x + radius_cells + 1):
                if (mx - center_x) ** 2 + (my - center_y) ** 2 <= radius_cells ** 2:
                    if self.is_wall_cell(mx, my):
                        return True
        return False

    def get_path_min_wall_clearance(self, path_xy):
        """OccupancyGrid 기준 후보 경로의 최소 벽 간격을 반환한다."""
        if not self.has_map() or self.map_wall_clearance is None:
            return float('inf')

        min_clearance = float('inf')
        for x, y in path_xy:
            mx, my = self.world_to_map(x, y)
            if mx < 0 or my < 0 or mx >= self.map_width or my >= self.map_height:
                return 0.0
            min_clearance = min(
                min_clearance,
                float(self.map_wall_clearance[my, mx])
            )
        return min_clearance

    @staticmethod
    def scalar(value):
        return float(np.asarray(value).reshape(-1)[0])

    def signed_track_s_delta(self, from_s, to_s):
        """랩 경계를 포함한 장애물의 signed Frenet s 이동량."""
        if self.gb_max_s is None or self.gb_max_s <= 0.0:
            return float(to_s) - float(from_s)
        max_s = float(self.gb_max_s)
        return float(
            (float(to_s) - float(from_s) + 0.5 * max_s) % max_s
            - 0.5 * max_s
        )

    def point_wall_clearance_sd(self, s_value, d_value):
        """동적 오검출 제거용 map 벽 거리. 정적 경로 평가는 건드리지 않는다."""
        if (
            not self.has_map()
            or self.map_wall_clearance is None
            or self.converter is None
            or self.gb_max_s is None
        ):
            return float("inf")

        xy = self.converter.get_cartesian(
            np.asarray([float(s_value) % self.gb_max_s]),
            np.asarray([float(d_value)])
        )
        if np.any(~np.isfinite(xy)):
            return 0.0

        mx, my = self.world_to_map(float(xy[0, 0]), float(xy[1, 0]))
        if mx < 0 or my < 0 or mx >= self.map_width or my >= self.map_height:
            return 0.0
        return float(self.map_wall_clearance[my, mx])

    def obstacle_track_id(self, obstacle):
        return self.current_obstacle_track_ids.get(id(obstacle))

    @staticmethod
    def obstacle_speed(obstacle):
        value = getattr(obstacle, "v_s", getattr(obstacle, "vs", 0.0))
        return float(np.asarray(value).reshape(-1)[0])

    def reset_dynamic_follow_state(self):
        self.follow_target_track_id = None
        self.follow_target_cache = None
        self.follow_target_last_seen = rospy.Time(0)
        self.follow_speed_command = None
        self.follow_speed_update_time = rospy.Time(0)
        self.follow_filtered_gap = None
        self.follow_filtered_opponent_speed = None

    def update_motion_tracks(self, msg):
        """raw obstacle 위치열로 고속에서도 안정적인 동적 차량을 확정한다."""
        stamp = getattr(getattr(msg, "header", None), "stamp", rospy.Time())
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
        now = stamp.to_sec()

        self.current_obstacle_track_ids = {}
        self.current_dynamic_object_ids = set()
        if self.gb_max_s is None or self.gb_max_s <= 0.0:
            return

        stale_ids = [
            track_id for track_id, track in self.motion_tracks.items()
            if now - track["last_time"] > self.motion_track_timeout
        ]
        for track_id in stale_ids:
            self.motion_tracks.pop(track_id, None)

        unused_ids = set(self.motion_tracks.keys())
        observations = list(getattr(msg, "obstacles", []))
        observations.sort(
            key=lambda obs: self.forward_s_distance(
                self.cur_s, getattr(obs, "s_center", 0.0)
            )
        )

        for obs in observations:
            obs_s = self.scalar(getattr(obs, "s_center", 0.0)) % self.gb_max_s
            obs_d = self.scalar(getattr(obs, "d_center", 0.0))
            best_id = None
            best_cost = float("inf")

            for track_id in list(unused_ids):
                track = self.motion_tracks[track_id]
                dt = max(now - track["last_time"], 0.0)
                ds = abs(self.signed_track_s_delta(track["last_s"], obs_s))
                dd = abs(obs_d - track["last_d"])
                adaptive_s_gate = min(
                    self.motion_match_s_max,
                    0.30 + 8.0 * dt
                )
                if ds > adaptive_s_gate or dd > self.motion_match_d:
                    continue
                cost = ds + 1.2 * dd
                if cost < best_cost:
                    best_cost = cost
                    best_id = track_id

            if best_id is None:
                best_id = self.next_motion_track_id
                self.next_motion_track_id += 1
                self.motion_tracks[best_id] = {
                    "history": deque(maxlen=60),
                    "last_s": obs_s,
                    "last_d": obs_d,
                    "unwrapped_s": obs_s,
                    "last_time": now,
                    "speed": 0.0,
                    "confirmed_dynamic": False,
                    "wall_like": False,
                }
            else:
                unused_ids.discard(best_id)

            track = self.motion_tracks[best_id]
            if track["history"]:
                track["unwrapped_s"] += self.signed_track_s_delta(
                    track["last_s"], obs_s
                )
            else:
                track["unwrapped_s"] = obs_s

            # 같은 stamp가 반복 발행된 경우 속도 표본으로 중복 사용하지 않는다.
            if not track["history"] or now - track["history"][-1][0] > 1e-4:
                track["history"].append(
                    (now, track["unwrapped_s"], obs_d)
                )
            while (
                len(track["history"]) > 2
                and now - track["history"][0][0] > self.motion_history_time
            ):
                track["history"].popleft()

            track["last_s"] = obs_s
            track["last_d"] = obs_d
            track["last_time"] = now
            track["wall_like"] = (
                self.point_wall_clearance_sd(obs_s, obs_d)
                < self.dynamic_wall_reject_clearance
            )

            history = list(track["history"])
            interval_speeds = []
            for previous, current in zip(history[:-1], history[1:]):
                dt = current[0] - previous[0]
                if dt > 1e-3:
                    interval_speeds.append(
                        (current[1] - previous[1]) / dt
                    )

            duration = (
                history[-1][0] - history[0][0]
                if len(history) >= 2 else 0.0
            )
            net_move = (
                history[-1][1] - history[0][1]
                if len(history) >= 2 else 0.0
            )
            lateral_move = (
                history[-1][2] - history[0][2]
                if len(history) >= 2 else 0.0
            )
            average_speed = net_move / duration if duration > 1e-3 else 0.0
            average_vd = lateral_move / duration if duration > 1e-3 else 0.0
            median_speed = (
                float(np.median(interval_speeds))
                if interval_speeds else 0.0
            )
            direction_consistency = (
                float(np.mean(np.asarray(interval_speeds) > 0.15))
                if interval_speeds else 0.0
            )

            if (
                not track["confirmed_dynamic"]
                and self.has_map()
                and not track["wall_like"]
                and duration >= self.motion_confirm_time
                and net_move >= self.motion_confirm_distance
                and average_speed >= self.dynamic_speed_threshold
                and median_speed >= self.dynamic_speed_threshold
                and direction_consistency >= self.motion_direction_consistency
                and abs(average_vd) <= self.motion_max_abs_vd
                and average_speed <= self.motion_max_abs_vs
            ):
                track["confirmed_dynamic"] = True

            dynamic_now = (
                self.dynamic_avoidance_enabled
                and track["confirmed_dynamic"]
            )
            if dynamic_now:
                robust_speed = 0.5 * average_speed + 0.5 * median_speed
                track["speed"] = float(np.clip(
                    robust_speed, 0.0, self.motion_max_abs_vs
                ))
                self.current_dynamic_object_ids.add(id(obs))
            else:
                track["speed"] = 0.0

            if hasattr(obs, "vs"):
                obs.vs = track["speed"]
            if hasattr(obs, "v_s"):
                obs.v_s = track["speed"]
            if hasattr(obs, "vd"):
                obs.vd = 0.0
            if hasattr(obs, "is_visible"):
                obs.is_visible = True
            if hasattr(obs, "is_static"):
                obs.is_static = not dynamic_now

            self.current_obstacle_track_ids[id(obs)] = best_id
            rospy.loginfo_throttle(
                0.5,
                "[DYN_TRACK] s=%.2f d=%.2f vs=%.2f age=%.2f "
                "move=%.2f consistency=%.2f wall=%s dyn=%s",
                obs_s, obs_d, track["speed"], duration, net_move,
                direction_consistency, str(track["wall_like"]), str(dynamic_now)
            )

    def obstacle_callback(self, msg):
        """detect.py raw obstacle을 저장하고 내부 동적 판정을 갱신한다."""
        if self.use_internal_lidar:
            return
        self.update_motion_tracks(msg)
        self.obs = msg

    def make_lidar_obstacle(self, obstacle_id, world_points):
        """map 좌표 LiDAR 군집 하나를 lattice가 쓰는 Frenet Obstacle로 변환한다."""
        sd_points = []
        cur_s = self.scalar(self.cur_s)
        max_s = float(self.gb_max_s)
        for x, y in world_points:
            frenet = self.converter.get_frenet(float(x), float(y))
            point_s = self.scalar(frenet[0])
            point_d = self.scalar(frenet[1])
            # 랩 경계 부근에서도 min/max가 트랙 전체로 벌어지지 않게 한다.
            rel_s = (point_s - cur_s + 0.5 * max_s) % max_s - 0.5 * max_s
            sd_points.append((rel_s, point_d))

        rel_s_values = np.asarray([point[0] for point in sd_points])
        d_values = np.asarray([point[1] for point in sd_points])
        obs = Obstacle()
        obs.id = int(obstacle_id)
        obs.s_start = float((cur_s + np.min(rel_s_values)) % max_s)
        obs.s_end = float((cur_s + np.max(rel_s_values)) % max_s)
        obs.s_center = float((cur_s + np.mean(rel_s_values)) % max_s)
        obs.d_right = float(np.min(d_values))
        obs.d_left = float(np.max(d_values))
        obs.d_center = float(np.mean(d_values))
        obs.size = float(max(np.ptp(rel_s_values), np.ptp(d_values), 0.05))
        obs.vs = 0.0
        obs.vd = 0.0
        obs.is_static = True
        obs.is_visible = True
        return obs

    def scan_callback(self, scan):
        """opponent_track.launch 없이 /scan을 직접 정적 ObstacleArray로 만든다."""
        if (
            self.converter is None or
            self.gb_max_s is None or
            not self.pose_received or
            (self.wall_filter_enabled and not self.has_map())
        ):
            rospy.logwarn_throttle(2.0, "[Lattice LiDAR] Waiting for global path, pose and map.")
            return

        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        valid = (
            np.isfinite(ranges) &
            (ranges >= max(scan.range_min, 1e-3)) &
            (ranges <= min(scan.range_max, self.lidar_max_range)) &
            (np.abs(angles) <= math.radians(self.lidar_fov_deg) * 0.5)
        )

        clusters = []
        current = []
        for index in np.flatnonzero(valid):
            distance = float(ranges[index])
            angle = float(angles[index])
            heading = self.cur_yaw + angle
            world_point = np.array([
                self.vehicle_x + distance * math.cos(heading),
                self.vehicle_y + distance * math.sin(heading)
            ])

            if self.wall_filter_enabled and self.is_near_map_wall(*world_point):
                if current:
                    clusters.append(current)
                    current = []
                continue

            if current:
                previous_index, previous_point = current[-1]
                if index != previous_index + 1 or np.linalg.norm(world_point - previous_point) > self.lidar_cluster_max_gap:
                    clusters.append(current)
                    current = []
            current.append((int(index), world_point))
        if current:
            clusters.append(current)

        max_width = math.sqrt(2.0) * self.lidar_obstacle_max_size + self.lidar_obstacle_size_tolerance
        msg = ObstacleArray()
        msg.header.stamp = scan.header.stamp if scan.header.stamp != rospy.Time() else rospy.Time.now()
        msg.header.frame_id = "map"
        for cluster in clusters:
            if len(cluster) < self.lidar_cluster_min_points:
                continue
            points = np.asarray([item[1] for item in cluster])
            if np.linalg.norm(points[-1] - points[0]) > max_width:
                continue
            obs = self.make_lidar_obstacle(len(msg.obstacles), points)
            rel_s = self.forward_s_distance(self.cur_s, obs.s_center)
            # 뒤쪽 물체와 트랙 바깥의 잔여 점은 플래너 입력에서 제외한다.
            if 0.0 < rel_s <= self.lidar_max_range and abs(obs.d_center) <= self.track_half_width:
                msg.obstacles.append(obs)
        self.obs = msg

    # 글로벌 웨이포인트 콜백 함수
    def gb_cb(self, data): 
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m, wpnt.s_m] for wpnt in data.wpnts]) 
        self.gb_wpnts = data 
        if self.gb_max_s is None: 
            self.gb_max_s = data.wpnts[-1].s_m 

    # 현재 속도 기반 lookahead 계산
    def get_dynamic_lookahead(self): 
        ld = self.m_map * self.cur_vs + self.q_map 
        return np.clip(ld, self.t_clip_min, self.t_clip_max) 

    # ==========================================
    # 🟢 모드 1. 정적 플래너 (평상시 완벽한 주행용)
    # ==========================================

    # 정적 장애물 회피용 후보 경로 생성 함수
    def generate_static_paths(self, start_s, start_d, aggressive=False) -> List[dict]:
        paths = []
        ld_dist = max(self.get_dynamic_lookahead() * 2.5, 3.0)
        offsets = np.linspace(-self.max_offset, self.max_offset, self.num_samples)

        for d_target in offsets:
            if aggressive:
                s_points = [
                    start_s,
                    start_s + ld_dist * 0.2,
                    start_s + ld_dist * 0.5,
                    start_s + ld_dist
                ]

                d_points = [
                    start_d,
                    (start_d + d_target) / 2,
                    d_target,
                    d_target
                ]
            else:
                s_points = [
                    start_s,
                    start_s + ld_dist * 0.2,
                    start_s + ld_dist * 0.5,
                    start_s + ld_dist
                ]

                d_points = [
                    start_d,
                    (start_d * 2 + d_target) / 3,
                    (start_d + d_target * 2) / 3,
                    d_target
                ]

            spatial_spline = Spline(s_points, d_points)
            s_range = np.arange(start_s + self.visual_start_offset, start_s + ld_dist, 0.15)
            d_range = np.clip(spatial_spline(s_range), -self.max_offset, self.max_offset)

            paths.append({'s': s_range, 'd': d_range, 'target_d': d_target})

        return paths
    
    # global path 또는 현재 주행 lane을 막는 앞쪽 장애물을 찾는다.
    def find_front_blocking_obstacle(self, obstacles):
        blocking_obs = None
        nearest_rel_s = float("inf")

        for obs in obstacles.obstacles:
            obs_rel_s = self.forward_s_distance(self.cur_s, obs.s_center)

            if not (self.avoid_trigger_s_min < obs_rel_s < self.avoid_trigger_s_max):
                continue

            obs_d_low = min(obs.d_left, obs.d_right) - self.collision_margin
            obs_d_high = max(obs.d_left, obs.d_right) + self.collision_margin

            blocks_global_path = (
                obs_d_low <= 0.0 <= obs_d_high
                or not (
                    obs_d_high < -self.global_block_d_margin
                    or obs_d_low > self.global_block_d_margin
                )
            )

            blocks_current_lane = obs_d_low <= self.cur_d <= obs_d_high

            if (blocks_global_path or blocks_current_lane) and obs_rel_s < nearest_rel_s:
                nearest_rel_s = obs_rel_s
                blocking_obs = obs

        return blocking_obs

    # 회피 latch 시작
    def start_avoidance_latch(self, best_path, blocking_obs):
        if best_path is None or blocking_obs is None:
            return

        self.avoidance_active = True
        self.locked_target_d = best_path.get("target_d", 0.0)
        self.avoidance_start_s = self.cur_s
        self.avoidance_release_s = (
            blocking_obs.s_end
            + self.rear_s_buffer
            + self.avoid_release_buffer
        ) % self.gb_max_s

        rospy.logwarn_throttle(
            0.5,
            f"[Lattice] avoidance LATCH ON: locked_target_d={self.locked_target_d:.2f}, "
            f"release_s={self.avoidance_release_s:.2f}"
        )

    # 회피 latch 해제 조건 검사
    def update_avoidance_latch_release(self):
        if not self.avoidance_active:
            return

        if self.avoidance_start_s is None or self.avoidance_release_s is None:
            self.reset_avoidance_latch()
            return

        traveled_s = self.forward_s_distance(self.avoidance_start_s, self.cur_s)
        release_dist = self.forward_s_distance(self.avoidance_start_s, self.avoidance_release_s)

        if traveled_s >= release_dist:
            rospy.logwarn_throttle(
                0.5,
                "[Lattice] avoidance LATCH OFF: obstacle passed"
            )
            self.reset_avoidance_latch()

    # 회피 latch 초기화
    def reset_avoidance_latch(self):
        self.avoidance_active = False
        self.locked_target_d = None
        self.avoidance_start_s = None
        self.avoidance_release_s = None

    # 정적 후보 경로들을 평가해서 best path 고르는 함수
    def score_static_paths(self, paths, obstacles, gate_center_d=None, gate_width=None, locked_target_d=None):

        valid_paths = []
        
        for path in paths: 
            collision = False 
            min_obs_dist = float('inf') 

            s_arr = path['s'] % self.gb_max_s 
            d_arr = path['d']

            # ==================================================
            # 1순위: /map 실제 벽 검사 최우선
            # Global path 기준 대칭 track_half_width 제한은 사용하지 않는다.
            # ==================================================
            resp = self.converter.get_cartesian(
                s_arr,
                d_arr
            )

            if np.any(np.isnan(resp)) or np.any(np.isinf(resp)):
                continue

            path_xy = np.column_stack(
                (resp[0], resp[1])
            )

            min_wall_clearance = \
                self.get_path_min_wall_clearance(
                    path_xy
                )

            # 실제 /map 벽에서 map_wall_margin 이상 떨어져야 함
            if min_wall_clearance < self.map_wall_margin:
                continue

            # ==================================================
            # 2. 장애물 영역 검사 → 걸리면 바로 탈락
            # ==================================================
            obstacle_hard_margin = self.collision_margin

            for i in range(len(s_arr)):
                curr_s = s_arr[i]
                curr_d = d_arr[i]

                for obs in obstacles.obstacles:
                    danger_start, danger_end = self.get_obstacle_s_interval(obs)

                    if not self.is_s_in_interval(curr_s, danger_start, danger_end):
                        continue

                    obs_d_low = min(obs.d_left, obs.d_right)
                    obs_d_high = max(obs.d_left, obs.d_right)

                    inflated_low = obs_d_low - obstacle_hard_margin
                    inflated_high = obs_d_high + obstacle_hard_margin

                    if inflated_low <= curr_d <= inflated_high:
                        collision = True
                        break

                    if curr_d < obs_d_low:
                        dist_to_obs = obs_d_low - curr_d
                    elif curr_d > obs_d_high:
                        dist_to_obs = curr_d - obs_d_high
                    else:
                        dist_to_obs = 0.0

                    min_obs_dist = min(min_obs_dist, dist_to_obs)

                if collision:
                    break

            if collision:
                continue

            # ==================================================
            # 4. 경로가 너무 급하게 꺾이는지 검사
            # ==================================================
            for j in range(1, len(path_xy) - 1):
                v1 = path_xy[j] - path_xy[j-1]
                v2 = path_xy[j+1] - path_xy[j]

                d1 = np.linalg.norm(v1)
                d2 = np.linalg.norm(v2)

                if d1 < 0.05 or d2 < 0.05:
                    collision = True
                    break

                angle = np.arccos(
                    np.clip(
                        np.dot(v1, v2) / (d1 * d2 + 1e-6),
                        -1.0,
                        1.0
                    )
                )

                if abs(angle) > 1.4:
                    collision = True
                    break

            if collision:
                continue

            valid_paths.append((path, min_obs_dist, min_wall_clearance))

        if valid_paths:
            best_path, min_cost = None, float('inf')

            W_CENTER = 1.0
            W_SMOOTH = 1.0
            W_OBS = 1.5
            W_WALL = 4.0

            W_GATE = 5.0

            use_gate = (
                gate_center_d is not None and
                gate_width is not None
            )

            for path, obs_dist, wall_clearance in valid_paths:
                obs_penalty = (1.0 / (obs_dist + 1e-3)) if obs_dist != float('inf') else 0.0
                wall_penalty = (
                    1.0 / max(wall_clearance, 1e-3)
                    if wall_clearance != float('inf')
                    else 0.0
                )

                if locked_target_d is not None:
                    cost = (
                        self.locked_target_weight * abs(path['target_d'] - locked_target_d)
                        + W_SMOOTH * abs(path['target_d'] - self.cur_d)
                        + W_OBS * obs_penalty
                        + W_WALL * wall_penalty
                    )
                elif use_gate:
                    cost = (
                        W_GATE * abs(path['target_d'] - gate_center_d)
                        + W_SMOOTH * abs(path['target_d'] - self.cur_d)
                        + W_OBS * obs_penalty
                        + W_WALL * wall_penalty
                    )
                else:
                    cost = (
                        W_CENTER * abs(path['target_d'])
                        + W_SMOOTH * abs(path['target_d'] - self.cur_d)
                        + W_OBS * obs_penalty
                        + W_WALL * wall_penalty
                    )

                if cost < min_cost:
                    min_cost, best_path = cost, path

            return best_path
        
        rospy.logwarn("[Lattice] All paths blocked! (Static Fallback)")
        return self.get_fallback_path(paths)

    # ==========================================
    # 🔴 모드 2. 동적 플래너 (움직이는 차 회피용)
    # ==========================================
    # 추가로 시간 배열 t와 목표 속도 target_v가 들어간다.
    def generate_dynamic_paths(self, start_s, start_d) -> List[dict]: 
        paths = [] 
        # 🔥 여기도 똑같이 길이 증가
        ld_dist = max(self.get_dynamic_lookahead() * 2.0, 2.0)  
        offsets = np.linspace(-self.max_offset, self.max_offset, self.num_samples) 
        
        target_speeds = [self.cur_vs + 3.0, max(self.cur_vs, 1.5), max(self.cur_vs * 0.5, 0.5)]
        s_range_local = np.arange(0, ld_dist, 0.2)
        s_range_global = start_s + s_range_local
        
        for d_target in offsets: 
            s_points = [start_s, start_s + ld_dist * 0.2, start_s + ld_dist * 0.5, start_s + ld_dist] 
            
            # 🔥 부드러운 S자 조향 제어점 적용
            d_points = [start_d, (start_d * 2 + d_target) / 3, (start_d + d_target * 2) / 3, d_target] 
            
            spatial_spline = Spline(s_points, d_points) 
            d_range = np.clip(spatial_spline(s_range_global), -self.max_offset, self.max_offset)
            
            for v_target in target_speeds:
                paths.append({
                    's': s_range_global, 'd': d_range, 't': s_range_local / v_target, 
                    'target_d': d_target, 'target_v': v_target
                }) 
        return paths

    # 동적 후보 경로들을 평가해서 best path 고르는 함수
    # 정적 검사와 거의 같지만, 장애물의 현재 위치만 보는 게 아니라 v_s를 이용해 미래 s 위치를 예측
    def score_dynamic_paths(self, paths, obstacles):
        valid_paths = []

        for path in paths:
            # 1. 현재 프레임 기준 벽/장애물 안전검사 먼저 수행
            safe, base_obs_dist, reason = self.check_path_safety(
                path,
                obstacles,
                obstacle_margin=self.collision_margin
            )

            if not safe:
                continue

            s_arr = path["s"] % self.gb_max_s
            d_arr = path["d"]
            t_arr = path["t"]

            # 2. Cartesian 변환 및 급격한 꺾임 검사
            resp = self.converter.get_cartesian(s_arr, d_arr)

            if np.any(np.isnan(resp)) or np.any(np.isinf(resp)):
                continue

            path_xy = np.column_stack((resp[0], resp[1]))

            geometry_collision = False

            for j in range(1, len(path_xy) - 1):
                v1 = path_xy[j] - path_xy[j - 1]
                v2 = path_xy[j + 1] - path_xy[j]

                d1 = np.linalg.norm(v1)
                d2 = np.linalg.norm(v2)

                if d1 < 0.05 or d2 < 0.05:
                    geometry_collision = True
                    break

                angle = np.arccos(
                    np.clip(
                        np.dot(v1, v2) / (d1 * d2 + 1e-6),
                        -1.0,
                        1.0
                    )
                )

                if abs(angle) > 1.4:
                    geometry_collision = True
                    break

            if geometry_collision:
                continue

            # 3. S-T 동적 장애물 검사
            # S-T: s축 위치와 시간 t를 같이 보는 충돌 검사
            dynamic_collision = False
            min_obs_dist = base_obs_dist

            for i in range(len(s_arr)):
                curr_s = s_arr[i]
                curr_d = d_arr[i]
                curr_t = t_arr[i] + self.dynamic_time_buffer

                for obs in obstacles.obstacles:
                    # 동적 장애물만 미래 예측 검사에 사용
                    if not self.is_dynamic_obstacle(obs):
                        continue

                    obs_speed = getattr(obs, "v_s", getattr(obs, "vs", 0.0))
                    obs_speed = float(np.asarray(obs_speed).reshape(-1)[0])

                    obs_s_center = getattr(obs, "s_center", 0.0)
                    predicted_s_center = (obs_s_center + obs_speed * curr_t) % self.gb_max_s

                    obs_length = max(
                        abs(
                            getattr(obs, "s_end", obs_s_center)
                            - getattr(obs, "s_start", obs_s_center)
                        ),
                        0.8
                    )

                    danger_start = predicted_s_center - obs_length * 0.5 - self.dynamic_front_s_buffer
                    danger_end = predicted_s_center + obs_length * 0.5 + self.dynamic_rear_s_buffer

                    obs_d_low = min(obs.d_left, obs.d_right) - self.collision_margin
                    obs_d_high = max(obs.d_left, obs.d_right) + self.collision_margin

                    in_s_range = self.is_s_in_interval(curr_s, danger_start, danger_end)
                    in_d_range = obs_d_low <= curr_d <= obs_d_high

                    if in_s_range and in_d_range:
                        dynamic_collision = True
                        break

                    if curr_d < obs_d_low:
                        dist_to_obs = obs_d_low - curr_d
                    elif curr_d > obs_d_high:
                        dist_to_obs = curr_d - obs_d_high
                    else:
                        dist_to_obs = 0.0

                    min_obs_dist = min(min_obs_dist, dist_to_obs)

                if dynamic_collision:
                    break

            if dynamic_collision:
                continue

            valid_paths.append((path, min_obs_dist))

        if valid_paths:
            best_path = None
            min_cost = float("inf")

            W_CENTER = 4.5
            W_SMOOTH = 0.8
            W_OBS = 6.0
            W_SPEED = 1.5
            W_WALL = 2.0

            d_limit = self.track_half_width - self.wall_margin

            for path, obs_dist in valid_paths:
                obs_penalty = (1.0 / (obs_dist + 1e-3)) if obs_dist != float("inf") else 0.0
                wall_clearance = max(d_limit - abs(path["target_d"]), 1e-3)
                wall_penalty = 1.0 / wall_clearance

                cost = (
                    W_CENTER * abs(path["target_d"])
                    + W_SMOOTH * abs(path["target_d"] - self.cur_d)
                    + W_OBS * obs_penalty
                    + W_SPEED * (1.0 / (path["target_v"] + 0.1))
                    + W_WALL * wall_penalty
                )

                if cost < min_cost:
                    min_cost = cost
                    best_path = path

            return best_path

        rospy.logwarn("[Lattice] All dynamic avoidance paths blocked. Switching to FOLLOW fallback.")
        follow_obs, follow_rel_s = self.find_nearest_dynamic_blocking_obstacle(obstacles)

        if follow_obs is not None:
            follow_path = self.make_dynamic_follow_path(follow_obs)

            if follow_path is not None:
                return follow_path

        rospy.logerr_throttle(
            0.5,
            "[Lattice] FOLLOW fallback failed. Publish no dynamic path."
        )

        return None
    
    # 현재 s에서 target_s까지 주행 방향 기준 거리 계산
    def forward_s_distance(self, from_s, target_s):
        if self.gb_max_s is None or self.gb_max_s <= 0.0:
            return 0.0

        from_s = float(np.asarray(from_s).reshape(-1)[0])
        target_s = float(np.asarray(target_s).reshape(-1)[0])
        gb_max_s = float(np.asarray(self.gb_max_s).reshape(-1)[0])

        return (target_s - from_s + gb_max_s) % gb_max_s

    # s 좌표가 트랙 끝에서 0으로 넘어가는 경우까지 포함해서 구간 내부 여부 검사
    def is_s_in_interval(self, curr_s, start_s, end_s):
        if self.gb_max_s is None or self.gb_max_s <= 0.0:
            return start_s <= curr_s <= end_s

        curr_s = curr_s % self.gb_max_s
        start_s = start_s % self.gb_max_s
        end_s = end_s % self.gb_max_s

        if start_s <= end_s:
            return start_s <= curr_s <= end_s

        return curr_s >= start_s or curr_s <= end_s

    # 장애물의 기본 s 위험 구간 생성
    def get_obstacle_s_interval(self, obs, front_buffer=None, rear_buffer=None):
        if front_buffer is None:
            front_buffer = self.front_s_buffer
        if rear_buffer is None:
            rear_buffer = self.rear_s_buffer

        s_start = getattr(obs, "s_start", getattr(obs, "s_center", 0.0))
        s_end = getattr(obs, "s_end", getattr(obs, "s_center", 0.0))
        s_center = getattr(obs, "s_center", (s_start + s_end) * 0.5)

        raw_len = abs(s_end - s_start)

        # detect/tracker가 길이를 거의 0으로 줄 때를 보정
        if raw_len < 0.05:
            raw_len = 0.5
            s_start = s_center - raw_len * 0.5
            s_end = s_center + raw_len * 0.5

        danger_start = s_start - front_buffer
        danger_end = s_end + rear_buffer

        return danger_start, danger_end

    # 정적/기본 안전검사: 벽과 현재 장애물 영역을 먼저 탈락
    def check_path_safety(self, path, obstacles, obstacle_margin=None, check_track_limit=True):
        if obstacle_margin is None:
            obstacle_margin = self.collision_margin

        if self.gb_max_s is None or self.gb_max_s <= 0.0:
            return False, 0.0, "no_global_s"

        s_arr = path["s"] % self.gb_max_s
        d_arr = path["d"]

        if check_track_limit:

            d_limit = (
                self.track_half_width
                - self.wall_margin
            )

            if d_limit <= 0.0:

                rospy.logwarn_throttle(
                    1.0,
                    f"[Lattice] invalid d_limit="
                    f"{d_limit:.2f}. "
                    f"track_half_width="
                    f"{self.track_half_width:.2f}, "
                    f"wall_margin="
                    f"{self.wall_margin:.2f}"
                )

                return (
                    False,
                    0.0,
                    "invalid_wall_limit"
                )

            if np.any(
                np.abs(d_arr) > d_limit
            ):
                return (
                    False,
                    0.0,
                    "wall"
                )

        min_obs_dist = float("inf")

        for i in range(len(s_arr)):
            curr_s = s_arr[i]
            curr_d = d_arr[i]

            for obs in obstacles.obstacles:
                danger_start, danger_end = self.get_obstacle_s_interval(obs)

                if not self.is_s_in_interval(curr_s, danger_start, danger_end):
                    continue

                obs_d_low = min(obs.d_left, obs.d_right)
                obs_d_high = max(obs.d_left, obs.d_right)

                inflated_low = obs_d_low - obstacle_margin
                inflated_high = obs_d_high + obstacle_margin

                if inflated_low <= curr_d <= inflated_high:
                    return False, 0.0, "obstacle"

                if curr_d < obs_d_low:
                    dist_to_obs = obs_d_low - curr_d
                elif curr_d > obs_d_high:
                    dist_to_obs = curr_d - obs_d_high
                else:
                    dist_to_obs = 0.0

                min_obs_dist = min(min_obs_dist, dist_to_obs)

        return True, min_obs_dist, "safe"

    # 동적 장애물인지 판단
    def is_dynamic_obstacle(self, obs):
        if not self.dynamic_avoidance_enabled:
            return False

        obs_is_visible = getattr(obs, "is_visible", True)
        if not obs_is_visible:
            return False

        # raw detect.py를 사용할 때는 플래너 내부에서 충분히 확인된
        # motion track만 동적으로 인정한다.
        if id(obs) in self.current_dynamic_object_ids:
            return True

        # 외부 tracker가 속도와 is_static을 채우는 구성도 호환한다.
        obs_speed = getattr(obs, "v_s", getattr(obs, "vs", 0.0))
        obs_speed = float(np.asarray(obs_speed).reshape(-1)[0])

        obs_is_static = getattr(obs, "is_static", None)
        if obs_is_static is True:
            return False

        return abs(obs_speed) >= self.dynamic_speed_threshold

    # 동적 장애물이 실제로 내 주행 경로를 막는지 판단
    def is_dynamic_blocking_obstacle(self, obs):
        if not self.is_dynamic_obstacle(obs):
            return False

        if (
            self.point_wall_clearance_sd(obs.s_center, obs.d_center)
            < self.dynamic_wall_reject_clearance
        ):
            return False

        obs_rel_s = self.forward_s_distance(self.cur_s, obs.s_center)

        if not (self.dynamic_trigger_s_min < obs_rel_s < self.dynamic_trigger_s_max):
            return False

        obs_d_low = min(obs.d_left, obs.d_right) - self.dynamic_block_d_margin
        obs_d_high = max(obs.d_left, obs.d_right) + self.dynamic_block_d_margin

        blocks_global_path = obs_d_low <= 0.0 <= obs_d_high
        blocks_current_lane = obs_d_low <= self.cur_d <= obs_d_high

        return blocks_global_path or blocks_current_lane

    # 현재 프레임에서 동적 모드로 들어갈지 판단
    def should_enter_dynamic_mode(self, obstacles):

        if not self.dynamic_avoidance_enabled:
            return False
        
        nearest_dynamic = None
        nearest_rel_s = float("inf")

        for obs in obstacles.obstacles:
            if not self.is_dynamic_blocking_obstacle(obs):
                continue

            obs_rel_s = self.forward_s_distance(self.cur_s, obs.s_center)
            obs_rel_s = float(np.asarray(obs_rel_s).reshape(-1)[0])

            if obs_rel_s < nearest_rel_s:
                nearest_rel_s = obs_rel_s
                nearest_dynamic = obs

        if nearest_dynamic is not None:
            obs_speed = getattr(nearest_dynamic, "v_s", getattr(nearest_dynamic, "vs", 0.0))
            obs_speed = float(np.asarray(obs_speed).reshape(-1)[0])

            d_left = float(np.asarray(nearest_dynamic.d_left).reshape(-1)[0])
            d_right = float(np.asarray(nearest_dynamic.d_right).reshape(-1)[0])

            rospy.logwarn_throttle(
                0.5,
                f"[Lattice] dynamic mode ON: rel_s={nearest_rel_s:.2f}, "
                f"obs_vs={obs_speed:.2f}, "
                f"d=({d_left:.2f}, {d_right:.2f})"
            )
            return True

        return False
    
    # 현재 차선/global path를 막는 가장 가까운 동적 장애물을 찾는다.
    def find_nearest_dynamic_blocking_obstacle(self, obstacles):
        nearest_obs = None
        nearest_rel_s = float("inf")

        for obs in obstacles.obstacles:
            if not self.is_dynamic_blocking_obstacle(obs):
                continue

            obs_rel_s = self.forward_s_distance(self.cur_s, obs.s_center)
            obs_rel_s = float(np.asarray(obs_rel_s).reshape(-1)[0])

            if obs_rel_s < nearest_rel_s:
                nearest_rel_s = obs_rel_s
                nearest_obs = obs

        return nearest_obs, nearest_rel_s

    def static_obstacle_view(self, obstacles):
        """동적 ON일 때만 확정 차량을 정적 planner 입력에서 분리한다."""
        if not self.dynamic_avoidance_enabled:
            return obstacles
        result = ObstacleArray()
        if hasattr(result, "header") and hasattr(obstacles, "header"):
            result.header = obstacles.header
        result.obstacles.extend([
            obs for obs in getattr(obstacles, "obstacles", [])
            if (
                not self.is_dynamic_obstacle(obs)
                and not self.matches_latched_follow_target(obs)
            )
        ])
        return result

    def follow_reacquire_motion_is_plausible(self, obs):
        """새 track의 최소 순방향 이동 증거가 있을 때만 앞차로 재연결한다."""
        track_id = self.obstacle_track_id(obs)
        track = self.motion_tracks.get(track_id)
        if track is None:
            return False

        history = list(track.get("history", []))
        if len(history) < 2:
            return False
        duration = history[-1][0] - history[0][0]
        if duration <= 1e-3:
            return False

        net_move = history[-1][1] - history[0][1]
        average_speed = net_move / duration
        return (
            net_move >= 0.03
            and average_speed >= self.dynamic_speed_threshold
            and average_speed <= self.motion_max_abs_vs
        )

    def matches_latched_follow_target(self, obs):
        """FOLLOW 중 track id가 바뀐 같은 앞차를 정적으로 되돌리지 않는다."""
        if (
            self.follow_target_cache is None
            or self.gb_max_s is None
            or self.gb_max_s <= 0.0
        ):
            return False

        age = (rospy.Time.now() - self.follow_target_last_seen).to_sec()
        if age < 0.0 or age > self.follow_target_lost_timeout:
            return False

        obs_s = self.scalar(obs.s_center) % self.gb_max_s
        obs_d = self.scalar(obs.d_center)
        if (
            self.point_wall_clearance_sd(obs_s, obs_d)
            < self.dynamic_wall_reject_clearance
        ):
            return False

        cached_speed = max(self.obstacle_speed(self.follow_target_cache), 0.0)
        predicted_s = (
            self.scalar(self.follow_target_cache.s_center)
            + cached_speed * age
        ) % self.gb_max_s
        predicted_d = self.scalar(self.follow_target_cache.d_center)
        ds = abs(self.signed_track_s_delta(predicted_s, obs_s))
        dd = abs(predicted_d - obs_d)

        cached_width = abs(
            self.scalar(self.follow_target_cache.d_left)
            - self.scalar(self.follow_target_cache.d_right)
        )
        obs_width = abs(
            self.scalar(obs.d_left) - self.scalar(obs.d_right)
        )
        width_matches = abs(cached_width - obs_width) <= 0.30

        return (
            ds <= self.follow_reacquire_s_max
            and dd <= self.follow_reacquire_d_max
            and width_matches
            and self.follow_reacquire_motion_is_plausible(obs)
        )

    def current_dynamic_follow_target(self, obstacles):
        """가장 가까운 앞차를 고정하고 짧은 인지 dropout도 이어서 추종한다."""
        nearest, nearest_rel_s = self.find_nearest_dynamic_blocking_obstacle(
            obstacles
        )
        now = rospy.Time.now()

        # 이미 FOLLOW 중인 차량은 track id로 먼저 찾는다. 차량이 GLOBAL
        # 경로를 벗어나면 한 프레임 흔들림에는 반응하지 않고, 설정 시간
        # 동안 계속 비켜 있을 때 FOLLOW를 해제한다.
        if self.follow_target_track_id is not None:
            for obs in getattr(obstacles, "obstacles", []):
                if self.obstacle_track_id(obs) != self.follow_target_track_id:
                    continue
                if not self.is_dynamic_obstacle(obs):
                    continue

                obs_d_low = (
                    min(self.scalar(obs.d_left), self.scalar(obs.d_right))
                    - self.follow_release_d_margin
                )
                obs_d_high = (
                    max(self.scalar(obs.d_left), self.scalar(obs.d_right))
                    + self.follow_release_d_margin
                )
                still_on_global = obs_d_low <= 0.0 <= obs_d_high

                if still_on_global:
                    self.follow_off_path_since = rospy.Time(0)
                else:
                    if self.follow_off_path_since == rospy.Time(0):
                        self.follow_off_path_since = now
                    elif (
                        now - self.follow_off_path_since
                    ).to_sec() >= self.follow_release_persist_time:
                        rospy.logwarn(
                            "[Lattice] DYNAMIC_FOLLOW -> GLOBAL: "
                            "target left global path"
                        )
                        self.reset_dynamic_follow_state()
                        return None, float("inf")

                self.follow_target_cache = deepcopy(obs)
                self.follow_target_last_seen = now
                rel_s = self.forward_s_distance(self.cur_s, obs.s_center)
                if not (0.0 <= rel_s < self.dynamic_release_s_max):
                    self.reset_dynamic_follow_state()
                    return None, float("inf")
                return obs, float(rel_s)

        if nearest is not None:
            track_id = self.obstacle_track_id(nearest)
            if (
                self.follow_target_track_id is not None
                and track_id != self.follow_target_track_id
            ):
                for obs in getattr(obstacles, "obstacles", []):
                    if (
                        self.obstacle_track_id(obs) == self.follow_target_track_id
                        and self.is_dynamic_blocking_obstacle(obs)
                    ):
                        nearest = obs
                        nearest_rel_s = self.forward_s_distance(
                            self.cur_s, obs.s_center
                        )
                        track_id = self.follow_target_track_id
                        break

            self.follow_target_track_id = track_id
            self.follow_target_cache = deepcopy(nearest)
            self.follow_target_last_seen = now
            self.follow_off_path_since = rospy.Time(0)
            return nearest, float(nearest_rel_s)

        if self.follow_target_cache is None:
            return None, float("inf")

        # LiDAR cluster가 갈라져 track id가 새로 생겨도 직전 앞차의 예측
        # 위치/폭과 일치하면 같은 차량으로 즉시 재연결한다. 이 과정에서
        # 한 프레임 STATIC_AVOID로 튀는 현상도 함께 막는다.
        for obs in getattr(obstacles, "obstacles", []):
            if not self.matches_latched_follow_target(obs):
                continue

            track_id = self.obstacle_track_id(obs)
            cached_speed = max(
                self.obstacle_speed(self.follow_target_cache),
                0.0
            )
            if track_id in self.motion_tracks:
                self.motion_tracks[track_id]["confirmed_dynamic"] = True
                self.motion_tracks[track_id]["speed"] = cached_speed
            if hasattr(obs, "vs"):
                obs.vs = cached_speed
            if hasattr(obs, "v_s"):
                obs.v_s = cached_speed
            if hasattr(obs, "is_static"):
                obs.is_static = False
            self.current_dynamic_object_ids.add(id(obs))
            self.follow_target_track_id = track_id
            self.follow_target_cache = deepcopy(obs)
            self.follow_target_last_seen = now
            self.follow_off_path_since = rospy.Time(0)
            rel_s = self.forward_s_distance(self.cur_s, obs.s_center)
            return obs, float(rel_s)

        if (
            self.follow_off_path_since != rospy.Time(0)
            and (now - self.follow_off_path_since).to_sec()
                >= self.follow_release_persist_time
        ):
            self.reset_dynamic_follow_state()
            return None, float("inf")

        age = (now - self.follow_target_last_seen).to_sec()
        if age < 0.0 or age > self.follow_target_lost_timeout:
            self.reset_dynamic_follow_state()
            return None, float("inf")

        predicted = deepcopy(self.follow_target_cache)
        advance = max(self.obstacle_speed(predicted), 0.0) * age
        for field in ("s_start", "s_end", "s_center"):
            if hasattr(predicted, field):
                setattr(
                    predicted,
                    field,
                    (self.scalar(getattr(predicted, field)) + advance)
                    % self.gb_max_s
                )
        rel_s = self.forward_s_distance(self.cur_s, predicted.s_center)
        if not (0.0 <= rel_s < self.dynamic_release_s_max):
            self.reset_dynamic_follow_state()
            return None, float("inf")
        return predicted, float(rel_s)

    def global_speed_at_s(self, s_value):
        if (
            not self.gb_wpnts.wpnts
            or self.gb_max_s is None
            or self.gb_max_s <= 0.0
        ):
            return max(float(self.cur_vs), 0.0)
        index = int(
            ((float(s_value) % self.gb_max_s) / self.gb_max_s)
            * len(self.gb_wpnts.wpnts)
        ) % len(self.gb_wpnts.wpnts)
        return max(float(self.gb_wpnts.wpnts[index].vx_mps), 0.0)
    
    # 동적 장애물 기준 좌우 추월 가능 공간을 계산한다.
    # 기준은 내 차와 장애물 사이의 종방향 거리가 아니라,
    # 동적 장애물의 좌우 d 방향으로 남아 있는 안전 공간이다.
    def get_dynamic_overtake_gap_width(self, dynamic_obs, obstacles):
        if dynamic_obs is None:
            return 0.0, 0.0, 0.0

        d_limit = self.track_half_width - self.wall_margin

        if d_limit <= 0.0:
            rospy.logwarn_throttle(
                1.0,
                f"[Lattice] invalid d_limit for dynamic overtake gap: {d_limit:.2f}"
            )
            return 0.0, 0.0, 0.0

        dyn_d_low = min(dynamic_obs.d_left, dynamic_obs.d_right) - self.collision_margin
        dyn_d_high = max(dynamic_obs.d_left, dynamic_obs.d_right) + self.collision_margin

        right_boundary = -d_limit
        left_boundary = d_limit

        dyn_s = getattr(dynamic_obs, "s_center", 0.0)

        for obs in obstacles.obstacles:
            if obs is dynamic_obs:
                continue

            obs_s = getattr(obs, "s_center", 0.0)

            # 동적 장애물과 거의 같은 s 구간에 있는 벽/정적 장애물만 좌우 공간 제한 요소로 사용한다.
            # 너무 앞이나 뒤의 장애물까지 넣으면 실제 추월 공간보다 과하게 좁게 판단될 수 있다.
            forward_gap = self.forward_s_distance(dyn_s, obs_s)
            backward_gap = self.forward_s_distance(obs_s, dyn_s)
            same_s_band_dist = min(forward_gap, backward_gap)

            if same_s_band_dist > self.dynamic_overtake_s_window:
                continue

            obs_d_low = min(obs.d_left, obs.d_right) - self.gate_margin
            obs_d_high = max(obs.d_left, obs.d_right) + self.gate_margin

            # 동적 장애물 오른쪽에 있는 장애물/벽이면 오른쪽 경계를 안쪽으로 당긴다.
            if obs_d_high <= dyn_d_low:
                right_boundary = max(right_boundary, obs_d_high)

            # 동적 장애물 왼쪽에 있는 장애물/벽이면 왼쪽 경계를 안쪽으로 당긴다.
            elif obs_d_low >= dyn_d_high:
                left_boundary = min(left_boundary, obs_d_low)

            # d 범위가 동적 장애물과 겹치는 다른 장애물은 이미 같은 통로를 막고 있다고 보고,
            # 좌우 gap 계산에는 별도 경계로 넣지 않는다. 실제 충돌 여부는 check_path_safety에서 다시 검사된다.

        right_gap = max(dyn_d_low - right_boundary, 0.0)
        left_gap = max(left_boundary - dyn_d_high, 0.0)
        best_gap = max(left_gap, right_gap)

        return best_gap, left_gap, right_gap

    # 동적 장애물이 앞에 있을 때, 동적 장애물과 양옆 벽/장애물 사이 공간이 좁으면 추종해야 하는지 판단한다.
    def should_follow_dynamic_obstacle_in_narrow_space(self, obstacles):
        follow_obs, follow_rel_s = self.find_nearest_dynamic_blocking_obstacle(obstacles)

        if follow_obs is None:
            return False, None, None, None

        best_gap, left_gap, right_gap = self.get_dynamic_overtake_gap_width(follow_obs, obstacles)

        if best_gap < self.dynamic_overtake_min_gap:
            rospy.logwarn_throttle(
                0.5,
                f"[Lattice] dynamic FOLLOW selected: narrow side gap, "
                f"best_gap={best_gap:.2f} < min_gap={self.dynamic_overtake_min_gap:.2f}, "
                f"left_gap={left_gap:.2f}, right_gap={right_gap:.2f}, "
                f"rel_s={follow_rel_s:.2f}"
            )
            return True, follow_obs, None, best_gap

        rospy.logwarn_throttle(
            0.5,
            f"[Lattice] dynamic OVERTAKE allowed: "
            f"best_gap={best_gap:.2f} >= min_gap={self.dynamic_overtake_min_gap:.2f}, "
            f"left_gap={left_gap:.2f}, right_gap={right_gap:.2f}, "
            f"rel_s={follow_rel_s:.2f}"
        )

        return False, follow_obs, None, best_gap

    # 동적 차량을 추월하지 않고 GLOBAL 형상과 1 m 간격으로 추종한다.
    def make_dynamic_follow_path(self, obstacle):
        if obstacle is None or self.gb_max_s is None or self.gb_max_s <= 0.0:
            return None

        raw_obs_speed = max(self.obstacle_speed(obstacle), 0.0)
        obs_rel_s = float(self.forward_s_distance(
            self.cur_s, obstacle.s_center
        ))
        obs_start = self.scalar(getattr(
            obstacle, "s_start", obstacle.s_center
        ))
        raw_bumper_gap = max(
            float(self.forward_s_distance(self.cur_s, obs_start))
            - self.follow_ego_front_offset,
            0.0
        )

        now = rospy.Time.now()
        dt = (now - self.follow_speed_update_time).to_sec()
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / 20.0

        # LiDAR bbox 경계와 내부 속도 추정이 흔들리더라도 1 m gap 제어에
        # 그대로 들어가지 않도록 거리와 앞차 속도를 각각 저역통과시킨다.
        if self.follow_filtered_gap is None:
            self.follow_filtered_gap = raw_bumper_gap
        else:
            alpha_gap = 1.0 - math.exp(
                -dt / max(self.follow_gap_filter_tau, 1e-3)
            )
            self.follow_filtered_gap += alpha_gap * (
                raw_bumper_gap - self.follow_filtered_gap
            )

        if self.follow_filtered_opponent_speed is None:
            self.follow_filtered_opponent_speed = raw_obs_speed
        else:
            alpha_speed = 1.0 - math.exp(
                -dt / max(self.follow_opponent_speed_filter_tau, 1e-3)
            )
            self.follow_filtered_opponent_speed += alpha_speed * (
                raw_obs_speed - self.follow_filtered_opponent_speed
            )

        bumper_gap = float(self.follow_filtered_gap)
        obs_speed = max(float(self.follow_filtered_opponent_speed), 0.0)
        gap_error = bumper_gap - self.follow_min_distance

        # 1 m +/- deadband 안에서는 정확히 앞차 속도를 사용한다. 범위를
        # 벗어난 오차만 제한된 보정속도로 더해 과도한 가감속을 막는다.
        if abs(gap_error) <= self.follow_gap_deadband:
            effective_gap_error = 0.0
        else:
            effective_gap_error = math.copysign(
                abs(gap_error) - self.follow_gap_deadband,
                gap_error
            )
        speed_correction = float(np.clip(
            self.follow_gap_kp * effective_gap_error,
            -self.follow_max_speed_correction,
            self.follow_max_speed_correction
        ))
        desired_speed = (
            obs_speed + speed_correction - self.follow_speed_margin
        )

        # 너무 가까울 때도 고정 50% 감속이나 즉시 정지를 걸지 않는다.
        # 앞차 속도보다 제한된 값만 낮춰 자연스럽게 간격을 다시 벌린다.
        close_gap = raw_bumper_gap <= self.follow_close_gap
        if close_gap:
            desired_speed = min(
                desired_speed,
                max(
                    obs_speed - self.follow_close_speed_reduction,
                    self.follow_min_speed
                )
            )

        global_speed = self.global_speed_at_s(self.cur_s)
        desired_speed = float(np.clip(
            desired_speed,
            self.follow_min_speed,
            min(self.follow_max_speed, global_speed)
        ))

        # 검출 속도와 간격 오차가 프레임마다 흔들려도 명령 속도가 튀지 않게 한다.
        if self.follow_speed_command is None:
            # FOLLOW 진입 첫 프레임도 현재 차속에서 시작한다. 앞차 목표속도로
            # 즉시 점프시키지 않고 아래의 가감속 제한을 다음 주기부터 적용한다.
            self.follow_speed_command = float(np.clip(
                float(self.cur_vs),
                self.follow_min_speed,
                min(self.follow_max_speed, global_speed)
            ))
        else:
            alpha = 1.0 - math.exp(
                -dt / max(self.follow_speed_tau, 1e-3)
            )
            filtered_command = self.follow_speed_command + alpha * (
                desired_speed - self.follow_speed_command
            )
            max_up = max(self.follow_accel_limit, 0.0) * dt
            max_down = max(self.follow_decel_limit, 0.0) * dt
            self.follow_speed_command = float(np.clip(
                filtered_command,
                self.follow_speed_command - max_down,
                self.follow_speed_command + max_up
            ))

        self.follow_speed_update_time = now
        target_v = float(np.clip(
            self.follow_speed_command,
            self.follow_min_speed,
            min(self.follow_max_speed, global_speed)
        ))

        # 고속 controller의 lookahead가 경로 끝을 넘지 않도록 GLOBAL 구간을
        # 최소 12 m, 또는 현재 속도의 2초분 중 큰 길이만큼 복사한다.
        path_length = max(
            self.follow_path_length,
            max(abs(float(self.cur_vs)), target_v) * 2.0
        )
        cur_s = self.scalar(self.cur_s)
        cur_d = self.scalar(self.cur_d)
        s_range = np.arange(
            cur_s,
            cur_s + path_length + 0.5 * self.follow_path_ds,
            self.follow_path_ds
        )

        if len(s_range) < 3:
            s_range = np.array([
                cur_s,
                cur_s + 0.3,
                cur_s + 0.6
            ])

        # 별도 추월 차선은 만들지 않는다. 정적 회피 직후라면 현재 d에서
        # 2.5 m 동안 부드럽게 d=0 GLOBAL로 합류한 뒤 GLOBAL을 그대로 따른다.
        s_rel = s_range - cur_s
        merge_phase = 0.5 - 0.5 * np.cos(
            math.pi * np.clip(
                s_rel / max(self.follow_global_merge_length, 1e-3),
                0.0,
                1.0
            )
        )
        d_range = cur_d * (1.0 - merge_phase)

        follow_path = {
            "s": s_range,
            "d": d_range,
            "target_d": 0.0,
            "target_v": target_v,
            "is_follow": True,
            "planner_mode": "DYNAMIC_FOLLOW",
            "bumper_gap": bumper_gap,
            "raw_bumper_gap": raw_bumper_gap,
            "opponent_speed": obs_speed,
            # RViz에서 접근 가능 구간과 목표 1 m 안전 구간을 서로 다른
            # 색으로 표시하기 위한 앞차 뒤쪽 Frenet 위치이다.
            "obstacle_rear_s": obs_start,
            "obstacle_d": self.scalar(obstacle.d_center),
        }

        rospy.logwarn_throttle(
            0.5,
            f"[Lattice] DYNAMIC_FOLLOW_GLOBAL: rel_s={obs_rel_s:.2f}, "
            f"gap_raw={raw_bumper_gap:.2f}, gap={bumper_gap:.2f}, "
            f"obs_vs_raw={raw_obs_speed:.2f}, obs_vs={obs_speed:.2f}, "
            f"target_v={target_v:.2f}, close={close_gap}"
        )

        return follow_path

    def reset_overtake_state(self, keep_follow_timer=False):
        """동적 추월 상태만 초기화한다. 정적 회피 latch에는 손대지 않는다."""
        self.overtake_phase = "FOLLOW"
        self.overtake_target_track_id = None
        self.overtake_target_cache = None
        self.overtake_target_last_seen = rospy.Time(0)
        self.overtake_locked_side = 0
        self.overtake_locked_d = None
        self.overtake_started_at = rospy.Time(0)
        self.overtake_gap_since = rospy.Time(0)
        self.overtake_candidate_side = 0
        self.overtake_candidate_d = None
        self.overtake_candidate_path = None
        self.overtake_reject_reason = "waiting_for_dynamic_target"
        self.overtake_shift_start_s = None
        self.overtake_shift_start_d = None
        self.overtake_shift_length = None
        self.overtake_return_start_s = None
        self.overtake_return_start_d = None
        self.overtake_return_length_locked = None
        self.overtake_locked_rollout = None
        if not keep_follow_timer:
            self.overtake_follow_since = rospy.Time(0)
            self.overtake_ready_track_id = None

    def overtake_bumper_gap(self, obstacle):
        if obstacle is None:
            return float("inf")
        obs_start = self.scalar(getattr(
            obstacle, "s_start", obstacle.s_center
        ))
        return max(
            float(self.forward_s_distance(self.cur_s, obs_start))
            - self.follow_ego_front_offset,
            0.0
        )

    def current_overtake_target(self, obstacles):
        """추월 중 상대 차량을 ID 변경과 짧은 dropout에도 유지한다."""
        if self.overtake_target_track_id is None:
            return None

        now = rospy.Time.now()
        for obs in getattr(obstacles, "obstacles", []):
            if self.obstacle_track_id(obs) == self.overtake_target_track_id:
                self.overtake_target_cache = deepcopy(obs)
                self.overtake_target_last_seen = now
                return obs

        if self.overtake_target_cache is None:
            return None

        age = (now - self.overtake_target_last_seen).to_sec()
        if age < 0.0 or age > self.overtake_target_lost_timeout:
            return None

        predicted = deepcopy(self.overtake_target_cache)
        advance = max(self.obstacle_speed(predicted), 0.0) * age
        for field in ("s_start", "s_end", "s_center"):
            if hasattr(predicted, field):
                setattr(
                    predicted,
                    field,
                    (self.scalar(getattr(predicted, field)) + advance)
                    % self.gb_max_s
                )

        # Exact ID가 사라졌더라도 예측 위치/횡위치/속도가 가까운 확정 동적
        # 물체를 같은 차량으로 재결합한다. 정적 물체는 후보에서 제외한다.
        predicted_s = self.scalar(predicted.s_center)
        predicted_d = self.scalar(predicted.d_center)
        predicted_v = max(self.obstacle_speed(predicted), 0.0)
        best_match = None
        best_cost = float("inf")
        for obs in getattr(obstacles, "obstacles", []):
            if not self.is_dynamic_obstacle(obs):
                continue
            ds = abs(self.signed_track_s_delta(predicted_s, obs.s_center))
            dd = abs(self.scalar(obs.d_center) - predicted_d)
            dv = abs(max(self.obstacle_speed(obs), 0.0) - predicted_v)
            if (
                ds > self.overtake_reacquire_s_max
                or dd > self.overtake_reacquire_d_max
                or dv > self.overtake_reacquire_speed_max
            ):
                continue
            cost = ds + 1.5 * dd + 0.15 * dv
            if cost < best_cost:
                best_cost = cost
                best_match = obs

        if best_match is not None:
            new_track_id = self.obstacle_track_id(best_match)
            if new_track_id is not None:
                self.overtake_target_track_id = new_track_id
            self.overtake_target_cache = deepcopy(best_match)
            self.overtake_target_last_seen = now
            rospy.logwarn_throttle(
                0.5,
                "[Lattice] OVERTAKE target reacquired after track-id change"
            )
            return best_match

        return predicted

    def overtake_segment_is_straight(self, required_length=None):
        """GLOBAL 앞 구간의 90-percentile 곡률로 긴 직선 여부를 확인한다."""
        if self.converter is None or self.gb_max_s is None:
            return False

        if required_length is None:
            required_length = self.overtake_min_straight_length
        required_length = max(
            float(required_length),
            self.overtake_min_straight_length
        )

        cur_s = self.scalar(self.cur_s)
        sample_s = np.arange(
            cur_s,
            cur_s + required_length + 0.25,
            0.25
        )
        if len(sample_s) < 5:
            return False

        xy = self.converter.get_cartesian(
            sample_s % self.gb_max_s,
            np.zeros_like(sample_s)
        )
        if np.any(~np.isfinite(xy)):
            return False

        points = np.column_stack((xy[0], xy[1]))
        delta = np.diff(points, axis=0)
        seg_len = np.linalg.norm(delta, axis=1)
        valid = seg_len > 0.03
        if np.count_nonzero(valid) < 3:
            return False

        headings = np.unwrap(np.arctan2(delta[:, 1], delta[:, 0]))
        d_heading = np.abs(np.diff(headings))
        ds_mid = 0.5 * (seg_len[:-1] + seg_len[1:])
        curvatures = d_heading / np.maximum(ds_mid, 1e-3)
        if len(curvatures) == 0:
            return False

        robust_curvature = float(np.percentile(curvatures, 90.0))
        return robust_curvature <= self.overtake_max_curvature

    def overtake_pass_target_speed(self, obstacle_speed, global_speed):
        """횡간격 확보 뒤 사용할 공격적이지만 GLOBAL 이하인 추월 속도."""
        return float(np.clip(
            max(
                obstacle_speed + self.overtake_speed_gain,
                global_speed * self.overtake_global_speed_fraction
            ),
            self.follow_min_speed,
            min(self.overtake_max_speed, global_speed)
        ))

    def overtake_time_profile(self, distances, start_speed, target_speed):
        """컨트롤러 가감속 한계를 반영한 거리별 도달시간 예측."""
        distances = np.maximum(np.asarray(distances, dtype=float), 0.0)
        v0 = max(float(start_speed), 0.20)
        vt = max(float(target_speed), 0.20)
        result = np.zeros_like(distances)

        if vt >= v0:
            accel = max(self.overtake_prediction_accel, 1e-3)
            accel_distance = max((vt * vt - v0 * v0) / (2.0 * accel), 0.0)
            accel_time = max((vt - v0) / accel, 0.0)
            accelerating = distances <= accel_distance
            result[accelerating] = (
                np.sqrt(v0 * v0 + 2.0 * accel * distances[accelerating]) - v0
            ) / accel
            result[~accelerating] = (
                accel_time
                + (distances[~accelerating] - accel_distance) / vt
            )
        else:
            decel = max(self.overtake_prediction_decel, 1e-3)
            decel_distance = max((v0 * v0 - vt * vt) / (2.0 * decel), 0.0)
            decel_time = max((v0 - vt) / decel, 0.0)
            decelerating = distances <= decel_distance
            inside = np.maximum(
                v0 * v0 - 2.0 * decel * distances[decelerating],
                vt * vt
            )
            result[decelerating] = (v0 - np.sqrt(inside)) / decel
            result[~decelerating] = (
                decel_time
                + (distances[~decelerating] - decel_distance) / vt
            )
        return result

    def overtake_longitudinal_uncertainty(self, prediction_time):
        """상대 속도 오차와 추가 가속 가능성을 종방향 안전구간으로 변환한다."""
        prediction_time = max(float(prediction_time), 0.0)
        uncertainty = (
            self.overtake_prediction_position_uncertainty
            + self.overtake_prediction_speed_uncertainty * prediction_time
            + 0.5 * self.overtake_opponent_accel_bound
                * prediction_time * prediction_time
        )
        return float(np.clip(
            uncertainty,
            0.0,
            self.overtake_prediction_uncertainty_max
        ))

    def simulate_overtake_pass(self, initial_gap, obstacle_speed,
                               start_speed, pass_target_speed,
                               time_offset=0.0):
        """가속 지연을 포함해 상대 중심보다 pass_margin 앞설 때까지 적분."""
        dt = max(self.overtake_prediction_dt, 0.01)
        max_time = max(self.overtake_prediction_max_time, dt)
        accel = max(self.overtake_prediction_accel, 1e-3)
        ego_speed = max(float(start_speed), 0.0)
        ego_distance = 0.0
        elapsed = 0.0

        while elapsed < max_time:
            relative_gain = ego_distance - obstacle_speed * elapsed
            required_gain = max(
                float(initial_gap)
                + self.overtake_pass_margin
                + self.overtake_longitudinal_uncertainty(
                    time_offset + elapsed
                ),
                0.0
            )
            if relative_gain >= required_gain:
                return elapsed, ego_distance
            previous_speed = ego_speed
            ego_speed = min(pass_target_speed, ego_speed + accel * dt)
            ego_distance += 0.5 * (previous_speed + ego_speed) * dt
            elapsed += dt

        return None, None

    def make_full_overtake_rollout(self, target_d, obstacle):
        """SHIFT+PASS+RETURN 전 구간을 한 번에 만들어 추월 시작 전 검사한다."""
        if obstacle is None or self.gb_max_s is None or self.gb_max_s <= 0.0:
            return None

        cur_s = self.scalar(self.cur_s)
        cur_d = self.scalar(self.cur_d)
        global_speed = self.global_speed_at_s(cur_s)
        obstacle_speed = max(self.obstacle_speed(obstacle), 0.0)
        shift_speed = max(min(obstacle_speed, global_speed), 0.50)
        pass_speed = self.overtake_pass_target_speed(
            obstacle_speed,
            global_speed
        )
        if pass_speed - obstacle_speed < self.overtake_min_speed_advantage:
            return None

        shift_length = max(
            self.overtake_entry_min_length,
            max(abs(float(self.cur_vs)), shift_speed) * self.overtake_entry_time
        )
        shift_time = float(self.overtake_time_profile(
            np.asarray([shift_length]),
            abs(float(self.cur_vs)),
            shift_speed
        )[0])

        initial_center_gap = float(self.forward_s_distance(
            cur_s,
            obstacle.s_center
        ))
        gap_after_shift = max(
            initial_center_gap + obstacle_speed * shift_time - shift_length,
            0.0
        )
        pass_time, pass_distance = self.simulate_overtake_pass(
            gap_after_shift,
            obstacle_speed,
            shift_speed,
            pass_speed,
            time_offset=shift_time
        )
        if pass_time is None or pass_distance is None:
            return None

        return_length = max(
            self.overtake_return_min_length,
            pass_speed * self.overtake_return_time
        )
        return_start = shift_length + pass_distance
        maneuver_length = return_start + return_length
        path_length = maneuver_length + self.overtake_full_plan_margin
        if path_length > self.overtake_path_max_length:
            return None

        s_rel = np.arange(
            0.0,
            path_length + 0.5 * self.overtake_path_ds,
            self.overtake_path_ds
        )
        if len(s_rel) < 3:
            return None
        s_range = cur_s + s_rel

        d_range = np.full_like(s_rel, float(target_d))
        shift_ratio = np.clip(s_rel / max(shift_length, 1e-3), 0.0, 1.0)
        shift_ease = 0.5 - 0.5 * np.cos(math.pi * shift_ratio)
        shift_mask = s_rel <= shift_length
        d_range[shift_mask] = (
            cur_d
            + (float(target_d) - cur_d) * shift_ease[shift_mask]
        )
        return_ratio = np.clip(
            (s_rel - return_start) / max(return_length, 1e-3),
            0.0,
            1.0
        )
        return_ease = 0.5 - 0.5 * np.cos(math.pi * return_ratio)
        return_mask = s_rel >= return_start
        d_range[return_mask] = float(target_d) * (1.0 - return_ease[return_mask])

        time_range = np.zeros_like(s_rel)
        time_range[shift_mask] = self.overtake_time_profile(
            s_rel[shift_mask],
            abs(float(self.cur_vs)),
            shift_speed
        )
        pass_mask = (s_rel > shift_length) & (s_rel <= return_start)
        pass_local = s_rel[pass_mask] - shift_length
        time_range[pass_mask] = shift_time + self.overtake_time_profile(
            pass_local,
            shift_speed,
            pass_speed
        )
        pass_end_time = shift_time + pass_time
        time_range[return_mask] = (
            pass_end_time
            + (s_rel[return_mask] - return_start) / max(pass_speed, 0.20)
        )

        return {
            "s": s_range,
            "d": d_range,
            "t": time_range,
            "target_d": float(target_d),
            "target_v": pass_speed,
            "is_overtake": True,
            "overtake_phase": "FULL_PREVIEW",
            "planner_mode": "DYNAMIC_OVERTAKE",
            "opponent_speed": obstacle_speed,
            "shift_length": float(shift_length),
            "pass_distance": float(pass_distance),
            "return_start": float(return_start),
            "return_length": float(return_length),
            "maneuver_length": float(maneuver_length),
            "maneuver_time": float(pass_end_time + return_length / max(pass_speed, 0.20)),
        }

    def make_overtake_path(self, target_d, target_v, phase, obstacle=None):
        """고정된 s 기준점으로 SHIFT/PASS/RETURN 경로를 재발행한다."""
        cur_s = self.scalar(self.cur_s)
        cur_d = self.scalar(self.cur_d)
        global_speed = self.global_speed_at_s(cur_s)
        target_v = float(np.clip(
            target_v,
            self.follow_min_speed,
            min(self.overtake_max_speed, global_speed)
        ))

        if phase == "RETURN":
            target_d = 0.0
            transition_length = (
                self.overtake_return_length_locked
                if self.overtake_return_length_locked is not None
                else max(
                    self.overtake_return_min_length,
                    max(abs(float(self.cur_vs)), target_v)
                    * self.overtake_return_time
                )
            )
        else:
            transition_length = (
                self.overtake_shift_length
                if self.overtake_shift_length is not None
                else max(
                    self.overtake_entry_min_length,
                    max(abs(float(self.cur_vs)), target_v)
                    * self.overtake_entry_time
                )
            )

        path_length = max(
            self.overtake_path_min_length,
            max(abs(float(self.cur_vs)), target_v) * 2.0,
            transition_length + 2.0
        )
        path_length = min(path_length, self.overtake_path_max_length)

        s_range = np.arange(
            cur_s,
            cur_s + path_length + 0.5 * self.overtake_path_ds,
            self.overtake_path_ds
        )
        if len(s_range) < 3:
            return None

        s_rel = s_range - cur_s
        if phase == "RETURN" and self.overtake_return_start_s is not None:
            progress_now = max(
                self.signed_track_s_delta(
                    self.overtake_return_start_s,
                    cur_s
                ),
                0.0
            )
            start_d = (
                self.overtake_return_start_d
                if self.overtake_return_start_d is not None else cur_d
            )
            phase_ratio = np.clip(
                (progress_now + s_rel) / max(transition_length, 1e-3),
                0.0,
                1.0
            )
            ease = 0.5 - 0.5 * np.cos(math.pi * phase_ratio)
            d_range = float(start_d) * (1.0 - ease)
        elif self.overtake_shift_start_s is not None:
            progress_now = max(
                self.signed_track_s_delta(
                    self.overtake_shift_start_s,
                    cur_s
                ),
                0.0
            )
            start_d = (
                self.overtake_shift_start_d
                if self.overtake_shift_start_d is not None else cur_d
            )
            phase_ratio = np.clip(
                (progress_now + s_rel) / max(transition_length, 1e-3),
                0.0,
                1.0
            )
            ease = 0.5 - 0.5 * np.cos(math.pi * phase_ratio)
            d_range = float(start_d) + (float(target_d) - float(start_d)) * ease
        else:
            phase_ratio = np.clip(
                s_rel / max(transition_length, 1e-3),
                0.0,
                1.0
            )
            ease = 0.5 - 0.5 * np.cos(math.pi * phase_ratio)
            d_range = cur_d + (float(target_d) - cur_d) * ease

        time_range = self.overtake_time_profile(
            s_rel,
            abs(float(self.cur_vs)),
            target_v
        )

        return {
            "s": s_range,
            "d": d_range,
            "t": time_range,
            "target_d": float(target_d),
            "target_v": target_v,
            "is_overtake": True,
            "overtake_phase": phase,
            "planner_mode": "DYNAMIC_OVERTAKE",
            "opponent_speed": (
                self.obstacle_speed(obstacle)
                if obstacle is not None else 0.0
            ),
        }

    def overtake_path_is_safe(self, path, obstacles):
        """실제 map 벽, 정적 장애물, 동적 미래 위치를 모두 검사한다."""
        if path is None or self.converter is None:
            return False, 0.0

        s_arr = np.asarray(path["s"], dtype=float) % self.gb_max_s
        d_arr = np.asarray(path["d"], dtype=float)
        t_arr = np.asarray(path.get("t", np.zeros_like(s_arr)), dtype=float)

        xy = self.converter.get_cartesian(s_arr, d_arr)
        if np.any(~np.isfinite(xy)):
            return False, 0.0
        path_xy = np.column_stack((xy[0], xy[1]))
        wall_clearance = self.get_path_min_wall_clearance(path_xy)
        if wall_clearance < self.map_wall_margin:
            return False, wall_clearance

        static_obstacles = self.static_obstacle_view(obstacles)
        static_safe, _, _ = self.check_path_safety(
            path,
            static_obstacles,
            obstacle_margin=self.collision_margin,
            check_track_limit=False
        )
        if not static_safe:
            return False, wall_clearance

        for obs in getattr(obstacles, "obstacles", []):
            if not self.is_dynamic_obstacle(obs):
                continue

            obs_speed = max(self.obstacle_speed(obs), 0.0)
            obs_center = self.scalar(obs.s_center)
            obs_len = max(
                abs(
                    self.scalar(getattr(obs, "s_end", obs_center))
                    - self.scalar(getattr(obs, "s_start", obs_center))
                ),
                0.50
            )
            obs_d_low = min(obs.d_left, obs.d_right) - self.collision_margin
            obs_d_high = max(obs.d_left, obs.d_right) + self.collision_margin

            for curr_s, curr_d, curr_t in zip(s_arr, d_arr, t_arr):
                prediction_time = max(
                    float(curr_t) + self.dynamic_time_buffer,
                    0.0
                )
                longitudinal_uncertainty = (
                    self.overtake_longitudinal_uncertainty(prediction_time)
                )
                predicted_center = (
                    obs_center
                    + obs_speed * prediction_time
                ) % self.gb_max_s
                danger_start = (
                    predicted_center
                    - 0.5 * obs_len
                    - self.dynamic_front_s_buffer
                    - longitudinal_uncertainty
                )
                danger_end = (
                    predicted_center
                    + 0.5 * obs_len
                    + self.dynamic_rear_s_buffer
                    + longitudinal_uncertainty
                )
                if (
                    self.is_s_in_interval(curr_s, danger_start, danger_end)
                    and obs_d_low <= curr_d <= obs_d_high
                ):
                    return False, wall_clearance

        return True, wall_clearance

    def find_safe_overtake_candidate(self, target, obstacles, follow_speed):
        """양쪽의 FULL SHIFT+PASS+RETURN rollout을 평가한다."""
        if target is None:
            return None

        target_center_d = self.scalar(target.d_center)
        target_low = (
            min(target.d_left, target.d_right)
            - self.collision_margin
            - self.overtake_target_extra_clearance
        )
        target_high = (
            max(target.d_left, target.d_right)
            + self.collision_margin
            + self.overtake_target_extra_clearance
        )

        candidates = []
        for d_target in np.linspace(
            -self.max_offset,
            self.max_offset,
            self.num_samples
        ):
            side = 1 if d_target > target_center_d else -1
            if side > 0 and d_target <= target_high:
                continue
            if side < 0 and d_target >= target_low:
                continue
            if (
                abs(d_target - target_center_d)
                < self.overtake_min_lateral_clearance
            ):
                continue

            path = self.make_full_overtake_rollout(d_target, target)
            if path is None:
                continue
            if path.get("maneuver_time", float("inf")) > self.overtake_timeout:
                continue
            if (
                self.overtake_require_full_straight
                and not self.overtake_segment_is_straight(
                    path.get("maneuver_length", self.overtake_min_straight_length)
                )
            ):
                continue
            safe, wall_clearance = self.overtake_path_is_safe(path, obstacles)
            if not safe:
                continue

            if side > 0:
                target_clearance = d_target - target_high
            else:
                target_clearance = target_low - d_target

            # SHIFT가 끝난 뒤부터 RETURN 시작 전까지 실제 추월 차선의 벽
            # 여유를 잰다. 복귀부가 평가값을 왜곡하지 않게 분리한다.
            path_rel = np.asarray(path["s"], dtype=float) - self.scalar(self.cur_s)
            lane_mask = (
                (path_rel >= path.get("shift_length", 0.0))
                & (path_rel <= path.get("return_start", path_rel[-1]))
            )
            lane_indices = np.flatnonzero(lane_mask)
            if len(lane_indices) == 0:
                continue
            lane_start = int(lane_indices[0])
            lane_end = int(lane_indices[-1]) + 1
            tail_xy_raw = self.converter.get_cartesian(
                np.asarray(path["s"][lane_start:lane_end]) % self.gb_max_s,
                np.asarray(path["d"][lane_start:lane_end])
            )
            tail_xy = np.column_stack((tail_xy_raw[0], tail_xy_raw[1]))
            lane_wall_clearance = self.get_path_min_wall_clearance(tail_xy)

            # 상대와의 여유와 벽 여유 중 작은 쪽을 최대화하므로 단순히
            # 벽 끝이나 max_offset 끝으로 붙지 않고 빈 통로 중심을 고른다.
            usable_clearance = min(
                target_clearance,
                lane_wall_clearance - self.map_wall_margin
            )
            if usable_clearance < self.dynamic_overtake_min_gap:
                continue

            score = (
                3.0 * usable_clearance
                + 0.5 * lane_wall_clearance
                + 0.2 * wall_clearance
                - 0.08 * abs(d_target - self.cur_d)
            )
            path["usable_clearance"] = float(usable_clearance)
            path["lane_wall_clearance"] = float(lane_wall_clearance)
            candidates.append((score, side, float(d_target), path))

        if not candidates:
            return None
        _, side, d_target, path = max(candidates, key=lambda item: item[0])
        return side, d_target, path

    def try_start_overtake(self, target, obstacles, follow_path):
        """FOLLOW와 안전 gap이 각각 일정 시간 유지됐을 때만 추월을 고정한다."""
        if (
            not self.dynamic_avoidance_enabled
            or not self.dynamic_overtake_enabled
            or target is None
            or follow_path is None
        ):
            self.overtake_gap_since = rospy.Time(0)
            self.overtake_candidate_path = None
            self.overtake_reject_reason = "overtake_disabled_or_no_target"
            return False

        now = rospy.Time.now()
        track_id = self.obstacle_track_id(target)
        if track_id is None:
            self.overtake_reject_reason = "no_stable_track_id"
            return False

        if track_id != self.overtake_ready_track_id:
            self.overtake_ready_track_id = track_id
            self.overtake_follow_since = now
            self.overtake_gap_since = rospy.Time(0)
            self.overtake_candidate_side = 0
            self.overtake_candidate_d = None
            self.overtake_candidate_path = None
            self.overtake_reject_reason = "new_target_follow_timer"

        follow_age = (now - self.overtake_follow_since).to_sec()
        bumper_gap = self.overtake_bumper_gap(target)
        global_speed = self.global_speed_at_s(self.cur_s)
        speed_advantage = global_speed - max(self.obstacle_speed(target), 0.0)

        basic_ready = (
            follow_age >= self.overtake_follow_min_time
            and self.overtake_min_start_gap
                <= bumper_gap <= self.overtake_max_start_gap
            and speed_advantage >= self.overtake_min_speed_advantage
            and self.overtake_segment_is_straight()
        )
        if not basic_ready:
            self.overtake_gap_since = rospy.Time(0)
            self.overtake_candidate_side = 0
            self.overtake_candidate_d = None
            self.overtake_candidate_path = None
            if follow_age < self.overtake_follow_min_time:
                self.overtake_reject_reason = "stabilizing_follow"
            elif bumper_gap < self.overtake_min_start_gap:
                self.overtake_reject_reason = "front_gap_too_small"
            elif bumper_gap > self.overtake_max_start_gap:
                self.overtake_reject_reason = "front_gap_too_large"
            elif speed_advantage < self.overtake_min_speed_advantage:
                self.overtake_reject_reason = "not_enough_speed_advantage"
            else:
                self.overtake_reject_reason = "not_straight"
            return False

        candidate = self.find_safe_overtake_candidate(
            target,
            obstacles,
            follow_path.get("target_v", self.obstacle_speed(target))
        )
        if candidate is None:
            self.overtake_gap_since = rospy.Time(0)
            self.overtake_candidate_side = 0
            self.overtake_candidate_d = None
            self.overtake_candidate_path = None
            self.overtake_reject_reason = "full_maneuver_not_safe"
            return False

        side, target_d, candidate_path = candidate
        self.overtake_candidate_path = candidate_path
        candidate_changed = (
            side != self.overtake_candidate_side
            or self.overtake_candidate_d is None
            or abs(target_d - self.overtake_candidate_d)
                > self.overtake_candidate_switch_tolerance
        )
        if candidate_changed:
            self.overtake_candidate_side = side
            self.overtake_candidate_d = target_d
            self.overtake_gap_since = now
            self.overtake_reject_reason = "safe_gap_persistence"
            return False

        if self.overtake_gap_since == rospy.Time(0):
            self.overtake_gap_since = now
            self.overtake_reject_reason = "safe_gap_persistence"
            return False
        if (now - self.overtake_gap_since).to_sec() < self.overtake_gap_persist_time:
            self.overtake_reject_reason = "safe_gap_persistence"
            return False

        self.overtake_phase = "SHIFT"
        self.overtake_target_track_id = track_id
        self.overtake_target_cache = deepcopy(target)
        self.overtake_target_last_seen = now
        self.overtake_locked_side = side
        self.overtake_locked_d = target_d
        self.overtake_started_at = now
        self.overtake_shift_start_s = self.scalar(self.cur_s)
        self.overtake_shift_start_d = self.scalar(self.cur_d)
        self.overtake_shift_length = candidate_path.get(
            "shift_length",
            self.overtake_entry_min_length
        )
        self.overtake_return_start_s = None
        self.overtake_return_start_d = None
        self.overtake_return_length_locked = candidate_path.get(
            "return_length",
            self.overtake_return_min_length
        )
        self.overtake_locked_rollout = deepcopy(candidate_path)
        self.overtake_reject_reason = "full_maneuver_safe"
        rospy.logwarn(
            "[Lattice] OVERTAKE START: side=%s target_d=%.2f gap=%.2f",
            "LEFT" if side > 0 else "RIGHT",
            target_d,
            bumper_gap
        )
        return True

    def begin_overtake_return(self, reason):
        """RETURN 시작 위치를 한 번만 고정해 매 프레임 곡선이 늘어지지 않게 한다."""
        if self.overtake_phase != "RETURN":
            self.overtake_phase = "RETURN"
            self.overtake_return_start_s = self.scalar(self.cur_s)
            self.overtake_return_start_d = self.scalar(self.cur_d)
            self.overtake_return_length_locked = max(
                self.overtake_return_min_length,
                max(abs(float(self.cur_vs)), 0.50) * self.overtake_return_time
            )
            rospy.logwarn(
                "[Lattice] OVERTAKE -> RETURN: %s, length=%.2f",
                reason,
                self.overtake_return_length_locked
            )

    def make_active_overtake_path(self, target, obstacles):
        """SHIFT→PASS→RETURN을 진행하고 target_d/방향은 끝까지 고정한다."""
        if self.overtake_phase == "FOLLOW":
            return None

        now = rospy.Time.now()
        elapsed = (now - self.overtake_started_at).to_sec()
        if elapsed > self.overtake_timeout and self.overtake_phase != "RETURN":
            self.begin_overtake_return("timeout")

        if target is None:
            # 상대 위치를 모르는 상태에서 GLOBAL을 가로질러 복귀하지 않는다.
            # RETURN 중이라도 현재 횡위치를 유지하며 재획득을 기다린다.
            hold_speed = min(
                self.global_speed_at_s(self.cur_s),
                max(abs(float(self.cur_vs)), self.follow_min_speed)
            )
            hold_path = self.make_overtake_path(
                self.scalar(self.cur_d),
                hold_speed,
                "SHIFT",
                None
            )
            hold_path["overtake_phase"] = "HOLD_TARGET_LOST"
            hold_path["target_d"] = self.scalar(self.cur_d)
            safe, _ = self.overtake_path_is_safe(hold_path, obstacles)
            if safe:
                rospy.logwarn_throttle(
                    0.5,
                    "[Lattice] OVERTAKE target lost -> HOLD and reacquire"
                )
                return hold_path
            return None

        obs_speed = max(self.obstacle_speed(target), 0.0) if target is not None else 0.0
        global_speed = self.global_speed_at_s(self.cur_s)

        if self.overtake_phase == "SHIFT":
            lateral_clear = (
                target is not None
                and abs(self.scalar(self.cur_d) - self.scalar(target.d_center))
                    >= self.overtake_min_lateral_clearance
            )
            if lateral_clear:
                self.overtake_phase = "PASS"
                rospy.logwarn("[Lattice] OVERTAKE SHIFT -> PASS")

        if self.overtake_phase == "PASS" and target is not None:
            ego_ahead = self.signed_track_s_delta(
                target.s_center,
                self.cur_s
            )
            required_observed_ahead = (
                self.overtake_pass_margin
                + self.overtake_return_observed_margin
            )
            if ego_ahead >= required_observed_ahead:
                return_speed = self.overtake_pass_target_speed(
                    obs_speed,
                    global_speed
                )
                return_preview = self.make_overtake_path(
                    0.0,
                    return_speed,
                    "RETURN",
                    target
                )
                return_safe, _ = self.overtake_path_is_safe(
                    return_preview,
                    obstacles
                )
                if return_safe:
                    self.begin_overtake_return("pass_complete_and_return_safe")
                    rospy.logwarn(
                        "[Lattice] OVERTAKE PASS -> RETURN: ahead=%.2f",
                        ego_ahead
                    )
                else:
                    rospy.logwarn_throttle(
                        0.5,
                        "[Lattice] PASS hold: ahead=%.2f but RETURN corridor unsafe",
                        ego_ahead
                    )

        if self.overtake_phase == "RETURN" and abs(self.scalar(self.cur_d)) <= 0.08:
            final_ahead = self.signed_track_s_delta(
                target.s_center,
                self.cur_s
            )
            required_final_ahead = (
                self.overtake_pass_margin
                + self.overtake_return_observed_margin
            )
            final_speed = min(
                global_speed,
                self.overtake_max_speed,
                max(abs(float(self.cur_vs)), obs_speed)
            )
            final_global_path = self.make_overtake_path(
                0.0,
                final_speed,
                "RETURN",
                target
            )
            final_safe, _ = self.overtake_path_is_safe(
                final_global_path,
                obstacles
            )
            if final_ahead >= required_final_ahead and final_safe:
                self.reset_overtake_state()
                rospy.logwarn("[Lattice] OVERTAKE COMPLETE -> GLOBAL")
                return None
            rospy.logwarn_throttle(
                0.5,
                "[Lattice] RETURN hold: final GLOBAL corridor not safe, ahead=%.2f",
                final_ahead
            )

        if self.overtake_phase == "SHIFT":
            # 횡간격이 확보되기 전에는 상대보다 빠르게 가속하지 않는다.
            target_v = min(obs_speed, global_speed)
            target_d = self.overtake_locked_d
        elif self.overtake_phase == "PASS":
            target_v = self.overtake_pass_target_speed(
                obs_speed,
                global_speed
            )
            target_d = self.overtake_locked_d
        else:
            target_v = min(
                global_speed,
                self.overtake_max_speed,
                max(obs_speed + self.overtake_speed_gain, self.cur_vs)
            )
            target_d = 0.0

        path = self.make_overtake_path(
            target_d,
            target_v,
            self.overtake_phase,
            target
        )
        safe, _ = self.overtake_path_is_safe(path, obstacles)
        if safe:
            return path

        # 진행 경로가 새 정적 장애물/벽 때문에 불가능해지면 GLOBAL 복귀를
        # 먼저 검사한다. 그것도 불가능하면 None을 반환해 정적 planner가
        # 다음 루프에서 우선권을 갖게 한다.
        if self.overtake_phase != "RETURN":
            return_path = self.make_overtake_path(
                0.0,
                min(global_speed, max(obs_speed, 0.50)),
                "RETURN",
                target
            )
            return_safe, _ = self.overtake_path_is_safe(
                return_path,
                obstacles
            )
            if return_safe:
                self.begin_overtake_return("active_path_unsafe")
                return_path = self.make_overtake_path(
                    0.0,
                    min(global_speed, max(obs_speed, 0.50)),
                    "RETURN",
                    target
                )
                return return_path

        rospy.logerr_throttle(
            0.5,
            "[Lattice] OVERTAKE path blocked; yielding to static safety"
        )

        hold_path = self.make_overtake_path(
            self.scalar(self.cur_d),
            min(global_speed, max(obs_speed, 0.50)),
            "SHIFT",
            target
        )
        hold_safe, _ = self.overtake_path_is_safe(hold_path, obstacles)
        if hold_safe:
            hold_path["overtake_phase"] = "HOLD"
            return hold_path
        return None

    # 현재 차량 앞쪽 장애물들을 기준으로 가장 넓은 빈 공간의 중심 d를 찾는다.
    def find_gate_center(self, obstacles):
        # 후보 경로가 실제로 생성되는 lateral 범위
        d_min = -self.max_offset
        d_max = self.max_offset

        gate_s_min = self.gate_s_min
        gate_s_max = self.gate_s_max
        gate_margin = self.gate_margin

        blocked_intervals = []

        for obs in obstacles.obstacles:
            obs_rel_s = self.forward_s_distance(self.cur_s, obs.s_center)

            # 현재 차량 앞쪽 gate_s_min ~ gate_s_max 안의 장애물만 사용
            if not (gate_s_min < obs_rel_s < gate_s_max):
                continue

            # Obstacle.msg에 d_left, d_right가 있으므로 이걸 사용
            obs_d_low = min(obs.d_left, obs.d_right) - gate_margin
            obs_d_high = max(obs.d_left, obs.d_right) + gate_margin

            # 후보 경로 범위 밖은 잘라냄
            obs_d_low = max(obs_d_low, d_min)
            obs_d_high = min(obs_d_high, d_max)

            if obs_d_low < obs_d_high:
                blocked_intervals.append((obs_d_low, obs_d_high))

        # 앞쪽에 관련 장애물이 없으면 gap center를 따로 쓰지 않음
        if len(blocked_intervals) == 0:
            return None, None

        # d 기준으로 정렬
        blocked_intervals.sort(key=lambda x: x[0])

        # 겹치는 blocked interval 병합
        merged = []
        for interval in blocked_intervals:
            if not merged:
                merged.append(list(interval))
            else:
                prev = merged[-1]
                if interval[0] <= prev[1]:
                    prev[1] = max(prev[1], interval[1])
                else:
                    merged.append(list(interval))

        # blocked interval 사이의 free gap 계산
        free_gaps = []
        cursor = d_min

        for low, high in merged:
            if cursor < low:
                free_gaps.append((cursor, low))
            cursor = max(cursor, high)

        if cursor < d_max:
            free_gaps.append((cursor, d_max))

        if len(free_gaps) == 0:
            return None, None

        # 가장 넓은 free gap 선택
        best_gap = max(free_gaps, key=lambda g: g[1] - g[0])
        gap_width = best_gap[1] - best_gap[0]
        gate_center_d = (best_gap[0] + best_gap[1]) / 2.0

        return gate_center_d, gap_width
    
    # 모든 후보가 막혔을 때 fallback 경로를 고르는 함수
    def get_fallback_path(self, paths, obstacles=None):
        if obstacles is None:
            obstacles = self.obs

        valid_fb = []

        for p in paths:

            s_arr = p["s"] % self.gb_max_s
            d_arr = p["d"]

            # 0순위: 실제 /map 벽 검사. 
            resp = self.converter.get_cartesian(
                s_arr,
                d_arr
            )

            if np.any(np.isnan(resp)) or np.any(np.isinf(resp)):
                continue

            path_xy = np.column_stack(
                (resp[0], resp[1])
            )

            min_wall_clearance = \
                self.get_path_min_wall_clearance(
                    path_xy
                )

            # fallback이라고 벽 margin을 완화하지 않는다.
            if min_wall_clearance < self.map_wall_margin:
                continue

            # 그 다음 장애물 검사
            safe, obs_clearance, reason = \
                self.check_path_safety(
                    p,
                    obstacles,
                    obstacle_margin=self.fallback_collision_margin,
                    check_track_limit=False

                )

            if not safe:
                continue

            valid_fb.append(
                (p, obs_clearance, min_wall_clearance)
            )

        if not valid_fb:
            rospy.logerr_throttle(
                0.5,
                "[Lattice] No safe fallback path. Publish empty avoidance."
            )
            return None

        def fallback_cost(item):

            p, obs_clearance, wall_clearance = item

            obs_penalty = (
                1.0 / (obs_clearance + 1e-3)
                if obs_clearance != float("inf")
                else 0.0
            )

            wall_penalty = (
                1.0 / max(wall_clearance, 1e-3)
            )

            return (
                1.0 * abs(
                    p["target_d"] - self.cur_d
                )
                + 4.0 * obs_penalty
                + 6.0 * wall_penalty
            )

        return min(valid_fb, key=fallback_cost)[0]

    def publish_empty_avoidance(self):
        self.evasion_pub.publish(self.new_avoidance_array())
        self.mode_pub.publish(String(data="GLOBAL"))
        clear_markers = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        clear_markers.markers.append(marker)
        self.mrks_pub.publish(clear_markers)
    
    def delete_dynamic_mode_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "dynamic_mode_text"
        marker.id = 9999
        marker.action = Marker.DELETE

        return marker

    def publish_dynamic_obstacle_markers(self):
        """플래너가 동적으로 확정한 물체를 별도 토픽에 초록색으로 표시한다."""
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = rospy.Time.now()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if (
            self.dynamic_avoidance_enabled
            and self.converter is not None
            and self.gb_max_s is not None
            and self.gb_max_s > 0.0
        ):
            dynamic_obstacles = [
                obs for obs in getattr(self.obs, "obstacles", [])
                if self.is_dynamic_obstacle(obs)
            ]
            for index, obs in enumerate(dynamic_obstacles):
                s_center = self.scalar(obs.s_center) % self.gb_max_s
                d_center = self.scalar(obs.d_center)
                xy = self.converter.get_cartesian(
                    np.asarray([s_center]),
                    np.asarray([d_center])
                )
                if np.any(~np.isfinite(xy)):
                    continue

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "confirmed_dynamic_obstacles"
                marker.id = index
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = float(xy[0, 0])
                marker.pose.position.y = float(xy[1, 0])
                marker.pose.position.z = 0.30
                marker.pose.orientation.w = 1.0
                obstacle_size = max(
                    self.scalar(getattr(obs, "size", 0.0)),
                    abs(self.scalar(obs.d_left) - self.scalar(obs.d_right)),
                    0.25
                )
                marker.scale.x = obstacle_size
                marker.scale.y = obstacle_size
                marker.scale.z = 0.30
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.95
                marker.lifetime = rospy.Duration(0.30)
                markers.markers.append(marker)

        self.dynamic_obstacle_marker_pub.publish(markers)

    def overtake_display_state(self, target=None):
        if self.overtake_phase != "FOLLOW":
            return self.overtake_phase
        if target is None:
            return "GLOBAL"
        if (
            self.overtake_candidate_path is not None
            and self.overtake_gap_since != rospy.Time(0)
        ):
            return "READY"
        return "FOLLOW"

    def publish_overtake_debug_markers(self, target=None):
        """추월 판단 상태, full rollout, 상대 예측 궤적을 RViz에 표시한다."""
        state = self.overtake_display_state(target)
        self.overtake_state_pub.publish(String(data=state))

        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = "map"
        clear.header.stamp = rospy.Time.now()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        colors = {
            "GLOBAL": (0.75, 0.75, 0.75),
            "FOLLOW": (0.10, 0.35, 1.00),
            "READY": (1.00, 0.55, 0.00),
            "SHIFT": (0.00, 0.90, 1.00),
            "PASS": (1.00, 0.00, 0.85),
            "RETURN": (0.10, 1.00, 0.20),
        }
        color = colors.get(state, (1.00, 0.15, 0.15))

        status = Marker()
        status.header.frame_id = "map"
        status.header.stamp = rospy.Time.now()
        status.ns = "overtake_state"
        status.id = 0
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.position.x = self.vehicle_x
        status.pose.position.y = self.vehicle_y
        status.pose.position.z = 1.55
        status.pose.orientation.w = 1.0
        status.scale.z = 0.42
        status.color.r, status.color.g, status.color.b = color
        status.color.a = 1.0
        status.lifetime = rospy.Duration(0.20)

        detail = self.overtake_reject_reason
        if target is not None:
            gap = self.overtake_bumper_gap(target)
            detail = "gap=%.2fm  %s" % (gap, detail)
        if state == "READY" and self.overtake_gap_since != rospy.Time(0):
            ready_age = max(
                (rospy.Time.now() - self.overtake_gap_since).to_sec(),
                0.0
            )
            detail += "  hold=%.2f/%.2fs" % (
                ready_age,
                self.overtake_gap_persist_time
            )
        selected_d = (
            self.overtake_locked_d
            if self.overtake_phase != "FOLLOW"
            else self.overtake_candidate_d
        )
        if selected_d is not None:
            detail += "  d=%.2f" % float(selected_d)
        status.text = "%s\n%s" % (state, detail)
        markers.markers.append(status)

        preview = (
            self.overtake_locked_rollout
            if self.overtake_phase != "FOLLOW"
            else self.overtake_candidate_path
        )
        if preview is not None and self.converter is not None:
            s_arr = np.asarray(preview.get("s", []), dtype=float)
            d_arr = np.asarray(preview.get("d", []), dtype=float)
            if len(s_arr) == len(d_arr) and len(s_arr) >= 2:
                xy = self.converter.get_cartesian(
                    s_arr % self.gb_max_s,
                    d_arr
                )
                if not np.any(~np.isfinite(xy)):
                    rollout = Marker()
                    rollout.header.frame_id = "map"
                    rollout.header.stamp = rospy.Time.now()
                    rollout.ns = "overtake_full_rollout"
                    rollout.id = 1
                    rollout.type = Marker.LINE_STRIP
                    rollout.action = Marker.ADD
                    rollout.pose.orientation.w = 1.0
                    rollout.scale.x = 0.07
                    rollout.color.r = color[0]
                    rollout.color.g = color[1]
                    rollout.color.b = color[2]
                    rollout.color.a = 0.90
                    rollout.lifetime = rospy.Duration(0.20)
                    for x, y in zip(xy[0], xy[1]):
                        point = Point()
                        point.x = float(x)
                        point.y = float(y)
                        point.z = 0.18
                        rollout.points.append(point)
                    markers.markers.append(rollout)

        if (
            target is not None
            and self.converter is not None
            and self.gb_max_s is not None
            and self.gb_max_s > 0.0
        ):
            target_s = self.scalar(target.s_center)
            target_d = self.scalar(target.d_center)
            target_speed = max(self.obstacle_speed(target), 0.0)
            horizon = 3.0
            if preview is not None and len(preview.get("t", [])) > 0:
                horizon = min(
                    max(float(np.max(preview["t"])), 1.0),
                    self.overtake_prediction_max_time
                )
            sample_t = np.arange(0.0, horizon + 0.10, 0.10)
            predicted_s = (target_s + target_speed * sample_t) % self.gb_max_s
            predicted_d = np.full_like(predicted_s, target_d)
            predicted_xy = self.converter.get_cartesian(
                predicted_s,
                predicted_d
            )
            if not np.any(~np.isfinite(predicted_xy)):
                trajectory = Marker()
                trajectory.header.frame_id = "map"
                trajectory.header.stamp = rospy.Time.now()
                trajectory.ns = "overtake_target_prediction"
                trajectory.id = 2
                trajectory.type = Marker.LINE_STRIP
                trajectory.action = Marker.ADD
                trajectory.pose.orientation.w = 1.0
                trajectory.scale.x = 0.08
                trajectory.color.r = 0.0
                trajectory.color.g = 1.0
                trajectory.color.b = 0.15
                trajectory.color.a = 0.95
                trajectory.lifetime = rospy.Duration(0.20)
                for x, y in zip(predicted_xy[0], predicted_xy[1]):
                    point = Point()
                    point.x = float(x)
                    point.y = float(y)
                    point.z = 0.25
                    trajectory.points.append(point)
                markers.markers.append(trajectory)

                # 주황색 횡선은 속도 오차와 상대 추가 가속까지 포함한 종방향
                # 예측 범위다. 제어 충돌검사에서도 같은 범위를 사용한다.
                safety_time = sample_t + self.dynamic_time_buffer
                uncertainty = np.asarray([
                    self.overtake_longitudinal_uncertainty(value)
                    for value in safety_time
                ])
                safety_center_s = (
                    target_s + target_speed * safety_time
                ) % self.gb_max_s
                lower_s = (safety_center_s - uncertainty) % self.gb_max_s
                upper_s = (safety_center_s + uncertainty) % self.gb_max_s
                lower_xy = self.converter.get_cartesian(
                    lower_s,
                    predicted_d
                )
                upper_xy = self.converter.get_cartesian(
                    upper_s,
                    predicted_d
                )
                if (
                    not np.any(~np.isfinite(lower_xy))
                    and not np.any(~np.isfinite(upper_xy))
                ):
                    corridor = Marker()
                    corridor.header.frame_id = "map"
                    corridor.header.stamp = rospy.Time.now()
                    corridor.ns = "overtake_target_uncertainty"
                    corridor.id = 3
                    corridor.type = Marker.LINE_LIST
                    corridor.action = Marker.ADD
                    corridor.pose.orientation.w = 1.0
                    corridor.scale.x = 0.035
                    corridor.color.r = 1.0
                    corridor.color.g = 0.45
                    corridor.color.b = 0.0
                    corridor.color.a = 0.55
                    corridor.lifetime = rospy.Duration(0.20)
                    for index in range(0, len(sample_t), 2):
                        low_point = Point()
                        low_point.x = float(lower_xy[0, index])
                        low_point.y = float(lower_xy[1, index])
                        low_point.z = 0.22
                        high_point = Point()
                        high_point.x = float(upper_xy[0, index])
                        high_point.y = float(upper_xy[1, index])
                        high_point.z = 0.22
                        corridor.points.extend([low_point, high_point])
                    markers.markers.append(corridor)

        self.overtake_marker_pub.publish(markers)

    # ==========================================
    # 🧠 메인 하이브리드 루프 (뇌 스위칭)
    # ==========================================
    def loop(self): 
        rospy.loginfo("[Lattice Planner] System Ready! Starting Hybrid loop...") 
        
        while not rospy.is_shutdown(): 
            try:
                self.publish_dynamic_obstacle_markers()

                # 동적 ON일 때만 확정 앞차와 짧은 dropout cache를 사용한다.
                if self.dynamic_avoidance_enabled:
                    dynamic_target, dynamic_rel_s = \
                        self.current_dynamic_follow_target(self.obs)
                else:
                    dynamic_target, dynamic_rel_s = None, float("inf")
                    self.reset_dynamic_follow_state()
                    self.reset_overtake_state()

                # 장애물이 전혀 없고 FOLLOW cache도 끝났다면 두 번째 원본과
                # 동일하게 avoidance 경로를 비우고 GLOBAL에 제어권을 돌린다.
                if (
                    len(self.obs.obstacles) == 0
                    and dynamic_target is None
                    and not self.avoidance_active
                ):
                    self.publish_empty_avoidance()
                    self.rate.sleep()
                    continue

                # 1-1. 가까운 앞쪽 장애물이 있는지 확인
                near_obstacle = False

                for obs in self.obs.obstacles:
                    obs_rel_s = self.forward_s_distance(self.cur_s, obs.s_center)

                    if 0.3 < obs_rel_s < self.gate_s_max:
                        near_obstacle = True
                        break

                # 정적 장애물과 이미 진행 중인 정적 latch가 항상 FOLLOW보다 우선한다.
                self.update_avoidance_latch_release()
                if not self.avoidance_active:
                    self.static_latch_track_id = None

                static_obstacles = self.static_obstacle_view(self.obs)
                blocking_obs = self.find_front_blocking_obstacle(
                    static_obstacles
                )

                # 같은 물체를 확정 전에는 정적으로 보아 latch가 켜졌더라도,
                # 고속 동적 조건을 통과하면 그 물체의 latch만 FOLLOW로 전환한다.
                if (
                    self.avoidance_active
                    and blocking_obs is None
                    and dynamic_target is not None
                    and self.static_latch_track_id is not None
                    and self.static_latch_track_id
                        == self.follow_target_track_id
                ):
                    self.reset_avoidance_latch()
                    self.static_latch_track_id = None
                    rospy.logwarn_throttle(
                        0.5,
                        "[Lattice] pending obstacle confirmed dynamic: "
                        "STATIC latch -> FOLLOW"
                    )

                is_dynamic_mode = False
                run_static_planner = (
                    blocking_obs is not None
                    or self.avoidance_active
                    or (
                        dynamic_target is None
                        and len(static_obstacles.obstacles) > 0
                    )
                )

                if run_static_planner:
                    gate_center_d, gate_width = self.find_gate_center(self.obs)

                    if gate_center_d is not None:
                        rospy.logwarn_throttle(
                            0.5,
                            f"[Lattice] gate_center_d={gate_center_d:.2f}, gate_width={gate_width:.2f}"
                        )

                    candidate_paths = self.generate_static_paths(
                        self.cur_s,
                        self.cur_d,
                        aggressive=(near_obstacle or self.avoidance_active)
                    )

                    locked_target_d = self.locked_target_d if self.avoidance_active else None

                    best = self.score_static_paths(
                        candidate_paths,
                        self.obs,
                        gate_center_d=gate_center_d,
                        gate_width=gate_width,
                        locked_target_d=locked_target_d
                    )

                    if (
                        self.avoidance_active
                        and self.locked_target_d is not None
                        and best is not None
                        and abs(best.get("target_d", 0.0) - self.locked_target_d) > self.lock_replan_target_threshold
                    ):
                        rospy.logwarn_throttle(
                            0.5,
                            f"[Lattice] locked target changed for safety: "
                            f"{self.locked_target_d:.2f} -> {best.get('target_d', 0.0):.2f}"
                        )
                        self.locked_target_d = best.get("target_d", 0.0)

                    if (not self.avoidance_active) and blocking_obs is not None:
                        if best is not None and abs(best.get("target_d", 0.0)) >= self.avoidance_off_target_d_threshold:
                            self.start_avoidance_latch(best, blocking_obs)
                            self.static_latch_track_id = self.obstacle_track_id(
                                blocking_obs
                            )
                elif dynamic_target is not None:
                    # 추월 없이 GLOBAL 형상과 속도만 조정해 앞차를 추종한다.
                    is_dynamic_mode = True
                    follow_path = self.make_dynamic_follow_path(dynamic_target)
                    best = follow_path
                    candidate_paths = [best] if best is not None else []

                    rospy.logwarn_throttle(
                        0.5,
                        f"[Lattice] DYNAMIC_FOLLOW selected: "
                        f"rel_s={dynamic_rel_s:.2f}"
                    )
                else:
                    self.reset_overtake_state()
                    self.publish_empty_avoidance()
                    self.rate.sleep()
                    continue
                
                # 3. 퍼블리시
                if best is None:
                    self.publish_empty_avoidance()
                    self.rate.sleep()
                    continue

                if best is not None:
                    if (not is_dynamic_mode) and (not near_obstacle) and abs(best.get('target_d', 0.0)) < 0.25:
                        self.publish_empty_avoidance()
                        rospy.logwarn_throttle(
                            1.0,
                            "[Lattice] avoidance OFF: best path is close to global path"
                        )
                    else:
                        self.publish_path(best, candidate_paths, is_dynamic_mode) 
                        
                    
            except Exception as e:
                import traceback
                rospy.logerr(f"플래너 연산 에러 발생 (죽지 않고 계속 실행됨): {e}")
                rospy.logerr(traceback.format_exc())
                
            self.rate.sleep()

    # 현재 차량 속도를 저장
    def speed_callback(self, msg):
        self.cur_vs = float(msg.twist.twist.linear.x)
        self.speed_received = True
        
    # 현재 차량 AMCL pose를 저장하고 Frenet 좌표로 변환
    def pose_callback(self, msg):
        if self.converter is None:
            return

        pose = msg.pose.pose

        self.vehicle_x = float(pose.position.x)
        self.vehicle_y = float(pose.position.y)
        self.pose_received = True

        _, _, self.cur_yaw = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ])

        self.cur_s, self.cur_d = self.converter.get_frenet(
            self.vehicle_x,
            self.vehicle_y
        )

        # /odom 초기 수신 전에는 global waypoint 속도를
        # lookahead 계산용 임시 속도로 사용한다.
        if (
            not self.speed_received
            and self.gb_wpnts.wpnts
            and self.gb_max_s is not None
            and self.gb_max_s > 0.0
        ):
            cur_s_scalar = float(
                np.asarray(self.cur_s).reshape(-1)[0]
            )
            idx = int(
                (cur_s_scalar / self.gb_max_s)
                * len(self.gb_wpnts.wpnts)
            ) % len(self.gb_wpnts.wpnts)

            self.cur_vs = float(
                self.gb_wpnts.wpnts[idx].vx_mps
            )

        # detect_2.py용 Frenet Odometry.
        # pose.position.x/y는 각각 트랙 종방향 s와 횡방향 d이다.
        frenet_odom = Odometry()
        frenet_odom.header.stamp = (
            msg.header.stamp
            if msg.header.stamp != rospy.Time()
            else rospy.Time.now()
        )
        frenet_odom.header.frame_id = "frenet"
        frenet_odom.child_frame_id = "base_link"
        frenet_odom.pose.pose.position.x = self.scalar(self.cur_s)
        frenet_odom.pose.pose.position.y = self.scalar(self.cur_d)
        frenet_odom.pose.pose.position.z = 0.0
        frenet_odom.pose.pose.orientation.w = 1.0
        frenet_odom.twist.twist.linear.x = float(self.cur_vs)
        self.odom_frenet_pub.publish(frenet_odom)

    # 후보 경로들과 최종 선택 경로를 publish
    def publish_path(self, best_path, all_paths, is_dynamic): 
        wpnts, mrks = self.new_avoidance_array(), MarkerArray() 

        clear_marker = Marker()
        clear_marker.header.frame_id = "map"
        clear_marker.header.stamp = rospy.Time.now()
        clear_marker.action = Marker.DELETEALL
        mrks.markers.append(clear_marker)
            
        for idx, path in enumerate(all_paths):
            # FOLLOW는 아래에서 전체 복사 경로, 접근 구간, 1 m 안전 구간을
            # 따로 표시하므로 동일 경로의 후보 마커를 중복해서 그리지 않는다.
            if path.get("is_follow", False):
                continue

            s_arr, d_arr = path['s'] % self.gb_max_s, np.clip(path['d'], -self.max_offset, self.max_offset) 
            resp = self.converter.get_cartesian(s_arr, d_arr) 
            if np.any(np.isnan(resp)) or np.any(np.isinf(resp)): 
                continue

            path_xy = np.column_stack((resp[0], resp[1]))

            if len(path_xy) > 0:
                path_xy[0, 0] = self.vehicle_x
                path_xy[0, 1] = self.vehicle_y

            m = self.create_marker(path_xy.tolist(), idx + 1)

            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.45, 0.45, 0.25
            mrks.markers.append(m) 

        if best_path: 
            s_arr, d_arr = best_path['s'] % self.gb_max_s, np.clip(best_path['d'], -self.max_offset, self.max_offset) 
            resp = self.converter.get_cartesian(s_arr, d_arr) 
            
            if not (np.any(np.isnan(resp)) or np.any(np.isinf(resp))):
                points_for_marker = [] 
                path_xy = np.column_stack((resp[0], resp[1]))

                if len(path_xy) > 0:
                    path_xy[0, 0] = self.vehicle_x
                    path_xy[0, 1] = self.vehicle_y
                
                planned_speed = best_path.get('target_v', None) if is_dynamic else None
                
                for i in range(len(s_arr)): 
                    wpnt_idx = int((s_arr[i] / self.gb_max_s) * len(self.gb_wpnts.wpnts)) % len(self.gb_wpnts.wpnts) 
                    vi_global = self.gb_wpnts.wpnts[wpnt_idx].vx_mps 
                    
                    if best_path.get("is_follow", False):
                        vi = min(best_path.get("target_v", self.follow_min_speed), vi_global)
                        vi = max(vi, self.follow_min_speed)

                    elif best_path.get("is_overtake", False):
                        vi = min(best_path.get("target_v", 0.0), vi_global)
                        vi = max(vi, self.follow_min_speed)

                    elif is_dynamic and planned_speed is not None:
                        vi = min(planned_speed, vi_global)
                        vi = max(vi * (1.0 - 0.2 * (abs(d_arr[i]) / self.max_offset)), 0.5)

                    else:
                        vi = max(vi_global * (1.0 - 0.3 * (abs(d_arr[i]) / self.max_offset)), 0.5)
                    wpnts.wpnts.append(
                        self.make_ot_wpnt(
                            i,
                            resp[0, i],
                            resp[1, i],
                            s_arr[i],
                            d_arr[i],
                            vi
                        )
                    )
                    points_for_marker.append([path_xy[i, 0], path_xy[i, 1]]) 
                
                best_m = self.create_marker(points_for_marker, 0)
                if best_path.get("is_follow", False):
                    # 실제 controller에 발행되는 GLOBAL 복사 경로는 회색으로
                    # 약하게 남기고, 의미가 있는 두 FOLLOW 구간을 위에 덧그린다.
                    best_m.ns = "follow_full_global_path"
                    best_m.color.r = 0.65
                    best_m.color.g = 0.65
                    best_m.color.b = 0.65
                    best_m.color.a = 0.35
                    mrks.markers.append(best_m)
                    mrks.markers.extend(
                        self.create_follow_distance_markers(best_path)
                    )
                else:
                    best_m.color.r = best_m.color.g = best_m.color.a = 1.0
                    mrks.markers.append(best_m)
                self.evasion_pub.publish(wpnts) 

                output_mode = best_path.get(
                    "planner_mode",
                    "DYNAMIC_OVERTAKE" if is_dynamic else "STATIC_AVOID"
                )
                self.mode_pub.publish(String(data=str(output_mode)))

        if is_dynamic:
            mrks.markers.append(self.create_dynamic_mode_marker())
        else:
            mrks.markers.append(self.delete_dynamic_mode_marker())

        self.mrks_pub.publish(mrks)
        
    # RViz에 그릴 line marker를 만든다.
    def create_marker(self, waypoints, marker_id): 
        marker = Marker() 
        marker.header.frame_id, marker.header.stamp = "map", rospy.Time.now() 
        marker.type, marker.action = Marker.LINE_STRIP, Marker.ADD 
        marker.pose.orientation.w = 1.0  
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15  
        marker.color.a, marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 1.0, 0.0 
        for wp in waypoints: 
            p = Point()
            p.x, p.y, p.z = wp[0], wp[1], 1.0
            marker.points.append(p) 
        marker.id = marker_id 
        return marker 

    def create_follow_distance_markers(self, follow_path):
        """FOLLOW 접근 구간과 앞차까지의 목표 1 m 구간을 색으로 구분한다."""
        if (
            self.converter is None
            or self.gb_max_s is None
            or self.gb_max_s <= 0.0
            or "obstacle_rear_s" not in follow_path
        ):
            return []

        path_s = np.asarray(follow_path.get("s", []), dtype=float)
        path_d = np.asarray(follow_path.get("d", []), dtype=float)
        if len(path_s) < 2 or len(path_s) != len(path_d):
            return []

        cur_s = float(path_s[0])
        obstacle_rear_s = self.scalar(follow_path["obstacle_rear_s"])
        obstacle_rear_unwrapped = cur_s + float(
            self.forward_s_distance(cur_s, obstacle_rear_s)
        )

        # 이 위치에 자차 앞 범퍼가 도착하면 앞차 뒤쪽과 정확히
        # follow_min_distance만큼 떨어진다.
        target_front_s = obstacle_rear_unwrapped - self.follow_min_distance
        target_d = float(np.interp(target_front_s, path_s, path_d))
        obstacle_d = self.scalar(follow_path.get("obstacle_d", target_d))
        markers = []

        # 청록색: 현재 위치에서 목표 1 m 지점까지 실제로 접근할 수 있는 구간.
        if target_front_s > cur_s:
            approach_mask = path_s < target_front_s
            approach_s = path_s[approach_mask]
            approach_d = path_d[approach_mask]
            approach_s = np.append(approach_s, target_front_s)
            approach_d = np.append(approach_d, target_d)
            approach_xy = self.converter.get_cartesian(
                approach_s % self.gb_max_s,
                np.clip(approach_d, -self.max_offset, self.max_offset)
            )
            if not np.any(~np.isfinite(approach_xy)):
                points = np.column_stack(
                    (approach_xy[0], approach_xy[1])
                ).tolist()
                marker = self.create_marker(points, 0)
                marker.ns = "follow_approach_to_1m"
                marker.scale.x = 0.18
                marker.color.r = 0.00
                marker.color.g = 0.90
                marker.color.b = 1.00
                marker.color.a = 1.00
                markers.append(marker)

        # 빨간색: 목표 지점부터 앞차 뒤쪽까지 비워 두어야 하는 1 m 구간.
        gap_samples = max(
            int(math.ceil(self.follow_min_distance / 0.05)) + 1,
            2
        )
        gap_s = np.linspace(
            target_front_s,
            obstacle_rear_unwrapped,
            gap_samples
        )
        gap_d = np.linspace(target_d, obstacle_d, gap_samples)
        gap_xy = self.converter.get_cartesian(
            gap_s % self.gb_max_s,
            np.clip(gap_d, -self.max_offset, self.max_offset)
        )
        if not np.any(~np.isfinite(gap_xy)):
            points = np.column_stack((gap_xy[0], gap_xy[1])).tolist()
            marker = self.create_marker(points, 1)
            marker.ns = "follow_reserved_1m_gap"
            marker.scale.x = 0.22
            marker.color.r = 1.00
            marker.color.g = 0.10
            marker.color.b = 0.00
            marker.color.a = 1.00
            markers.append(marker)

        return markers

    # 동적 장애물 모드일 때 차량 위에 표시할 text marker를 만든다.
    def create_dynamic_mode_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        marker.ns = "dynamic_mode_text"
        marker.id = 9999
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = self.vehicle_x
        marker.pose.position.y = self.vehicle_y
        marker.pose.position.z = 1.2
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.60

        state = self.overtake_display_state(
            self.follow_target_cache if self.follow_target_cache is not None else None
        )
        colors = {
            "FOLLOW": (0.10, 0.35, 1.00),
            "READY": (1.00, 0.55, 0.00),
            "SHIFT": (0.00, 0.90, 1.00),
            "PASS": (1.00, 0.00, 0.85),
            "RETURN": (0.10, 1.00, 0.20),
        }
        marker.color.r, marker.color.g, marker.color.b = colors.get(
            state,
            (0.0, 0.0, 1.0)
        )
        marker.color.a = 1.0

        marker.text = state
        marker.lifetime = rospy.Duration(0.3)

        return marker

    # 실차 global trajectory를 기다린 뒤 FrenetConverter를 만든다.
    def initialize_converter(self):
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message(
                self.global_topic,
                WpntArray
            )

            if not msg.wpnts:
                rospy.logwarn_throttle(
                    1.0,
                    "[REAL GLOBAL] empty waypoint array"
                )
                continue

            init_waypoints = np.array([
                [wpnt.x_m, wpnt.y_m, wpnt.s_m]
                for wpnt in msg.wpnts
            ])

            return FrenetConverter(
                init_waypoints[:, 0],
                init_waypoints[:, 1],
                init_waypoints[:, 2]
            )

        return None

if __name__ == "__main__": 
    planner = StaticObstacleLatticePlanner() 
    planner.loop()
