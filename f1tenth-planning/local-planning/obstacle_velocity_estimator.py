#!/usr/bin/env python3

import rospy
import copy
import math
import numpy as np

from f110_msgs.msg import ObstacleArray, WpntArray
from visualization_msgs.msg import Marker, MarkerArray
from frenet_utils import FrenetConverter


class ObstacleVelocityEstimator:
    def __init__(self):
        rospy.init_node("obstacle_velocity_estimator")

        # ----------------------------
        # Parameters
        # ----------------------------

        self.alpha = rospy.get_param("~velocity_alpha", 0.3)

        # 이전/현재 장애물을 같은 물체로 볼 최대 거리
        self.match_max_distance = rospy.get_param(
            "~match_max_distance", 0.8
        )

        # 동적 장애물 판단용 속도
        self.dynamic_speed_threshold = rospy.get_param(
            "~dynamic_speed_threshold", 0.6
        )

        self.track_length = None
        self.converter = None

        # 이전 프레임 track
        self.prev_tracks = []

        # 영구적인 obstacle ID
        self.next_id = 0


        # ----------------------------
        # Publisher
        # ----------------------------

        self.pub = rospy.Publisher(
            "/perception/detection/tracked_obstacles",
            ObstacleArray,
            queue_size=5
        )
        self.dynamic_marker_pub = rospy.Publisher(
            "/perception/dynamic_obstacles_markers",
            MarkerArray,
            queue_size=5
        )


        # ----------------------------
        # Subscribers
        # ----------------------------

        rospy.Subscriber(
            "/global_path/optimal_trajectory_wpnt",
            WpntArray,
            self.path_callback,
            queue_size=1
        )

        rospy.Subscriber(
            "/perception/detection/raw_obstacles",
            ObstacleArray,
            self.obstacle_callback,
            queue_size=1
        )


    def path_callback(self, msg):
        if msg.wpnts:
            self.track_length = msg.wpnts[-1].s_m
            self.converter = FrenetConverter(
                np.asarray([wpnt.x_m for wpnt in msg.wpnts]),
                np.asarray([wpnt.y_m for wpnt in msg.wpnts]),
                np.asarray([wpnt.s_m for wpnt in msg.wpnts])
            )


    def publish_dynamic_markers(self, obstacles, stamp):
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        if self.converter is None:
            self.dynamic_marker_pub.publish(markers)
            return

        for obs in obstacles:
            obs_speed = float(getattr(obs, "vs", getattr(obs, "v_s", 0.0)))
            if abs(obs_speed) < self.dynamic_speed_threshold:
                continue

            xy = self.converter.get_cartesian(
                np.asarray([float(obs.s_center)]),
                np.asarray([float(obs.d_center)])
            )

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = stamp
            marker.ns = "dynamic_obstacles"
            marker.id = int(obs.id)
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(np.asarray(xy[0]).reshape(-1)[0])
            marker.pose.position.y = float(np.asarray(xy[1]).reshape(-1)[0])
            marker.pose.position.z = max(float(obs.size), 0.05) * 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(float(obs.size), 0.05)
            marker.scale.y = max(float(obs.size), 0.05)
            marker.scale.z = max(float(obs.size), 0.05)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration(0.3)
            markers.markers.append(marker)

        self.dynamic_marker_pub.publish(markers)


    def normalize_s(self, ds):
        if self.track_length is None:
            return ds

        ds = ds % self.track_length

        if ds > self.track_length / 2.0:
            ds -= self.track_length

        return ds


    def obstacle_callback(self, msg):

        if self.track_length is None:
            return

        # 메시지 timestamp 사용
        stamp = msg.header.stamp

        if stamp == rospy.Time():
            stamp = rospy.Time.now()

        current_time = stamp.to_sec()

        output = copy.deepcopy(msg)

        current_tracks = []

        # 한 previous track을 여러 장애물이 동시에 가져가지 않도록
        used_prev = set()


        # =============================================
        # 현재 장애물 각각에 대해 이전 장애물 찾기
        # =============================================

        for obs in output.obstacles:

            s_now = float(obs.s_center)
            d_now = float(obs.d_center)

            best_idx = None
            best_distance = float("inf")


            # -----------------------------------------
            # 1. 이전 프레임과 nearest-neighbor matching
            # -----------------------------------------

            for i, prev in enumerate(self.prev_tracks):

                if i in used_prev:
                    continue

                ds = self.normalize_s(
                    s_now - prev["s"]
                )

                dd = d_now - prev["d"]

                distance = math.sqrt(
                    ds * ds + dd * dd
                )

                if distance < best_distance:
                    best_distance = distance
                    best_idx = i


            # =========================================
            # 같은 장애물이라고 판단
            # =========================================

            if (
                best_idx is not None
                and best_distance < self.match_max_distance
            ):

                prev = self.prev_tracks[best_idx]
                used_prev.add(best_idx)

                dt = current_time - prev["time"]

                if dt > 1e-3:

                    # ---------------------------------
                    # 2. 위치 변화 / 시간 = 속도
                    # ---------------------------------

                    ds = self.normalize_s(
                        s_now - prev["s"]
                    )

                    dd = d_now - prev["d"]

                    raw_vs = ds / dt
                    raw_vd = dd / dt


                    # ---------------------------------
                    # 3. EMA filter
                    # ---------------------------------

                    vs = (
                        self.alpha * raw_vs
                        + (1.0 - self.alpha) * prev["vs"]
                    )

                    vd = (
                        self.alpha * raw_vd
                        + (1.0 - self.alpha) * prev["vd"]
                    )

                else:
                    vs = prev["vs"]
                    vd = prev["vd"]

                track_id = prev["id"]


            # =========================================
            # 새로 등장한 장애물
            # =========================================

            else:
                track_id = self.next_id
                self.next_id += 1

                vs = 0.0
                vd = 0.0


            # =========================================
            # ObstacleMessage에 결과 저장
            # =========================================

            obs.id = track_id

            if hasattr(obs, "vs"):
                obs.vs = vs

            if hasattr(obs, "vd"):
                obs.vd = vd

            if hasattr(obs, "is_static"):
                obs.is_static = (
                    abs(vs)
                    < self.dynamic_speed_threshold
                )

            if hasattr(obs, "is_visible"):
                obs.is_visible = True


            current_tracks.append({
                "id": track_id,
                "s": s_now,
                "d": d_now,
                "vs": vs,
                "vd": vd,
                "time": current_time
            })


        # 현재 프레임을 다음 프레임의 prev로 저장
        self.prev_tracks = current_tracks

        self.pub.publish(output)
        self.publish_dynamic_markers(output.obstacles, stamp)


if __name__ == "__main__":
    node = ObstacleVelocityEstimator()
    rospy.spin()
