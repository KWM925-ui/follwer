#!/usr/bin/env python3
import argparse
import json
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from human_follow_msgs.msg import FollowState


def yaw_from_quaternion(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
    )


def rounded_or_none(value):
    if value is None:
        return None
    return round(float(value), 3)


class Stage2GoalChainHistoryProbe:
    def __init__(self, extra_clearance_m=None):
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        self.goal_send_time = None
        self.armed_external_goal_after_time = None
        self.external_goal_point = None
        self.last_odom = None
        self.last_state = None
        self.last_state_recv_time = None
        self.last_occupancy_stamp = None
        self.last_occupancy_header_stamp_sec = None
        self.occupied_cells = set()

        self.goal_events = []
        self.ego_goal_events = []
        self.waypoint_events = []

        self.occupancy_resolution_m = float(rospy.get_param("/ego_planner_node/grid_map/resolution", 0.1))
        self.map_inflation_m = max(
            float(rospy.get_param("/ego_planner_node/grid_map/obstacles_inflation", 0.1)),
            0.0,
        )
        if extra_clearance_m is None:
            self.extra_clearance_m = 0.05
        else:
            self.extra_clearance_m = max(float(extra_clearance_m), 0.0)

        rospy.Subscriber("/follow/sim/vehicle_odom", Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/state", FollowState, self._state_cb, queue_size=50)
        rospy.Subscriber("/grid_map/occupancy_inflate", PointCloud2, self._occupancy_cb, queue_size=5)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self._nav_goal_cb, queue_size=20)
        rospy.Subscriber("/follow/stage2/goal", PoseStamped, self._stage2_goal_cb, queue_size=100)
        rospy.Subscriber("/follow/stage2/ego_goal", PoseStamped, self._ego_goal_cb, queue_size=100)
        rospy.Subscriber("/waypoint_generator/waypoints", Path, self._waypoint_cb, queue_size=100)

    def _odom_cb(self, msg):
        self.last_odom = msg

    def _state_cb(self, msg):
        self.last_state = msg
        self.last_state_recv_time = time.time()

    def _occupancy_cb(self, msg):
        occupied = set()
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            occupied.add(self._cell_key(point[0], point[1], point[2]))
        self.occupied_cells = occupied
        self.last_occupancy_stamp = time.time()
        self.last_occupancy_header_stamp_sec = (
            None if msg.header.stamp is None else float(msg.header.stamp.to_sec())
        )

    def _nav_goal_cb(self, msg):
        recv_time = time.time()
        if self.armed_external_goal_after_time is None:
            return
        if recv_time < self.armed_external_goal_after_time:
            return
        if self.goal_send_time is not None:
            return
        self.goal_send_time = recv_time
        self.external_goal_point = {
            "frame_id": str(msg.header.frame_id),
            "point": self._point_dict(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ),
        }

    def _stage2_goal_cb(self, msg):
        self._record_pose_event(
            self.goal_events,
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.header.stamp.to_sec() if msg.header.stamp else None,
            time.time(),
            "stage2_goal",
        )

    def _ego_goal_cb(self, msg):
        self._record_pose_event(
            self.ego_goal_events,
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.header.stamp.to_sec() if msg.header.stamp else None,
            time.time(),
            "ego_goal",
        )

    def _waypoint_cb(self, msg):
        recv_time = time.time()
        if not msg.poses:
            return
        pose = msg.poses[0]
        event = self._build_event(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.header.stamp.to_sec() if pose.header.stamp else None,
            recv_time,
            "waypoint",
        )
        if event is None:
            return
        event["path_pose_count"] = int(len(msg.poses))
        self.waypoint_events.append(event)

    def _record_pose_event(self, target_list, x_value, y_value, z_value, header_stamp_sec, recv_time, topic_name):
        event = self._build_event(x_value, y_value, z_value, header_stamp_sec, recv_time, topic_name)
        if event is None:
            return
        target_list.append(event)

    def _build_event(self, x_value, y_value, z_value, header_stamp_sec, recv_time, topic_name):
        if self.goal_send_time is None or recv_time < self.goal_send_time:
            return None
        occupancy_age_sec = None
        occupancy_rel_t_sec = None
        if self.last_occupancy_stamp is not None:
            occupancy_age_sec = max(0.0, float(recv_time - self.last_occupancy_stamp))
            occupancy_rel_t_sec = max(0.0, float(self.last_occupancy_stamp - self.goal_send_time))
        odom_snapshot = None
        relative_from_odom = None
        if self.last_odom is not None:
            odom_x = float(self.last_odom.pose.pose.position.x)
            odom_y = float(self.last_odom.pose.pose.position.y)
            odom_z = float(self.last_odom.pose.pose.position.z)
            odom_snapshot = self._point_dict(odom_x, odom_y, odom_z)
            relative_from_odom = {
                "distance_xy_m": round(math.hypot(float(x_value) - odom_x, float(y_value) - odom_y), 3),
                "bearing_deg": round(math.degrees(math.atan2(float(y_value) - odom_y, float(x_value) - odom_x)), 3),
            }
        return {
            "topic": topic_name,
            "recv_t_sec": round(float(recv_time - self.goal_send_time), 3),
            "header_stamp_sec": rounded_or_none(header_stamp_sec),
            "point": self._point_dict(x_value, y_value, z_value),
            "point_key": self._point_key(x_value, y_value, z_value),
            "occupied_now": bool(self.point_is_in_inflated_obstacle(x_value, y_value, z_value)),
            "occupancy_age_sec": rounded_or_none(occupancy_age_sec),
            "occupancy_rel_t_sec": rounded_or_none(occupancy_rel_t_sec),
            "occupancy_header_stamp_sec": rounded_or_none(self.last_occupancy_header_stamp_sec),
            "odom_snapshot": odom_snapshot,
            "relative_from_odom": relative_from_odom,
            "state": self._state_snapshot(),
        }

    def _cell_key(self, x_value, y_value, z_value):
        resolution = max(self.occupancy_resolution_m, 0.02)
        return (
            int(round(float(x_value) / resolution)),
            int(round(float(y_value) / resolution)),
            int(round(float(z_value) / resolution)),
        )

    def _point_key(self, x_value, y_value, z_value):
        return (
            round(float(x_value), 3),
            round(float(y_value), 3),
            round(float(z_value), 3),
        )

    def _point_dict(self, x_value, y_value, z_value):
        return {
            "x": round(float(x_value), 3),
            "y": round(float(y_value), 3),
            "z": round(float(z_value), 3),
        }

    def _state_snapshot(self):
        if self.last_state is None:
            return None
        return {
            "state_name": str(self.last_state.state_name),
            "detail": str(self.last_state.detail),
        }

    def point_is_in_inflated_obstacle(self, x_value, y_value, z_value):
        if not self.occupied_cells:
            return False
        radius_cells = max(
            int(math.ceil(self.extra_clearance_m / max(self.occupancy_resolution_m, 0.02))),
            0,
        )
        center = self._cell_key(x_value, y_value, z_value)
        if radius_cells == 0:
            return center in self.occupied_cells
        for dx_value in range(-radius_cells, radius_cells + 1):
            for dy_value in range(-radius_cells, radius_cells + 1):
                for dz_value in range(-radius_cells, radius_cells + 1):
                    if dx_value * dx_value + dy_value * dy_value + dz_value * dz_value > radius_cells * radius_cells:
                        continue
                    if (
                        center[0] + dx_value,
                        center[1] + dy_value,
                        center[2] + dz_value,
                    ) in self.occupied_cells:
                        return True
        return False

    def wait_odom(self, timeout_sec=8.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.last_odom is not None:
                return True
            time.sleep(0.05)
        return False

    def wait_occupancy_after(self, earliest_time, timeout_sec=8.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.last_occupancy_stamp is not None and self.last_occupancy_stamp >= earliest_time and self.occupied_cells:
                return True
            time.sleep(0.05)
        return False

    def send_goal(self, x_value, y_value, z_value=0.0):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = x_value
        msg.pose.position.y = y_value
        msg.pose.position.z = z_value
        msg.pose.orientation.w = 1.0
        for _ in range(3):
            self.goal_pub.publish(msg)
            time.sleep(0.05)

    def wait_external_goal(self, timeout_sec=8.0):
        self.goal_send_time = None
        self.external_goal_point = None
        self.armed_external_goal_after_time = time.time()
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.goal_send_time is not None:
                self.armed_external_goal_after_time = None
                return True
            time.sleep(0.05)
        self.armed_external_goal_after_time = None
        return False

    def _first_dirty_event(self, events):
        for event in events:
            if event["occupied_now"]:
                return event
        return None

    def _find_exact_stage2_counterpart(self, ego_event):
        if ego_event is None:
            return None
        target_header = ego_event.get("header_stamp_sec")
        target_point_key = tuple(ego_event.get("point_key", ()))
        for event in self.goal_events:
            if tuple(event.get("point_key", ())) != target_point_key:
                continue
            if target_header is None and event.get("header_stamp_sec") is None:
                return event
            if target_header is not None and abs(float(event.get("header_stamp_sec", 0.0)) - float(target_header)) <= 1e-3:
                return event
        return None

    def _find_point_counterpart(self, source_event, candidate_events, max_recv_dt_sec=0.2):
        if source_event is None:
            return None
        target_point_key = tuple(source_event.get("point_key", ()))
        best_event = None
        best_recv_dt = None
        for event in candidate_events:
            if tuple(event.get("point_key", ())) != target_point_key:
                continue
            recv_dt = abs(float(event["recv_t_sec"]) - float(source_event["recv_t_sec"]))
            if best_recv_dt is None or recv_dt < best_recv_dt:
                best_recv_dt = recv_dt
                best_event = event
        if best_event is None:
            return None
        if best_recv_dt is not None and best_recv_dt > max_recv_dt_sec:
            return None
        return best_event

    def _event_counts(self, events):
        dirty_count = 0
        for event in events:
            if event["occupied_now"]:
                dirty_count += 1
        return {
            "total": int(len(events)),
            "dirty": int(dirty_count),
        }

    def _pair_summary(self, source_event, counterpart_event, match_rule):
        if source_event is None:
            return None
        summary = {
            "match_rule": match_rule,
            "source": source_event,
            "counterpart_found": counterpart_event is not None,
        }
        if counterpart_event is None:
            return summary
        summary["counterpart"] = counterpart_event
        summary["recv_dt_sec"] = round(
            abs(float(counterpart_event["recv_t_sec"]) - float(source_event["recv_t_sec"])),
            3,
        )
        return summary

    def run(self, goal_forward_m=10.5, dwell_sec=10.0, prewait_occupancy_sec=0.0, wait_external_goal=False):
        if not self.wait_odom():
            return {"status": "failed", "reason": "odom_timeout"}

        odom = self.last_odom
        yaw_rad = yaw_from_quaternion(odom.pose.pose.orientation)
        start_x = float(odom.pose.pose.position.x)
        start_y = float(odom.pose.pose.position.y)
        start_z = float(odom.pose.pose.position.z)
        goal_xy = (
            start_x + math.cos(yaw_rad) * float(goal_forward_m),
            start_y + math.sin(yaw_rad) * float(goal_forward_m),
        )

        had_occupancy_before_goal = self.last_occupancy_stamp is not None and bool(self.occupied_cells)
        if prewait_occupancy_sec > 0.0:
            if not self.wait_occupancy_after(0.0):
                return {
                    "status": "failed",
                    "reason": "occupancy_timeout_before_goal",
                    "had_occupancy_before_goal": bool(had_occupancy_before_goal),
                }
            time.sleep(float(prewait_occupancy_sec))
            had_occupancy_before_goal = self.last_occupancy_stamp is not None and bool(self.occupied_cells)

        if wait_external_goal:
            if not self.wait_external_goal(timeout_sec=8.0):
                return {"status": "failed", "reason": "external_goal_timeout"}
        else:
            self.goal_send_time = time.time()
            self.external_goal_point = {
                "frame_id": "map",
                "point": self._point_dict(goal_xy[0], goal_xy[1], 0.0),
            }
            self.send_goal(*goal_xy)
        if not self.wait_occupancy_after(self.goal_send_time):
            return {
                "status": "failed",
                "reason": "occupancy_timeout_after_goal",
                "had_occupancy_before_goal": bool(had_occupancy_before_goal),
            }

        deadline = time.time() + float(dwell_sec)
        while time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.05)

        first_dirty_stage2 = self._first_dirty_event(self.goal_events)
        first_dirty_ego = self._first_dirty_event(self.ego_goal_events)
        first_dirty_waypoint = self._first_dirty_event(self.waypoint_events)

        exact_stage2_for_ego = self._find_exact_stage2_counterpart(first_dirty_ego)
        waypoint_for_ego = self._find_point_counterpart(first_dirty_ego, self.waypoint_events)
        ego_for_waypoint = self._find_point_counterpart(first_dirty_waypoint, self.ego_goal_events)

        return {
            "status": "ok",
            "goal_xy": [round(goal_xy[0], 3), round(goal_xy[1], 3)],
            "start_xyz": [round(start_x, 3), round(start_y, 3), round(start_z, 3)],
            "wait_external_goal": bool(wait_external_goal),
            "external_goal_point": self.external_goal_point,
            "had_occupancy_before_goal": bool(had_occupancy_before_goal),
            "map_inflation_m": round(float(self.map_inflation_m), 3),
            "extra_clearance_m": round(float(self.extra_clearance_m), 3),
            "effective_total_clearance_m": round(float(self.map_inflation_m + self.extra_clearance_m), 3),
            "counts": {
                "stage2_goal": self._event_counts(self.goal_events),
                "ego_goal": self._event_counts(self.ego_goal_events),
                "waypoint": self._event_counts(self.waypoint_events),
            },
            "first_dirty_stage2_goal": first_dirty_stage2,
            "first_dirty_ego_goal": first_dirty_ego,
            "first_dirty_waypoint": first_dirty_waypoint,
            "first_dirty_ego_exact_stage2_counterpart": self._pair_summary(
                first_dirty_ego,
                exact_stage2_for_ego,
                "same_header_stamp_and_same_point",
            ),
            "first_dirty_ego_waypoint_counterpart": self._pair_summary(
                first_dirty_ego,
                waypoint_for_ego,
                "same_point_closest_recv_time",
            ),
            "first_dirty_waypoint_ego_counterpart": self._pair_summary(
                first_dirty_waypoint,
                ego_for_waypoint,
                "same_point_closest_recv_time",
            ),
            "recent_stage2_goal_events": self.goal_events[-8:],
            "recent_ego_goal_events": self.ego_goal_events[-8:],
            "recent_waypoint_events": self.waypoint_events[-8:],
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-forward-m", type=float, default=10.5)
    parser.add_argument("--dwell-sec", type=float, default=10.0)
    parser.add_argument("--prewait-occupancy-sec", type=float, default=0.0)
    parser.add_argument("--extra-clearance-m", type=float, default=0.05)
    parser.add_argument("--wait-external-goal", action="store_true")
    args = parser.parse_args()

    rospy.init_node("stage2_goal_chain_history_probe", anonymous=True)
    probe = Stage2GoalChainHistoryProbe(extra_clearance_m=args.extra_clearance_m)
    result = probe.run(
        goal_forward_m=float(args.goal_forward_m),
        dwell_sec=float(args.dwell_sec),
        prewait_occupancy_sec=float(args.prewait_occupancy_sec),
        wait_external_goal=bool(args.wait_external_goal),
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result.get("status") != "ok":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
