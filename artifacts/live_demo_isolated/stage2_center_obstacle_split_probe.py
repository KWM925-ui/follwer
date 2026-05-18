#!/usr/bin/env python3
import json
import math
import time

import rospy
from ego_planner.msg import Bspline
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

from human_follow_msgs.msg import FollowState


def yaw_from_quaternion(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
    )


class CenterObstacleSplitProbe:
    def __init__(self, clearance_override_m=None):
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.last_odom = None
        self.last_state = None
        self.last_stage2_goal = None
        self.last_ego_goal = None
        self.last_waypoints = None
        self.last_bspline = None
        self.last_position_cmd = None
        self.last_occupancy_stamp = None
        self.occupied_cells = set()

        self.occupancy_resolution_m = float(rospy.get_param("/ego_planner_node/grid_map/resolution", 0.1))
        default_clearance_m = max(
            float(rospy.get_param("/stage2_follow_goal_generator/goal_obstacle_clearance_m", 0.18)),
            0.0,
        )
        if clearance_override_m is None:
            self.goal_clearance_m = default_clearance_m
        else:
            self.goal_clearance_m = max(float(clearance_override_m), 0.0)

        rospy.Subscriber("/follow/sim/vehicle_odom", Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/state", FollowState, self._state_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/goal", PoseStamped, self._stage2_goal_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/ego_goal", PoseStamped, self._ego_goal_cb, queue_size=50)
        rospy.Subscriber("/waypoint_generator/waypoints", Path, self._waypoints_cb, queue_size=50)
        rospy.Subscriber("/planning/bspline", Bspline, self._bspline_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/ego_position_cmd", PositionCommand, self._position_cmd_cb, queue_size=50)
        rospy.Subscriber("/grid_map/occupancy_inflate", PointCloud2, self._occupancy_cb, queue_size=5)

    def _odom_cb(self, msg):
        self.last_odom = msg

    def _state_cb(self, msg):
        self.last_state = msg

    def _stage2_goal_cb(self, msg):
        self.last_stage2_goal = msg

    def _ego_goal_cb(self, msg):
        self.last_ego_goal = msg

    def _waypoints_cb(self, msg):
        self.last_waypoints = msg

    def _bspline_cb(self, msg):
        self.last_bspline = msg

    def _position_cmd_cb(self, msg):
        self.last_position_cmd = msg

    def _occupancy_cb(self, msg):
        occupied = set()
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            occupied.add(self._cell_key(point[0], point[1], point[2]))
        self.occupied_cells = occupied
        self.last_occupancy_stamp = time.time()

    def _cell_key(self, x_value, y_value, z_value):
        resolution = max(self.occupancy_resolution_m, 0.02)
        return (
            int(round(float(x_value) / resolution)),
            int(round(float(y_value) / resolution)),
            int(round(float(z_value) / resolution)),
        )

    def _state_snapshot(self):
        if self.last_state is None:
            return None
        return {
            "state_name": str(self.last_state.state_name),
            "detail": str(self.last_state.detail),
        }

    def _point_dict(self, x_value, y_value, z_value):
        return {
            "x": round(float(x_value), 3),
            "y": round(float(y_value), 3),
            "z": round(float(z_value), 3),
        }

    def _point_is_in_inflated_obstacle(self, x_value, y_value, z_value):
        if not self.occupied_cells:
            return False
        radius_cells = max(
            int(math.ceil(self.goal_clearance_m / max(self.occupancy_resolution_m, 0.02))),
            0,
        )
        center = self._cell_key(x_value, y_value, z_value)
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

    def _wait_basic_ready(self, timeout_sec=8.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.last_odom is not None and self.last_state is not None:
                return True
            time.sleep(0.05)
        return False

    def _wait_occupancy(self, timeout_sec=8.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.last_occupancy_stamp is not None and self.occupied_cells:
                return True
            time.sleep(0.05)
        return False

    def _publish_center_goal(self):
        odom = self.last_odom
        yaw_rad = yaw_from_quaternion(odom.pose.pose.orientation)
        start_x = float(odom.pose.pose.position.x)
        start_y = float(odom.pose.pose.position.y)
        goal_x = start_x + math.cos(yaw_rad) * 10.5
        goal_y = start_y + math.sin(yaw_rad) * 10.5

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        for _ in range(3):
            self.goal_pub.publish(msg)
            time.sleep(0.05)
        return goal_x, goal_y

    def _first_point_in_occ_from_path(self, path_msg):
        if path_msg is None:
            return None
        for pose_stamped in path_msg.poses:
            point = pose_stamped.pose.position
            if self._point_is_in_inflated_obstacle(point.x, point.y, point.z):
                return self._point_dict(point.x, point.y, point.z)
        return None

    def _first_point_in_occ_from_bspline(self, bspline_msg):
        if bspline_msg is None:
            return None
        for point in bspline_msg.pos_pts:
            if self._point_is_in_inflated_obstacle(point.x, point.y, point.z):
                return self._point_dict(point.x, point.y, point.z)
        return None

    def run(self, dwell_sec=8.0):
        if not self._wait_basic_ready():
            return {"status": "failed", "reason": "wait_basic_ready_timeout"}

        had_occupancy_before_goal = bool(self.last_occupancy_stamp is not None and self.occupied_cells)
        if not had_occupancy_before_goal and self._wait_occupancy():
            had_occupancy_before_goal = True

        published_goal_xy = self._publish_center_goal()
        if not self.occupied_cells and not self._wait_occupancy():
            return {"status": "failed", "reason": "occupancy_timeout_after_goal"}
        start_time = time.time()
        first_stage2_goal_in_occ = None
        first_ego_goal_in_occ = None
        first_waypoint_in_occ = None
        first_bspline_in_occ = None
        first_position_cmd_in_occ = None
        first_odom_in_occ = None

        while time.time() < start_time + dwell_sec and not rospy.is_shutdown():
            now_rel = time.time() - start_time
            state_snapshot = self._state_snapshot()

            if first_stage2_goal_in_occ is None and self.last_stage2_goal is not None:
                point = self.last_stage2_goal.pose.position
                if self._point_is_in_inflated_obstacle(point.x, point.y, point.z):
                    first_stage2_goal_in_occ = {
                        "t_sec": round(now_rel, 3),
                        "point": self._point_dict(point.x, point.y, point.z),
                        "state": state_snapshot,
                    }

            if first_ego_goal_in_occ is None and self.last_ego_goal is not None:
                point = self.last_ego_goal.pose.position
                if self._point_is_in_inflated_obstacle(point.x, point.y, point.z):
                    first_ego_goal_in_occ = {
                        "t_sec": round(now_rel, 3),
                        "point": self._point_dict(point.x, point.y, point.z),
                        "state": state_snapshot,
                    }

            if first_waypoint_in_occ is None and self.last_waypoints is not None:
                point_dict = self._first_point_in_occ_from_path(self.last_waypoints)
                if point_dict is not None:
                    first_waypoint_in_occ = {
                        "t_sec": round(now_rel, 3),
                        "point": point_dict,
                        "state": state_snapshot,
                    }

            if first_bspline_in_occ is None and self.last_bspline is not None:
                point_dict = self._first_point_in_occ_from_bspline(self.last_bspline)
                if point_dict is not None:
                    first_bspline_in_occ = {
                        "t_sec": round(now_rel, 3),
                        "point": point_dict,
                        "state": state_snapshot,
                    }

            if first_position_cmd_in_occ is None and self.last_position_cmd is not None:
                point = self.last_position_cmd.position
                if self._point_is_in_inflated_obstacle(point.x, point.y, point.z):
                    first_position_cmd_in_occ = {
                        "t_sec": round(now_rel, 3),
                        "point": self._point_dict(point.x, point.y, point.z),
                        "state": state_snapshot,
                    }

            if first_odom_in_occ is None and self.last_odom is not None:
                point = self.last_odom.pose.pose.position
                if self._point_is_in_inflated_obstacle(point.x, point.y, point.z):
                    first_odom_in_occ = {
                        "t_sec": round(now_rel, 3),
                        "point": self._point_dict(point.x, point.y, point.z),
                        "state": state_snapshot,
                    }

            time.sleep(0.05)

        return {
            "status": "ok",
            "input_goal_map": {
                "x": round(float(published_goal_xy[0]), 3),
                "y": round(float(published_goal_xy[1]), 3),
            },
            "had_occupancy_before_goal": had_occupancy_before_goal,
            "goal_clearance_m": round(float(self.goal_clearance_m), 3),
            "occupancy_resolution_m": round(float(self.occupancy_resolution_m), 3),
            "first_stage2_goal_in_occ": first_stage2_goal_in_occ,
            "first_ego_goal_in_occ": first_ego_goal_in_occ,
            "first_waypoint_in_occ": first_waypoint_in_occ,
            "first_bspline_in_occ": first_bspline_in_occ,
            "first_position_cmd_in_occ": first_position_cmd_in_occ,
            "first_odom_in_occ": first_odom_in_occ,
            "final_state": self._state_snapshot(),
        }


def main():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--clearance-m", type=float, default=None)
    args = parser.parse_args()

    rospy.init_node("stage2_center_obstacle_split_probe", anonymous=True)
    probe = CenterObstacleSplitProbe(clearance_override_m=args.clearance_m)
    result = probe.run()
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    if result.get("status") != "ok":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
