#!/usr/bin/env python3
import argparse
import json
import math
import time

import rospy
from geometry_msgs.msg import PoseStamped
from human_follow_msgs.msg import FollowState
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool


def yaw_from_quaternion(quat):
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z),
    )


class DemoVerifier:
    def __init__(self, extra_clearance_override_m=None):
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.last_odom = None
        self.last_stage2_state = None
        self.last_occupancy_stamp = None
        self.occupancy_resolution_m = float(rospy.get_param("/ego_planner_node/grid_map/resolution", 0.1))
        self.map_inflation_m = max(
            float(rospy.get_param("/ego_planner_node/grid_map/obstacles_inflation", 0.1)),
            0.0,
        )
        if extra_clearance_override_m is None:
            self.extra_clearance_m = 0.05
        else:
            self.extra_clearance_m = max(float(extra_clearance_override_m), 0.0)
        self.occupied_cells = set()
        rospy.Subscriber("/follow/sim/vehicle_odom", Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/state", FollowState, self._stage2_cb, queue_size=50)
        rospy.Subscriber("/grid_map/occupancy_inflate", PointCloud2, self._occupancy_cb, queue_size=5)

    def _odom_cb(self, msg):
        self.last_odom = msg

    def _stage2_cb(self, msg):
        self.last_stage2_state = msg

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

    def wait_occupancy(self, timeout_sec=8.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.last_occupancy_stamp is not None and self.occupied_cells:
                return True
            time.sleep(0.05)
        return False

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

    def set_visible(self, visible_value, timeout_sec=4.0):
        service = rospy.ServiceProxy("/human_truth_keyboard_teleop/set_visible", SetBool)
        service.wait_for_service(timeout=timeout_sec)
        return service(bool(visible_value))

    def _run_case(self, goal_xy, duration_sec, obstacle_box=None):
        start_x = self.last_odom.pose.pose.position.x
        start_y = self.last_odom.pose.pose.position.y
        start_z = self.last_odom.pose.pose.position.z
        max_travel = 0.0
        entered_obstacle = False
        max_abs_lateral_offset = 0.0
        samples = []
        deadline = time.time() + duration_sec
        while time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.25)
            if self.last_odom is None:
                continue
            x_value = self.last_odom.pose.pose.position.x
            y_value = self.last_odom.pose.pose.position.y
            z_value = self.last_odom.pose.pose.position.z
            travel = math.hypot(x_value - start_x, y_value - start_y)
            max_travel = max(max_travel, travel)
            max_abs_lateral_offset = max(max_abs_lateral_offset, abs(y_value - start_y))
            if obstacle_box is not None:
                x_min, x_max, y_min, y_max = obstacle_box
                if x_min <= x_value <= x_max and y_min <= y_value <= y_max:
                    entered_obstacle = True
            if self.point_is_in_inflated_obstacle(x_value, y_value, z_value):
                entered_obstacle = True
            if self.last_stage2_state is not None:
                samples.append((self.last_stage2_state.state_name, self.last_stage2_state.detail))
        return {
            "goal_xy": [round(goal_xy[0], 3), round(goal_xy[1], 3)],
            "max_travel_xy_m": round(max_travel, 3),
            "max_abs_lateral_offset_m": round(max_abs_lateral_offset, 3),
            "entered_obstacle_box": entered_obstacle,
            "map_inflation_m": round(float(self.map_inflation_m), 3),
            "extra_clearance_m": round(float(self.extra_clearance_m), 3),
            "effective_total_clearance_m": round(float(self.map_inflation_m + self.extra_clearance_m), 3),
            "last_stage2": samples[-1] if samples else None,
            "end_xy": [
                round(self.last_odom.pose.pose.position.x, 3),
                round(self.last_odom.pose.pose.position.y, 3),
            ],
            "start_xyz": [
                round(start_x, 3),
                round(start_y, 3),
                round(start_z, 3),
            ],
        }

    def run_case(self, mode):
        if not self.wait_odom():
            return {"status": "failed", "reason": "odom_timeout"}

        odom = self.last_odom
        yaw_rad = yaw_from_quaternion(odom.pose.pose.orientation)
        start_x = odom.pose.pose.position.x
        start_y = odom.pose.pose.position.y

        if mode == "center_obstacle":
            goal_xy = (
                start_x + math.cos(yaw_rad) * 10.5,
                start_y + math.sin(yaw_rad) * 10.5,
            )
            self.send_goal(*goal_xy)
            if not self.wait_occupancy():
                return {"status": "failed", "reason": "occupancy_timeout_after_goal"}
            result = self._run_case(goal_xy, 14.0)
            ok_motion = result["max_travel_xy_m"] >= 3.0
            ok_clear = not result["entered_obstacle_box"]
            ok_state = result["last_stage2"] is not None and result["last_stage2"][0] == "follow"
            ok_replan = result["max_abs_lateral_offset_m"] >= 0.45
            status = "passed" if (ok_motion and ok_clear and ok_state and ok_replan) else "failed"
            return {
                "status": status,
                "mode": mode,
                "result": result,
                "checks": {
                    "motion_ge_3m": ok_motion,
                    "obstacle_box_clear": ok_clear,
                    "stage2_follow_state_observed": ok_state,
                    "lateral_detour_observed": ok_replan,
                },
            }

        if mode == "center_obstacle_hidden_search":
            goal_xy = (
                start_x + math.cos(yaw_rad) * 10.5,
                start_y + math.sin(yaw_rad) * 10.5,
            )
            self.send_goal(*goal_xy)
            if not self.wait_occupancy():
                return {"status": "failed", "reason": "occupancy_timeout_after_goal"}
            time.sleep(1.0)
            hide_resp = self.set_visible(False)
            hide_samples = []
            hide_deadline = time.time() + 3.0
            while time.time() < hide_deadline and not rospy.is_shutdown():
                time.sleep(0.2)
                if self.last_stage2_state is not None:
                    hide_samples.append((self.last_stage2_state.state_name, self.last_stage2_state.detail))
                    if self.last_stage2_state.state_name in ("search", "lost"):
                        break

            show_resp = self.set_visible(True)
            reacquire_samples = []
            reacquire_deadline = time.time() + 3.0
            while time.time() < reacquire_deadline and not rospy.is_shutdown():
                time.sleep(0.2)
                if self.last_stage2_state is not None:
                    reacquire_samples.append((self.last_stage2_state.state_name, self.last_stage2_state.detail))
                    if self.last_stage2_state.state_name == "follow":
                        break

            result = self._run_case(goal_xy, 8.0)
            saw_search = any(sample[0] in ("search", "lost") for sample in hide_samples)
            reacquired_follow = any(sample[0] == "follow" for sample in reacquire_samples)
            ok_motion = result["max_travel_xy_m"] >= 2.5
            ok_clear = not result["entered_obstacle_box"]
            status = "passed" if (saw_search and reacquired_follow and ok_motion and ok_clear) else "failed"
            return {
                "status": status,
                "mode": mode,
                "result": result,
                "hide_response": {
                    "success": bool(getattr(hide_resp, "success", False)),
                    "message": str(getattr(hide_resp, "message", "")),
                },
                "show_response": {
                    "success": bool(getattr(show_resp, "success", False)),
                    "message": str(getattr(show_resp, "message", "")),
                },
                "hide_samples": hide_samples,
                "reacquire_samples": reacquire_samples,
                "checks": {
                    "search_or_lost_observed_after_hide": saw_search,
                    "follow_reacquired_after_show": reacquired_follow,
                    "motion_ge_2_5m": ok_motion,
                    "obstacle_box_clear": ok_clear,
                },
            }

        if mode == "right_bias_follow":
            goal_xy = (
                start_x + math.cos(yaw_rad) * 10.5 - math.sin(yaw_rad) * 1.8,
                start_y + math.sin(yaw_rad) * 10.5 + math.cos(yaw_rad) * 1.8,
            )
            self.send_goal(*goal_xy)
            if not self.wait_occupancy():
                return {"status": "failed", "reason": "occupancy_timeout_after_goal"}
            result = self._run_case(goal_xy, 10.0)
            ok_motion = result["max_travel_xy_m"] >= 3.0
            ok_state = result["last_stage2"] is not None and result["last_stage2"][0] == "follow"
            ok_clear = not result["entered_obstacle_box"]
            ok_lateral = result["max_abs_lateral_offset_m"] >= 0.8
            status = "passed" if (ok_motion and ok_state and ok_clear and ok_lateral) else "failed"
            return {
                "status": status,
                "mode": mode,
                "result": result,
                "checks": {
                    "motion_ge_3m": ok_motion,
                    "stage2_follow_state_observed": ok_state,
                    "obstacle_box_clear": ok_clear,
                    "strong_lateral_follow_observed": ok_lateral,
                },
            }

        return {"status": "failed", "reason": "unknown_mode:%s" % mode}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=("center_obstacle", "right_bias_follow", "center_obstacle_hidden_search"),
        default="center_obstacle",
    )
    parser.add_argument("--extra-clearance-m", type=float, default=None)
    args = parser.parse_args()

    rospy.init_node("stage2_real_ego_visual_demo_verifier", anonymous=True)
    verifier = DemoVerifier(extra_clearance_override_m=args.extra_clearance_m)
    result = verifier.run_case(args.mode)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if result["status"] != "passed":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
