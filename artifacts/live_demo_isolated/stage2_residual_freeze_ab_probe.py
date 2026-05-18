#!/usr/bin/env python3
import json
import math
import time

import rospy
from ego_planner.msg import Bspline
from geometry_msgs.msg import PoseStamped
from human_follow_msgs.msg import FollowState, Target3D
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand


GOAL_SEQUENCE = [
    ("center_detour", (9.0, 0.0)),
    ("escape_rightish", (10.0, 0.5)),
    ("low_lane", (8.0, -1.0)),
    ("upper_lane", (8.0, 1.6)),
    ("far_lower", (10.6, -1.4)),
    ("near_escape", (9.2, -2.6)),
]


def _pose_xy(msg):
    return float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)


class ResidualFreezeABProbe:
    def __init__(self):
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.last_odom = None
        self.last_state = None
        self.last_target = None
        self.last_stage2_goal = None
        self.last_cmd = None
        self.bspline_count = 0
        self.bspline_last_stamp = None

        rospy.Subscriber("/follow/sim/vehicle_odom", Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/state", FollowState, self._state_cb, queue_size=50)
        rospy.Subscriber("/follow/fusion/target_world", Target3D, self._target_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/goal", PoseStamped, self._goal_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/ego_position_cmd", PositionCommand, self._cmd_cb, queue_size=50)
        rospy.Subscriber("/planning/bspline", Bspline, self._bspline_cb, queue_size=50)

    def _odom_cb(self, msg):
        self.last_odom = msg

    def _state_cb(self, msg):
        self.last_state = msg

    def _target_cb(self, msg):
        self.last_target = msg

    def _goal_cb(self, msg):
        self.last_stage2_goal = msg

    def _cmd_cb(self, msg):
        self.last_cmd = msg

    def _bspline_cb(self, _msg):
        self.bspline_count += 1
        self.bspline_last_stamp = time.time()

    def wait_ready(self, timeout_sec=8.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.last_odom is not None and self.last_state is not None:
                return True
            time.sleep(0.05)
        return False

    def publish_base_link_goal(self, x_value, y_value):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = float(x_value)
        msg.pose.position.y = float(y_value)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        for _ in range(3):
            self.goal_pub.publish(msg)
            time.sleep(0.05)

    def run_case(self, label, goal_xy, dwell_sec=6.0):
        if self.last_odom is None:
            return {"label": label, "status": "failed", "reason": "no_odom"}

        start_x, start_y = _pose_xy(self.last_odom)
        start_bspline_count = self.bspline_count
        start_bspline_stamp = self.bspline_last_stamp
        max_disp = 0.0
        max_cmd_speed = 0.0

        self.publish_base_link_goal(goal_xy[0], goal_xy[1])
        deadline = time.time() + dwell_sec
        while time.time() < deadline and not rospy.is_shutdown():
            time.sleep(0.1)
            if self.last_odom is not None:
                current_x, current_y = _pose_xy(self.last_odom)
                max_disp = max(max_disp, math.hypot(current_x - start_x, current_y - start_y))
            if self.last_cmd is not None:
                velocity = self.last_cmd.velocity
                speed = math.sqrt(
                    float(velocity.x) * float(velocity.x)
                    + float(velocity.y) * float(velocity.y)
                    + float(velocity.z) * float(velocity.z)
                )
                max_cmd_speed = max(max_cmd_speed, speed)

        end_x, end_y = _pose_xy(self.last_odom) if self.last_odom is not None else (None, None)
        bspline_delta = self.bspline_count - start_bspline_count
        last_bspline_advanced = False
        if self.bspline_last_stamp is not None and start_bspline_stamp is not None:
            last_bspline_advanced = self.bspline_last_stamp > start_bspline_stamp
        elif self.bspline_last_stamp is not None and start_bspline_stamp is None:
            last_bspline_advanced = True

        target_far_m = None
        if self.last_target is not None and self.last_odom is not None:
            target_far_m = math.sqrt(
                (float(self.last_target.position.x) - float(self.last_odom.pose.pose.position.x)) ** 2
                + (float(self.last_target.position.y) - float(self.last_odom.pose.pose.position.y)) ** 2
            )

        state_name = getattr(self.last_state, "state_name", "") if self.last_state is not None else ""
        detail = getattr(self.last_state, "detail", "") if self.last_state is not None else ""
        stage2_goal = None
        if self.last_stage2_goal is not None:
            stage2_goal = {
                "x": float(self.last_stage2_goal.pose.position.x),
                "y": float(self.last_stage2_goal.pose.position.y),
                "z": float(self.last_stage2_goal.pose.position.z),
            }
        target_world = None
        if self.last_target is not None:
            target_world = {
                "x": float(self.last_target.position.x),
                "y": float(self.last_target.position.y),
                "z": float(self.last_target.position.z),
                "valid": bool(self.last_target.valid),
            }

        freeze = bool(target_far_m is not None and target_far_m > 5.0 and max_disp < 0.05 and bspline_delta == 0 and max_cmd_speed == 0.0)
        return {
            "label": label,
            "input_goal_base_link": {"x": float(goal_xy[0]), "y": float(goal_xy[1])},
            "end_xy": {"x": end_x, "y": end_y},
            "max_disp_m": max_disp,
            "max_cmd_speed": max_cmd_speed,
            "bspline_delta": int(bspline_delta),
            "bspline_advanced": bool(last_bspline_advanced),
            "target_far_m": target_far_m,
            "state_name": state_name,
            "detail": detail,
            "target_world": target_world,
            "stage2_goal": stage2_goal,
            "classified_freeze": freeze,
        }

    def run(self):
        if not self.wait_ready():
            return {"status": "failed", "reason": "wait_ready_timeout"}
        results = []
        for label, goal_xy in GOAL_SEQUENCE:
            results.append(self.run_case(label, goal_xy))
        return {"status": "ok", "sequence": results}


def main():
    rospy.init_node("stage2_residual_freeze_ab_probe", anonymous=True)
    probe = ResidualFreezeABProbe()
    result = probe.run()
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
