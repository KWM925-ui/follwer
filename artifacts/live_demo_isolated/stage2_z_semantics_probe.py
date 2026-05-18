#!/usr/bin/env python3
import argparse
import json
import time

import rospy
from geometry_msgs.msg import PoseStamped
from human_follow_msgs.msg import FollowState, Target3D
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


def new_stats():
    return {
        "samples": 0,
        "min": None,
        "max": None,
        "last": None,
    }


def update_stats(stats, value):
    value = float(value)
    stats["samples"] += 1
    stats["min"] = value if stats["min"] is None else min(stats["min"], value)
    stats["max"] = value if stats["max"] is None else max(stats["max"], value)
    stats["last"] = value


class Probe:
    def __init__(self):
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.vehicle_odom = None
        self.truth_world = None
        self.target_world = None
        self.stage2_goal = None
        self.ego_goal = None
        self.cmd = None
        self.state = None
        self.occ_summary = None

        self.data = {
            "truth_world_z": new_stats(),
            "truth_relative_z": new_stats(),
            "target_world_z": new_stats(),
            "stage2_goal_z": new_stats(),
            "ego_goal_z": new_stats(),
            "cmd_z": new_stats(),
            "odom_z": new_stats(),
            "occ_z": {
                "samples": 0,
                "min_of_min": None,
                "max_of_max": None,
                "last": None,
            },
            "state_tail": [],
        }

        rospy.Subscriber("/follow/sim/vehicle_odom", Odometry, self._odom_cb, queue_size=50)
        rospy.Subscriber("/follow/sim/human_truth_world", Target3D, self._truth_world_cb, queue_size=50)
        rospy.Subscriber("/follow/fusion/target_world", Target3D, self._target_world_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/goal", PoseStamped, self._stage2_goal_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/ego_goal", PoseStamped, self._ego_goal_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/ego_position_cmd", PositionCommand, self._cmd_cb, queue_size=50)
        rospy.Subscriber("/follow/stage2/state", FollowState, self._state_cb, queue_size=50)
        rospy.Subscriber("/grid_map/occupancy_inflate", PointCloud2, self._occ_cb, queue_size=5)

    def _odom_cb(self, msg):
        self.vehicle_odom = msg
        update_stats(self.data["odom_z"], msg.pose.pose.position.z)
        if self.truth_world is not None and self.truth_world.valid:
            update_stats(
                self.data["truth_relative_z"],
                float(self.truth_world.position.z) - float(msg.pose.pose.position.z),
            )

    def _truth_world_cb(self, msg):
        self.truth_world = msg
        if msg.valid:
            update_stats(self.data["truth_world_z"], msg.position.z)
            if self.vehicle_odom is not None:
                update_stats(
                    self.data["truth_relative_z"],
                    float(msg.position.z) - float(self.vehicle_odom.pose.pose.position.z),
                )

    def _target_world_cb(self, msg):
        self.target_world = msg
        if msg.valid:
            update_stats(self.data["target_world_z"], msg.position.z)

    def _stage2_goal_cb(self, msg):
        self.stage2_goal = msg
        update_stats(self.data["stage2_goal_z"], msg.pose.position.z)

    def _ego_goal_cb(self, msg):
        self.ego_goal = msg
        update_stats(self.data["ego_goal_z"], msg.pose.position.z)

    def _cmd_cb(self, msg):
        self.cmd = msg
        update_stats(self.data["cmd_z"], msg.position.z)

    def _state_cb(self, msg):
        item = [str(msg.state_name), str(msg.detail)]
        self.data["state_tail"].append(item)
        self.data["state_tail"] = self.data["state_tail"][-20:]

    def _occ_cb(self, msg):
        min_z = None
        max_z = None
        count = 0
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            z_value = float(point[2])
            min_z = z_value if min_z is None else min(min_z, z_value)
            max_z = z_value if max_z is None else max(max_z, z_value)
            count += 1
        if count <= 0:
            return
        occ = self.data["occ_z"]
        occ["samples"] += 1
        occ["min_of_min"] = min_z if occ["min_of_min"] is None else min(occ["min_of_min"], min_z)
        occ["max_of_max"] = max_z if occ["max_of_max"] is None else max(occ["max_of_max"], max_z)
        occ["last"] = {"min": min_z, "max": max_z, "count": count}

    def wait_odom(self, timeout_sec=8.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not rospy.is_shutdown():
            if self.vehicle_odom is not None:
                return True
            time.sleep(0.05)
        return False

    def send_goal_ahead(self, forward_m=10.5):
        if self.vehicle_odom is None:
            return
        start_x = float(self.vehicle_odom.pose.pose.position.x)
        start_y = float(self.vehicle_odom.pose.pose.position.y)
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.position.x = start_x + float(forward_m)
        msg.pose.position.y = start_y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        for _ in range(3):
            self.goal_pub.publish(msg)
            time.sleep(0.05)


def rounded_stats(stats):
    out = dict(stats)
    for key in ("min", "max", "last"):
        if out[key] is not None:
            out[key] = round(float(out[key]), 3)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dwell-sec", type=float, default=14.0)
    parser.add_argument("--goal-forward-m", type=float, default=10.5)
    args = parser.parse_args()

    rospy.init_node("stage2_z_semantics_probe", anonymous=True)
    probe = Probe()
    if not probe.wait_odom():
        raise SystemExit("odom_timeout")

    probe.send_goal_ahead(args.goal_forward_m)
    deadline = time.time() + max(float(args.dwell_sec), 1.0)
    rate = rospy.Rate(25.0)
    while time.time() < deadline and not rospy.is_shutdown():
        rate.sleep()

    result = {
        "truth_world_z": rounded_stats(probe.data["truth_world_z"]),
        "truth_relative_z": rounded_stats(probe.data["truth_relative_z"]),
        "target_world_z": rounded_stats(probe.data["target_world_z"]),
        "stage2_goal_z": rounded_stats(probe.data["stage2_goal_z"]),
        "ego_goal_z": rounded_stats(probe.data["ego_goal_z"]),
        "cmd_z": rounded_stats(probe.data["cmd_z"]),
        "odom_z": rounded_stats(probe.data["odom_z"]),
        "occ_z": probe.data["occ_z"],
        "state_tail": probe.data["state_tail"],
    }
    if result["occ_z"]["min_of_min"] is not None:
        result["occ_z"]["min_of_min"] = round(float(result["occ_z"]["min_of_min"]), 3)
    if result["occ_z"]["max_of_max"] is not None:
        result["occ_z"]["max_of_max"] = round(float(result["occ_z"]["max_of_max"]), 3)
    if result["occ_z"]["last"] is not None:
        result["occ_z"]["last"]["min"] = round(float(result["occ_z"]["last"]["min"]), 3)
        result["occ_z"]["last"]["max"] = round(float(result["occ_z"]["last"]["max"]), 3)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
