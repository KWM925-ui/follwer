#!/usr/bin/env python3
import math

import rospy
from ego_planner.msg import Bspline
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def _evaluate_bspline_t(bspline_msg, t_sec):
    order = int(bspline_msg.order)
    knots = [float(value) for value in bspline_msg.knots]
    control_points = [
        [float(point.x), float(point.y), float(point.z)]
        for point in bspline_msg.pos_pts
    ]
    if not control_points or len(knots) < (len(control_points) + order + 1):
        return None

    m_value = len(knots) - 1
    lower_u = knots[order]
    upper_u = knots[m_value - order]
    if upper_u <= lower_u:
        return None
    u_value = min(max(lower_u, float(t_sec) + lower_u), upper_u)

    k_index = order
    while k_index + 1 < len(knots) and knots[k_index + 1] < u_value:
        k_index += 1

    left_index = k_index - order
    if left_index < 0 or left_index + order >= len(control_points):
        return None
    work = [control_points[left_index + i][:] for i in range(order + 1)]
    for r_value in range(1, order + 1):
        for i_value in range(order, r_value - 1, -1):
            left = knots[i_value + k_index - order]
            right = knots[i_value + 1 + k_index - r_value]
            denom = right - left
            alpha = 0.0 if abs(denom) <= 1e-9 else (u_value - left) / denom
            prev_point = work[i_value - 1]
            curr_point = work[i_value]
            work[i_value] = [
                (1.0 - alpha) * prev_point[axis] + alpha * curr_point[axis]
                for axis in range(3)
            ]
    return work[order]


class Stage2BsplinePathVisualizerNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/planning/bspline")
        self.path_topic = rospy.get_param("~path_topic", "/follow/stage2/ego_bspline_path")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.sample_dt_sec = max(float(rospy.get_param("~sample_dt_sec", 0.05)), 0.01)
        self.max_samples = max(int(rospy.get_param("~max_samples", 240)), 10)

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)
        self.bspline_sub = rospy.Subscriber(self.input_topic, Bspline, self._bspline_callback, queue_size=10)
        rospy.loginfo(
            "stage2_bspline_path_visualizer ready: %s -> %s frame=%s",
            self.input_topic,
            self.path_topic,
            self.frame_id,
        )

    def _duration_sec(self, msg):
        order = int(msg.order)
        knots = [float(value) for value in msg.knots]
        if len(knots) <= order * 2:
            return None
        duration = knots[-order - 1] - knots[order]
        if not math.isfinite(duration) or duration <= 0.0:
            return None
        return float(duration)

    def _bspline_callback(self, msg):
        duration = self._duration_sec(msg)
        if duration is None:
            return

        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id

        sample_count = min(int(math.ceil(duration / self.sample_dt_sec)) + 1, self.max_samples)
        if sample_count < 2:
            return

        for index in range(sample_count):
            if sample_count == 1:
                t_sec = 0.0
            else:
                t_sec = duration * float(index) / float(sample_count - 1)
            point = _evaluate_bspline_t(msg, t_sec)
            if point is None:
                continue
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        if len(path.poses) >= 2:
            try:
                self.path_pub.publish(path)
            except rospy.ROSException:
                return


if __name__ == "__main__":
    rospy.init_node("stage2_bspline_path_visualizer")
    Stage2BsplinePathVisualizerNode()
    rospy.spin()
