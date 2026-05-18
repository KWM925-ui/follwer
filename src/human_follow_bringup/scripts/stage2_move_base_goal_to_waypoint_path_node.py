#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class Stage2MoveBaseGoalToWaypointPathNode:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/move_base_simple/goal")
        self.output_topic = rospy.get_param("~output_topic", "/waypoint_generator/waypoints")
        self.output_frame_id = rospy.get_param("~output_frame_id", "")

        self.publisher = rospy.Publisher(self.output_topic, Path, queue_size=10, latch=True)
        self.subscriber = rospy.Subscriber(self.input_topic, PoseStamped, self._goal_callback, queue_size=10)

        rospy.loginfo(
            "stage2_move_base_goal_to_waypoint_path ready input=%s output=%s",
            self.input_topic,
            self.output_topic,
        )

    def _goal_callback(self, msg):
        out = Path()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.output_frame_id or msg.header.frame_id or "map"

        goal_pose = PoseStamped()
        goal_pose.header = out.header
        goal_pose.pose = msg.pose
        out.poses.append(goal_pose)

        try:
            self.publisher.publish(out)
        except rospy.ROSException:
            return


if __name__ == "__main__":
    rospy.init_node("stage2_move_base_goal_to_waypoint_path")
    Stage2MoveBaseGoalToWaypointPathNode()
    rospy.spin()
