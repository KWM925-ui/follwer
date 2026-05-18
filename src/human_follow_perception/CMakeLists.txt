cmake_minimum_required(VERSION 3.10)
project(human_follow_perception)

find_package(catkin REQUIRED COMPONENTS
  human_follow_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    human_follow_msgs
    rospy
    sensor_msgs
    std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/detector_node.py
    scripts/tracker_node.py
    scripts/video_replay_node.py
    scripts/target_overlay_node.py
    scripts/target_sequence_publisher_node.py
    scripts/synthetic_camera_canvas_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
