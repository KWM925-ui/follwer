cmake_minimum_required(VERSION 3.10)
project(human_follow_user)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  human_follow_msgs
  nav_msgs
  quadrotor_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    human_follow_msgs
    nav_msgs
    quadrotor_msgs
    rospy
    sensor_msgs
    std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/user_control_node.py
    scripts/user_planning_node.py
    scripts/user_stage2_goal_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES README_CN.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
