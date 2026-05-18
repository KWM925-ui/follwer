cmake_minimum_required(VERSION 3.10)
project(human_follow_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  human_follow_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    human_follow_msgs
    nav_msgs
    rospy
    sensor_msgs
    std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/follow_controller_node.py
    scripts/control_algorithm_template_node.py
    scripts/planning_algorithm_template_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
