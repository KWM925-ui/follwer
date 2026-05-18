cmake_minimum_required(VERSION 3.10)
project(human_follow_px4_bridge)

set(_mavros_overlay_candidates
  "$ENV{MAVROS_OVERLAY_PREFIX}"
  "/home/coco/.local/ros_noetic_overlay/opt/ros/noetic"
)

foreach(_candidate IN LISTS _mavros_overlay_candidates)
  if(_candidate AND EXISTS "${_candidate}/share/mavros_msgs/package.xml")
    list(PREPEND CMAKE_PREFIX_PATH "${_candidate}")
    message(STATUS "human_follow_px4_bridge using MAVROS overlay: ${_candidate}")
    break()
  endif()
endforeach()

find_package(catkin REQUIRED COMPONENTS
  human_follow_msgs
  mavros_msgs
  quadrotor_msgs
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    human_follow_msgs
    mavros_msgs
    quadrotor_msgs
    rospy
    std_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/follow_px4_bridge_node.py
    scripts/ego_position_command_bridge_node.py
    scripts/offboard_mode_gate_node.py
    scripts/fake_mavros_gate_harness_node.py
    scripts/bridge_command_sequence_publisher_node.py
    scripts/position_target_sequence_publisher_node.py
    scripts/bridge_output_regression_monitor_node.py
    scripts/controller_bridge_regression_monitor_node.py
    scripts/ego_bridge_regression_monitor_node.py
    scripts/truth_chain_regression_monitor_node.py
    scripts/offboard_gate_regression_monitor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/runtime_mavros_import.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
