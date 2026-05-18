cmake_minimum_required(VERSION 3.10)
project(human_follow_fusion)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  human_follow_msgs
  nav_msgs
  quadrotor_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
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
    visualization_msgs
)

catkin_install_python(
  PROGRAMS
    scripts/calibration_asset_validator.py
    scripts/target_fusion_node.py
    scripts/projection_sanity_node.py
    scripts/association_sanity_node.py
    scripts/fusion_synthetic_scene_publisher_node.py
    scripts/truth_driven_scene_publisher_node.py
    scripts/truth_follow_visualizer_node.py
    scripts/fusion_output_regression_monitor_node.py
    scripts/projection_overlay_debug_node.py
    scripts/target_frame_adapter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
