cmake_minimum_required(VERSION 3.10)
project(human_follow_bringup)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  human_follow_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    human_follow_msgs
    nav_msgs
    rospy
    sensor_msgs
    std_msgs
    tf2_ros
)

catkin_install_python(
  PROGRAMS
    scripts/calibration_bag_marker_audit.py
    scripts/external_odom_emulator_node.py
    scripts/calibration_record_monitor_node.py
    scripts/odom_topic_relay_node.py
    scripts/sequence_regression_monitor_node.py
    scripts/offline_video_smoke_monitor_node.py
    scripts/live_idle_safety_monitor_node.py
    scripts/human_truth_sequence_publisher_node.py
    scripts/vehicle_kinematic_sim_node.py
    scripts/body_truth_world_projector_node.py
    scripts/target_world_projector_node.py
    scripts/stage2_follow_goal_generator_node.py
    scripts/stage2_ego_topic_adapter_node.py
    scripts/stage2_face_target_yaw_override_node.py
    scripts/stage2_goal_input_fixture_node.py
    scripts/stage2_goal_adapter_regression_monitor_node.py
    scripts/stage2_search_goal_regression_monitor_node.py
    scripts/stage2_placeholder_planner_stub_node.py
    scripts/stage2_placeholder_full_chain_regression_monitor_node.py
    scripts/stage2_move_base_goal_to_waypoint_path_node.py
    scripts/stage2_real_ego_full_chain_regression_monitor_node.py
    scripts/stage2_paper_line_regression_monitor_node.py
    scripts/stage2_real_ego_search_full_chain_regression_monitor_node.py
    scripts/stage2_paper_line_regression_monitor_node.py
    scripts/stage2_visual_demo_goal_replay_node.py
    scripts/external_controller_ingress_monitor_node.py
    scripts/truth_case_validation_monitor_node.py
    scripts/human_truth_keyboard_teleop_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
