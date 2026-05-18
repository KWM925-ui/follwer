cmake_minimum_required(VERSION 3.10)
project(human_follow_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
  FollowCommand.msg
  FollowState.msg
  Target2D.msg
  Target3D.msg
  TrackerStatus.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)
