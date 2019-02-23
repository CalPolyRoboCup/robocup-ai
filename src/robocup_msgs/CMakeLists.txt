cmake_minimum_required(VERSION 2.8.3)
project(robocup_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Position.msg
  Strategy.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)