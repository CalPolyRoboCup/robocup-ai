cmake_minimum_required(VERSION 2.8.3)
project(coach)

find_package(catkin REQUIRED COMPONENTS
  strategy_selection
  rospy
  roscpp
  robocup_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coach
#  CATKIN_DEPENDS strategy_selection rospy roscpp std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)