cmake_minimum_required(VERSION 2.8.3)
project(player)

find_package(catkin REQUIRED COMPONENTS
  strategy_selection
  coach
  rospy
  std_msgs
  robocup_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS strategy_selection rospy std_msgs message_runtime geometry_msgs coach robocup_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
