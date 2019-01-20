cmake_minimum_required(VERSION 2.8.3)
project(robocup_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  hardware_interface
  controller_manager
  message_generation
)

## System dependencies are found with CMake's conventions
find_package(Protobuf REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Insn.srv
  Data.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs
  geometry_msgs
  message_runtime
  hardware_interface
  controller_manager
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${PROTOBUF_INCLUDE_DIRS})

## Generate Protobuf messages
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS 
  proto/grSim_Replacement.proto
  proto/grSim_Commands.proto
  proto/grSim_Packet.proto 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/player_hw_interface.cpp
  src/player_control_loop.cpp
  src/sim_spi_service.cpp
  src/drive_controller.cpp
  ${PROTO_SRCS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

## Declare a C++ executable
add_executable(sim_spi_thread src/sim_spi_thread.cpp) 
add_executable(player_thread src/main.cpp)
add_executable(pose_thread src/dummy_pose_pub.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
  sim_spi_thread
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(
  player_thread
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

target_link_libraries(
  pose_thread
  ${catkin_LIBRARIES}
)

add_dependencies(sim_spi_thread robocup_control_gencpp)
add_dependencies(player_thread robocup_control_gencpp)
add_dependencies(pose_thread robocup_control_gencpp)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES drive_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robocup_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
