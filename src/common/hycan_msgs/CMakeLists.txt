cmake_minimum_required(VERSION 2.8.3)
project(hycan_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
  geometry_msgs
	message_generation
  sensor_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Head.msg
  VehicleFeedback.msg
  FourImages.msg  
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS
	message_runtime
	roscpp
	std_msgs
  geometry_msgs
  sensor_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
)
