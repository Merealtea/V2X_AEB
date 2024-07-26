cmake_minimum_required(VERSION 2.8.3)
project(e100_msgs)

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
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  brakecmd.msg
  xboxone.msg
  speedcmd.msg
  steercmd.msg
  canframe.msg
  supervisor.msg
  VehicleSpeedFeedback.msg
  VehicleSteerFeedback.msg
  VehicleBrakeFeedback.msg
  communicationHeart.msg
  cansteer703.msg
  canspeed301.msg
  canbrake131.msg
  iduOn632.msg
  enablecmd.msg
  zigbee.msg
  FollowerStatus.msg
  pi_status.msg
  SpeedTarget.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS 
	message_runtime 
	roscpp 
	std_msgs
  geometry_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
)