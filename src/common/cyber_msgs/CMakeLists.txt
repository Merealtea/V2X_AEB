cmake_minimum_required(VERSION 2.8.3)
project(cyber_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
  geometry_msgs
  sensor_msgs
	message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  speedcmd.msg
  steercmd.msg
  brakecmd.msg
  canframe.msg
  BrakeFeedback.msg
  BrakeStateFeedback.msg
  SteerFeedback.msg
  SteerStateFeedback.msg
  SpeedFeedback.msg
  BodyworkControl.msg
  LaneSegement.msg
  LaneWaypoint.msg
  LocalTrajPoint.msg
  LocalTrajList.msg
  LocalizationEstimate.msg
  Object.msg
  ObjectArray.msg
  RoutingResult.msg
  TrafficInformation.msg
  VehicleState.msg
  Heading.msg
  VehicleSpeedFeedback.msg
  V2VPacket.msg
  LqrDebug.msg
  PlatoonControlDebug.msg
  PlatoonControlTarget.msg
  LongitudinalControlDebug.msg
  PlatoonMPCDebug.msg
  Box2D.msg
  Box2DArray.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS
	message_runtime
	roscpp
	std_msgs
  geometry_msgs
)
# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
)
