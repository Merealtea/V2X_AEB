cmake_minimum_required(VERSION 2.8.3)
project(cyber_e100_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CanFrame.msg
  EsrObj.msg
  EsrObjMultiArray.msg
  LaneSegement.msg
  LaneWaypoint.msg
  LocalizationEstimate.msg
  LocalTrajPoint.msg
  LocalTrajList.msg
  Object.msg
  ObjectArray.msg
  OccupancyIndex.msg
  OccupancyIndexArray.msg
  VehicleBrakeFeedback.msg
  VehicleSpeedFeedback.msg
  VehicleSteerFeedback.msg
  CommunicationHeart.msg
  JY901.msg
  GPGGA_MSG.msg
  Heading.msg
  GPTRA_MSG.msg
  VehicleState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs roscpp sensor_msgs std_msgs
)
