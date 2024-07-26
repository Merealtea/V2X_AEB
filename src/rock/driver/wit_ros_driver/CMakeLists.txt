cmake_minimum_required(VERSION 2.8.3)
project(wit_ros_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf2
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/wit_modbus_gps_ros.py scripts/wit_modbus_imu_ros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
