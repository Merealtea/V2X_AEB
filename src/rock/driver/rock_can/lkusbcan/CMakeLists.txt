cmake_minimum_required(VERSION 2.8.3)
project(lkusbcan)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

link_directories("lib/ver 2.0.6/acusb linux demo/64bit/dll")

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	std_msgs
	cyber_msgs
)

catkin_package(
	CATKIN_DEPENDS
  	message_runtime
  	roscpp
  	std_msgs
  	cyber_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Build the executable that will be used to run this node.
add_executable (lcan_read_write src/lcan_read_write.cpp)
add_dependencies(lcan_read_write ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lcan_read_write ${catkin_LIBRARIES})
target_link_libraries(lcan_read_write libCanCmd.so)

