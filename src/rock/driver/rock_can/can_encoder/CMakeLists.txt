cmake_minimum_required(VERSION 2.8.3)
project(can_encoder)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED  COMPONENTS
  rospy
  roscpp
  std_msgs
  cyber_msgs
)

catkin_package()

catkin_package(
	CATKIN_DEPENDS
   	roscpp
  	std_msgs
  	message_runtime
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/can_encoder.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
