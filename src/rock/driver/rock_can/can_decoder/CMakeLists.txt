cmake_minimum_required(VERSION 2.8.3)
project(can_decoder)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

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
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Build the executable that will be used to run this node.
# add_dependencies(can_decoder ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_executable (can_decoder src/can_decoder.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(can_decoder ${catkin_LIBRARIES})
