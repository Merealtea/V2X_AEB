cmake_minimum_required(VERSION 2.8.3)
project(control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O3)
if(${CMAKE_CXX_COMPILER} STREQUAL "/usr/bin/c++")
  add_compile_options(-fext-numeric-literals)
elseif(${CMAKE_CXX_COMPILER} STREQUAL "/usr/bin/clang++")
#  MESSAGE(STATUS "complier is ${CMAKE_CXX_COMPILER}, so DO NOT add compile options: -fext-numeric-literals" )
endif()

include(${CMAKE_SOURCE_DIR}/common/utils/FindGlog.cmake)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  cyber_msgs
  sensor_msgs
  std_msgs
  tf
)

generate_dynamic_reconfigure_options(
  cfg/Param.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tracking
#  CATKIN_DEPENDS geometry_msgs nodelet pcl_ros roscpp rospy std_msgs tf visualization_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  src/common
  src/vehicle
  src/mapping
  src/control
  src/draw
  src/smoother
  /usr/include/eigen3
)

add_executable(vehicle_node
  src/main.cpp
  src/top.cpp
  src/vehicle/vehicle.cpp
  src/mapping/mapping.cpp
  src/control/control.cpp
  src/draw/draw.cpp
)

add_library(control_common STATIC
  src/common/discretized_path.cc)
target_link_libraries(control_common linear_interpolation glog)

add_subdirectory(src/smoother/math)
add_subdirectory(src/smoother/reference_line)

add_library(track_smoother STATIC src/smoother/track_smoother.cc)
target_link_libraries(track_smoother control_common discrete_points_math reference_line_smoother glog)

target_link_libraries(vehicle_node ${catkin_LIBRARIES}
  track_smoother glog digital_filters)

add_dependencies(vehicle_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
