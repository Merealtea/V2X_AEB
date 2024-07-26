cmake_minimum_required(VERSION 2.8.3)
project(lidar_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O3 -Wall)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  mrpt_bridge
  nodelet
  pcl_ros
  roscpp
  rospy
  std_msgs
  e100_msgs
  tf
  visualization_msgs
  cyber_msgs
)
find_package(PCL 1.7 REQUIRED)
find_package(MRPT REQUIRED
  base
  slam
  gui
  opengl
  maps
  obs
)

generate_dynamic_reconfigure_options(
  cfg/ParamLoc.cfg
  cfg/PclEditor.cfg
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
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${MRPT_INCLUDE_DIRS}
  src
)

# include_directories( ${PCL_INCLUDE_DIRS} )
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})
add_definitions( -DPCL_NO_PRECOMPILE )

include_directories(
  ${MRPT_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # src/location
)

add_executable(tracker_node
  src/tracker_node/main.cc
  src/tracker_node/top.cc
  src/ground_filter/ground_filter.cpp
  src/ekf/ekf_pose.cpp
)
target_link_libraries(tracker_node
  ${catkin_LIBRARIES}
  ${MRPT_LIBS}
)
add_dependencies(tracker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg)

add_executable(pcl_editor_node
  src/pcl_editor_node/main.cc
  src/pcl_editor_node/top.cc
)
target_link_libraries(pcl_editor_node
  ${catkin_LIBRARIES}
)
add_dependencies(pcl_editor_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg)
