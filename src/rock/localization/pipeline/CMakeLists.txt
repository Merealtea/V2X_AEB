cmake_minimum_required(VERSION 3.5)
project(localization_pipeline)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  cyber_msgs
  tf
  roslib
  std_msgs
)

find_package(Eigen3)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy
)

###########
## Build ##
###########

include_directories(
    src
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(localization src/main.cpp src/top.cpp src/common/map_frame.cpp src/ekf/ekf_pose.cpp)
add_dependencies(localization ${catkin_EXPORTED_TARGETS})
target_link_libraries(localization ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})
