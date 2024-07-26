cmake_minimum_required(VERSION 2.8.3)
project(lqr)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  cyber_msgs
  geometry_msgs
  roscpp
  sensor_msgs
  rospy
  std_msgs
  tf
  visualization_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)
# find_package(glog REQUIRED NO_MODULE)
include(${CMAKE_SOURCE_DIR}/common/utils/FindGlog.cmake)

generate_dynamic_reconfigure_options(
  cfg/LQR.cfg
)


catkin_package(

)

include_directories(
  ${CMAKE_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_library(leadlag STATIC leadlag_controller.cc)
target_link_libraries(leadlag glog)

add_executable(${PROJECT_NAME}_node LQR.cpp )

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} math_utils linear_interpolation lqr glog leadlag digital_filters)
add_dependencies(${PROJECT_NAME}_node  ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
