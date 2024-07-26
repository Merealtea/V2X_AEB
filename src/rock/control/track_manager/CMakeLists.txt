cmake_minimum_required(VERSION 2.8.3)
project(track_manager)

set(CMAKE_CXX_STANDARD 14)

# ##############################################################################
# ROS set
# ##############################################################################
find_package(catkin REQUIRED
  COMPONENTS dynamic_reconfigure
  roscpp
  rospy
  cyber_msgs
  sensor_msgs
  std_msgs
  tf
)
find_package(Eigen3 REQUIRED)
include(${CMAKE_SOURCE_DIR}/common/utils/FindGlog.cmake)

catkin_package()

# generate_dynamic_reconfigure_options( cfg/Param.cfg )
link_directories(${catkin_LIBRARY_DIRS})
include_directories(${CMAKE_SOURCE_DIR} ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src/common/data_type
  ${PROJECT_SOURCE_DIR}/src/common/third_party
  ${EIGEN3_INCLUDE_DIR}
)

# link_libraries(${catkin_LIBRARIES})
add_subdirectory(src/common/math)
add_subdirectory(src/common/data_type)

add_library(track_smoother STATIC src/module/track_smoother/track_smoother.cc)
target_include_directories(
  track_smoother PUBLIC ${PROJECT_SOURCE_DIR}/src/module/track_smoother
)

target_link_libraries(
  track_smoother
  discretized_trajectory
  discretized_path
  discrete_points_math
  reference_line_smoother
  glog
)

## load in pkg-config support
find_package(PkgConfig)
## use pkg-config to get hints for 0mq locations
pkg_check_modules(PC_ZeroMQ QUIET zmq)

## use the hint from above to find where 'zmq.hpp' is located
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

## use the hint from about to find the location of libzmq
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )

add_library(speed_predictor STATIC
  src/module/speed_predictor/speed_predictor.cc
  src/module/speed_predictor/kalman_speed_predictor.cc)

target_include_directories(
  speed_predictor PUBLIC ${PROJECT_SOURCE_DIR}/src/module/speed_predictor
)

add_library(julia_mpc_adapter STATIC src/module/julia_mpc_adapter/julia_mpc_adapter.cc)
target_include_directories(
  julia_mpc_adapter PUBLIC ${PROJECT_SOURCE_DIR}/src/module/julia_mpc_adapter
)
target_link_libraries(
  julia_mpc_adapter
  discretized_trajectory
  glog
  ${ZeroMQ_LIBRARY}
  -pthread
  speed_predictor
)

add_library(track_manager STATIC src/module/track_manager/track_manager.cc)

target_include_directories(
  track_manager PUBLIC ${PROJECT_SOURCE_DIR}/src/module/track_manager
)

target_link_libraries(
  track_manager
  discretized_trajectory
  discretized_path
  track_smoother
  glog
  julia_mpc_adapter
  speed_predictor
)

add_executable(track_manager_node src/node/track_manager_node.cpp)

target_include_directories(
  track_manager_node PUBLIC ${PROJECT_SOURCE_DIR}/src/module
)

target_link_libraries(
  track_manager_node
  ${catkin_LIBRARIES}
  track_smoother
  track_manager
  glog
  digital_filters
)

add_dependencies(track_manager_node ${catkin_EXPORTED_TARGETS})
