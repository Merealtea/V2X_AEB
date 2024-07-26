# find_package(glog REQUIRED NO_MODULE)
# include(../common/FindGlog.cmake)
find_package(osqp REQUIRED NO_MODULE)
include_directories(${CMAKE_SOURCE_DIR}
  ../common)

aux_source_directory(./discretized_points_smoothing DISCRETIZED_POINTS_SMOOTHING_SRCS_DIR)
add_library(discretized_points_smoothing STATIC ${DISCRETIZED_POINTS_SMOOTHING_SRCS_DIR})
target_link_libraries(discretized_points_smoothing glog osqp)

add_library(discrete_points_math STATIC discrete_points_math.cc)
target_link_libraries(discrete_points_math glog)
