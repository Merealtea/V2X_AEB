cmake_minimum_required(VERSION 3.5)
project(math)

add_compile_options(-std=c++14)


find_package(Eigen3 REQUIRED)

# find_package(glog REQUIRED NO_MODULE)
include(${CMAKE_SOURCE_DIR}/common/utils/FindGlog.cmake)
find_package(osqp REQUIRED NO_MODULE)

include_directories(
  ${CMAKE_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(math_utils STATIC math_utils.cc)
target_link_libraries(math_utils  vec2d)

add_library(vec2d STATIC vec2d.cc)
target_link_libraries(vec2d glog)

add_library(linear_interpolation STATIC linear_interpolation.cc)
target_link_libraries(linear_interpolation  glog math_utils)

add_library(lqr STATIC linear_quadratic_regulator.cc)
target_link_libraries(lqr glog)

add_library(digital_filters STATIC filters/digital_filter_coefficients.cc filters/digital_filter.cc)
target_link_libraries(digital_filters glog)
