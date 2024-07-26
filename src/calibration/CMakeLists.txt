cmake_minimum_required(VERSION 2.8.3)
project(calibration)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  geometry_msgs
)
catkin_package()
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
include_directories(
  ${CMAKE_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)
add_executable(${PROJECT_NAME} src/static_tf_broadcaster.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
