cmake_minimum_required(VERSION 2.8.3)
project(longitudinal_controller)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
		dynamic_reconfigure
		roscpp
    rospy
		std_msgs
		cyber_msgs)

generate_dynamic_reconfigure_options(
  cfg/lon_controller.cfg
)

catkin_package(
# INCLUDE_DIRS include
# CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp std_msgs
)

include_directories(
  ${CMAKE_SOURCE_DIR}
	${catkin_INCLUDE_DIRS}
	src/
)

# Set the source files to use with the executable.
set (SRCS1 src/lon_controller_node.cpp src/pid_controller.cc)

# Build the executable that will be used to run this node.
add_executable (${PROJECT_NAME}
                ${SRCS1}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} linear_interpolation)
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS
  scripts/controller_test_ui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
