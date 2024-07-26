cmake_minimum_required(VERSION 2.8.3)
project(speed_controller)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
		dynamic_reconfigure
		message_generation
		roscpp
    rospy
		std_msgs
		cyber_msgs)


#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

generate_dynamic_reconfigure_options(
  cfg/speed_controller.cfg
  cfg/speed_controller_test.cfg
)

catkin_package(
# INCLUDE_DIRS include
# CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp std_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
	src/
)

# Set the source files to use with the executable.
set (SRCS1 src/speed_controller.cpp)

set (SRCS2 src/speed_controller_test.cpp)

#set (USB_KEY_SRCS
#    ../../common/Pwd_8/syunew3.cpp
#    ../../common/Pwd_8/libusb/core.c
#    ../../common/Pwd_8/libusb/descriptor.c
#    ../../common/Pwd_8/libusb/io.c
#    ../../common/Pwd_8/libusb/sync.c
#    ../../common/Pwd_8/libusb/os/linux_usbfs.c
#)

# Build the executable that will be used to run this node.
add_executable (${PROJECT_NAME}
                ${SRCS1}
                # ${USB_KEY_SRCS}
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)

add_executable(speed_controller_test  ${SRCS2})

target_link_libraries(speed_controller_test  ${catkin_LIBRARIES})

add_dependencies(speed_controller_test ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

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
