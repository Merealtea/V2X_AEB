cmake_minimum_required(VERSION 2.8.3)
project(steer_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	dynamic_reconfigure
	message_generation
	roscpp
	std_msgs
  	sensor_msgs
  	tf
  	cyber_msgs
)

#add_message_files(
#  FILES
##  NodeExampleData.msg
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  tiggo_msgs
#)

generate_dynamic_reconfigure_options(
  cfg/stanley_param.cfg
  cfg/pursuit_param.cfg
)

catkin_package(
  	CATKIN_DEPENDS
  	dynamic_reconfigure
  	message_runtime
  	roscpp
  	std_msgs
	sensor_msgs
	cyber_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}

)

# Set the name to use for the executable.
set (BINNAME1 stanley_controller)
set (BINNAME2 pure_pursuit)
set (BINNAME3 rear_feedback)


# Set the source files to use with the executable.
set (SRCS1 ${SRCS1} src/stanley_controller.cpp)
set (SRCS2 ${SRCS2} src/pure_pursuit.cpp)
set (SRCS3 ${SRCS3} src/rear_feedback.cpp)

#set (USB_KEY_SRCS
#    ../../common/Pwd_8/syunew3.cpp
#    ../../common/Pwd_8/libusb/core.c
#    ../../common/Pwd_8/libusb/descriptor.c
#    ../../common/Pwd_8/libusb/io.c
#    ../../common/Pwd_8/libusb/sync.c
#    ../../common/Pwd_8/libusb/os/linux_usbfs.c
#)

#Build the executable that will be used to run this node.
add_executable (${BINNAME1}
                ${SRCS1}
                # ${USB_KEY_SRCS}
)
add_dependencies(${BINNAME1} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})

install(TARGETS ${BINNAME1}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

add_executable (${BINNAME2}
                ${SRCS2}
                # ${USB_KEY_SRCS}
)
add_dependencies(${BINNAME2} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(${BINNAME2} ${catkin_LIBRARIES})

install(TARGETS ${BINNAME2}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

add_executable (${BINNAME3}
                ${SRCS3}
                # ${USB_KEY_SRCS}
)
add_dependencies(${BINNAME3} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${BINNAME3} ${catkin_LIBRARIES})

install(TARGETS ${BINNAME3}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
