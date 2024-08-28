cmake_minimum_required(VERSION 3.5)
project(rock_localization)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  hycan_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(localization 
        src/localization.cpp
      )
add_dependencies(localization 
        ${catkin_EXPORTED_TARGETS}
      )
target_link_libraries(localization 
        ${catkin_LIBRARIES}
      )
