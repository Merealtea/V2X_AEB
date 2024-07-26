add_library(discretized_path STATIC path/discretized_path.cc)
target_include_directories(
  discretized_path PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/path
  )
target_link_libraries(discretized_path linear_interpolation glog)

add_library(discretized_trajectory STATIC trajectory/discretized_trajectory.cc)
target_include_directories(
  discretized_trajectory PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/trajectory
  )
target_link_libraries(discretized_trajectory linear_interpolation glog)
