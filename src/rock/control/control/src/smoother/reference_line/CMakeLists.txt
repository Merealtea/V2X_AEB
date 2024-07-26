include_directories(${CMAKE_SOURCE_DIR}
  ../common
  ../math)

add_library(reference_line_smoother STATIC discrete_points_reference_line_smoother.cc)
target_link_libraries(reference_line_smoother discrete_points_math
discretized_points_smoothing)
