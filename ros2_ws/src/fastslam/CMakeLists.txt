cmake_minimum_required(VERSION 3.8)
project(fastslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenMP REQUIRED)

# Pure C++ SLAM engine — no ROS dependencies
add_library(fastslam_core STATIC
  src/fastslam2.cpp
  src/occupancy_grid_map.cpp
  src/cell.cpp
  src/scan_integrator.cpp
  src/motion_model.cpp
  src/particle.cpp
  src/scan_matcher.cpp
)

target_include_directories(fastslam_core PUBLIC include)

target_link_libraries(fastslam_core PUBLIC OpenMP::OpenMP_CXX)

add_executable(clf_runner
  src/clf_runner.cpp
)

target_link_libraries(clf_runner fastslam_core)

# ROS2 node — plumbing only
add_executable(fastslam2_node
  src/fastslam2_node.cpp
)

target_link_libraries(fastslam2_node fastslam_core)

ament_target_dependencies(fastslam2_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS fastslam2_node clf_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
