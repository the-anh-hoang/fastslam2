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
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(mapper_test_node
  src/mapper_test_node.cpp
  src/occupancy_grid_map.cpp
  src/scan_integrator.cpp
)

add_executable(fastslam_node
  src/fastslam_node.cpp
  src/occupancy_grid_map.cpp
  src/scan_integrator.cpp
  src/motion_model.cpp
  src/particle.cpp
)

add_executable(fastslam2_node
  src/fastslam2_node.cpp
  src/occupancy_grid_map.cpp
  src/scan_integrator.cpp
  src/motion_model.cpp
  src/particle.cpp
  src/scan_matcher.cpp
)

target_include_directories(mapper_test_node PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(fastslam_node PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(fastslam2_node PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
)



ament_target_dependencies(mapper_test_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(fastslam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(fastslam2_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)


target_link_libraries(mapper_test_node
  ${OpenCV_LIBS}
)

target_link_libraries(fastslam_node
  ${OpenCV_LIBS}
)

target_link_libraries(fastslam2_node
  ${OpenCV_LIBS}
)


install(TARGETS mapper_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS fastslam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS fastslam2_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
