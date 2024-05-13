cmake_minimum_required(VERSION 3.8)
project(ros2_dwa_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include directory 

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add executable
add_executable(dwa_planner_node src/dwa_planner.cpp)
ament_target_dependencies(dwa_planner_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  Eigen3
  builtin_interfaces
)
target_include_directories(dwa_planner_node PUBLIC
  ./include
)

add_executable(goal_publisher src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher
  rclcpp
  geometry_msgs
  tf2
  rosgraph_msgs
)

add_executable(footprint_publisher src/footprint_publisher.cpp)
ament_target_dependencies(footprint_publisher
  rclcpp
  geometry_msgs
  tf2
  rosgraph_msgs
)

# Installing the node
install(TARGETS dwa_planner_node goal_publisher footprint_publisher
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
