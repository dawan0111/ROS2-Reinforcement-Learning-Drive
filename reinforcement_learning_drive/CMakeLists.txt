cmake_minimum_required(VERSION 3.8)
project(reinforcement_learning_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-DTBB_ENABLE_IPO=OFF)

# find dependencies
set(library_name rld_lib)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(TBB REQUIRED)
find_package(reinforcement_learning_drive_interface REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(${library_name}
  src/reinforcement_learning_drive.cpp
  src/actor/actor.cpp
  src/actor/ros2_actor.cpp
  src/actor/ackermann_steering_actor.cpp
  src/environment/occupancy_grid_environment.cpp
  src/reward/path_tracking_reward.cpp
  src/reward/scan_reward.cpp
)

ament_target_dependencies(${library_name}
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  reinforcement_learning_drive_interface
)

target_link_libraries(${library_name}
  TBB::tbb
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${G2O_INCLUDE_DIRS}
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(reinforcement_learning_drive_node
  apps/reinforcement_learning_drive_node.cpp
)
ament_target_dependencies(reinforcement_learning_drive_node
  rclcpp
)
target_link_libraries(reinforcement_learning_drive_node
  ${library_name}
)

install(TARGETS
reinforcement_learning_drive_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch map rviz
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
