cmake_minimum_required(VERSION 3.8)
project(gripper_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gripper_action_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_action_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
)


add_executable(gripper_action_server src/gripper_action_server.cpp)
ament_target_dependencies(gripper_action_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  gripper_action_interfaces
  moveit_core
  moveit_ros_planning
  geometric_shapes
  Eigen3
  rclcpp_components
  tf2_ros
)

install(TARGETS
  gripper_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)


  
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
