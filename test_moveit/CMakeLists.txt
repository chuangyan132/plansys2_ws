cmake_minimum_required(VERSION 3.8)
project(test_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(test_moveit src/test_moveit.cpp)
add_executable(gripper_control src/gripper_control.cpp)
add_executable(target_move src/target_move.cpp)
add_executable(gripper_down src/gripper_down.cpp)

target_include_directories(test_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(test_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  test_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(gripper_control
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  moveit_ros_planning
  geometric_shapes
  Eigen3
  rclcpp_components
  tf2_ros
)

ament_target_dependencies(
  target_move
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
)

ament_target_dependencies(
  gripper_down
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
)
  

install(TARGETS 
  test_moveit
  gripper_control
  target_move
  gripper_down
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
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
