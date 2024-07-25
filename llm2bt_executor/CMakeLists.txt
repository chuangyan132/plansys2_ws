cmake_minimum_required(VERSION 3.8)
project(llm2bt_executor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(gripper_action_interfaces REQUIRED)
find_package(plansys2_object_sorting_interfaces REQUIRED)
find_package(objects_action_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
)

set(THIS_PACKAGE_DEPS
  rclcpp
  rclcpp_action
  rclcpp_components
  behaviortree_ros2
  btcpp_ros2_interfaces
  gripper_action_interfaces
  plansys2_object_sorting_interfaces
  objects_action_interfaces
  geometry_msgs
  ament_index_cpp
)

add_executable(llm2bt_executor 
  src/llm2bt_executor.cpp
  src/MoveItClient.cpp
  src/SetSceneClient.cpp
  )
ament_target_dependencies(llm2bt_executor ${THIS_PACKAGE_DEPS})

install(TARGETS
  llm2bt_executor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
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
