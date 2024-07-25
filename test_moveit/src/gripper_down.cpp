#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <stdexcept>
#include <iostream>
#include <string>
#include <limits>
#include <thread>
#include <mutex>


int main(int argc, char * argv[])
{


  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("gripper_down", node_options);



  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create a ROS logger ========================================================================
  auto const logger = rclcpp::get_logger("gripper_down");
  static const rclcpp::Logger LOGGER = logger;

  // Get the z_decrement parameter
  double z_decrement;
  if (!move_group_node->get_parameter("z_decrement", z_decrement)) {
    RCLCPP_ERROR(logger, "Failed to get 'z_decrement' parameter");
    return 1;
  }

  static const std::string PLANNING_GROUP = "panda_arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose("panda_link8");
  RCLCPP_INFO(logger, "Current Pose: position(%.2f, %.2f, %.2f), orientation(%.2f, %.2f, %.2f, %.2f)",
              current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
              current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w);


  
  // Set a target Pose
  geometry_msgs::msg::Pose target_pose = current_pose.pose;
  target_pose.position.z -= z_decrement;
  RCLCPP_INFO(logger, "================================NEW_POSITION===========================");
  RCLCPP_INFO(logger, "Target Pose: position(%.2f, %.2f, %.2f), orientation(%.2f, %.2f, %.2f, %.2f)",
              target_pose.position.x, target_pose.position.y, target_pose.position.z,
              target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
  move_group.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group.plan(msg));
      return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
      move_group.execute(plan);
  } else {
      RCLCPP_ERROR(logger, "Planning failed!");
  }
  RCLCPP_INFO(logger, "================================NOW_POSITION===========================");
  // Get the new pose of the end effector after execution
  auto new_pose = move_group.getCurrentPose();
  RCLCPP_INFO(logger, "New Pose: position(%.2f, %.2f, %.2f), orientation(%.2f, %.2f, %.2f, %.2f)",
              new_pose.pose.position.x, new_pose.pose.position.y, new_pose.pose.position.z,
              new_pose.pose.orientation.x, new_pose.pose.orientation.y, new_pose.pose.orientation.z, new_pose.pose.orientation.w);
  

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
