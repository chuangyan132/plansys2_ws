#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "target_move",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("target_move");

  // Declare parameters and get their values
  node->declare_parameter<double>("position_x", 0.2532);
  node->declare_parameter<double>("position_y", -0.35107);
  node->declare_parameter<double>("position_z", 0.05);
  node->declare_parameter<double>("orientation_x", 0.0);
  node->declare_parameter<double>("orientation_y", 0.0);
  node->declare_parameter<double>("orientation_z", 0.0);
  node->declare_parameter<double>("orientation_w", 0.0);

  double position_x = node->get_parameter("position_x").as_double();
  double position_y = node->get_parameter("position_y").as_double();
  double position_z = node->get_parameter("position_z").as_double();
  double orientation_x = node->get_parameter("orientation_x").as_double();
  double orientation_y = node->get_parameter("orientation_y").as_double();
  double orientation_z = node->get_parameter("orientation_z").as_double();
  double orientation_w = node->get_parameter("orientation_w").as_double();

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Set a target Pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = position_x;
  target_pose.position.y = position_y;
  target_pose.position.z = position_z;
  target_pose.orientation.x = orientation_x;
  target_pose.orientation.y = orientation_y;
  target_pose.orientation.z = orientation_z;
  target_pose.orientation.w = orientation_w;
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
