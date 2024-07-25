#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("gripper_control", node_options);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    // Create a ROS logger ========================================================================
    auto const logger = rclcpp::get_logger("hello_moveit");
    static const rclcpp::Logger LOGGER = logger;

    static const std::string PLANNING_GROUP = "panda_arm_hand";

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    // // Set a target joint position ================================================================
    // std::vector<double> joint_group_positions(2);
    // joint_group_positions[0] = 0.0;
    // joint_group_positions[1] = 0.0; //0.4
    
    const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    std::vector<double> joint_group_positions;
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // Print the joint positions
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    for (std::size_t i = 0; i < joint_group_positions.size(); ++i)
    {
        std::cout << joint_names[i] << ": " << joint_group_positions[i] << std::endl;
    }
    joint_group_positions[7] = 0.0;
    joint_group_positions[8] = 0.0;
    move_group.setJointValueTarget(joint_group_positions);

    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

    // Execute the plan
    if(success) {
    move_group.execute(my_plan);
    } else {
    RCLCPP_ERROR(LOGGER, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;

}