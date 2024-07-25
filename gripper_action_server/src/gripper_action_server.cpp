/*
Project:      gripper_action_server
Description:  A server node that receives a goal to control the gripper and executes the action.
Author:       Chuang Yan
Email:        yanchuang1122@gmailcom
Date:         2024.07.13
*/

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include "gripper_action_interfaces/action/gripper_control.hpp"



class GripperActionServer : public rclcpp::Node
{
public:

    using GripperControl = gripper_action_interfaces::action::GripperControl;
    using GoalHandleGripperControl = rclcpp_action::ServerGoalHandle<GripperControl>;
 
    GripperActionServer() : Node("gripper_action_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<GripperControl>(
            this,
            "gripper_control",
            std::bind(&GripperActionServer::handle_goal, this, _1, _2),
            std::bind(&GripperActionServer::handle_cancel, this, _1),
            std::bind(&GripperActionServer::handle_accepted, this, _1)
            );
    }


private:
    rclcpp_action::Server<GripperControl>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GripperControl::Goal> goal
        )
    {
        RCLCPP_INFO(this->get_logger(), "Received Goal: gripper distance %f", goal->distance);
        (void)uuid; 
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGripperControl> goal_handle
        )
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<GoalHandleGripperControl> goal_handle
        )
    {
        std::thread{std::bind(&GripperActionServer::execute, this, goal_handle), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleGripperControl> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<GripperControl::Feedback>();
        auto result = std::make_shared<GripperControl::Result>();

        static const std::string PLANNING_GROUP = "panda_arm_hand";
        moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), PLANNING_GROUP);

        const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        std::vector<double> joint_group_positions;
        moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

        current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

        joint_group_positions[7] = goal->distance;
        joint_group_positions[8] = joint_group_positions[7];

        move_group.setJointValueTarget(joint_group_positions);

        move_group.setMaxVelocityScalingFactor(0.05);
        move_group.setMaxAccelerationScalingFactor(0.05);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if(success)
        {
            auto exec_result = move_group.execute(my_plan);

            // !!!==================For demo only. not correct!!===================
            result->success = true;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");

            // if(exec_result == moveit::core::MoveItErrorCode::SUCCESS)
            // {
            //     result->success = true;
            //     goal_handle->succeed(result);
            //     RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            // } else {
            //     result->success = false;
            //     goal_handle->abort(result);
            //     RCLCPP_ERROR(this->get_logger(), "gripper control failed");
            // }
        } else {
            result->success = false;
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
        }    
    
    }

};

int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}


