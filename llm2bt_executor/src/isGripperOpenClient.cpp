/*
Description: isGripperOpenClient will check if the gripper is open.
Maintainer:  Chuang Yan
Email:       yanchuang1122@gmail.com
Date:        25.07.2024
*/

#include "isGripperOpenClient.hpp"

IsGripperOpenClient::IsGripperOpenClient(
  const std::string& name,
  const NodeConfig& conf,
  const RosNodeParams& params)
  : RosActionNode<GripperControl>(name, conf, params)
{
    client = params.nh.lock();
}

PortsList IsGripperOpenClient::providedPorts()
{
    return {};
}

bool IsGripperOpenClient::setGoal(Goal& goal)
{
    double gripper_state = 0.04;
    goal.distance = gripper_state;
    return true;
}

NodeStatus IsGripperOpenClient::onFeedback(const std::shared_ptr<const GripperControl::Feedback> feedback)
{
    RCLCPP_INFO(client->get_logger(), "Feedback received");
    return NodeStatus::RUNNING;
}

NodeStatus IsGripperOpenClient::onResultReceived(const WrappedResult& wr)
{
    switch (wr.code) 
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        if(wr.result->success)
        {
            RCLCPP_INFO(client->get_logger(), "Gripper is open");
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(client->get_logger(), "Gripper is not open");
            return NodeStatus::FAILURE;
        }
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(client->get_logger(), "Gripper action was aborted");
        return NodeStatus::FAILURE;
    default:
        RCLCPP_ERROR(client->get_logger(), "Gripper action failed");
        return NodeStatus::FAILURE;
    }
}

NodeStatus IsGripperOpenClient::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(client->get_logger(), "Gripper action failed");
    return NodeStatus::FAILURE;
}

void IsGripperOpenClient::onHalt()
{
    RCLCPP_INFO(client->get_logger(), "Gripper halt");
}

