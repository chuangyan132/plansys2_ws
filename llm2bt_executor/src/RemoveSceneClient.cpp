/*
Description: RemoveSceneClient will clean all the objects from movegroup.
Maintainer:  Chuang Yan
Email:       yanchuang1122@gmail.com
Date:        25.07.2024
*/

#include "RemoveSceneClient.hpp"

RemoveSceneClient::RemoveSceneClient(
  const std::string& name,
  const NodeConfig& conf,
  const RosNodeParams& params)
  : RosActionNode<RemoveObject>(name, conf, params)
{
    client = params.nh.lock();
}

PortsList RemoveSceneClient::providedPorts()
{
    return {};
}

bool RemoveSceneClient::setGoal(Goal& goal)
{
    return true;
}

NodeStatus RemoveSceneClient::onFeedback(const std::shared_ptr<const RemoveObject::Feedback> feedback)
{
    RCLCPP_INFO(client->get_logger(), "Feedback received");
    return NodeStatus::RUNNING;
}

NodeStatus RemoveSceneClient::onResultReceived(const WrappedResult& wr)
{
    switch (wr.code) 
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        if(wr.result->success)
        {
            RCLCPP_INFO(client->get_logger(), "RemoveScene action succeeded");
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(client->get_logger(), "RemoveScene action completed but failed");
            return NodeStatus::FAILURE;
        }
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(client->get_logger(), "RemoveScene action was aborted");
        return NodeStatus::FAILURE;
    default:
        RCLCPP_ERROR(client->get_logger(), "RemoveScene action failed");
        return NodeStatus::FAILURE;
    }
}

NodeStatus RemoveSceneClient::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(client->get_logger(), "RemoveScene action failed");
    return NodeStatus::FAILURE;
}

void RemoveSceneClient::onHalt()
{
    RCLCPP_INFO(client->get_logger(), "RemoveScene halt");
}

