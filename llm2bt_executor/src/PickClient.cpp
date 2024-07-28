/*
Description: PickClient will pick the object at the given pose
Maintainer:  Chuang Yan
Email:       yanchuang1122@gmail.com
Date:        26.07.2024
*/

#include "PickClient.hpp"

PickClient::PickClient(
  const std::string& name,
  const NodeConfig& conf,
  const RosNodeParams& params)
  : RosActionNode<AttachIt>(name, conf, params)
{
    client = params.nh.lock();
}

PortsList PickClient::providedPorts()
{
    return { InputPort<std::string>("object_name") };
}

bool PickClient::setGoal(Goal& goal)
{
    std::string object_name_;
    
    if(!getInput("object_name", object_name_))
    {
        RCLCPP_ERROR(client->get_logger(), "Failed to get object_name from port");
        return false;
    }

    goal.object_id = object_name_;
    return true;
}

NodeStatus PickClient::onFeedback(const std::shared_ptr<const AttachIt::Feedback> feedback)
{
    RCLCPP_INFO(client->get_logger(), "Feedback received");
    return NodeStatus::RUNNING;
}

NodeStatus PickClient::onResultReceived(const WrappedResult& wr)
{
    switch (wr.code) 
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        if(wr.result->success)
        {
            RCLCPP_INFO(client->get_logger(), "Pick action succeeded");
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(client->get_logger(), "Pick action completed but failed");
            return NodeStatus::FAILURE;
        }
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(client->get_logger(), "Pick action was aborted");
        return NodeStatus::FAILURE;
    default:
        RCLCPP_ERROR(client->get_logger(), "Pick action failed");
        return NodeStatus::FAILURE;
    }
}

NodeStatus PickClient::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(client->get_logger(), "Pick action failed");
    return NodeStatus::FAILURE;
}

void PickClient::onHalt()
{
    RCLCPP_INFO(client->get_logger(), "Pick halt");
}
