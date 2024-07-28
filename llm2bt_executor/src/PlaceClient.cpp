/*
Description: PickClient will pick the object at the given pose
Maintainer:  Chuang Yan
Email:       yanchuang1122@gmail.com
Date:        26.07.2024
*/

#include "PlaceClient.hpp"

PlaceClient::PlaceClient(
  const std::string& name,
  const NodeConfig& conf,
  const RosNodeParams& params)
  : RosActionNode<DetachIt>(name, conf, params)
{
    client = params.nh.lock();
}

PortsList PlaceClient::providedPorts()
{
    return { InputPort<std::string>("object_name") };
}

bool PlaceClient::setGoal(Goal& goal)
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

NodeStatus PlaceClient::onFeedback(const std::shared_ptr<const DetachIt::Feedback> feedback)
{
    RCLCPP_INFO(client->get_logger(), "Feedback received");
    return NodeStatus::RUNNING;
}

NodeStatus PlaceClient::onResultReceived(const WrappedResult& wr)
{
    switch (wr.code) 
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        if(wr.result->success)
        {
            RCLCPP_INFO(client->get_logger(), "Place action succeeded");
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(client->get_logger(), "Place action completed but failed");
            return NodeStatus::FAILURE;
        }
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(client->get_logger(), "Place action was aborted");
        return NodeStatus::FAILURE;
    default:
        RCLCPP_ERROR(client->get_logger(), "Place action failed");
        return NodeStatus::FAILURE;
    }
}

NodeStatus PlaceClient::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(client->get_logger(), "Place action failed");
    return NodeStatus::FAILURE;
}

void PlaceClient::onHalt()
{
    RCLCPP_INFO(client->get_logger(), "Place halt");
}
