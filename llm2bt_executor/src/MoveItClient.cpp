/*
Description: This file is the implementation of the llm2bt_executor 
Maintainer:  Chuang Yan
Email:       yanchuang1122@gmail.com
Date:        25.07.2024
*/


#include "MoveItClient.hpp"

MoveItClient::MoveItClient(
  const std::string& name,
  const NodeConfig& conf,
  const RosNodeParams& params)
  : RosActionNode<MoveIt>(name, conf, params)
{
    client = params.nh.lock();
    initializePredefinedPoses();
}

PortsList MoveItClient::providedPorts()
{
    return { InputPort<std::string>("target_pose_name") };
}

bool MoveItClient::setGoal(Goal& goal)
{
    Pose target_pose;
    std::string tp_name;
    
    if(!getInput("target_pose_name", tp_name))
    {
        RCLCPP_ERROR(client->get_logger(), "Failed to get target_pose from port");
        return false;
    }

    goal.target_pose = predefined_poses_[tp_name];
    return true;
}

NodeStatus MoveItClient::onFeedback(const std::shared_ptr<const MoveIt::Feedback> feedback)
{
    RCLCPP_INFO(client->get_logger(), "Feedback received");
    return NodeStatus::RUNNING;
}

NodeStatus MoveItClient::onResultReceived(const WrappedResult& wr)
{
    switch (wr.code) 
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        if(wr.result->success)
        {
            RCLCPP_INFO(client->get_logger(), "MoveIt action succeeded");
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(client->get_logger(), "MoveIt action completed but failed");
            return NodeStatus::FAILURE;
        }
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(client->get_logger(), "MoveIt action was aborted");
        return NodeStatus::FAILURE;

    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(client->get_logger(), "MoveIt action was canceled");
        return NodeStatus::FAILURE;
 
    default:
        RCLCPP_ERROR(client->get_logger(), "Unknown result code");
        return NodeStatus::FAILURE;
    }
}

NodeStatus MoveItClient::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(node_->get_logger(), "MoveIt action failed with error code %d", error);
    return NodeStatus::FAILURE;
}

void MoveItClient::onHalt()
{
    RCLCPP_WARN(node_->get_logger(), "MoveIt action halted");
}

void MoveItClient::initializePredefinedPoses()
{
    Pose pose;
    pose.position.x = 0.3;
    pose.position.y = 0.3;
    pose.position.z = 0.14;
    pose.orientation.x = -0.3827;
    pose.orientation.y = 0.9239;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    predefined_poses_["pose1_i"] = pose; // initial pose for pose1(first cube)

    pose.position.x = 0.3;
    pose.position.y = -0.3;
    pose.position.z = 0.14;
    predefined_poses_["pose1_d"] = pose; // desired pose for pose1(first cube)

    pose.position.x = -0.3;
    pose.position.y = -0.3;
    pose.position.z = 0.14;
    predefined_poses_["pose2_i"] = pose; // initial pose for pose2(second cube) and so on...

    pose.position.x = -0.3;
    pose.position.y = 0.3;
    pose.position.z = 0.14;
    predefined_poses_["pose2_d"] = pose;

    pose.position.x = 0.3;
    pose.position.y = 0.2;
    pose.position.z = 0.4;
    predefined_poses_["pose0"] = pose;
}
