/*
Description: This file is the implementation of the llm2bt_executor 
Maintainer:  Chuang Yan
Email:       yanchuang1122@gmail.com
Date:        25.07.2024
*/

#include "SetSceneClient.hpp"

SetSceneClient::SetSceneClient(
  const std::string& name,
  const NodeConfig& conf,
  const RosNodeParams& params)
  : RosActionNode<AddObject>(name, conf, params)
{
    client = params.nh.lock();
    initializePredefinedCase();
}

PortsList SetSceneClient::providedPorts()
{
    return { InputPort<std::string>("case_name") };
}

bool SetSceneClient::setGoal(Goal& goal)
{
    std::string case_name;
    
    if(!getInput("case_name", case_name))
    {
        RCLCPP_ERROR(client->get_logger(), "Failed to get case_name from port");
        return false;
    }
    auto it = predefined_case_.find(case_name);
    if(it != predefined_case_.end())
    {
        const CaseInfo& case_info = it->second;
        goal.config_file_path = case_info.config_file_path;
        goal.object_names = case_info.object_names;

        RCLCPP_INFO(client->get_logger(), "SetScene goal for %s with config_file_path: %s", 
                    case_name.c_str(), goal.config_file_path.c_str());
        return true;

    } else {
        RCLCPP_ERROR(client->get_logger(), "Unsupported case_name: %s", case_name.c_str());
        return false;
    }

}

NodeStatus SetSceneClient::onFeedback(const std::shared_ptr<const AddObject::Feedback> feedback)
{
    RCLCPP_INFO(client->get_logger(), "Feedback received");
    return NodeStatus::RUNNING;
}

NodeStatus SetSceneClient::onResultReceived(const WrappedResult& wr)
{
    switch (wr.code) 
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        if(wr.result->success)
        {
            RCLCPP_INFO(client->get_logger(), "SetScene add objects to movegroup succeeded");
            return NodeStatus::SUCCESS;
        } else {
            RCLCPP_ERROR(client->get_logger(), "SetScene action completed but failed");
            return NodeStatus::FAILURE;
        }
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(client->get_logger(), "SetScene action was aborted");
        return NodeStatus::FAILURE;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(client->get_logger(), "SetScene action was canceled");
        return NodeStatus::FAILURE;
    default:
        RCLCPP_ERROR(client->get_logger(), "Unknown result code");
        return NodeStatus::FAILURE;
    }
}

NodeStatus SetSceneClient::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(client->get_logger(), "SetScene action failed with error code %d", error);
    return NodeStatus::FAILURE;
}

void SetSceneClient::onHalt()
{
    RCLCPP_WARN(client->get_logger(), "SetScene action halted");
}

void SetSceneClient::initializePredefinedCase()
{
    std::string package_name = "llm2bt_executor";
    std::string config_file_name = "case1.yaml";
    std::string config_file_path = ament_index_cpp::get_package_share_directory(package_name) + "/config/" + config_file_name;
    predefined_case_["case1"] = CaseInfo{
        config_file_path,
        {"blue_cube", "red_cube"}
    };

    config_file_name = "case2.yaml";
    config_file_path = ament_index_cpp::get_package_share_directory(package_name) + "/config/" + config_file_name;
    predefined_case_["case2"] = CaseInfo{
        config_file_path,
        {"blue_cube", "red_cube", "green_cube", "yellow_cube"}
    };
}

    