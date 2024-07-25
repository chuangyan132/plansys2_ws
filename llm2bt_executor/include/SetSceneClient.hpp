/*
Description: This file is the implementation of the llm2bt_executor 
Maintainer:  Chuang Yan
Email:       yanchuang1122@gmail.com
Date:        25.07.2024
*/
#ifndef SET_SCENE_CLIENT_HPP
#define SET_SCENE_CLIENT_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "objects_action_interfaces/action/add_object.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <vector>
#include <map>
#include <string>

using AddObject = objects_action_interfaces::action::AddObject;
using namespace BT;

class SetSceneClient : public RosActionNode<AddObject>
{
public:
    SetSceneClient(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    static PortsList providedPorts();

    bool setGoal(Goal& goal) override;

    NodeStatus onFeedback(const std::shared_ptr<const AddObject::Feedback> feedback) override;

    NodeStatus onResultReceived(const WrappedResult& wr) override;

    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;

    void onHalt() override;
private:
    std::shared_ptr<rclcpp::Node> client;
    struct CaseInfo
    {
        std::string config_file_path;
        std::vector <std::string> object_names;
    };
        
    std::map<std::string, CaseInfo> predefined_case_;
    void initializePredefinedCase();
    
};

#endif // SET_SCENE_CLIENT_HPP