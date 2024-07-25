/*
Description: This file is the implementation of the llm2bt_executor 
Maintainer:  Chuang Yan
Email:       yanchuang1122@gmail.com
Date:        25.07.2024
*/

#ifndef REMOVE_SCENE_CLIENT_HPP
#define REMOVE_SCENE_CLIENT_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "objects_action_interfaces/action/remove_object.hpp"

using RemoveObject = objects_action_interfaces::action::RemoveObject;
using namespace BT;

class RemoveSceneClient : public RosActionNode<RemoveObject>
{
public:
    RemoveSceneClient(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    static PortsList providedPorts();

    bool setGoal(Goal& goal) override;

    NodeStatus onFeedback(const std::shared_ptr<const RemoveObject::Feedback> feedback) override;

    NodeStatus onResultReceived(const WrappedResult& wr) override;

    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;

    void onHalt() override;
private:
    std::shared_ptr<rclcpp::Node> client;
};

#endif // REMOVE_SCENE_CLIENT_HPP