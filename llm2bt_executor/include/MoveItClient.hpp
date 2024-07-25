/*
Description: This file is the implementation of the llm2bt_executor 
Maintainer:  Chuang Yan
Email:       yanchuang1122@gmail.com
Date:        25.07.2024
*/
#ifndef MOVEIT_CLIENT_HPP
#define MOVEIT_CLIENT_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "plansys2_object_sorting_interfaces/action/move_it.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <map>


using MoveIt = plansys2_object_sorting_interfaces::action::MoveIt;
using Pose = geometry_msgs::msg::Pose;
using namespace BT;

class MoveItClient : public RosActionNode<MoveIt>
{
public:

    MoveItClient(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
    static PortsList providedPorts();
    bool setGoal(Goal& goal) override;

    NodeStatus onFeedback(const std::shared_ptr<const MoveIt::Feedback> feedback) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;

    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
    void onHalt() override;

private:
    std::shared_ptr<rclcpp::Node> client;
    std::map<std::string, geometry_msgs::msg::Pose> predefined_poses_;
    void initializePredefinedPoses();
    rclcpp::Node::SharedPtr node_;
};

#endif // MOVEIT_CLIENT_HPP