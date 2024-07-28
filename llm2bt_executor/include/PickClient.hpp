/*
Description: PickClient will pick the object at the given pose
Maintainer:  Chuang Yan
Email:       yanchuang1122@gmail.com
Date:        26.07.2024
*/

#ifndef PICK_CLIENT_HPP
#define PICK_CLIENT_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "plansys2_object_sorting_interfaces/action/attach_it.hpp"

using AttachIt = plansys2_object_sorting_interfaces::action::AttachIt;
using namespace BT;

class PickClient : public RosActionNode<AttachIt>
{
public:
    PickClient(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    static PortsList providedPorts();

    bool setGoal(Goal& goal) override;

    NodeStatus onFeedback(const std::shared_ptr<const AttachIt::Feedback> feedback) override;

    NodeStatus onResultReceived(const WrappedResult& wr) override;

    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;

    void onHalt() override;
private:
    std::shared_ptr<rclcpp::Node> client;
    
};



#endif // PICK_CLIENT_HPP