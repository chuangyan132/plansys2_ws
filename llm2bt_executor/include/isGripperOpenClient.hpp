/*
Description: isGripperOpenClient will check if the gripper is open.
Maintainer:  Chuang Yan
Email:       yanchuang1122@gmail.com
Date:        25.07.2024
*/

#include "behaviortree_ros2/bt_action_node.hpp"
#include "gripper_action_interfaces/action/gripper_control.hpp"

using GripperControl = gripper_action_interfaces::action::GripperControl;
using namespace BT;

class IsGripperOpenClient : public RosActionNode<GripperControl>
{
public:
    IsGripperOpenClient(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    static PortsList providedPorts();

    bool setGoal(Goal& goal) override;

    NodeStatus onFeedback(const std::shared_ptr<const GripperControl::Feedback> feedback) override;

    NodeStatus onResultReceived(const WrappedResult& wr) override;

    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;

    void onHalt() override;
private:
    std::shared_ptr<rclcpp::Node> client;
    
};