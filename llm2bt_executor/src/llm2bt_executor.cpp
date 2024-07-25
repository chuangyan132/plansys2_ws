#include <rclcpp/rclcpp.hpp>
#include "behaviortree_ros2/plugins.hpp"
#include <MoveItClient.hpp>

using namespace BT;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("llm2bt_executor");

    BehaviorTreeFactory factory;
    
    

    RosNodeParams params;
    params.nh = node;
    params.default_port_value = "move_it";

    factory.registerNodeType<MoveItClient>("MoveItClient", params);

    std::string tree_string = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <SetSceneClient case_name="case1"/>
                <MoveItClient target_pose_name="pose1_i"/>
                <RemoveSceneClient />
            </Sequence>
        </BehaviorTree>
    </root>
    )";

    auto tree = factory.createTreeFromText(tree_string);

    RCLCPP_INFO(node->get_logger(), "Behavir Tree is created");
    tree.tickWhileRunning();
    RCLCPP_INFO(node->get_logger(), "Behavir Tree is finished");

    rclcpp::shutdown();
    return 0;
}