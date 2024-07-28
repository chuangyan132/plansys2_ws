#include <rclcpp/rclcpp.hpp>
#include "behaviortree_ros2/plugins.hpp"
#include "MoveItClient.hpp"
#include "SetSceneClient.hpp"
#include "PickClient.hpp"
#include "PlaceClient.hpp"
#include "RemoveSceneClient.hpp"
#include "isGripperOpenClient.hpp"


using namespace BT;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    

    BehaviorTreeFactory factory;
    
    
    auto node = std::make_shared<rclcpp::Node>("llm2bt_executor");
    RosNodeParams params;
    params.nh = node;
    params.default_port_value = "move_it";
    params.wait_for_server_timeout = std::chrono::milliseconds(2000);

    factory.registerNodeType<MoveItClient>("MoveItClient", params);


    RosNodeParams set_scene_client_params;
    set_scene_client_params.nh = node;
    set_scene_client_params.default_port_value = "add_object"; 
    set_scene_client_params.wait_for_server_timeout = std::chrono::milliseconds(2000);

    factory.registerNodeType<SetSceneClient>("SetSceneClient", set_scene_client_params);


    RosNodeParams pick_client_params;
    pick_client_params.nh = node;
    pick_client_params.default_port_value = "attach_it";
    pick_client_params.wait_for_server_timeout = std::chrono::milliseconds(2000);

    factory.registerNodeType<PickClient>("PickClient", pick_client_params);


    RosNodeParams place_client_params;
    place_client_params.nh = node;
    place_client_params.default_port_value = "detach_it";
    place_client_params.wait_for_server_timeout = std::chrono::milliseconds(2000);

    factory.registerNodeType<PlaceClient>("PlaceClient", place_client_params);


    RosNodeParams remove_scene_client_params;
    remove_scene_client_params.nh = node;
    remove_scene_client_params.default_port_value = "remove_object";
    remove_scene_client_params.wait_for_server_timeout = std::chrono::milliseconds(2000);

    factory.registerNodeType<RemoveSceneClient>("RemoveSceneClient", remove_scene_client_params);


    RosNodeParams is_gripper_open_client_params;
    is_gripper_open_client_params.nh = node;
    is_gripper_open_client_params.default_port_value = "gripper_control";
    is_gripper_open_client_params.wait_for_server_timeout = std::chrono::milliseconds(2000);

    factory.registerNodeType<IsGripperOpenClient>("IsGripperOpenClient", is_gripper_open_client_params);

    std::string tree_string = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <Fallback name="main_fallback">
                    <Sequence name="main_sequence">
                        <SetSceneClient case_name="case1"/>
                        <IsGripperOpenClient />
                        <MoveItClient target_pose_name="pose1_ia"/>
                        <MoveItClient target_pose_name="pose1_i"/>
                        <PickClient object_name="blue_cube" />
                        <MoveItClient target_pose_name="pose1_da"/>
                        <MoveItClient target_pose_name="pose1_d"/>
                        <PlaceClient object_name="blue_cube" />
                    </Sequence>
                    <AlwaysSuccess />
                </Fallback>
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