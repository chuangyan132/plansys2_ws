/*
* Project:     Plansys2 Object Sorting
* File:        plansys2_object_sorting_server.hpp
* Author:      Chuang Yan
* Email:       yanchuang1122@gmail.com
* Date:        19.07.2024
* Description: This file is the server node for the object sorting task.
*/

#ifndef PLANSYS2_OBJECT_SORTING_SERVER__HPP
#define PLANSYS2_OBJECT_SORTING_SERVER__HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/plane.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "plansys2_object_sorting_interfaces/action/move_it.hpp"
#include "plansys2_object_sorting_interfaces/action/attach_it.hpp"
#include "plansys2_object_sorting_interfaces/action/detach_it.hpp"
#include "objects_action_interfaces/action/add_object.hpp"
#include "objects_action_interfaces/action/remove_object.hpp"

#include "gripper_action_interfaces/action/gripper_control.hpp"
#include "isaac_object_interfaces/msg/isaac_object_info.hpp"

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <thread>
#include <future>
#include <yaml-cpp/yaml.h>

class MultiActionServer : public rclcpp::Node
{
public:
    using AddObject = objects_action_interfaces::action::AddObject;
    using MoveIt = plansys2_object_sorting_interfaces::action::MoveIt;
    using AttachIt = plansys2_object_sorting_interfaces::action::AttachIt;
    using DetachIt = plansys2_object_sorting_interfaces::action::DetachIt;
    using RemoveObject = objects_action_interfaces::action::RemoveObject;
    using IsaacObjectInfo = isaac_object_interfaces::msg::IsaacObjectInfo;

    // we use explicit here because we want to avoid the constructor to be used for implicit conversions
    explicit MultiActionServer(
        const rclcpp::NodeOptions & options,
        const std::shared_ptr<rclcpp::Node>& move_group_node
        );

private:
    std::string planning_group_;
    std::string end_effector_link_;
    std::vector<std::string> touch_links_;
    rclcpp::Subscription<IsaacObjectInfo>::SharedPtr isaac_subscriber_;

    // ================================== Action Servers ==================================
    rclcpp_action::Server<AddObject>::SharedPtr add_object_action_server_;
    rclcpp_action::Server<MoveIt>::SharedPtr move_it_action_server_;
    rclcpp_action::Server<AttachIt>::SharedPtr attach_it_action_server_;
    rclcpp_action::Server<DetachIt>::SharedPtr detach_it_action_server_;
    rclcpp_action::Server<RemoveObject>::SharedPtr remove_object_action_server_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp_action::Client<gripper_action_interfaces::action::GripperControl>::SharedPtr gripper_client_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;


    // TODO: add more objects struct
    struct CubeInfo  
    {
        std::string id;
        double size_x, size_y, size_z;
        geometry_msgs::msg::Pose pose;
    };
    std::map<std::string, CubeInfo> cubes_;
    //std::map<std::string, CubeInfo> cube_objects_;
    //std::vector<CubeInfo> cubes_;
    
    

    void initialize_moveit(const std::shared_ptr<rclcpp::Node>& move_group_node);
    // void initialize_cubes();
    void add_ground_plane(double height);
    void publish_ground_plane_marker(double height);
    void add_cube_to_planning_scene(const CubeInfo & cube);
    void update_planning_scene();
    std::future<bool> control_gripper(double distance);
    void isaac_object_info_callback(const IsaacObjectInfo::SharedPtr msg);

    // ================================== Template Action Handlers ==========================
    template <typename T>
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const typename T::Goal> goal);

    template <typename T>
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle);

    template <typename T>
    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle);

    // ================================== Action Callbacks ==================================
    void execute_goal(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<AddObject>> goal_handle);

    void execute_goal(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveIt>> goal_handle);

    void execute_goal(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<AttachIt>> goal_handle);

    void execute_goal(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<DetachIt>> goal_handle);

    void execute_goal(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<RemoveObject>> goal_handle);



};


#endif  // PLANSYS2_OBJECT_SORTING_SERVER__HPP