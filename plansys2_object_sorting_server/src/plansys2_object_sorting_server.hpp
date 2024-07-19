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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "plansys2_object_sorting_interfaces/action/move_it.hpp"
#include "plansys2_object_sorting_interfaces/action/attach_it.hpp"
#include "plansys2_object_sorting_interfaces/action/detach_it.hpp"

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <thread>

class MultiActionServer : public rclcpp::Node
{
public:
    using MoveIt = plansys2_object_sorting_interfaces::action::MoveIt;
    using AttachIt = plansys2_object_sorting_interfaces::action::AttachIt;
    using DetachIt = plansys2_object_sorting_interfaces::action::DetachIt;

    explicit MultiActionServer(
        const rclcpp::NodeOptions & options,
        const std::shared_ptr<rclcpp::Node>& move_group_node
        );

private:
    rclcpp_action::Server<MoveIt>::SharedPtr move_it_action_server_;
    rclcpp_action::Server<AttachIt>::SharedPtr attach_it_action_server_;
    rclcpp_action::Server<DetachIt>::SharedPtr detach_it_action_server_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

    std::string planning_group_;
    std::string end_effector_link_;
    std::vector<std::string> touch_links_;

    struct CubeInfo
    {
        std::string id;
        double size_x, size_y, size_z;
        geometry_msgs::msg::Pose pose;
    };

    std::vector<CubeInfo> cubes_;
    
    

    void initialize_moveit(const std::shared_ptr<rclcpp::Node>& move_group_node);
    void initialize_cubes();
    void add_cube_to_planning_scene(const CubeInfo & cube);
    void update_planning_scene();

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

    void execute_goal(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveIt>> goal_handle);

    void execute_goal(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<AttachIt>> goal_handle);

    void execute_goal(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<DetachIt>> goal_handle);



};


#endif  // PLANSYS2_OBJECT_SORTING_SERVER__HPP