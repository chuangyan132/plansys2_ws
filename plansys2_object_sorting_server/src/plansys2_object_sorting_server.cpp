/*
* Project:     Plansys2 Object Sorting
* File:        plansys2_object_sorting_server.cpp
* Author:      Chuang Yan
* Email:       yanchuang1122@gmail.com
* Date:        19.07.2024
* Description: This file is the server node for the object sorting task.
*/

#include <functional>
#include <memory>
#include <thread>
#include "plansys2_object_sorting_server.hpp"

MultiActionServer::MultiActionServer(
    const rclcpp::NodeOptions & options,
    const std::shared_ptr<rclcpp::Node>& move_group_node
    ) : Node("multi_action_server", options)
{
    using namespace std::placeholders;
    this->declare_parameter("planning_group", "panda_arm");
    this->declare_parameter("end_effector_link", "panda_link8");

    move_it_action_server_ = rclcpp_action::create_server<MoveIt>(
        this,
        "move_it",
        std::bind(&MultiActionServer::handle_goal<MoveIt>, this, _1, _2),
        std::bind(&MultiActionServer::handle_cancel<MoveIt>, this, _1),
        std::bind(&MultiActionServer::handle_accepted<MoveIt>, this, _1)
    );

    attach_it_action_server_ = rclcpp_action::create_server<AttachIt>(
        this,
        "attach_it",
        std::bind(&MultiActionServer::handle_goal<AttachIt>, this, _1, _2),
        std::bind(&MultiActionServer::handle_cancel<AttachIt>, this, _1),
        std::bind(&MultiActionServer::handle_accepted<AttachIt>, this, _1)
    );

    detach_it_action_server_ = rclcpp_action::create_server<DetachIt>(
        this,
        "detach_it",
        std::bind(&MultiActionServer::handle_goal<DetachIt>, this, _1, _2),
        std::bind(&MultiActionServer::handle_cancel<DetachIt>, this, _1),
        std::bind(&MultiActionServer::handle_accepted<DetachIt>, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "Multi Action Server has been started");
    initialize_moveit(move_group_node);
    initialize_cubes();
}

void MultiActionServer::initialize_moveit(const std::shared_ptr<rclcpp::Node>& move_group_node)
{
    planning_group_ = this->get_parameter("planning_group").as_string();
    end_effector_link_ = this->get_parameter("end_effector_link").as_string();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node, planning_group_);
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // !!! double check if planning_scene_monitor work as expected
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        move_group_node,
        "robot_description");
    planning_scene_monitor_->startStateMonitor();
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();

    touch_links_ = {"panda_leftfinger", "panda_rightfinger"};

    RCLCPP_INFO(this->get_logger(), "MoveIt has been initialized");

}
void MultiActionServer::initialize_cubes()
{
    cubes_ = 
    {
        {"red_cube", 0.05, 0.05, 0.05,  geometry_msgs::msg::Pose()},
        {"blue_cube", 0.05, 0.05, 0.05,  geometry_msgs::msg::Pose()}
        // {"green_cube", 0.05, 0.05, 0.05,  geometry_msgs::msg::Pose()},
        // {"yellow_cube", 0.05, 0.05, 0.05,  geometry_msgs::msg::Pose()}
    };

    cubes_[0].pose.position.x = 0.5;
    cubes_[0].pose.position.y = 0.0;
    cubes_[0].pose.position.z = 0.0;

    cubes_[1].pose.position.x = 0.3;
    cubes_[1].pose.position.y = -0.3;
    cubes_[1].pose.position.z = 0.0;

    for (auto & cube : cubes_)
    {
        cube.pose.orientation.w = 1.0;
        cube.pose.orientation.x = 0.0;
        cube.pose.orientation.y = 0.0;
        cube.pose.orientation.z = 0.0;
        add_cube_to_planning_scene(cube);
    }
}

void MultiActionServer::add_cube_to_planning_scene(const CubeInfo & cube)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_->getPlanningFrame();
    collision_object.id = cube.id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {cube.size_x, cube.size_y, cube.size_z};

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cube.pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface_->applyCollisionObject(collision_object);
}

void MultiActionServer::update_planning_scene()
{
    planning_scene_monitor_->requestPlanningSceneState();
    planning_scene_monitor_->waitForCurrentRobotState(this->now());
    planning_scene_monitor_->getPlanningScene()->getCurrentStateNonConst().update();
}

template <typename T>
rclcpp_action::GoalResponse MultiActionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const typename T::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal requst");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

template <typename T>
rclcpp_action::CancelResponse MultiActionServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

template <typename T>
void MultiActionServer::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<T>> goal_handle)
{
    std::thread{[this, goal_handle]() { this->execute_goal(goal_handle); }}.detach();
}

void MultiActionServer::execute_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveIt>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing MoveIt goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveIt::Feedback>();
    auto result = std::make_shared<MoveIt::Result>();

    // Why we do not need to update the planning scene here? 
    // Its because the planning scene is updated in the constructor
    // update_planning_scene();
    try
    {
        move_group_->setPoseTarget(goal->target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            move_group_->execute(my_plan);
            result->success = true;
            RCLCPP_INFO(this->get_logger(), "MoveIt plan succeeded, executing...");
        }
        else
        {
            result->success = false;
            RCLCPP_INFO(this->get_logger(), "MoveIt plan failed");
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "MoveIt execution failed: %s", e.what());
        result->success = false;
        result->error_code = -1;
    }


    if (rclcpp::ok())
    {
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "MoveIt goal succeeded");
    }
}
    
void MultiActionServer::execute_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<AttachIt>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing AttachIt goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<AttachIt::Result>();

    try 
    {
        // we use lambda function to find the cube with the same id as the goal object_id
        // "It" is the iterator that points to the first element that satisfies the condition
       auto it = std::find_if(cubes_.begin(), cubes_.end(), 
            [&](const CubeInfo & cube) { return cube.id == goal->object_id; });
        
        /*
        explanation: cubes_.end() is the end of the vector, "it" is the iterator that points to 
        the first element that satisfies the condition
        */
        if (it == cubes_.end()){
            throw std::runtime_error("Cube not found" + goal->object_id);        
        }

        const CubeInfo& cube = *it;

        move_group_->attachObject(cube.id, end_effector_link_, touch_links_);
        update_planning_scene();
        result->success = true;
        RCLCPP_INFO(this->get_logger(), "Object %s attached successfully", goal->object_id.c_str());

    }
    catch (const std::runtime_error & e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to attach object: %s", e.what());
        result->success = false;
        result->error_message = e.what();
    }

    if (rclcpp::ok())
    {
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "AttachIt goal succeeded");
    }
}

void MultiActionServer::execute_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<DetachIt>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing DetachIt goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<DetachIt::Result>();

    try
    {
        move_group_->detachObject(goal->object_id);
        auto it = std::find_if(cubes_.begin(), cubes_.end(), 
            [&](const CubeInfo & cube) { return cube.id == goal->object_id; });

        if (it != cubes_.end())
        {
            it->pose = goal->new_pose;
            add_cube_to_planning_scene(*it);
        }

        update_planning_scene();
        result->success = true;
        RCLCPP_INFO(this->get_logger(), "Object %s detached successfully", goal->object_id.c_str());


    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "Failed to detach object: %s", e.what());
        result->success = false;
        result->error_message = e.what();
    }

    if (rclcpp::ok())
    {
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "DetachIt goal succeeded");
    }
    
}

// Explicit instantiation handle_goal of MoveIt, AttachIt, and DetachIt
template rclcpp_action::GoalResponse MultiActionServer::handle_goal<MultiActionServer::MoveIt>(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MultiActionServer::MoveIt::Goal>);
template rclcpp_action::GoalResponse MultiActionServer::handle_goal<MultiActionServer::AttachIt>(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MultiActionServer::AttachIt::Goal>);
template rclcpp_action::GoalResponse MultiActionServer::handle_goal<MultiActionServer::DetachIt>(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MultiActionServer::DetachIt::Goal>);

// Explicit instantiation handle_cancel of MoveIt, AttachIt, and DetachIt
template rclcpp_action::CancelResponse MultiActionServer::handle_cancel<MultiActionServer::MoveIt>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::MoveIt>>);
template rclcpp_action::CancelResponse MultiActionServer::handle_cancel<MultiActionServer::AttachIt>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::AttachIt>>);
template rclcpp_action::CancelResponse MultiActionServer::handle_cancel<MultiActionServer::DetachIt>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::DetachIt>>);

// Explicit instantiation handle_accepted of MoveIt, AttachIt, and DetachIt
template void MultiActionServer::handle_accepted<MultiActionServer::MoveIt>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::MoveIt>>);
template void MultiActionServer::handle_accepted<MultiActionServer::AttachIt>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::AttachIt>>);
template void MultiActionServer::handle_accepted<MultiActionServer::DetachIt>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::DetachIt>>);



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = std::make_shared<rclcpp::Node>("plansys2_object_sorting_server", node_options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    auto multi_action_server = std::make_shared<MultiActionServer>(node_options, move_group_node);
    rclcpp::spin(multi_action_server);
    rclcpp::shutdown();
    return 0;
}

