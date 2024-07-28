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
    
    double ground_plane_height = -0.01;
    

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("ground_plane_marker", 10);
    
    add_object_action_server_ = rclcpp_action::create_server<AddObject>(
        this,
        "add_object",
        std::bind(&MultiActionServer::handle_goal<AddObject>, this, _1, _2),
        std::bind(&MultiActionServer::handle_cancel<AddObject>, this, _1),
        std::bind(&MultiActionServer::handle_accepted<AddObject>, this, _1)
    );
    
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

    remove_object_action_server_ = rclcpp_action::create_server<RemoveObject>(
        this,
        "remove_object",
        std::bind(&MultiActionServer::handle_goal<RemoveObject>, this, _1, _2),
        std::bind(&MultiActionServer::handle_cancel<RemoveObject>, this, _1),
        std::bind(&MultiActionServer::handle_accepted<RemoveObject>, this, _1)
    );

    gripper_client_ =  rclcpp_action::create_client<gripper_action_interfaces::action::GripperControl>
    (
        this,
        "gripper_control"
    );

    isaac_subscriber_ = this->create_subscription<IsaacObjectInfo>(
        "isaac_object_info", 1,
        std::bind(&MultiActionServer::isaac_object_info_callback, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "Multi Action Server has been started");
    initialize_moveit(move_group_node);
    // initialize_cubes();
    add_ground_plane(ground_plane_height);
    
}

void MultiActionServer::isaac_object_info_callback(const IsaacObjectInfo::SharedPtr msg)
{
    CubeInfo cube;
    cube.id = msg->object_id;
    cube.size_x = msg->size_x;
    cube.size_y = msg->size_y;
    cube.size_z = msg->size_z;
    cube.pose = msg->pose;

    add_cube_to_planning_scene(cube);
    cubes_[cube.id] = cube;
}

void MultiActionServer::initialize_moveit(const std::shared_ptr<rclcpp::Node>& move_group_node)
{
    planning_group_ = this->get_parameter("planning_group").as_string();
    end_effector_link_ = this->get_parameter("end_effector_link").as_string();

    // no possible to have two move_group_interface in the same node
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

void MultiActionServer::add_ground_plane(double height)
{
    moveit_msgs::msg::CollisionObject ground_plane;
    ground_plane.header.frame_id = move_group_->getPlanningFrame();
    ground_plane.id = "ground_plane";

    shape_msgs::msg::Plane plane;
    plane.coef = {0.0, 0.0, 1.0, height};

    ground_plane.planes.push_back(plane);

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.z = height;
    ground_plane.plane_poses.push_back(pose);

    ground_plane.operation = ground_plane.ADD;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(ground_plane);
    planning_scene_interface_->applyCollisionObjects(collision_objects);

    publish_ground_plane_marker(height);
    RCLCPP_INFO(this->get_logger(), "Ground plane added successfully at height %f", height);
}

std::future<bool> MultiActionServer::control_gripper(double distance)
{
    return std::async(std::launch::async, [this, distance]()
    {
        if (!gripper_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Gripper action server not available");
            return false;
        }

        auto goal_msg = gripper_action_interfaces::action::GripperControl::Goal();
        goal_msg.distance = distance;

        auto send_goal_options = rclcpp_action::Client<gripper_action_interfaces::action::GripperControl>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const auto&) { RCLCPP_INFO(this->get_logger(), "Gripper goal accepted"); };

        auto future = gripper_client_->async_send_goal(goal_msg, send_goal_options);

        if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send gripper goal");
            return false;
        }

        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            return false;
        }

        auto result_future = gripper_client_->async_get_result(goal_handle);

        if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get gripper result");
            return false;
        }

        auto wrapped_result = result_future.get();
        return wrapped_result.result->success;
    });

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

    // Check if the object is already in the planning scene
    if (planning_scene_interface_->getObjects().count(cube.id) > 0)
    {
        collision_object.operation = collision_object.MOVE;
    }
    else
    {
        collision_object.operation = collision_object.ADD;
    }

    planning_scene_interface_->applyCollisionObject(collision_object);
}

void MultiActionServer::publish_ground_plane_marker(double height)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = move_group_->getPlanningFrame();
    marker.header.stamp = this->now();
    marker.ns = "ground_plane";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;
    marker.pose.position.z = height - 0.005;

    marker.scale.x = 20.0;
    marker.scale.y = 20.0;
    marker.scale.z = 0.01;

    marker.color.a = 0.5;
    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 0.8;

    marker_pub_->publish(marker);
    
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

// ============================== AddObject ======================================
void MultiActionServer::execute_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<AddObject>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing AddObject goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<AddObject::Result>();

    try
    {
        YAML::Node config = YAML::LoadFile(goal->config_file_path);
        std::vector<std::string> object_id_to_add;
        if(goal->object_names.empty())
        {
            // For now we only consider cubes as objects
            for (const auto & object : config["cubes"])
            {
                object_id_to_add.push_back(object.first.as<std::string>());
            }
        }
        else
        {
            object_id_to_add = goal->object_names; // object_names is a vector of strings
        }

        for (const auto& object_id : object_id_to_add)
        {
            const auto& object_config = config["cubes"][object_id];

            // For now we only consider cubes as objects
            CubeInfo cube;
            cube.id = object_id;
            cube.size_x = object_config["size_x"].as<double>();
            cube.size_y = object_config["size_y"].as<double>();
            cube.size_z = object_config["size_z"].as<double>();
            cube.pose.position.x = object_config["pose"]["position"]["x"].as<double>();
            cube.pose.position.y = object_config["pose"]["position"]["y"].as<double>();
            cube.pose.position.z = object_config["pose"]["position"]["z"].as<double>();
            cube.pose.orientation.x = object_config["pose"]["orientation"]["x"].as<double>();
            cube.pose.orientation.y = object_config["pose"]["orientation"]["y"].as<double>();
            cube.pose.orientation.z = object_config["pose"]["orientation"]["z"].as<double>();
            cube.pose.orientation.w = object_config["pose"]["orientation"]["w"].as<double>();

            add_cube_to_planning_scene(cube);
            cubes_[cube.id] = cube;

            result->success = true;
            result->message = "Objects added successfully";
            

        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_INFO(this->get_logger(), "Failed to add object: %s", e.what());
        result->success = false;
        result->message = e.what();
    }

    goal_handle->succeed(result);
}

// ============================== MoveIt =========================================
void MultiActionServer::execute_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveIt>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing MoveIt goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveIt::Feedback>();
    auto result = std::make_shared<MoveIt::Result>();

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

// ==============================AttachIt execution================================
void MultiActionServer::execute_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<AttachIt>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing AttachIt goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<AttachIt::Result>();

    try 
    {
        auto it = cubes_.find(goal->object_id);
        if (it == cubes_.end())
        {
            throw std::runtime_error("Object not found" + goal->object_id);
        }
        
        const CubeInfo & cube = it->second;

        
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

// ==============================DetachIt execution================================
void MultiActionServer::execute_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<DetachIt>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing DetachIt goal");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<DetachIt::Result>();

    try
    {
        
        move_group_->detachObject(goal->object_id);
        // TODO: update the planning scene should be callback function
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

// ============================== RemoveObject ====================================
void MultiActionServer::execute_goal(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<RemoveObject>> goal_handle
)
{
    RCLCPP_INFO(this->get_logger(), "Executing RemoveObject goal");
    auto result = std::make_shared<RemoveObject::Result>();

    //TODO: Optimize this part
    std::vector<std::string> object_ids;

    for(const auto&[cube_name, _] : cubes_)
    {
        object_ids.push_back(cube_name);
    }

    cubes_.clear();
    planning_scene_interface_->removeCollisionObjects(object_ids);
    result->success = true;
    result->message = "All objects removed successfully";
    goal_handle->succeed(result);
}

// Explicit instantiation handle_goal of AddObject, MoveIt, AttachIt, DetachIt, and RemoveObject
template rclcpp_action::GoalResponse MultiActionServer::handle_goal<MultiActionServer::AddObject>(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MultiActionServer::AddObject::Goal>);
template rclcpp_action::GoalResponse MultiActionServer::handle_goal<MultiActionServer::MoveIt>(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MultiActionServer::MoveIt::Goal>);
template rclcpp_action::GoalResponse MultiActionServer::handle_goal<MultiActionServer::AttachIt>(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MultiActionServer::AttachIt::Goal>);
template rclcpp_action::GoalResponse MultiActionServer::handle_goal<MultiActionServer::DetachIt>(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MultiActionServer::DetachIt::Goal>);
template rclcpp_action::GoalResponse MultiActionServer::handle_goal<MultiActionServer::RemoveObject>(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const MultiActionServer::RemoveObject::Goal>);

// Explicit instantiation handle_cancel of AddObject, MoveIt, AttachIt, DetachIt, and RemoveObject
template rclcpp_action::CancelResponse MultiActionServer::handle_cancel<MultiActionServer::AddObject>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::AddObject>>);
template rclcpp_action::CancelResponse MultiActionServer::handle_cancel<MultiActionServer::MoveIt>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::MoveIt>>);
template rclcpp_action::CancelResponse MultiActionServer::handle_cancel<MultiActionServer::AttachIt>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::AttachIt>>);
template rclcpp_action::CancelResponse MultiActionServer::handle_cancel<MultiActionServer::DetachIt>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::DetachIt>>);
template rclcpp_action::CancelResponse MultiActionServer::handle_cancel<MultiActionServer::RemoveObject>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::RemoveObject>>);

// Explicit instantiation handle_accepted of AddObject, MoveIt, AttachIt, DetachIt, and RemoveObject
template void MultiActionServer::handle_accepted<MultiActionServer::AddObject>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::AddObject>>);
template void MultiActionServer::handle_accepted<MultiActionServer::MoveIt>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::MoveIt>>);
template void MultiActionServer::handle_accepted<MultiActionServer::AttachIt>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::AttachIt>>);
template void MultiActionServer::handle_accepted<MultiActionServer::DetachIt>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::DetachIt>>);
template void MultiActionServer::handle_accepted<MultiActionServer::RemoveObject>(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<MultiActionServer::RemoveObject>>);



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