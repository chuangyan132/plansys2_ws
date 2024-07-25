#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include "moveit_action_interfaces/action/move_to_pose.hpp"



class MoveItActionServer : public rclcpp::Node
{
public:

    using MoveToPose = moveit_action_interfaces::action::MoveToPose;
    using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

    MoveItActionServer() : Node("moveit_action_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<MoveToPose>(
            this,
            "move_to_pose",
            std::bind(&MoveItActionServer::handle_goal, this, _1, _2),
            std::bind(&MoveItActionServer::handle_cancel, this, _1),
            std::bind(&MoveItActionServer::handle_accepted, this, _1)
            );
    }

private:
    rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;


    // Callback function for handling a goal
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveToPose::Goal> goal
    )
    {
        const auto& pose = goal->target_pose;
        RCLCPP_INFO(this->get_logger(), "Received Goal: Position: [x: %f, y: %f, z: %f], Orientation: [x: %f, y: %f, z: %f, w: %f]",
        pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        );
        (void)uuid; // This is to suppress unused variable warning
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // Callback function for handling a cancel request
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveToPose> goal_handle
    )
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    // Callback function for handling an accepted goal
    void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        /*
        This needs to return quickly to avoid blocking the executor, so spin up a new thread
        */

       std::thread{std::bind(&MoveItActionServer::execute, this, goal_handle), goal_handle}.detach();
    }



    // Callback function for executing a goal
    void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto goal = goal_handle-> get_goal();
        auto grasp = *goal;
        auto task = *goal;
        auto feedback = std::make_shared<MoveToPose::Feedback>();
        auto result = std::make_shared<MoveToPose::Result>();

        if(!move_group_interface_)
        {
            move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "panda_arm");
        }

        // Get the current pose
        geometry_msgs::msg::PoseStamped current_pose = move_group_interface_->getCurrentPose();
        RCLCPP_INFO(this->get_logger(), "Current position: [x: %f, y: %f, z: %f]",
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z);


        moveit::planning_interface::MoveGroupInterface::Plan plan_left;
        moveit::planning_interface::MoveGroupInterface::Plan plan_grasp;
        moveit::planning_interface::MoveGroupInterface::Plan plan_target;
        // Set the left pose
        current_pose.pose.position.z += 0.2;
        move_group_interface_->setPoseTarget(current_pose.pose);
        bool success_left = static_cast<bool>(move_group_interface_->plan(plan_left));

        if (success_left)
        {
            auto exec_left_result = move_group_interface_->execute(plan_left);
            if(exec_left_result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "1st left moving succeeded");
                grasp.target_pose.position.z += 0.2;
                move_group_interface_->setPoseTarget(grasp.target_pose);

                bool success_grasp = static_cast<bool>(move_group_interface_->plan(plan_grasp));
                if (success_grasp)
                {
                    auto exec_grasp_result = move_group_interface_->execute(plan_grasp);
                    if(exec_grasp_result == moveit::core::MoveItErrorCode::SUCCESS)
                    {
                        RCLCPP_INFO(this->get_logger(), "2nd Grasp moving succeeded");
                        move_group_interface_->setPoseTarget(task.target_pose);
                        bool success_target = static_cast<bool>(move_group_interface_->plan(plan_target));
                        if(success_target)
                        {
                            auto exec_target_result = move_group_interface_->execute(plan_target);
                            if(exec_target_result == moveit::core::MoveItErrorCode::SUCCESS)
                            {
                                result->success = true;
                                result->message = "Move completed successfully";
                                goal_handle->succeed(result);
                                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                            }
                            else
                            {
                                result->success = false;
                                result->message = "3rd Target move failed";
                                goal_handle->abort(result);
                                RCLCPP_ERROR(this->get_logger(), "3rd Target move failed");
                            }
                        }
                        else
                        {
                            result->success = false;
                            result->message = "3rd Target plan failed";
                            goal_handle->abort(result);
                            RCLCPP_ERROR(this->get_logger(), "3rd Target plan failed");
                        }

                    }
                    else
                    {
                        result->success = false;
                        result->message = "2nd Grasp move failed";
                        goal_handle->abort(result);
                        RCLCPP_ERROR(this->get_logger(), "2nd Grasp move failed");
                    }
                }
                else
                {
                    result->success = false;
                    result->message = "2nd Grasp plan failed";
                    goal_handle->abort(result);
                    RCLCPP_ERROR(this->get_logger(), "2nd Grasp plan failed");
                }
            } 
            else 
            {
                result->success = false;
                result->message = "1st left moving failed";
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "1st left moving failed");
            }
        } 
        else 
        {
            result->success = false;
            result->message = "1st Plan failed";
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "1st Plan failed");

        }

        // grasp.target_pose.position.z += 0.2;

        // move_group_interface_->setPoseTarget(grasp.target_pose);

        

        // // static_cast<bool> is used to convert to bool
        // bool success = static_cast<bool>(move_group_interface_->plan(plan));

        // if(success)
        // {
        //     auto exec_result = move_group_interface_->execute(plan);

        //     if(exec_result == moveit::core::MoveItErrorCode::SUCCESS)
        //     {
        //         result->success = true;
        //         result->message = "Move completed successfully";
        //         goal_handle->succeed(result);
        //         RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        //     } else {
        //         result->success = false;
        //         result->message = "Move execution failed";
        //         goal_handle->abort(result);
        //         RCLCPP_ERROR(this->get_logger(), "Move execution failed");
        //     }
        // } else {
        //     result->success = false;
        //     result->message = "Planning failed";
        //     goal_handle->abort(result);
        //     RCLCPP_ERROR(this->get_logger(), "Planning failed");
        // }

        

    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveItActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}