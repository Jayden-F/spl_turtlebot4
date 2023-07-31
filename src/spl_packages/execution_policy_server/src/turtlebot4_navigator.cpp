#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "execution_policy_server/turtlebot4_navigator.hpp"

namespace execution_policy_server
{
    
    TurtleBot4Navigator::TurtleBot4Navigator(
        std::string name,
        std::string robot_namespace,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node(name, options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting TurtleBot4Navigator");

        this->robot_namespace_ = robot_namespace;
        this->dock_client_ptr_ = rclcpp_action::create_client<Dock>(this, this->robot_namespace_ + "dock");
        this->undock_client_ptr_ = rclcpp_action::create_client<Undock>(this, this->robot_namespace_ + "undock");
        this->follow_waypoints_client_ptr_ = rclcpp_action::create_client<FollowWaypoints>(this, this->robot_namespace_ + "follow_waypoints");
    }

    void TurtleBot4Navigator::dock()
    {
        RCLCPP_INFO(this->get_logger(), "Docking Robot");

        this->dock_send_goal();
    }

    void TurtleBot4Navigator::undock()
    {
        RCLCPP_INFO(this->get_logger(), "Undocking Robot");

        this->undock_send_goal();
    }

    bool TurtleBot4Navigator::is_docked()
    {
        return this->is_docked_;
    }

    void TurtleBot4Navigator::follow_waypoints(std::vector<Pose> poses)
    {
        RCLCPP_INFO(this->get_logger(), "Sending Waypoints to Robot");

        this->follow_waypoints_send_goal(poses);
    }

    Pose TurtleBot4Navigator::get_pose_stamped(float x, float y)
    {
        Pose pose = Pose();
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        return pose;
    }

    void TurtleBot4Navigator::dock_send_goal()
    {
        using namespace std::placeholders;

        auto goal_msg = Dock::Goal();
        this->dock_client_ptr_->wait_for_action_server();

        auto send_goal_options = rclcpp_action::Client<Dock>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&TurtleBot4Navigator::dock_goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&TurtleBot4Navigator::dock_feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&TurtleBot4Navigator::dock_result_callback, this, _1);

        this->dock_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void TurtleBot4Navigator::dock_goal_response_callback(const GoalHandleDock::SharedPtr & goal_handle)
    {

        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void TurtleBot4Navigator::dock_feedback_callback(
        GoalHandleDock::SharedPtr,
        const std::shared_ptr<const Dock::Feedback> feedback)
    {
        if (feedback->sees_dock)
        {
            RCLCPP_INFO(this->get_logger(), "Located dock");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Looking for dock");
        }
    }

    void TurtleBot4Navigator::dock_result_callback(const GoalHandleDock::WrappedResult &result)
    {

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal was successful");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
    }

    void TurtleBot4Navigator::undock_send_goal()
    {
        auto goal_msg = Undock::Goal();
        this->undock_client_ptr_->wait_for_action_server();

        auto send_goal_options = rclcpp_action::Client<Undock>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&TurtleBot4Navigator::undock_goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&TurtleBot4Navigator::undock_result_callback, this, std::placeholders::_1);

        this->undock_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void TurtleBot4Navigator::undock_goal_response_callback(const GoalHandleUndock::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void TurtleBot4Navigator::undock_result_callback(const GoalHandleUndock::WrappedResult &result)
    {

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal was successful");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
    }

    void TurtleBot4Navigator::follow_waypoints_send_goal(std::vector<Pose> poses)
    {

        auto goal_msg = FollowWaypoints::Goal();
        goal_msg.poses = poses;

        this->follow_waypoints_client_ptr_->wait_for_action_server();

        auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&TurtleBot4Navigator::follow_waypoints_goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&TurtleBot4Navigator::follow_waypoints_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&TurtleBot4Navigator::follow_waypoints_result_callback, this, std::placeholders::_1);

        this->follow_waypoints_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    void TurtleBot4Navigator::follow_waypoints_goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr & goal_handle)
    {

        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void TurtleBot4Navigator::follow_waypoints_feedback_callback(GoalHandleFollowWaypoints::SharedPtr,
                                                                 const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Waypoint Number: %d", feedback->current_waypoint);
    }

    void TurtleBot4Navigator::follow_waypoints_result_callback(const GoalHandleFollowWaypoints::WrappedResult &result)
    {

        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal was successful");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
    }

}

using namespace execution_policy_server;

int main(int argc, char **argv)
{
    rclcpp::InitOptions init_options = rclcpp::InitOptions();
    init_options.set_domain_id(1);

    rclcpp::init(argc, argv, init_options);

    auto walle = std::make_shared<TurtleBot4Navigator>("Turtlebot4Navigator", "/walle/");

    // tina->undock();
    // walle->undock();

    rclcpp::sleep_for(std::chrono::seconds(10));

    std::vector<Pose> poses = std::vector<Pose>();
    poses.push_back(walle->get_pose_stamped(-1.0, 0.0));
    poses.push_back(walle->get_pose_stamped(-1.0, 1.0));
    poses.push_back(walle->get_pose_stamped(-1.0, 2.0));
    poses.push_back(walle->get_pose_stamped(-1.0, 3.0));

    // tina->follow_waypoints(poses);
    walle->follow_waypoints(poses);

    rclcpp::spin(walle);
    rclcpp::shutdown();
    return 0;
}
