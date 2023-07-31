#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.h"
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

namespace execution_policy_server
{
    using Dock = irobot_create_msgs::action::Dock;
    using GoalHandleDock = rclcpp_action::ClientGoalHandle<Dock>;
    using Undock = irobot_create_msgs::action::Undock;
    using GoalHandleUndock = rclcpp_action::ClientGoalHandle<Undock>;
    using Pose = geometry_msgs::msg::PoseStamped;
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    class TurtleBot4Navigator : public rclcpp::Node
    {

    public:
        TurtleBot4Navigator(
            std::string name,
            std::string robot_namespace,
            const rclcpp::NodeOptions &options);

        void dock();

        void undock();

        bool is_docked();

        void follow_waypoints(std::vector<Pose> poses);

        Pose get_pose_stamped(float x, float y);

    private:
        std::string robot_namespace_;
        bool is_docked_;
        rclcpp_action::Client<Dock>::SharedPtr dock_client_ptr_;
        rclcpp_action::Client<Undock>::SharedPtr undock_client_ptr_;
        rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_ptr_;

        // Dock Command
        //====================================================================================
        void dock_send_goal();

        void dock_goal_response_callback(const GoalHandleDock::SharedPtr & goal_handle);

        void dock_feedback_callback(
            GoalHandleDock::SharedPtr,
            const std::shared_ptr<const Dock::Feedback> feedback);

        void dock_result_callback(const GoalHandleDock::WrappedResult &result);

        // Undock Command
        //====================================================================================
        void undock_send_goal();

        void undock_goal_response_callback(const GoalHandleUndock::SharedPtr & goal_handle);

        void undock_feedback_callback(
            GoalHandleUndock::SharedPtr,
            const std::shared_ptr<const Undock::Feedback> feedback);

        void undock_result_callback(const GoalHandleUndock::WrappedResult &result);

        // Follow Waypoints Command
        //====================================================================================
        void follow_waypoints_send_goal(std::vector<Pose> poses);

        void follow_waypoints_goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr & goal_handle);

        void follow_waypoints_feedback_callback(
            GoalHandleFollowWaypoints::SharedPtr,
            const std::shared_ptr<const FollowWaypoints::Feedback> feedback);

        void follow_waypoints_result_callback(const GoalHandleFollowWaypoints::WrappedResult &result);
    };

}
