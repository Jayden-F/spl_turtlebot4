#include "turtlebot4_navigator.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace commander_server {

TurtleBot4Navigator::TurtleBot4Navigator(
    int robot_id, const std::string &ip, const std::string &port)

    const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node(name, options) {
  RCLCPP_INFO(this->get_logger(), "Starting TurtleBot4Navigator");

  this->navigate_to_pose_client_ptr_ =
      rclcpp_action::create_client<NavigateToPose>(this, );
}

void TurtleBot4Navigator::navigate_to_pose(Pose pose) {
  RCLCPP_INFO(this->get_logger(), "Sending Waypoints to Robot");

  this->navigate_to_pose_send_goal(pose);
}

Pose TurtleBot4Navigator::get_pose_stamped(float x, float y) {
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

void TurtleBot4Navigator::navigate_to_pose_send_goal(Pose pose) {

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = pose;

  this->navigate_to_pose_client_ptr_->wait_for_action_server();

  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&TurtleBot4Navigator::navigate_to_pose_goal_response_callback,
                this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&TurtleBot4Navigator::navigate_to_pose_feedback_callback, this,
                std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
      std::bind(&TurtleBot4Navigator::navigate_to_pose_result_callback, this,
                std::placeholders::_1);

  this->navigate_to_pose_client_ptr_->async_send_goal(goal_msg,
                                                      send_goal_options);
}

void TurtleBot4Navigator::navigate_to_pose_goal_response_callback(
    const GoalHandleNavigateToPose::SharedPtr &goal_handle) {

  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void TurtleBot4Navigator::navigate_to_pose_feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
  //RCLCPP_INFO(this->get_logger(), "Waypoint Number: %d",
              //feedback->current_waypoint);
}

void TurtleBot4Navigator::navigate_to_pose_result_callback(
    const GoalHandleNavigateToPose::WrappedResult &result) {

  switch (result.code) {
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

} // namespace commander_server

namespace po = boost::program_options;

int main(int argc, char **argv) {
  rclcpp::InitOptions init_options = rclcpp::InitOptions();

  rclcpp::init(argc, argv, init_options);

  auto node =
      std::make_shared<TurtleBot4Navigator>("Turtlebot4Navigator", "/walle/");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
