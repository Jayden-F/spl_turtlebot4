#include "commander_server/turtlebot4_commander.hpp"

commander_server::turtlebot4_commander::turtlebot4_commander(
    int robot_id, const std::string &ip, const std::string &port,
    const rclcpp::NodeOptions &options)
    : Node("Turtlebot4_Commander", options), ip_(ip), port_(port),
      id_(robot_id) {
  RCLCPP_INFO(this->get_logger(),
              "Starting commander_server::turtlebot4_commander");

  this->navigate_to_pose_client_ptr_ =
      rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
}

void commander_server::turtlebot4_commander::navigate_to_pose(Pose pose) {
  RCLCPP_INFO(this->get_logger(), "Sending Waypoints to Robot");

  this->navigate_to_pose_send_goal(pose);
}

commander_server::Pose
commander_server::turtlebot4_commander::get_pose_stamped(float x, float y) {
  Pose pose = Pose();
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  return pose;
}

// void commander_server::turtlebot4_commander::turtlebot4_status_callback(
// const Turtlebot4Status::SharedPtr msg) {
// auto battery = msg->status[0].values[0].value;
// RCLCPP_INFO(this->get_logger(), "%s", battery.c_str());
//// TODO
//// Get useful robot state information.
//}

void commander_server::turtlebot4_commander::navigate_to_pose_send_goal(
    Pose pose) {

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = pose;

  this->navigate_to_pose_client_ptr_->wait_for_action_server();

  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&commander_server::turtlebot4_commander::
                    navigate_to_pose_goal_response_callback,
                this, std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&commander_server::turtlebot4_commander::
                    navigate_to_pose_feedback_callback,
                this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(
      &commander_server::turtlebot4_commander::navigate_to_pose_result_callback,
      this, std::placeholders::_1);

  this->navigate_to_pose_client_ptr_->async_send_goal(goal_msg,
                                                      send_goal_options);
}

void commander_server::turtlebot4_commander::
    navigate_to_pose_goal_response_callback(
        const GoalHandleNavigateToPose::SharedPtr goal_handle) {

  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void commander_server::turtlebot4_commander::navigate_to_pose_feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
  // RCLCPP_INFO(this->get_logger(), "Waypoint Number: %d",
  // feedback->current_pose;
}

void commander_server::turtlebot4_commander::navigate_to_pose_result_callback(
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

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<commander_server::turtlebot4_commander>(
      0, "192.168.0.204", "8080");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
