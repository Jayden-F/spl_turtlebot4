#include "geometry_msgs/msg/pose_stamped.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "nav2_msgs/action/navigate_to_pose.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <boost/asio.hpp>
#include <boost/asio/ip/address.hpp>
#include <nlohmann/json.hpp>
#include <string>

namespace commander_server {

using json = nlohmann::json;
using Pose = geometry_msgs::msg::PoseStamped;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;

class turtlebot4_commander : public rclcpp::Node {

public:
  turtlebot4_commander(
      int robot_id, const std::string &ip, const std::string &port,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  void get_request();

  void post_request(Pose pose, std::string status);

  void navigate_to_pose(Pose pose);

  Pose get_pose_stamped(float x, float y);

private:
  const std::string ip_;
  const std::string port_;
  int id_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_ptr_;
  ip::tcp::socket sock_;

  // Follow Waypoints Command
  //====================================================================================
  void navigate_to_pose_send_goal(Pose pose);

  void navigate_to_pose_goal_response_callback(
      const GoalHandleNavigateToPose::SharedPtr goal_handle);

  void navigate_to_pose_feedback_callback(
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  void navigate_to_pose_result_callback(
      const GoalHandleNavigateToPose::WrappedResult &result);
};

} // namespace commander_server
