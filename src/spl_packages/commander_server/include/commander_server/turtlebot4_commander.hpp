#ifndef TURTLEBOT4_COMMANDER_HPP
#define TURTLEBOT4_COMMANDER_HPP

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "nav2_msgs/action/navigate_to_pose.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <boost/asio.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/program_options.hpp>
#include <cstdint>
#include <map>
#include <math.h>
#include <nlohmann/json.hpp>
#include <string>

namespace commander_server {

#define PI 3.14159265

using json = nlohmann::json;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;

class turtlebot4_commander : public rclcpp::Node {

public:
  turtlebot4_commander(
      const uint32_t id, const std::string &ip, const uint16_t port,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  PoseStamped get_request();

  void post_request(std::string path, json payload);

  void navigate_to_pose(PoseStamped pose);

  PoseStamped get_pose_stamped(float x, float y, float theta);

  json json_post_format(Pose pose, std::string status);

private:
  const std::string ip_;
  const uint16_t port_;
  const uint32_t id_;
  Pose pose_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_ptr_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr
      pose_subscriber_ptr_;
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool is_executing_;

  void navigate_to_pose_send_goal(PoseStamped pose);

  void navigate_to_pose_goal_response_callback(
      const GoalHandleNavigateToPose::SharedPtr goal_handle);

  void navigate_to_pose_feedback_callback(
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  void navigate_to_pose_result_callback(
      const GoalHandleNavigateToPose::WrappedResult &result);

  void pose_topic_callback(const PoseWithCovarianceStamped::SharedPtr msg);

  void polling_callback();
};

} // namespace commander_server
#endif
