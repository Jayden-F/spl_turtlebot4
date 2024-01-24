#ifndef TURTLEBOT4_COMMANDER_HPP
#define TURTLEBOT4_COMMANDER_HPP

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/beast.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/program_options.hpp>
#include <cstdint>
#include <map>
#include <math.h>
#include <nlohmann/json.hpp>
#include <string>

namespace commander_server {

#define PI 3.14159265

using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavigateThroughPoses =
    rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class turtlebot4_commander : public rclcpp::Node {

public:
  turtlebot4_commander(
      const uint32_t id, const std::string &ip, const uint16_t port,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  const std::string ip_;
  const uint16_t port_;
  const uint32_t id_;
  Pose pose_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr
      navigate_to_pose_client_ptr_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr
      pose_subscriber_ptr_;
  boost::asio::io_context ioc_;
  boost::beast::flat_buffer buffer_;
  boost::beast::tcp_stream stream_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool is_executing_;
  uint32_t num_poses_;

  void reset_state();

  void connect_central_controller();

  std::string make_request(boost::beast::http::verb verb, std::string target,
                           nlohmann::json body);

  nlohmann::json json_post_format(Pose pose, std::string status,
                                  uint32_t pose_number);

  std::vector<PoseStamped> get_request();

  void navigate_to_pose(std::vector<PoseStamped> poses);

  PoseStamped get_pose_stamped(float x, float y, float theta);

  void navigate_to_pose_send_goal(std::vector<PoseStamped> poses);

  void navigate_to_pose_goal_response_callback(
      const GoalHandleNavigateThroughPoses::SharedPtr goal_handle);

  void navigate_to_pose_feedback_callback(
      GoalHandleNavigateThroughPoses::SharedPtr,
      const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);

  void navigate_to_pose_result_callback(
      const GoalHandleNavigateThroughPoses::WrappedResult &result);

  void pose_topic_callback(const PoseWithCovarianceStamped::SharedPtr msg);

  void polling_callback();
};
} // namespace commander_server
#endif
