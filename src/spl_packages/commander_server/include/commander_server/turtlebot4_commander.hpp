#ifndef TURTLEBOT4_COMMANDER_HPP
#define TURTLEBOT4_COMMANDER_HPP

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/beast.hpp>
#include <boost/beast/core/error.hpp>
#include <boost/beast/http/message.hpp>
#include <boost/program_options.hpp>
#include <cstdint>
#include <map>
#include <math.h>
#include <nlohmann/json.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <string>

namespace commander_server {

#define PI 3.14159265

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

private:
  // robot state variables
  bool is_executing_;
  bool started_;
  int32_t end_timestep_;
  uint32_t current_progress_;
  PoseStamped pose_;
  std::vector<PoseStamped> poses_;

  // ros2 callbacks
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_ptr_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr
      pose_subscriber_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  // central controller variables
  const std::string ip_;
  const uint16_t port_;
  const uint32_t id_;

  boost::asio::io_context ioc_;
  boost::beast::flat_buffer buffer_;
  boost::beast::tcp_stream stream_;

  void reset_state();

  void connect_central_controller();

  std::string make_request(boost::beast::http::verb verb, std::string target,
                           nlohmann::json body);

  nlohmann::json json_post_format(PoseStamped pose, std::string status, uint32_t timestep);

  void request_next_poses();

  void navigate_to_pose();

  inline PoseStamped get_pose_stamped(float x, float y, float theta);

  void navigate_to_pose_send_goal(std::vector<PoseStamped> &poses);

  void navigate_to_pose_goal_response_callback(
      const GoalHandleNavigateToPose::SharedPtr goal_handle);

  void navigate_to_pose_feedback_callback(
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  void navigate_to_pose_result_callback(
      const GoalHandleNavigateToPose::WrappedResult &result);

  void pose_topic_callback(const PoseWithCovarianceStamped::SharedPtr msg);

  void polling_callback();

  bool check_at_position(PoseStamped current, PoseStamped target, float threshold_metres = 0.3);

};
} // namespace commander_server
#endif
