#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "nav2_msgs/action/navigate_to_pose.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <boost/asio.hpp>
#include <boost/asio/ip/address.hpp>
#include <cstdint>
#include <math.h>
#include <nlohmann/json.hpp>
#include <string>

namespace commander_server {

#define PI 3.14159265

using json = nlohmann::json;
using Pose = geometry_msgs::msg::PoseStamped;
using PoseCovaraince = geometry_msgs::msg::PoseWithCovariance;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose =
    rclcpp_action::ClientGoalHandle<NavigateToPose>;

class turtlebot4_commander : public rclcpp::Node {

public:
  turtlebot4_commander(
      uint32_t id, const std::string &ip, const uint16_t port,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  Pose get_request();

  void post_request(Pose pose, std::string status);

  void navigate_to_pose(Pose pose);

  Pose get_pose_stamped(float x, float y, float theta);

private:
  const std::string ip_;
  const uint16_t port_;
  const uint32_t id_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_ptr_;
  rclcpp::Subscription<PoseWithCovariance>::SharedPtr pose_subscriber_ptr_;
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;

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

  void pose_topic_callback(const PoseWithCovariance::SharedPtr msg) const;
};

} // namespace commander_server
