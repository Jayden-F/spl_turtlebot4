#include "geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class PositionPublisher : public rclcpp::Node {

public:
  using PointStamped = geometry_msgs::msg::PointStamped;
  using PoseWithCovarianceStamped =
      geometry_msgs::msg::PoseWithCovarianceStamped;

  PositionPublisher(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("position_publisher", options) {

    RCLCPP_INFO(this->get_logger(), "Starting position publisher node.");

    publisher_ =
        this->create_publisher<PoseWithCovarianceStamped>("/initialpose", 1);

    subscriber_ = this->create_subscription<PointStamped>(
        "/clicked_point", 1,
        std::bind(&PositionPublisher::callback, this, std::placeholders::_1));
  }

  void callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(),
                "Recieved Data:\n X : %f \n Y : %f \n Z : %f", msg->point.x,
                msg->point.y, msg->point.z);

    publish(msg->point.x, msg->point.y);
  }

  void publish(float x, float y) {
    auto msg = PoseWithCovarianceStamped();
    msg.header.frame_id = "/map";
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;

    RCLCPP_INFO(this->get_logger(),
                "Publishing Initial Position \n X: %f \n Y: %f",
                msg.pose.pose.position.x, msg.pose.pose.position.y);

    publisher_->publish(msg);
  }

private:
  rclcpp::Subscription<PointStamped>::SharedPtr subscriber_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionPublisher>());
  rclcpp::shutdown();
  return 0;
}