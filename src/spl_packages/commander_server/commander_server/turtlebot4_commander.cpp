#include "commander_server/turtlebot4_commander.hpp"

commander_server::turtlebot4_commander::turtlebot4_commander(
    const uint32_t id, const std::string &ip, const uint16_t port,
    const rclcpp::NodeOptions &options)
    : Node("Turtlebot4_Commander", options), ip_(ip), port_(port), id_(id),
      socket_(io_service_), is_executing_(0), pose_() {
  RCLCPP_INFO(
      this->get_logger(),
      "Starting turtlebot4_commander\n Agent ID: %d\n IP: %s\n Port: %d", id_,
      ip_.c_str(), port_);

  navigate_to_pose_client_ptr_ =
      rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

  pose_subscriber_ptr_ = create_subscription<PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&commander_server::turtlebot4_commander::pose_topic_callback,
                this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&commander_server::turtlebot4_commander::polling_callback,
                this));

  // Connect to the central controller
  while (true) {
    try {
      socket_.connect(boost::asio::ip::tcp::endpoint(
          boost::asio::ip::address::from_string(ip_), port_));
      break;
    } catch (boost::system::system_error &e) {
      RCLCPP_WARN(this->get_logger(), "Error: %s", e.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }
}

commander_server::PoseStamped
commander_server::turtlebot4_commander::get_request() {
  boost::system::error_code ec;

  // no need to build a string everytime should be const but lazy
  const std::string request("GET /?agent_id=" + std::to_string(id_) +
                            " HTTP/1.1\r\n\r\n");
  socket_.send(boost::asio::buffer(request));

  std::string response;
  do {
    char buf[1024];
    size_t bytes_transferred =
        socket_.receive(boost::asio::buffer(buf), {}, ec);
    if (!ec) {
      response.append(buf, buf + bytes_transferred);
    }
  } while (!ec);

  if (response.empty()) {
    RCLCPP_WARN(this->get_logger(), "Response was empty");
  }

  uint64_t split = response.rfind('\n', response.length());
  std::string data = response.substr(split, response.length() - split);
  json json_received = json::parse(data);

  // read the position from the json in format [x, y, theta]
  std::vector<json> position = json_received["position"];
  float x = position[0].get<float>();
  float y = position[1].get<float>();
  int theta = position[2].get<int>();

  return get_pose_stamped(x, y, theta);
}

void commander_server::turtlebot4_commander::post_request(Pose pose,
                                                          std::string status) {
  boost::system::error_code ec;
  json payload = json::object();
  payload["agent_id"] = id_;
  payload["status"] = status;

  std::string serial_payload = payload.dump();
  std::string request(
      "POST / HTTP/1.1\r\n Host: \r\n " + std::to_string(id_) + " \r\n" +
      "Connection: close\r\n Accept: */*\r\n User-Agent: " +
      std::to_string(id_) +
      "\r\n Content-Type: applications/json\r\n Content-Length: " +
      std::to_string(serial_payload.length()) + "\r\n" + "\r\n" +
      serial_payload);
  socket_.send(boost::asio::buffer(request));

  std::string response;
  do {
    char buf[1024];
    size_t bytes_transferred =
        socket_.receive(boost::asio::buffer(buf), {}, ec);
    if (!ec) {
      response.append(buf, buf + bytes_transferred);
    }
  } while (!ec);

  if (response.empty()) {
    RCLCPP_WARN(this->get_logger(), "Response was empty");
  }

  RCLCPP_INFO(this->get_logger(), "Response: %s", response.c_str());
}

void commander_server::turtlebot4_commander::navigate_to_pose(
    PoseStamped pose) {
  RCLCPP_INFO(this->get_logger(), "Sending Waypoints to Robot");

  this->navigate_to_pose_send_goal(pose);
}

commander_server::PoseStamped
commander_server::turtlebot4_commander::get_pose_stamped(float x, float y,
                                                         float theta) {

  PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->get_clock()->now();

  pose.pose.position.x = x;
  pose.pose.position.y = y;

  float rads = theta * (PI / 180);
  pose.pose.orientation.z = sin(rads / 2);
  pose.pose.orientation.w = cos(rads / 2);

  return pose;
}

void commander_server::turtlebot4_commander::navigate_to_pose_send_goal(
    PoseStamped pose) {

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = pose;

  navigate_to_pose_client_ptr_->wait_for_action_server();

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

  navigate_to_pose_client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void commander_server::turtlebot4_commander::
    navigate_to_pose_goal_response_callback(
        const GoalHandleNavigateToPose::SharedPtr goal_handle) {

  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    is_executing_ = false;
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void commander_server::turtlebot4_commander::navigate_to_pose_feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {

  pose_ = feedback->current_pose.pose;
  RCLCPP_INFO(this->get_logger(), "Received feedback: %f, %f, %f",
              feedback->current_pose.pose.position.x,
              feedback->current_pose.pose.position.y,
              feedback->current_pose.pose.orientation.z);
  post_request(feedback->current_pose.pose, "Executing");
}

void commander_server::turtlebot4_commander::navigate_to_pose_result_callback(
    const GoalHandleNavigateToPose::WrappedResult &result) {

  std::map<rclcpp_action::ResultCode, std::string> status{
      {rclcpp_action::ResultCode::SUCCEEDED, "success"},
      {rclcpp_action::ResultCode::ABORTED, "aborted"},
      {rclcpp_action::ResultCode::CANCELED, "cancelled"},
      {rclcpp_action::ResultCode::UNKNOWN, "unknown"},
  };

  RCLCPP_INFO(this->get_logger(), "Goal Status: %s",
              status[result.code].c_str());
  post_request(pose_, status[result.code]);
  is_executing_ = false;
}

void commander_server::turtlebot4_commander::pose_topic_callback(
    const PoseWithCovarianceStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Pose: %f, %f, %f", msg->pose.pose.position.x,
              msg->pose.pose.position.y, msg->pose.pose.orientation.z);

  Pose pose = msg->pose.pose;
  post_request(pose, "Executing");
  pose_subscriber_ptr_.reset();
}

void commander_server::turtlebot4_commander::polling_callback() {

  if (is_executing_) {
    return;
  };

  is_executing_ = true;
  PoseStamped pose = get_request();
  navigate_to_pose(pose);
}
namespace po = boost::program_options;

int main(int argc, char **argv) {

  std::string address;
  uint16_t port;
  uint32_t id;

  rclcpp::init(argc, argv);

  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")(
      "ip", po::value<std::string>(&address)->default_value("192.168.0.204"),
      "set ip address")("port", po::value<uint16_t>(&port)->default_value(8080),
                        "set port number")(
      "id", po::value<uint32_t>(&id)->default_value(0), "set agent id");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  auto node = std::make_shared<commander_server::turtlebot4_commander>(
      id, address, port);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
