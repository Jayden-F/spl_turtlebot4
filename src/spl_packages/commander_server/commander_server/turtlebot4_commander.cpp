#include "commander_server/turtlebot4_commander.hpp"
#include <boost/beast/core/error.hpp>
#include <rclcpp/logging.hpp>

commander_server::turtlebot4_commander::turtlebot4_commander(
    const uint32_t id, const std::string &ip, const uint16_t port,
    const rclcpp::NodeOptions &options)
    : Node("Turtlebot4_Commander", options), ip_(ip), port_(port), id_(id),
      pose_(), stream_(ioc_, boost::asio::ip::tcp::v4()), is_executing_(1),
      num_poses_(0) {

  RCLCPP_INFO(
      this->get_logger(),
      "Starting turtlebot4_commander\n Agent ID: %d\n IP: %s\n Port: %d", id_,
      ip_.c_str(), port_);
  navigate_to_pose_client_ptr_ =
      rclcpp_action::create_client<NavigateThroughPoses>(this,
                                                         "/navigate_to_pose");
  pose_subscriber_ptr_ = create_subscription<PoseWithCovarianceStamped>(
      "/amcl_pose", 10,
      std::bind(&commander_server::turtlebot4_commander::pose_topic_callback,
                this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&commander_server::turtlebot4_commander::polling_callback,
                this));
}

void commander_server::turtlebot4_commander::connect_central_controller() {

  // Connect to the central controller
  RCLCPP_INFO(this->get_logger(), "Connecting to %s:%d", ip_.c_str(), port_);
  boost::asio::ip::tcp::resolver r(ioc_);
  auto const results = r.resolve(ip_, std::to_string(port_));
  stream_.connect(results);
  RCLCPP_INFO(this->get_logger(), "Connected to %s:%d", ip_.c_str(), port_);
}

void commander_server::turtlebot4_commander::reset_state() {
  is_executing_ = false;
  num_poses_ = 0;
}

std::string commander_server::turtlebot4_commander::make_request(
    boost::beast::http::verb verb, std::string target, nlohmann::json body) {
  namespace http = boost::beast::http;
  boost::beast::error_code ec;

  http::request<http::string_body> req{verb, target, 11};
  req.set(http::field::host, ip_);
  req.set(http::field::user_agent, std::to_string(id_));
  req.set(http::field::content_type, "application/json");
  req.set(http::field::connection, "keep-alive"); // Keep the connection alive
  req.body() = body.dump();
  req.prepare_payload();

  connect_central_controller();

  http::write(stream_, req, ec);
  RCLCPP_INFO(this->get_logger(), ec.message().c_str());

  http::response<http::string_body> res;

  // Receive the HTTP response
  RCLCPP_INFO(this->get_logger(), "Reading Response");
  http::read(stream_, buffer_, res, ec);
  RCLCPP_INFO(this->get_logger(), ec.message().c_str());

  stream_.socket().close();
  RCLCPP_INFO(this->get_logger(), "Converting to string");
  // std::string data = boost::beast::buffers_to_string(res.body());
  RCLCPP_INFO(this->get_logger(), "%s", res.body().c_str());

  return res.body();
}

std::vector<commander_server::PoseStamped>
commander_server::turtlebot4_commander::get_request() {

  RCLCPP_INFO(this->get_logger(),
              "Requesting positions from central controller");

  nlohmann::json json_pose = nlohmann::json::object();
  json_pose["agent_id"] = id_;

  std::string data =
      make_request(boost::beast::http::verb::get, "/", json_pose);

  nlohmann::json json_received = nlohmann::json::parse(data);

  // read the position from the json in format [x, y, theta]
  std::vector<nlohmann::json> positions = json_received["positions"];

  std::cout << positions << ',' << positions << positions.size() << std::endl;

  std::vector<PoseStamped> poses(positions.size());

  for (std::vector<nlohmann::json> pose : positions) {
    float x = pose[0].get<float>();
    float y = pose[1].get<float>();
    int theta = pose[2].get<int>();

    poses.push_back(get_pose_stamped(x, y, theta));
  }
  return poses;
}

// TODO split how json is created for different post request paths /
// /extend_plan
// /
nlohmann::json commander_server::turtlebot4_commander::json_post_format(
    Pose pose, std::string status, uint32_t pose_number) {

  nlohmann::json json_pose = nlohmann::json::object();
  json_pose["agent_id"] = id_;
  json_pose["x"] = pose.position.x;
  json_pose["y"] = pose.position.y;
  json_pose["theta"] =
      asin(pose.orientation.z) * 2 * 180 / PI; // magic backtracking trust me

  nlohmann::json plans = nlohmann::json::array({json_pose});

  nlohmann::json progress = nlohmann::json::object();
  progress["current"] = pose_number;
  progress["total"] = num_poses_;

  nlohmann::json payload = nlohmann::json::object();
  payload["agent_id"] = id_;
  payload["status"] = status;
  payload["pose"] = json_pose;
  payload["plans"] = plans;
  payload["progress"] = progress;

  return payload;
}

void commander_server::turtlebot4_commander::navigate_to_pose(
    std::vector<PoseStamped> poses) {
  RCLCPP_INFO(this->get_logger(), "Sending Waypoints to Robot");
  for (uint32_t i = 0; i < poses.size(); i++) {
    PoseStamped &pose = poses[i];
    RCLCPP_INFO(this->get_logger(), "Pose %d, (%f, %f, %f)", i,
                pose.pose.position.x, pose.pose.position.y,
                pose.pose.orientation.z);
  }
  RCLCPP_INFO(this->get_logger(), "Sending Waypoints to Robot");

  navigate_to_pose_send_goal(poses);
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
    std::vector<PoseStamped> poses) {

  auto goal_msg = NavigateThroughPoses::Goal();
  goal_msg.poses = poses;
  num_poses_ = poses.size();
  // TODO: verify if we have to specify behaviour tree
  goal_msg.behavior_tree = "";

  navigate_to_pose_client_ptr_->wait_for_action_server();

  auto send_goal_options =
      rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
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
        const GoalHandleNavigateThroughPoses::SharedPtr goal_handle) {

  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    reset_state();
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void commander_server::turtlebot4_commander::navigate_to_pose_feedback_callback(
    GoalHandleNavigateThroughPoses::SharedPtr,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {

  pose_ = feedback->current_pose.pose;
  uint32_t progress = num_poses_ - feedback->number_of_poses_remaining;

  RCLCPP_INFO(this->get_logger(),
              "Received feedback: %f, %f, %f Progress: %d/%d",
              feedback->current_pose.pose.position.x,
              feedback->current_pose.pose.position.y,
              feedback->current_pose.pose.orientation.z, progress, num_poses_);

  nlohmann::json payload =
      json_post_format(feedback->current_pose.pose, "executing", progress);
  make_request(boost::beast::http::verb::post, "/", payload);
}

void commander_server::turtlebot4_commander::navigate_to_pose_result_callback(
    const GoalHandleNavigateThroughPoses::WrappedResult &result) {

  std::map<rclcpp_action::ResultCode, std::string> status{
      {rclcpp_action::ResultCode::SUCCEEDED, "success"},
      {rclcpp_action::ResultCode::ABORTED, "aborted"},
      {rclcpp_action::ResultCode::CANCELED, "cancelled"},
      {rclcpp_action::ResultCode::UNKNOWN, "unknown"},
  };

  RCLCPP_INFO(this->get_logger(), "Goal Status: %s",
              status[result.code].c_str());

  nlohmann::json payload = json_post_format(pose_, status[result.code], 0);
  make_request(boost::beast::http::verb::post, "/", payload);
  reset_state();
}

void commander_server::turtlebot4_commander::pose_topic_callback(
    const PoseWithCovarianceStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Pose: %f, %f, %f", msg->pose.pose.position.x,
              msg->pose.pose.position.y, msg->pose.pose.orientation.z);

  nlohmann::json payload = json_post_format(msg->pose.pose, "succeeded", 1);
  make_request(boost::beast::http::verb::post, "/extend_path", payload);
  pose_subscriber_ptr_.reset();

  reset_state();
}

void commander_server::turtlebot4_commander::polling_callback() {

  if (is_executing_) {
    return;
  };

  is_executing_ = true;
  std::vector<PoseStamped> poses = get_request();
  navigate_to_pose(poses);
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
