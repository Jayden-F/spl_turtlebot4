#include "commander_server/turtlebot4_commander.hpp"
#include <cstdint>

commander_server::turtlebot4_commander::turtlebot4_commander(
    const uint32_t id, const std::string &ip, const uint16_t port,
    const rclcpp::NodeOptions &options)
    : Node("Turtlebot4_Commander", options), is_executing_(true),
      started_(false), end_timestep_(-1), current_progress_(0), pose_(),
      poses_(0), ip_(ip), port_(port), id_(id),
      stream_(ioc_, boost::asio::ip::tcp::v4()) {

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
  // RCLCPP_INFO(this->get_logger(), "%s", ec.message().c_str());

  http::response<http::string_body> res;

  // Receive the HTTP response
  // RCLCPP_INFO(this->get_logger(), "Reading Response");
  http::read(stream_, buffer_, res, ec);
  // RCLCPP_INFO(this->get_logger(), "%s", ec.message().c_str());

  stream_.socket().close();
  // RCLCPP_INFO(this->get_logger(), "Converting to string");
  // RCLCPP_INFO(this->get_logger(), "%s", res.body().c_str());

  return res.body();
}

void commander_server::turtlebot4_commander::request_next_poses() {

  RCLCPP_INFO(this->get_logger(),
              "Requesting positions from central controller");

  nlohmann::json json_pose = nlohmann::json::object();
  json_pose["agent_id"] = id_;

  std::string data =
      make_request(boost::beast::http::verb::get, "/", json_pose);

  if (data.empty()) {
    RCLCPP_INFO(this->get_logger(), "No data received");
    return;
  }

  nlohmann::json json_received = nlohmann::json::parse(data);

  // read the position from the json in format [x, y, theta]
  std::vector<nlohmann::json> json_positions = json_received["positions"];

  // std::cout << json_positions << ',' << json_positions <<
  // json_positions.size()
  //<< std::endl;

  poses_.resize(json_positions.size());

  for (uint32_t i = 0; i < poses_.size(); i++) {
    nlohmann::json &json_position = json_positions[i];

    float x = json_position[0].get<float>();
    float y = json_position[1].get<float>();
    int theta = json_position[2].get<int>();

    poses_[i] = get_pose_stamped(x, y, theta);
  }
  end_timestep_ = json_received["end_timestep"].get<int32_t>();
  current_progress_ = 0;
}

nlohmann::json commander_server::turtlebot4_commander::json_post_format(
    PoseStamped pose, std::string status, uint32_t timestep) {

  nlohmann::json json_pose = nlohmann::json::object();
  json_pose["agent_id"] = id_;
  json_pose["x"] = pose.pose.position.x;
  json_pose["y"] = pose.pose.position.y;
  json_pose["theta"] = asin(pose.pose.orientation.z) * 2 * 180 /
                       PI; // magic backtracking trust me

  nlohmann::json plans = nlohmann::json::array({json_pose});

  nlohmann::json payload = nlohmann::json::object();
  payload["agent_id"] = id_;
  payload["status"] = status;
  payload["position"] = json_pose;
  payload["plans"] = plans;
  payload["timestep"] = timestep;

  return payload;
}

void commander_server::turtlebot4_commander::navigate_to_pose() {
  RCLCPP_INFO(this->get_logger(), "Sending Waypoints to Robot");

  for (uint32_t i = 0; i < poses_.size(); i++) {
    PoseStamped &pose = poses_[i];
    RCLCPP_INFO(this->get_logger(), "Pose %d, (%f, %f, %f)", i,
                pose.pose.position.x, pose.pose.position.y,
                pose.pose.orientation.z);
  }

  navigate_to_pose_send_goal(poses_);
}

commander_server::PoseStamped
commander_server::turtlebot4_commander::get_pose_stamped(float x, float y,
                                                         float theta) {

  PoseStamped pose;
  pose.header.stamp = this->get_clock()->now();
  pose.header.frame_id = "map";

  pose.pose.position.x = x;
  pose.pose.position.y = y;

  pose.pose.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(theta);

  return pose;
}

void commander_server::turtlebot4_commander::navigate_to_pose_send_goal(
    std::vector<PoseStamped> &poses) {

  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = poses.back();

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
    reset_state();
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void commander_server::turtlebot4_commander::navigate_to_pose_feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {

  pose_ = feedback->current_pose;

  // I am sorry, we were unable to get navigate through poses to consistently
  // accept requests.
  std::string status = "executing";
  for (uint32_t i = current_progress_ + 1; i < poses_.size(); i++) {
    if (check_at_position(pose_, poses_[i], 0.2)) {
      current_progress_ = i;
      status = "succeeded";

      nlohmann::json payload =
          json_post_format(feedback->current_pose, status,
                           end_timestep_ + current_progress_ - poses_.size());
      make_request(boost::beast::http::verb::post, "/", payload);
      break;
    }
  }

  RCLCPP_ERROR(this->get_logger(),
               "Received feedback: %f, %f, %f, Progress: %d/%zu",
               feedback->current_pose.pose.position.x,
               feedback->current_pose.pose.position.y,
               feedback->current_pose.pose.orientation.z, current_progress_,
               poses_.size());
}

void commander_server::turtlebot4_commander::navigate_to_pose_result_callback(
    const GoalHandleNavigateToPose::WrappedResult &result) {

  std::map<rclcpp_action::ResultCode, std::string> status{
      {rclcpp_action::ResultCode::SUCCEEDED, "succeeded"},
      {rclcpp_action::ResultCode::ABORTED, "aborted"},
      {rclcpp_action::ResultCode::CANCELED, "canceled"},
      {rclcpp_action::ResultCode::UNKNOWN, "unknown"},
  };

  RCLCPP_INFO(this->get_logger(), "Goal Status: %s",
              status[result.code].c_str());

  nlohmann::json payload =
      json_post_format(pose_, status[result.code], end_timestep_);
  make_request(boost::beast::http::verb::post, "/", payload);
  reset_state();
}

void commander_server::turtlebot4_commander::pose_topic_callback(
    const PoseWithCovarianceStamped::SharedPtr msg) {
  if (started_) {
    return;
  }
  started_ = true;

  RCLCPP_INFO(this->get_logger(), "Pose: %f, %f, %f", msg->pose.pose.position.x,
              msg->pose.pose.position.y, msg->pose.pose.orientation.z);

  pose_.pose.position.x = msg->pose.pose.position.x;
  pose_.pose.position.y = msg->pose.pose.position.y;
  pose_.pose.orientation.z = msg->pose.pose.orientation.z;

  nlohmann::json payload = json_post_format(pose_, "succeeded", 0);
  make_request(boost::beast::http::verb::post, "/extend_path", payload);
  reset_state();
}

void commander_server::turtlebot4_commander::polling_callback() {

  if (is_executing_) {
    return;
  };

  is_executing_ = true;

  request_next_poses();
  navigate_to_pose();
}

bool commander_server::turtlebot4_commander::check_at_position(
    PoseStamped current, PoseStamped target, float threshold_metres) {
  if (abs(current.pose.position.x - target.pose.position.x) >
      threshold_metres) {
    return false;
  }
  if (abs(current.pose.position.y - target.pose.position.y) >
      threshold_metres) {
    return false;
  }
  return true;
}

int main(int argc, char **argv) {

  namespace po = boost::program_options;

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
