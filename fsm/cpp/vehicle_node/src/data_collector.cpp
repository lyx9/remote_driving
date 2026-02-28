#include "fsm/vehicle/data_collector.hpp"
#include "fsm/utils.hpp"
#include <fstream>
#include <sstream>
#include <sys/sysinfo.h>

namespace fsm {
namespace vehicle {

// Build a rclcpp::QoS from our config struct.
static rclcpp::QoS BuildQos(const config::QosConfig& cfg) {
  rclcpp::QoS qos(static_cast<size_t>(cfg.depth));
  qos.reliability(cfg.reliability == "reliable"
      ? rclcpp::ReliabilityPolicy::Reliable
      : rclcpp::ReliabilityPolicy::BestEffort);
  qos.durability(cfg.durability == "transient_local"
      ? rclcpp::DurabilityPolicy::TransientLocal
      : rclcpp::DurabilityPolicy::Volatile);
  return qos;
}

DataCollector::DataCollector(
    rclcpp::Node::SharedPtr node,
    const config::VehicleConfigManager& config)
    : node_(node), config_(config) {
  FSM_LOG_INFO("DataCollector initialized: {}", config_.vehicle_id());
}

DataCollector::~DataCollector() {
  Stop();
}

void DataCollector::Start() {
  if (running_) {
    FSM_LOG_WARN("DataCollector already running");
    return;
  }
  running_ = true;

  SetupCameraSubscribers();
  SetupVehicleSubscribers();
  SetupLocalizationSubscribers();
  SetupSystemSubscribers();

  system_monitor_timer_ = node_->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&DataCollector::SystemMonitorCallback, this));

  FSM_LOG_INFO("DataCollector started");
}

void DataCollector::Stop() {
  if (!running_) return;
  running_ = false;

  if (system_monitor_timer_) {
    system_monitor_timer_->cancel();
    system_monitor_timer_.reset();
  }

  camera_subs_.clear();
  camera_it_subs_.clear();
  vehicle_subs_.clear();
  odom_sub_.reset();
  pose_sub_.reset();

  FSM_LOG_INFO("DataCollector stopped");
}

VehicleState DataCollector::GetVehicleState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return vehicle_state_;
}

SystemState DataCollector::GetSystemState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return system_state_;
}

bool DataCollector::IsHealthy() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (vehicle_state_.timestamp_ns == 0) return true;
  const int64_t age_ns = utils::NowNanos() - vehicle_state_.timestamp_ns;
  if (age_ns > 5000000000LL) {
    FSM_LOG_WARN("Vehicle state data is stale");
    return false;
  }
  return true;
}

void DataCollector::set_vehicle_state_callback(VehicleStateCallback cb) {
  vehicle_state_cb_ = std::move(cb);
}

void DataCollector::set_system_state_callback(SystemStateCallback cb) {
  system_state_cb_ = std::move(cb);
}

void DataCollector::set_camera_frame_callback(CameraFrameCallback cb) {
  camera_frame_cb_ = std::move(cb);
}

void DataCollector::SetupCameraSubscribers() {
  for (const auto& cam : config_.cameras()) {
    if (!cam.enabled) continue;
    // Use per-camera QoS config; bypass image_transport so QoS is fully
    // under our control (image_transport defaults to best_effort anyway).
    const auto qos = BuildQos(cam.qos);
    camera_subs_[cam.id] = node_->create_subscription<sensor_msgs::msg::Image>(
        cam.topic, qos,
        [this, id = cam.id](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
          OnCameraImage(msg, id);
        });
    camera_frame_counts_[cam.id] = 0;
    FSM_LOG_INFO("Subscribed to camera: {} ({}) reliability={} depth={}",
                 cam.id, cam.topic, cam.qos.reliability, cam.qos.depth);
  }
}

void DataCollector::OnCameraImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg,
    const std::string& camera_id) {
  if (!running_ || !camera_frame_cb_) return;
  try {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    CameraFrame frame;
    frame.camera_id    = camera_id;
    frame.image        = cv_ptr->image.clone();
    frame.timestamp_ns = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
    frame.frame_number = ++camera_frame_counts_[camera_id];
    camera_frame_cb_(frame);
  } catch (const cv_bridge::Exception& e) {
    FSM_LOG_ERROR("cv_bridge exception: {}", e.what());
  }
}

void DataCollector::OnCompressedImage(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg,
    const std::string& camera_id) {
  if (!running_ || !camera_frame_cb_) return;
  try {
    cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (image.empty()) {
      FSM_LOG_WARN("Failed to decode compressed image from {}", camera_id);
      return;
    }
    CameraFrame frame;
    frame.camera_id    = camera_id;
    frame.image        = image;
    frame.timestamp_ns = msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
    frame.frame_number = ++camera_frame_counts_[camera_id];
    camera_frame_cb_(frame);
  } catch (const std::exception& e) {
    FSM_LOG_ERROR("Exception in OnCompressedImage: {}", e.what());
  }
}

void DataCollector::SetupVehicleSubscribers() {
  const auto& topics = config_.sensor_topics();
  const auto  qos    = BuildQos(config_.sensor_qos().vehicle);

#ifdef HAVE_AUTOWARE_MSGS
  vehicle_subs_.push_back(node_->create_subscription<
      autoware_auto_vehicle_msgs::msg::VelocityReport>(
      topics.velocity_status, qos,
      std::bind(&DataCollector::OnVelocityReport, this, std::placeholders::_1)));

  vehicle_subs_.push_back(node_->create_subscription<
      autoware_auto_vehicle_msgs::msg::SteeringReport>(
      topics.steering_status, qos,
      std::bind(&DataCollector::OnSteeringReport, this, std::placeholders::_1)));

  vehicle_subs_.push_back(node_->create_subscription<
      autoware_auto_vehicle_msgs::msg::GearReport>(
      topics.gear_status, qos,
      std::bind(&DataCollector::OnGearReport, this, std::placeholders::_1)));

  vehicle_subs_.push_back(node_->create_subscription<
      autoware_auto_vehicle_msgs::msg::ControlModeReport>(
      topics.control_mode, qos,
      std::bind(&DataCollector::OnControlModeReport, this, std::placeholders::_1)));

  FSM_LOG_INFO("Subscribed to Autoware vehicle topics (reliability={})",
               config_.sensor_qos().vehicle.reliability);
#else
  vehicle_subs_.push_back(node_->create_subscription<geometry_msgs::msg::TwistStamped>(
      topics.velocity_status, qos,
      std::bind(&DataCollector::OnTwistStamped, this, std::placeholders::_1)));

  vehicle_subs_.push_back(node_->create_subscription<std_msgs::msg::Float64>(
      topics.steering_status, qos,
      std::bind(&DataCollector::OnFloat64Steering, this, std::placeholders::_1)));

  FSM_LOG_INFO("Subscribed to generic vehicle topics (reliability={})",
               config_.sensor_qos().vehicle.reliability);
#endif
}

#ifdef HAVE_AUTOWARE_MSGS
void DataCollector::OnVelocityReport(
    const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  vehicle_state_.longitudinal_velocity = msg->longitudinal_velocity;
  vehicle_state_.lateral_velocity      = msg->lateral_velocity;
  vehicle_state_.heading_rate          = msg->heading_rate;
  vehicle_state_.velocity_valid        = true;
  vehicle_state_.timestamp_ns =
      msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
  if (vehicle_state_cb_) vehicle_state_cb_(vehicle_state_);
}

void DataCollector::OnSteeringReport(
    const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  vehicle_state_.steering_tire_angle = msg->steering_tire_angle;
  vehicle_state_.steering_valid      = true;
}

void DataCollector::OnGearReport(
    const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  vehicle_state_.gear = static_cast<int>(msg->report);
}

void DataCollector::OnControlModeReport(
    const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  vehicle_state_.control_mode = static_cast<int>(msg->mode);
}

void DataCollector::OnTurnIndicatorsReport(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  vehicle_state_.turn_indicator = static_cast<int>(msg->report);
}
#else
void DataCollector::OnTwistStamped(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  vehicle_state_.longitudinal_velocity = msg->twist.linear.x;
  vehicle_state_.lateral_velocity      = msg->twist.linear.y;
  vehicle_state_.heading_rate          = msg->twist.angular.z;
  vehicle_state_.velocity_valid        = true;
  vehicle_state_.timestamp_ns =
      msg->header.stamp.sec * 1000000000LL + msg->header.stamp.nanosec;
  if (vehicle_state_cb_) vehicle_state_cb_(vehicle_state_);
}

void DataCollector::OnFloat64Steering(
    const std_msgs::msg::Float64::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  vehicle_state_.steering_tire_angle = msg->data;
  vehicle_state_.steering_valid      = true;
}
#endif

void DataCollector::SetupLocalizationSubscribers() {
  const auto& topics = config_.sensor_topics();
  const auto  qos    = BuildQos(config_.sensor_qos().localization);
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      topics.localization, qos,
      std::bind(&DataCollector::OnOdometry, this, std::placeholders::_1));
  FSM_LOG_INFO("Subscribed to localization: {} (reliability={})",
               topics.localization, config_.sensor_qos().localization.reliability);
}

void DataCollector::OnOdometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  vehicle_state_.pose_x        = msg->pose.pose.position.x;
  vehicle_state_.pose_y        = msg->pose.pose.position.y;
  vehicle_state_.pose_z        = msg->pose.pose.position.z;
  vehicle_state_.orientation_x = msg->pose.pose.orientation.x;
  vehicle_state_.orientation_y = msg->pose.pose.orientation.y;
  vehicle_state_.orientation_z = msg->pose.pose.orientation.z;
  vehicle_state_.orientation_w = msg->pose.pose.orientation.w;
  vehicle_state_.localization_valid = true;
  if (vehicle_state_cb_) vehicle_state_cb_(vehicle_state_);
}

void DataCollector::OnPoseStamped(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  vehicle_state_.pose_x        = msg->pose.position.x;
  vehicle_state_.pose_y        = msg->pose.position.y;
  vehicle_state_.pose_z        = msg->pose.position.z;
  vehicle_state_.orientation_x = msg->pose.orientation.x;
  vehicle_state_.orientation_y = msg->pose.orientation.y;
  vehicle_state_.orientation_z = msg->pose.orientation.z;
  vehicle_state_.orientation_w = msg->pose.orientation.w;
  vehicle_state_.localization_valid = true;
}

void DataCollector::SetupSystemSubscribers() {
  FSM_LOG_INFO("System subscribers setup");
}

void DataCollector::SystemMonitorCallback() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  static long prev_idle = 0, prev_total = 0;
  std::ifstream stat_file("/proc/stat");
  if (stat_file.is_open()) {
    std::string label;
    long user, nice, system, idle, iowait, irq, softirq;
    stat_file >> label >> user >> nice >> system >> idle >> iowait >> irq >> softirq;
    const long total      = user + nice + system + idle + iowait + irq + softirq;
    const long total_diff = total - prev_total;
    if (total_diff > 0) {
      system_state_.cpu_usage =
          100.0f * (1.0f - static_cast<float>(idle - prev_idle) / total_diff);
    }
    prev_idle  = idle;
    prev_total = total;
  }

  struct sysinfo si;
  if (sysinfo(&si) == 0) {
    const unsigned long total_mem = si.totalram * si.mem_unit;
    const unsigned long free_mem  = si.freeram  * si.mem_unit;
    system_state_.memory_usage =
        100.0f * (1.0f - static_cast<float>(free_mem) / total_mem);
  }

  static long prev_rx = 0, prev_tx = 0;
  std::ifstream net_file("/proc/net/dev");
  if (net_file.is_open()) {
    std::string line;
    while (std::getline(net_file, line)) {
      if (line.find("eth0")  == std::string::npos &&
          line.find("wlan0") == std::string::npos &&
          line.find("enp")   == std::string::npos) continue;
      std::istringstream iss(line);
      std::string iface;
      long rx_bytes, tx_bytes;
      iss >> iface >> rx_bytes;
      for (int i = 0; i < 7; ++i) iss >> tx_bytes;
      iss >> tx_bytes;
      system_state_.network_rx_bps = static_cast<float>(rx_bytes - prev_rx);
      system_state_.network_tx_bps = static_cast<float>(tx_bytes - prev_tx);
      prev_rx = rx_bytes;
      prev_tx = tx_bytes;
      break;
    }
  }

  system_state_.timestamp_ns = utils::NowNanos();
  if (system_state_cb_) system_state_cb_(system_state_);
}

}  // namespace vehicle
}  // namespace fsm
