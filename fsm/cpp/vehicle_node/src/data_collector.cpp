#include "fsm/vehicle/data_collector.hpp"
#include "fsm/utils.hpp"
#include <fstream>
#include <sys/sysinfo.h>

namespace fsm {
namespace vehicle {

DataCollector::DataCollector(
    rclcpp::Node::SharedPtr node,
    const config::VehicleConfigManager& config)
    : node_(node), config_(config) {

    FSM_LOG_INFO("DataCollector initialized for vehicle: {}", config_.getVehicleId());
}

DataCollector::~DataCollector() {
    stop();
}

void DataCollector::start() {
    if (running_) {
        FSM_LOG_WARN("DataCollector already running");
        return;
    }

    running_ = true;

    // 设置各类订阅者
    setupCameraSubscribers();
    setupVehicleSubscribers();
    setupLocalizationSubscribers();
    setupSystemSubscribers();

    // 系统监控定时器 (1Hz)
    system_monitor_timer_ = node_->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&DataCollector::systemMonitorCallback, this));

    FSM_LOG_INFO("DataCollector started");
}

void DataCollector::stop() {
    if (!running_) {
        return;
    }

    running_ = false;

    // 清理定时器
    if (system_monitor_timer_) {
        system_monitor_timer_->cancel();
        system_monitor_timer_.reset();
    }

    // 清理订阅者
    camera_subs_.clear();
    camera_it_subs_.clear();
    vehicle_subs_.clear();
    odom_sub_.reset();
    pose_sub_.reset();
    // diag_sub_.reset();  // Commented out - diagnostic_msgs not available

    FSM_LOG_INFO("DataCollector stopped");
}

void DataCollector::setupCameraSubscribers() {
    const auto& cameras = config_.getCameras();

    auto it = std::make_shared<image_transport::ImageTransport>(node_);

    for (const auto& cam : cameras) {
        if (!cam.enabled) {
            continue;
        }

        // 使用image_transport订阅 (支持压缩)
        auto callback = [this, cam_id = cam.id](
            const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
            this->onCameraImage(msg, cam_id);
        };

        camera_it_subs_[cam.id] = it->subscribe(
            cam.topic, 1, callback);

        camera_frame_counts_[cam.id] = 0;

        FSM_LOG_INFO("Subscribed to camera: {} ({})", cam.id, cam.topic);
    }
}

void DataCollector::onCameraImage(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg,
    const std::string& camera_id) {

    if (!running_ || !camera_frame_callback_) {
        return;
    }

    try {
        // 转换为OpenCV格式
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");

        CameraFrame frame;
        frame.camera_id = camera_id;
        frame.image = cv_ptr->image.clone();
        frame.timestamp_ns = msg->header.stamp.sec * 1000000000LL +
                             msg->header.stamp.nanosec;
        frame.frame_number = ++camera_frame_counts_[camera_id];

        // 调用回调
        camera_frame_callback_(frame);

    } catch (const cv_bridge::Exception& e) {
        FSM_LOG_ERROR("cv_bridge exception: {}", e.what());
    }
}

void DataCollector::onCompressedImage(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg,
    const std::string& camera_id) {

    if (!running_ || !camera_frame_callback_) {
        return;
    }

    try {
        // 解码压缩图像
        cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (image.empty()) {
            FSM_LOG_WARN("Failed to decode compressed image from {}", camera_id);
            return;
        }

        CameraFrame frame;
        frame.camera_id = camera_id;
        frame.image = image;
        frame.timestamp_ns = msg->header.stamp.sec * 1000000000LL +
                             msg->header.stamp.nanosec;
        frame.frame_number = ++camera_frame_counts_[camera_id];

        camera_frame_callback_(frame);

    } catch (const std::exception& e) {
        FSM_LOG_ERROR("Exception in onCompressedImage: {}", e.what());
    }
}

void DataCollector::setupVehicleSubscribers() {
    const auto& topics = config_.getSensorTopics();

#ifdef HAVE_AUTOWARE_MSGS
    // Autoware消息订阅
    auto velocity_sub = node_->create_subscription<
        autoware_auto_vehicle_msgs::msg::VelocityReport>(
        topics.velocity_status, 10,
        std::bind(&DataCollector::onVelocityReport, this, std::placeholders::_1));
    vehicle_subs_.push_back(velocity_sub);

    auto steering_sub = node_->create_subscription<
        autoware_auto_vehicle_msgs::msg::SteeringReport>(
        topics.steering_status, 10,
        std::bind(&DataCollector::onSteeringReport, this, std::placeholders::_1));
    vehicle_subs_.push_back(steering_sub);

    auto gear_sub = node_->create_subscription<
        autoware_auto_vehicle_msgs::msg::GearReport>(
        topics.gear_status, 10,
        std::bind(&DataCollector::onGearReport, this, std::placeholders::_1));
    vehicle_subs_.push_back(gear_sub);

    auto mode_sub = node_->create_subscription<
        autoware_auto_vehicle_msgs::msg::ControlModeReport>(
        topics.control_mode, 10,
        std::bind(&DataCollector::onControlModeReport, this, std::placeholders::_1));
    vehicle_subs_.push_back(mode_sub);

    FSM_LOG_INFO("Subscribed to Autoware vehicle topics");

#else
    // 通用消息订阅 (geometry_msgs)
    auto twist_sub = node_->create_subscription<geometry_msgs::msg::TwistStamped>(
        topics.velocity_status, 10,
        std::bind(&DataCollector::onTwistStamped, this, std::placeholders::_1));
    vehicle_subs_.push_back(twist_sub);

    auto steering_sub = node_->create_subscription<std_msgs::msg::Float64>(
        topics.steering_status, 10,
        std::bind(&DataCollector::onFloat64Steering, this, std::placeholders::_1));
    vehicle_subs_.push_back(steering_sub);

    FSM_LOG_INFO("Subscribed to generic vehicle topics");
#endif
}

#ifdef HAVE_AUTOWARE_MSGS
void DataCollector::onVelocityReport(
    const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr& msg) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    vehicle_state_.longitudinal_velocity = msg->longitudinal_velocity;
    vehicle_state_.lateral_velocity = msg->lateral_velocity;
    vehicle_state_.heading_rate = msg->heading_rate;
    vehicle_state_.velocity_valid = true;
    vehicle_state_.timestamp_ns = msg->header.stamp.sec * 1000000000LL +
                                   msg->header.stamp.nanosec;

    if (vehicle_state_callback_) {
        vehicle_state_callback_(vehicle_state_);
    }
}

void DataCollector::onSteeringReport(
    const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr& msg) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    vehicle_state_.steering_tire_angle = msg->steering_tire_angle;
    vehicle_state_.steering_valid = true;
}

void DataCollector::onGearReport(
    const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr& msg) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    vehicle_state_.gear = static_cast<int>(msg->report);
}

void DataCollector::onControlModeReport(
    const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr& msg) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    vehicle_state_.control_mode = static_cast<int>(msg->mode);
}

void DataCollector::onTurnIndicatorsReport(
    const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr& msg) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    vehicle_state_.turn_indicator = static_cast<int>(msg->report);
}

#else
void DataCollector::onTwistStamped(
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    vehicle_state_.longitudinal_velocity = msg->twist.linear.x;
    vehicle_state_.lateral_velocity = msg->twist.linear.y;
    vehicle_state_.heading_rate = msg->twist.angular.z;
    vehicle_state_.velocity_valid = true;
    vehicle_state_.timestamp_ns = msg->header.stamp.sec * 1000000000LL +
                                   msg->header.stamp.nanosec;

    if (vehicle_state_callback_) {
        vehicle_state_callback_(vehicle_state_);
    }
}

void DataCollector::onFloat64Steering(
    const std_msgs::msg::Float64::ConstSharedPtr& msg) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    vehicle_state_.steering_tire_angle = msg->data;
    vehicle_state_.steering_valid = true;
}
#endif

void DataCollector::setupLocalizationSubscribers() {
    const auto& topics = config_.getSensorTopics();

    // 里程计订阅
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        topics.localization, 10,
        std::bind(&DataCollector::onOdometry, this, std::placeholders::_1));

    FSM_LOG_INFO("Subscribed to localization topic: {}", topics.localization);
}

void DataCollector::onOdometry(
    const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    vehicle_state_.pose_x = msg->pose.pose.position.x;
    vehicle_state_.pose_y = msg->pose.pose.position.y;
    vehicle_state_.pose_z = msg->pose.pose.position.z;

    vehicle_state_.orientation_x = msg->pose.pose.orientation.x;
    vehicle_state_.orientation_y = msg->pose.pose.orientation.y;
    vehicle_state_.orientation_z = msg->pose.pose.orientation.z;
    vehicle_state_.orientation_w = msg->pose.pose.orientation.w;

    vehicle_state_.localization_valid = true;

    if (vehicle_state_callback_) {
        vehicle_state_callback_(vehicle_state_);
    }
}

void DataCollector::onPoseStamped(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    vehicle_state_.pose_x = msg->pose.position.x;
    vehicle_state_.pose_y = msg->pose.position.y;
    vehicle_state_.pose_z = msg->pose.position.z;

    vehicle_state_.orientation_x = msg->pose.orientation.x;
    vehicle_state_.orientation_y = msg->pose.orientation.y;
    vehicle_state_.orientation_z = msg->pose.orientation.z;
    vehicle_state_.orientation_w = msg->pose.orientation.w;

    vehicle_state_.localization_valid = true;
}

void DataCollector::setupSystemSubscribers() {
    // const auto& topics = config_.getSensorTopics();

    // 诊断消息订阅 - 已禁用，需要 diagnostic_msgs 包
    // diag_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    //     topics.diagnostics, 10,
    //     std::bind(&DataCollector::onDiagnostics, this, std::placeholders::_1));

    FSM_LOG_INFO("System subscribers setup (diagnostics disabled)");
}

/*
// Commented out - diagnostic_msgs not available
void DataCollector::onDiagnostics(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr& msg) {

    std::lock_guard<std::mutex> lock(state_mutex_);

    system_state_.diagnostics.clear();
    for (const auto& status : msg->status) {
        system_state_.diagnostics.emplace_back(
            status.name, static_cast<int>(status.level));
    }

    system_state_.timestamp_ns = msg->header.stamp.sec * 1000000000LL +
                                  msg->header.stamp.nanosec;
}
*/

void DataCollector::systemMonitorCallback() {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 读取CPU使用率
    static long prev_idle = 0, prev_total = 0;
    std::ifstream stat_file("/proc/stat");
    if (stat_file.is_open()) {
        std::string cpu_label;
        long user, nice, system, idle, iowait, irq, softirq;
        stat_file >> cpu_label >> user >> nice >> system >> idle >> iowait >> irq >> softirq;

        long total = user + nice + system + idle + iowait + irq + softirq;
        long idle_diff = idle - prev_idle;
        long total_diff = total - prev_total;

        if (total_diff > 0) {
            system_state_.cpu_usage = 100.0f * (1.0f - static_cast<float>(idle_diff) / total_diff);
        }

        prev_idle = idle;
        prev_total = total;
        stat_file.close();
    }

    // 读取内存使用率
    struct sysinfo si;
    if (sysinfo(&si) == 0) {
        unsigned long total_mem = si.totalram * si.mem_unit;
        unsigned long free_mem = si.freeram * si.mem_unit;
        system_state_.memory_usage = 100.0f * (1.0f - static_cast<float>(free_mem) / total_mem);
    }

    // 读取网络流量 (简化版)
    static long prev_rx = 0, prev_tx = 0;
    std::ifstream net_file("/proc/net/dev");
    if (net_file.is_open()) {
        std::string line;
        while (std::getline(net_file, line)) {
            if (line.find("eth0") != std::string::npos ||
                line.find("wlan0") != std::string::npos ||
                line.find("enp") != std::string::npos) {

                std::istringstream iss(line);
                std::string iface;
                long rx_bytes, tx_bytes;
                iss >> iface >> rx_bytes;
                // 跳过一些字段
                for (int i = 0; i < 7; ++i) iss >> tx_bytes;
                iss >> tx_bytes;

                system_state_.network_rx_bps = static_cast<float>(rx_bytes - prev_rx);
                system_state_.network_tx_bps = static_cast<float>(tx_bytes - prev_tx);

                prev_rx = rx_bytes;
                prev_tx = tx_bytes;
                break;
            }
        }
        net_file.close();
    }

    system_state_.timestamp_ns = utils::TimeUtils::nowNanos();

    if (system_state_callback_) {
        system_state_callback_(system_state_);
    }
}

VehicleState DataCollector::getVehicleState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return vehicle_state_;
}

SystemState DataCollector::getSystemState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return system_state_;
}

void DataCollector::setVehicleStateCallback(VehicleStateCallback callback) {
    vehicle_state_callback_ = std::move(callback);
}

void DataCollector::setSystemStateCallback(SystemStateCallback callback) {
    system_state_callback_ = std::move(callback);
}

void DataCollector::setCameraFrameCallback(CameraFrameCallback callback) {
    camera_frame_callback_ = std::move(callback);
}

bool DataCollector::isHealthy() const {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 检查各数据源是否有效
    bool healthy = true;

    // 检查车辆状态数据时效性 (5秒内)
    int64_t now = utils::TimeUtils::nowNanos();
    if (vehicle_state_.timestamp_ns > 0) {
        int64_t age_ns = now - vehicle_state_.timestamp_ns;
        if (age_ns > 5000000000LL) {  // 5秒
            healthy = false;
            FSM_LOG_WARN("Vehicle state data is stale");
        }
    }

    return healthy;
}

}  // namespace vehicle
}  // namespace fsm
