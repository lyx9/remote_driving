#include "fsm/vehicle/vehicle_node.hpp"
#include "fsm/utils.hpp"

// Protobuf 序列化
#include "fsm_messages.pb.h"

namespace fsm {
namespace vehicle {

VehicleNode::VehicleNode(const rclcpp::NodeOptions& options)
    : Node("fsm_vehicle_node", options) {

    // 声明参数
    this->declare_parameter<std::string>("config_file", "");
    this->declare_parameter<int>("telemetry_rate_hz", 20);
    this->declare_parameter<int>("health_check_interval_ms", 5000);

    FSM_LOG_INFO("VehicleNode created");
}

VehicleNode::~VehicleNode() {
    stop();
}

bool VehicleNode::initialize(const std::string& config_path) {
    // 加载配置
    std::string config_file = config_path;
    if (config_file.empty()) {
        config_file = this->get_parameter("config_file").as_string();
    }

    if (config_file.empty()) {
        FSM_LOG_ERROR("No config file specified");
        return false;
    }

    if (!config_manager_.loadFromFile(config_file)) {
        FSM_LOG_ERROR("Failed to load config from: {}", config_file);
        return false;
    }

    FSM_LOG_INFO("Initializing VehicleNode for: {}", config_manager_.getVehicleId());

    // 创建数据采集器
    data_collector_ = std::make_unique<DataCollector>(
        shared_from_this(), config_manager_);

    // 设置数据采集回调
    data_collector_->setVehicleStateCallback(
        std::bind(&VehicleNode::onVehicleStateUpdate, this, std::placeholders::_1));
    data_collector_->setSystemStateCallback(
        std::bind(&VehicleNode::onSystemStateUpdate, this, std::placeholders::_1));
    data_collector_->setCameraFrameCallback(
        std::bind(&VehicleNode::onCameraFrame, this, std::placeholders::_1));

    // 创建指令执行器
    command_executor_ = std::make_unique<CommandExecutor>(
        shared_from_this(), config_manager_);

    // 创建WebRTC客户端
    WebRTCClientConfig rtc_config;
    rtc_config.vehicle_id = config_manager_.getVehicleId();
    rtc_config.signaling_url = config_manager_.getWebRTCConfig().signaling_url;
    rtc_config.stun_servers = config_manager_.getWebRTCConfig().stun_servers;
    rtc_config.turn_servers = config_manager_.getWebRTCConfig().turn_servers;
    rtc_config.turn_username = config_manager_.getWebRTCConfig().turn_username;
    rtc_config.turn_password = config_manager_.getWebRTCConfig().turn_password;

    webrtc_client_ = std::make_unique<WebRTCClient>(rtc_config);

    // 设置WebRTC回调
    webrtc_client_->setConnectionStateCallback(
        std::bind(&VehicleNode::onConnectionStateChange, this, std::placeholders::_1));
    webrtc_client_->setControlCommandCallback(
        std::bind(&VehicleNode::onControlCommand, this, std::placeholders::_1));
    webrtc_client_->setLatencyUpdateCallback(
        std::bind(&VehicleNode::onLatencyUpdate, this, std::placeholders::_1));

    FSM_LOG_INFO("VehicleNode initialized successfully");
    return true;
}

void VehicleNode::start() {
    if (running_) {
        FSM_LOG_WARN("VehicleNode already running");
        return;
    }

    running_ = true;

    // 启动数据采集
    data_collector_->start();

    // 启动指令执行器
    command_executor_->start();

    // 连接WebRTC
    if (webrtc_client_->connect()) {
        FSM_LOG_INFO("WebRTC connection initiated");
    } else {
        FSM_LOG_WARN("Failed to initiate WebRTC connection");
    }

    // 启动遥测数据发送定时器
    int telemetry_rate = this->get_parameter("telemetry_rate_hz").as_int();
    int period_ms = 1000 / telemetry_rate;
    telemetry_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&VehicleNode::telemetryTimerCallback, this));

    // 启动健康检查定时器
    int health_check_ms = this->get_parameter("health_check_interval_ms").as_int();
    health_check_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(health_check_ms),
        std::bind(&VehicleNode::healthCheckCallback, this));

    FSM_LOG_INFO("VehicleNode started");
}

void VehicleNode::stop() {
    if (!running_) {
        return;
    }

    running_ = false;

    // 停止定时器
    if (telemetry_timer_) {
        telemetry_timer_->cancel();
        telemetry_timer_.reset();
    }

    if (health_check_timer_) {
        health_check_timer_->cancel();
        health_check_timer_.reset();
    }

    // 断开WebRTC
    if (webrtc_client_) {
        webrtc_client_->disconnect();
    }

    // 停止指令执行器
    if (command_executor_) {
        command_executor_->stop();
    }

    // 停止数据采集
    if (data_collector_) {
        data_collector_->stop();
    }

    FSM_LOG_INFO("VehicleNode stopped");
}

WebRTCState VehicleNode::getConnectionState() const {
    if (webrtc_client_) {
        return webrtc_client_->getState();
    }
    return WebRTCState::DISCONNECTED;
}

LatencyInfo VehicleNode::getLatencyInfo() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return cached_latency_info_;
}

void VehicleNode::onVehicleStateUpdate(const VehicleState& state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cached_vehicle_state_ = state;
}

void VehicleNode::onSystemStateUpdate(const SystemState& state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cached_system_state_ = state;
}

void VehicleNode::onCameraFrame(const CameraFrame& frame) {
    if (!webrtc_client_ || !webrtc_client_->isConnected()) {
        return;
    }

    // 发送视频帧
    webrtc_client_->sendVideoFrame(frame.camera_id, frame.image, frame.timestamp_ns);
    ++frames_sent_count_;
}

void VehicleNode::onConnectionStateChange(WebRTCState state) {
    connected_ = (state == WebRTCState::CONNECTED);

    const char* state_str = "UNKNOWN";
    switch (state) {
        case WebRTCState::DISCONNECTED: state_str = "DISCONNECTED"; break;
        case WebRTCState::CONNECTING: state_str = "CONNECTING"; break;
        case WebRTCState::CONNECTED: state_str = "CONNECTED"; break;
        case WebRTCState::RECONNECTING: state_str = "RECONNECTING"; break;
        case WebRTCState::FAILED: state_str = "FAILED"; break;
    }

    FSM_LOG_INFO("WebRTC connection state changed: {}", state_str);

    if (connected_) {
        // 连接成功，启用远程控制
        command_executor_->setRemoteControlMode();
    } else {
        // 断开连接，禁用远程控制
        command_executor_->exitRemoteControlMode();
    }
}

void VehicleNode::onControlCommand(const ControlCommand& cmd) {
    if (!running_) {
        return;
    }

    // 执行控制指令
    command_executor_->executeCommand(cmd);
    ++commands_received_count_;
}

void VehicleNode::onLatencyUpdate(const LatencyInfo& info) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cached_latency_info_ = info;
}

void VehicleNode::telemetryTimerCallback() {
    if (!webrtc_client_ || !webrtc_client_->isConnected()) {
        return;
    }

    // 获取当前状态
    VehicleState vehicle_state;
    SystemState system_state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        vehicle_state = cached_vehicle_state_;
        system_state = cached_system_state_;
    }

    // 序列化并发送遥测数据
    auto telemetry_data = serializeVehicleState(vehicle_state);
    webrtc_client_->sendTelemetry(telemetry_data);

    // 定期发送系统状态
    static int system_state_counter = 0;
    if (++system_state_counter >= 10) {  // 每10次发送一次系统状态
        auto system_data = serializeSystemState(system_state);
        webrtc_client_->sendSystemStatus(system_data);
        system_state_counter = 0;
    }

    ++telemetry_sent_count_;
}

void VehicleNode::healthCheckCallback() {
    // 检查数据采集器状态
    bool collector_healthy = data_collector_->isHealthy();

    // 检查WebRTC连接状态
    bool connection_healthy = webrtc_client_ &&
                              (webrtc_client_->getState() == WebRTCState::CONNECTED ||
                               webrtc_client_->getState() == WebRTCState::CONNECTING);

    // 检查指令执行器状态
    bool executor_healthy = !command_executor_->isEmergencyActive();

    if (!collector_healthy) {
        FSM_LOG_WARN("Data collector unhealthy");
    }

    if (!connection_healthy) {
        FSM_LOG_WARN("WebRTC connection unhealthy");
    }

    // 获取延迟信息
    auto latency = webrtc_client_ ? webrtc_client_->getLatencyInfo() : LatencyInfo{};

    FSM_LOG_DEBUG("Health check: collector={}, connection={}, executor={}, rtt={}ms",
                  collector_healthy, connection_healthy, executor_healthy,
                  latency.rtt_ms);

    // 统计信息
    FSM_LOG_DEBUG("Stats: telemetry_sent={}, commands_received={}, frames_sent={}",
                  telemetry_sent_count_, commands_received_count_, frames_sent_count_);
}

std::vector<uint8_t> VehicleNode::serializeVehicleState(const VehicleState& state) {
    fsm::proto::TelemetryData proto;

    // 设置时间戳
    auto* stamp = proto.mutable_stamp();
    stamp->set_seconds(state.timestamp_ns / 1000000000LL);
    stamp->set_nanos(state.timestamp_ns % 1000000000LL);

    proto.set_vehicle_id(config_manager_.getVehicleId());

    // 车辆状态
    auto* vehicle = proto.mutable_vehicle_status();
    vehicle->mutable_stamp()->CopyFrom(*stamp);
    vehicle->set_vehicle_id(config_manager_.getVehicleId());

    auto* velocity = vehicle->mutable_velocity();
    velocity->set_longitudinal_velocity(state.longitudinal_velocity);
    velocity->set_lateral_velocity(state.lateral_velocity);
    velocity->set_heading_rate(state.heading_rate);

    auto* steering = vehicle->mutable_steering();
    steering->set_steering_tire_angle(state.steering_tire_angle);

    vehicle->set_gear(static_cast<fsm::proto::GearPosition>(state.gear));
    vehicle->set_mode(static_cast<fsm::proto::VehicleMode>(state.control_mode));

    // 定位
    auto* loc = proto.mutable_localization();
    auto* pose = loc->mutable_pose();
    pose->mutable_position()->set_x(state.pose_x);
    pose->mutable_position()->set_y(state.pose_y);
    pose->mutable_position()->set_z(state.pose_z);
    pose->mutable_orientation()->set_x(state.orientation_x);
    pose->mutable_orientation()->set_y(state.orientation_y);
    pose->mutable_orientation()->set_z(state.orientation_z);
    pose->mutable_orientation()->set_w(state.orientation_w);

    auto* geo = loc->mutable_geo_point();
    geo->set_latitude(state.latitude);
    geo->set_longitude(state.longitude);
    geo->set_altitude(state.altitude);

    // 序列化
    std::string serialized;
    proto.SerializeToString(&serialized);

    return std::vector<uint8_t>(serialized.begin(), serialized.end());
}

std::vector<uint8_t> VehicleNode::serializeSystemState(const SystemState& state) {
    fsm::proto::SystemStatus proto;

    auto* stamp = proto.mutable_stamp();
    stamp->set_seconds(state.timestamp_ns / 1000000000LL);
    stamp->set_nanos(state.timestamp_ns % 1000000000LL);

    proto.set_vehicle_id(config_manager_.getVehicleId());
    proto.set_cpu_usage(state.cpu_usage);
    proto.set_memory_usage(state.memory_usage);
    proto.set_gpu_usage(state.gpu_usage);
    proto.set_disk_usage(state.disk_usage);
    proto.set_network_tx_bytes(state.network_tx_bps);
    proto.set_network_rx_bytes(state.network_rx_bps);

    for (const auto& diag : state.diagnostics) {
        auto* d = proto.add_diagnostics();
        d->set_name(diag.first);
        d->set_level(static_cast<fsm::proto::DiagnosticLevel>(diag.second));
    }

    std::string serialized;
    proto.SerializeToString(&serialized);

    return std::vector<uint8_t>(serialized.begin(), serialized.end());
}

}  // namespace vehicle
}  // namespace fsm

// Main函数
int main(int argc, char* argv[]) {
    // 初始化日志
    fsm::Logger::init("fsm_vehicle", "/tmp/fsm_vehicle.log", fsm::Logger::Level::INFO);

    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<fsm::vehicle::VehicleNode>();

    // 获取配置文件路径
    std::string config_path;
    if (argc > 1) {
        config_path = argv[1];
    } else {
        config_path = "config/vehicle_config.yaml";
    }

    // 初始化
    if (!node->initialize(config_path)) {
        FSM_LOG_ERROR("Failed to initialize VehicleNode");
        return 1;
    }

    // 启动
    node->start();

    // 运行
    FSM_LOG_INFO("FSM Vehicle Node running...");
    rclcpp::spin(node);

    // 清理
    node->stop();
    rclcpp::shutdown();
    fsm::Logger::shutdown();

    return 0;
}
