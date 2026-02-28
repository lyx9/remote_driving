#include "fsm/vehicle/vehicle_node.hpp"
#include "fsm/utils.hpp"
#include "fsm/build_mode.hpp"

#include "fsm_messages.pb.h"
#include <nlohmann/json.hpp>
#include <algorithm>

namespace fsm {
namespace vehicle {

// ── Constructor / Destructor ─────────────────────────────────────────────────

VehicleNode::VehicleNode(const rclcpp::NodeOptions& options)
    : Node("fsm_vehicle_node", options),
      safety_monitor_(500U),   // 500 ms watchdog deadline (ASIL-B)
      diagnostics_(64U) {
  this->declare_parameter<std::string>("config_file", "");
  this->declare_parameter<int>("telemetry_rate_hz", 20);
  this->declare_parameter<int>("health_check_interval_ms", 5000);

  FSM_LOG_INFO("VehicleNode ctor — build_variant={}", fsm::kBuildVariant);
}

VehicleNode::~VehicleNode() {
  Stop();
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

bool VehicleNode::Initialize(const std::string& config_path) {
  std::string path = config_path;
  if (path.empty()) {
    path = this->get_parameter("config_file").as_string();
  }
  if (path.empty()) {
    FSM_LOG_ERROR("No config file specified");
    return false;
  }
  if (!config_manager_.LoadFromFile(path)) {
    FSM_LOG_ERROR("Failed to load config: {}", path);
    return false;
  }

  FSM_LOG_INFO("Initializing VehicleNode: {} (build={})",
               config_manager_.vehicle_id(), fsm::kBuildVariant);

  // ── [VN-01] Safety monitor is first up ────────────────────────────────────
  // Fail-safe action: triggers emergency stop on ASIL-B fault.
  safety_monitor_.RegisterFailSafeAction([this]() {
    if (command_executor_) {
      command_executor_->TriggerEmergencyStop("SafetyMonitor fail-safe");
    }
  });

  // State change callback → update diagnostics + log.
  safety_monitor_.set_state_change_callback(
      [this](SafetyState new_state, const FaultEvent& evt) {
        const char* state_str = "UNKNOWN";
        switch (new_state) {
          case SafetyState::kNominal:  state_str = "NOMINAL";   break;
          case SafetyState::kDegraded: state_str = "DEGRADED";  break;
          case SafetyState::kFault:    state_str = "FAULT";     break;
          case SafetyState::kFailSafe: state_str = "FAIL-SAFE"; break;
          default:                     state_str = "UNKNOWN";   break;
        }
        diagnostics_.UpdateStr(
            "safety.state", "Safety State", state_str,
            (new_state >= SafetyState::kFault)
                ? DiagnosticStatus::kError
                : (new_state == SafetyState::kDegraded)
                    ? DiagnosticStatus::kWarn
                    : DiagnosticStatus::kOk,
            "safety_monitor");
        diagnostics_.UpdateStr(
            "safety.last_fault", "Last Fault", evt.description,
            DiagnosticStatus::kWarn, "safety_monitor");
      });

  // ── Data collector ────────────────────────────────────────────────────────
  data_collector_ = std::make_unique<DataCollector>(
      shared_from_this(), config_manager_);
  data_collector_->set_vehicle_state_callback(
      std::bind(&VehicleNode::OnVehicleStateUpdate, this,
                std::placeholders::_1));
  data_collector_->set_system_state_callback(
      std::bind(&VehicleNode::OnSystemStateUpdate, this,
                std::placeholders::_1));
  data_collector_->set_camera_frame_callback(
      std::bind(&VehicleNode::OnCameraFrame, this, std::placeholders::_1));

  // ── Command executor (receives SafetyMonitor* for fault reporting) ────────
  command_executor_ = std::make_unique<CommandExecutor>(
      shared_from_this(), config_manager_, &safety_monitor_);

  // ── WebRTC client ─────────────────────────────────────────────────────────
  const auto& rtc = config_manager_.webrtc_config();
  WebRtcClientConfig rtc_cfg;
  rtc_cfg.vehicle_id    = config_manager_.vehicle_id();
  rtc_cfg.signaling_url = rtc.signaling_url;
  rtc_cfg.stun_servers  = rtc.stun_servers;
  rtc_cfg.turn_servers  = rtc.turn_servers;
  rtc_cfg.turn_username = rtc.turn_username;
  rtc_cfg.turn_password = rtc.turn_password;

  const auto& vp = config_manager_.vehicle_params();
  rtc_cfg.max_steering_angle_deg =
      static_cast<float>(vp.max_steering_angle_deg);
  rtc_cfg.max_speed_kph  = static_cast<float>(vp.max_speed_kph);
  rtc_cfg.max_accel_mps2 = static_cast<float>(vp.max_acceleration_mps2);
  rtc_cfg.max_decel_mps2 = static_cast<float>(vp.max_deceleration_mps2);

  webrtc_client_ = std::make_unique<WebRtcClient>(rtc_cfg);
  webrtc_client_->set_connection_state_callback(
      std::bind(&VehicleNode::OnConnectionStateChange, this,
                std::placeholders::_1));
  webrtc_client_->set_control_command_callback(
      std::bind(&VehicleNode::OnControlCommand, this,
                std::placeholders::_1));
  webrtc_client_->set_latency_update_callback(
      std::bind(&VehicleNode::OnLatencyUpdate, this,
                std::placeholders::_1));

  // ── Optional MQTT transport ────────────────────────────────────────────────
  const auto& mqtt_cfg = config_manager_.mqtt_config();
  if (mqtt_cfg.enabled) {
    mqtt_bridge_ = std::make_unique<MqttBridge>(
        mqtt_cfg, config_manager_.vehicle_id());
    mqtt_bridge_->set_control_callback(
        [this](const std::string& payload) {
          try {
            auto j = nlohmann::json::parse(payload);
            const auto& d = j.contains("data") ? j.at("data") : j;

            ControlCommand cmd;
            static constexpr float kDegToRad = 3.14159265F / 180.0F;
            const float steer = d.value("steering", 0.0F);
            const float thr   = d.value("throttle", 0.0F);
            const float brk   = d.value("brake",    0.0F);
            const float max_steer_rad =
                static_cast<float>(
                    config_manager_.vehicle_params().max_steering_angle_deg)
                * kDegToRad;

            cmd.steering_tire_angle = std::clamp(steer, -1.0F, 1.0F) *
                                      max_steer_rad;
            if (brk > 0.01F) {
              cmd.speed = 0.0F;
              cmd.acceleration = -brk * static_cast<float>(
                  config_manager_.vehicle_params().max_deceleration_mps2);
            } else {
              cmd.speed = thr * static_cast<float>(
                  config_manager_.vehicle_params().max_speed_kph) / 3.6F;
              cmd.acceleration = thr * static_cast<float>(
                  config_manager_.vehicle_params().max_acceleration_mps2);
            }
            cmd.emergency_stop  = d.value("emergency",   false);
            cmd.sequence_number = d.value("sequence",    uint64_t{0U});
            cmd.gear            = d.value("gear",        int32_t{3});
            cmd.turn_signal     = d.value("turn_signal", int32_t{0});
            OnControlCommand(cmd);
          } catch (const std::exception& e) {
            FSM_LOG_ERROR("MQTT control parse error: {}", e.what());
          }
        });
    FSM_LOG_INFO("MQTT transport enabled");
  }

  // Initial diagnostics state.
  diagnostics_.UpdateStr("safety.state", "Safety State",
                          "NOMINAL", DiagnosticStatus::kOk, "safety_monitor");
  diagnostics_.UpdateStr("vehicle.id", "Vehicle ID",
                          config_manager_.vehicle_id(),
                          DiagnosticStatus::kOk, "vehicle");
  diagnostics_.UpdateStr("build.variant", "Build Variant",
                          fsm::kBuildVariant,
                          DiagnosticStatus::kOk, "build");

  FSM_LOG_INFO("VehicleNode initialized");
  return true;
}

void VehicleNode::Start() {
  if (running_.exchange(true, std::memory_order_acq_rel)) {
    FSM_LOG_WARN("VehicleNode already running");
    return;
  }

  // [VN-01] Safety monitor starts before subsystems.
  safety_monitor_.Start();

  data_collector_->Start();
  command_executor_->Start();

  if (!webrtc_client_->Connect()) {
    FSM_LOG_WARN("Failed to initiate WebRTC connection");
    diagnostics_.UpdateStr("webrtc.state", "WebRTC State",
                            "CONNECTING_FAILED",
                            DiagnosticStatus::kWarn, "webrtc");
  }

  if (mqtt_bridge_) {
    if (!mqtt_bridge_->Start()) {
      FSM_LOG_WARN("MQTT bridge failed to start; continuing without MQTT");
      diagnostics_.UpdateStr("mqtt.state", "MQTT State",
                              "START_FAILED",
                              DiagnosticStatus::kWarn, "mqtt");
    }
  }

  const int period_ms =
      1000 / std::max(1, this->get_parameter("telemetry_rate_hz").as_int());
  telemetry_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&VehicleNode::TelemetryTimerCallback, this));

  health_check_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(
          this->get_parameter("health_check_interval_ms").as_int()),
      std::bind(&VehicleNode::HealthCheckCallback, this));

  FSM_LOG_INFO("VehicleNode started");
}

void VehicleNode::Stop() {
  if (!running_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }

  if (telemetry_timer_)    { telemetry_timer_->cancel();    telemetry_timer_.reset(); }
  if (health_check_timer_) { health_check_timer_->cancel(); health_check_timer_.reset(); }

  if (webrtc_client_)    webrtc_client_->Disconnect();
  if (mqtt_bridge_)      mqtt_bridge_->Stop();
  if (command_executor_) command_executor_->Stop();
  if (data_collector_)   data_collector_->Stop();

  // [VN-01] Safety monitor stops last (reverse order of Start()).
  safety_monitor_.Stop();

  FSM_LOG_INFO("VehicleNode stopped");
}

// ── Accessors ─────────────────────────────────────────────────────────────────

bool VehicleNode::is_running() const noexcept {
  return running_.load(std::memory_order_acquire);
}

const std::string& VehicleNode::vehicle_id() const noexcept {
  return config_manager_.vehicle_id();
}

WebRtcState VehicleNode::connection_state() const {
  return webrtc_client_
      ? webrtc_client_->state()
      : WebRtcState::kDisconnected;
}

LatencyInfo VehicleNode::latency_info() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return cached_latency_info_;
}

SafetyState VehicleNode::safety_state() const noexcept {
  return safety_monitor_.state();
}

nlohmann::json VehicleNode::GetDiagnosticsJson() const {
  return diagnostics_.ToJson();
}

// ── Subsystem callbacks ────────────────────────────────────────────────────────

void VehicleNode::OnVehicleStateUpdate(const VehicleState& state) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  cached_vehicle_state_ = state;
}

void VehicleNode::OnSystemStateUpdate(const SystemState& state) {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cached_system_state_ = state;
  }

  // Update diagnostics from system state.
  auto status = [](double val, double warn_thresh, double error_thresh)
      -> DiagnosticStatus {
    if (val >= error_thresh) return DiagnosticStatus::kError;
    if (val >= warn_thresh)  return DiagnosticStatus::kWarn;
    return DiagnosticStatus::kOk;
  };

  diagnostics_.Update("sys.cpu_usage",    "CPU Usage",
                       static_cast<double>(state.cpu_usage),    "%",
                       status(state.cpu_usage,    85.0, 95.0), "system");
  diagnostics_.Update("sys.memory_usage", "Memory Usage",
                       static_cast<double>(state.memory_usage), "%",
                       status(state.memory_usage, 80.0, 90.0), "system");
  diagnostics_.Update("sys.gpu_usage",    "GPU Usage",
                       static_cast<double>(state.gpu_usage),    "%",
                       status(state.gpu_usage,    80.0, 95.0), "system");
  diagnostics_.Update("sys.disk_usage",   "Disk Usage",
                       static_cast<double>(state.disk_usage),   "%",
                       status(state.disk_usage,   80.0, 95.0), "system");
}

void VehicleNode::OnCameraFrame(const CameraFrame& frame) {
  if (!webrtc_client_ || !webrtc_client_->is_connected()) {
    return;
  }
  webrtc_client_->SendVideoFrame(
      frame.camera_id, frame.image, frame.timestamp_ns);
  frames_sent_count_.fetch_add(1U, std::memory_order_relaxed);
}

void VehicleNode::OnConnectionStateChange(WebRtcState state) {
  connected_.store(state == WebRtcState::kConnected,
                   std::memory_order_release);

  const char* s = "UNKNOWN";
  DiagnosticStatus diag_status = DiagnosticStatus::kOk;
  switch (state) {
    case WebRtcState::kDisconnected:
      s = "DISCONNECTED";
      diag_status = DiagnosticStatus::kWarn;
      break;
    case WebRtcState::kConnecting:
      s = "CONNECTING";
      diag_status = DiagnosticStatus::kWarn;
      break;
    case WebRtcState::kConnected:
      s = "CONNECTED";
      diag_status = DiagnosticStatus::kOk;
      safety_monitor_.Feed();
      break;
    case WebRtcState::kReconnecting:
      s = "RECONNECTING";
      diag_status = DiagnosticStatus::kWarn;
      break;
    case WebRtcState::kFailed:
      s = "FAILED";
      diag_status = DiagnosticStatus::kError;
      ReportSubsystemFault(FaultCode::kCommLinkLost, AsilLevel::kAsilB,
                            "webrtc", "WebRTC connection failed");
      break;
    default:
      s = "UNKNOWN";
      break;
  }
  FSM_LOG_INFO("WebRTC state: {}", s);
  diagnostics_.UpdateStr("webrtc.state", "WebRTC State",
                          s, diag_status, "webrtc");

  if (connected_.load(std::memory_order_acquire)) {
    command_executor_->SetRemoteControlMode();
  } else {
    command_executor_->ExitRemoteControlMode();
  }
}

void VehicleNode::OnControlCommand(const ControlCommand& cmd) {
  if (!running_.load(std::memory_order_acquire)) {
    return;
  }
  // Feed safety watchdog on every received command — [SM-01].
  safety_monitor_.Feed();

  if (!command_executor_->Execute(cmd)) {
    FSM_LOG_WARN("Control command rejected (seq={})", cmd.sequence_number);
  }
  commands_received_count_.fetch_add(1U, std::memory_order_relaxed);
}

void VehicleNode::OnLatencyUpdate(const LatencyInfo& info) {
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    cached_latency_info_ = info;
  }

  const double rtt = static_cast<double>(info.rtt_ms);
  auto lat_status = [](double ms) -> DiagnosticStatus {
    if (ms > 500.0) return DiagnosticStatus::kError;
    if (ms > 200.0) return DiagnosticStatus::kWarn;
    return DiagnosticStatus::kOk;
  };

  diagnostics_.Update("webrtc.rtt_ms",         "WebRTC RTT",
                       rtt, "ms", lat_status(rtt), "webrtc");
  diagnostics_.Update("webrtc.packet_loss_pct", "Packet Loss",
                       static_cast<double>(info.packet_loss_rate) * 100.0,
                       "%",
                       (info.packet_loss_rate > 0.05F)
                           ? DiagnosticStatus::kWarn
                           : DiagnosticStatus::kOk,
                       "webrtc");

  static constexpr double kCriticalLatencyMs = 500.0;
  if (rtt > kCriticalLatencyMs) {
    ReportSubsystemFault(FaultCode::kCommLatencyExceeded, AsilLevel::kAsilA,
                          "webrtc", "RTT latency exceeds 500ms safety threshold");
  }
}

// ── Timer callbacks ───────────────────────────────────────────────────────────

void VehicleNode::TelemetryTimerCallback() {
  if (!webrtc_client_ || !webrtc_client_->is_connected()) {
    return;
  }

  VehicleState vehicle_state;
  SystemState  system_state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    vehicle_state = cached_vehicle_state_;
    system_state  = cached_system_state_;
  }

  const auto telem_bytes = SerializeVehicleState(vehicle_state);
  webrtc_client_->SendTelemetry(telem_bytes);
  if (mqtt_bridge_) {
    mqtt_bridge_->PublishTelemetry(telem_bytes);
  }

  static int sys_counter = 0;
  ++sys_counter;
  if (sys_counter >= 10) {
    const auto sys_bytes = SerializeSystemState(system_state);
    webrtc_client_->SendSystemStatus(sys_bytes);
    if (mqtt_bridge_) {
      mqtt_bridge_->PublishStatus(sys_bytes);
    }
    sys_counter = 0;
  }

  telemetry_sent_count_.fetch_add(1U, std::memory_order_relaxed);
}

void VehicleNode::HealthCheckCallback() {
  const bool collector_ok  = data_collector_->IsHealthy();
  const bool executor_ok   = !command_executor_->is_emergency_active();
  const bool safety_ok     = safety_monitor_.is_remote_control_permitted();

  diagnostics_.UpdateStr(
      "sensor.health", "Sensor Health",
      collector_ok ? "ok" : "unhealthy",
      collector_ok ? DiagnosticStatus::kOk : DiagnosticStatus::kError,
      "sensor");
  diagnostics_.UpdateStr(
      "control.emergency", "Emergency Stop",
      executor_ok ? "inactive" : "ACTIVE",
      executor_ok ? DiagnosticStatus::kOk : DiagnosticStatus::kError,
      "control");
  diagnostics_.Update(
      "safety.watchdog_ms", "Watchdog Age",
      static_cast<double>(safety_monitor_.ms_since_last_feed()),
      "ms",
      safety_ok ? DiagnosticStatus::kOk : DiagnosticStatus::kError,
      "safety_monitor");

  if (!collector_ok) {
    ReportSubsystemFault(FaultCode::kSensorDataStale, AsilLevel::kAsilA,
                          "data_collector",
                          "Vehicle sensor data is stale or unavailable");
  }

  const uint64_t telem  = telemetry_sent_count_.load(std::memory_order_relaxed);
  const uint64_t cmds   = commands_received_count_.load(std::memory_order_relaxed);
  const uint64_t frames = frames_sent_count_.load(std::memory_order_relaxed);

  const auto lat = webrtc_client_
      ? webrtc_client_->latency_info()
      : LatencyInfo{};

  FSM_LOG_DEBUG(
      "Health: sensor={} executor_ok={} safety={} rtt={:.0f}ms "
      "telem={} cmds={} frames={}",
      collector_ok, executor_ok, safety_ok,
      static_cast<double>(lat.rtt_ms), telem, cmds, frames);

  diagnostics_.Update("stats.telemetry_sent",    "Telemetry Sent",
                       static_cast<double>(telem),   "frames",
                       DiagnosticStatus::kOk, "stats");
  diagnostics_.Update("stats.commands_received", "Commands Received",
                       static_cast<double>(cmds),    "cmds",
                       DiagnosticStatus::kOk, "stats");
  diagnostics_.Update("stats.frames_sent",       "Frames Sent",
                       static_cast<double>(frames),  "frames",
                       DiagnosticStatus::kOk, "stats");
  diagnostics_.Update("stats.missed_commands",   "Missed Commands",
                       static_cast<double>(command_executor_->missed_commands()),
                       "cmds",
                       (command_executor_->missed_commands() > 10U)
                           ? DiagnosticStatus::kWarn
                           : DiagnosticStatus::kOk,
                       "stats");
}

// ── Serialisation ─────────────────────────────────────────────────────────────

std::vector<uint8_t> VehicleNode::SerializeVehicleState(
    const VehicleState& state) const {
  fsm::proto::TelemetryData proto;

  auto* stamp = proto.mutable_stamp();
  stamp->set_seconds(state.timestamp_ns / 1'000'000'000LL);
  stamp->set_nanos(static_cast<int32_t>(state.timestamp_ns % 1'000'000'000LL));
  proto.set_vehicle_id(config_manager_.vehicle_id());

  auto* vehicle = proto.mutable_vehicle_status();
  vehicle->mutable_stamp()->CopyFrom(*stamp);
  vehicle->set_vehicle_id(config_manager_.vehicle_id());

  auto* vel = vehicle->mutable_velocity();
  vel->set_longitudinal_velocity(state.longitudinal_velocity);
  vel->set_lateral_velocity(state.lateral_velocity);
  vel->set_heading_rate(state.heading_rate);

  vehicle->mutable_steering()->set_steering_tire_angle(
      state.steering_tire_angle);
  vehicle->set_gear(
      static_cast<fsm::proto::GearPosition>(state.gear));
  vehicle->set_mode(
      static_cast<fsm::proto::VehicleMode>(state.control_mode));

  auto* loc  = proto.mutable_localization();
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

  std::string buf;
  proto.SerializeToString(&buf);
  return std::vector<uint8_t>(buf.begin(), buf.end());
}

std::vector<uint8_t> VehicleNode::SerializeSystemState(
    const SystemState& state) const {
  fsm::proto::SystemStatus proto;

  auto* stamp = proto.mutable_stamp();
  stamp->set_seconds(state.timestamp_ns / 1'000'000'000LL);
  stamp->set_nanos(static_cast<int32_t>(state.timestamp_ns % 1'000'000'000LL));
  proto.set_vehicle_id(config_manager_.vehicle_id());
  proto.set_cpu_usage(state.cpu_usage);
  proto.set_memory_usage(state.memory_usage);
  proto.set_gpu_usage(state.gpu_usage);
  proto.set_disk_usage(state.disk_usage);
  proto.set_network_tx_bytes(state.network_tx_bps);
  proto.set_network_rx_bytes(state.network_rx_bps);

  for (const auto& [name, level] : state.diagnostics) {
    auto* d = proto.add_diagnostics();
    d->set_name(name);
    d->set_level(static_cast<fsm::proto::DiagnosticLevel>(level));
  }

  std::string buf;
  proto.SerializeToString(&buf);
  return std::vector<uint8_t>(buf.begin(), buf.end());
}

// ── Safety helpers ────────────────────────────────────────────────────────────

void VehicleNode::ReportSubsystemFault(FaultCode          code,
                                        AsilLevel          level,
                                        const std::string& subsystem,
                                        const std::string& description) {
  safety_monitor_.ReportFault(code, level, subsystem, description);
  diagnostics_.UpdateStr(
      "safety.last_fault", "Last Fault",
      description, DiagnosticStatus::kWarn, "safety_monitor");
}

}  // namespace vehicle
}  // namespace fsm

// ── main() ────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
  fsm::Logger::Init("fsm_vehicle", "/tmp/fsm_vehicle.log",
                    fsm::Logger::Level::kInfo);
  rclcpp::init(argc, argv);

  FSM_LOG_INFO("FSM Vehicle Node — build_variant={} version={}",
               fsm::kBuildVariant, fsm::kFsmVersion);

  auto node = std::make_shared<fsm::vehicle::VehicleNode>();
  const std::string config_path =
      (argc > 1) ? argv[1] : "config/vehicle_config.yaml";

  if (!node->Initialize(config_path)) {
    FSM_LOG_CRITICAL("Failed to initialize VehicleNode — aborting");
    return 1;
  }

  node->Start();
  FSM_LOG_INFO("FSM Vehicle Node running (safety_state=NOMINAL)...");
  rclcpp::spin(node);

  node->Stop();
  rclcpp::shutdown();
  fsm::Logger::Shutdown();
  return 0;
}
