#include "fsm/vehicle/vehicle_node.hpp"
#include "fsm/utils.hpp"
#include "fsm/build_mode.hpp"

#include "fsm_messages.pb.h"
#include <nlohmann/json.hpp>
#include <algorithm>

namespace fsm {
namespace vehicle {

// ── Constructor ────────────────────────────────────────────────────────────

VehicleNode::VehicleNode(const rclcpp::NodeOptions& options)
    : Node("fsm_vehicle_node", options),
      safety_monitor_(500U),
      diagnostics_(64U) {
  this->declare_parameter<std::string>("config_file", "");
  this->declare_parameter<int>("telemetry_rate_hz", 20);
  this->declare_parameter<int>("health_check_interval_ms", 5000);
  FSM_LOG_INFO("VehicleNode — build={}", fsm::kBuildVariant);
}

VehicleNode::~VehicleNode() { Stop(); }

// ── Initialize ────────────────────────────────────────────────────────────

bool VehicleNode::Initialize(const std::string& config_path) {
  std::string path = config_path;
  if (path.empty()) path = this->get_parameter("config_file").as_string();
  if (path.empty()) { FSM_LOG_ERROR("No config file"); return false; }
  if (!config_manager_.LoadFromFile(path)) {
    FSM_LOG_ERROR("Config load failed: {}", path); return false;
  }

  FSM_LOG_INFO("Init VehicleNode id={} build={}", config_manager_.vehicle_id(),
               fsm::kBuildVariant);

  // ── [VN-01] Safety monitor first ─────────────────────────────────────────
  safety_monitor_.RegisterFailSafeAction([this]() {
    if (command_executor_) {
      command_executor_->TriggerEmergencyStop("SafetyMonitor fail-safe");
    }
  });
  safety_monitor_.set_state_change_callback(
      [this](SafetyState s, const FaultEvent& e) {
        static const char* kNames[] = {"NOMINAL","DEGRADED","FAULT","FAIL-SAFE"};
        const uint8_t idx = static_cast<uint8_t>(s);
        const char* name = (idx < 4U) ? kNames[idx] : "UNKNOWN";
        diagnostics_.UpdateStr("safety.state", "Safety State", name,
            (s >= SafetyState::kFault) ? DiagnosticStatus::kError
            : (s == SafetyState::kDegraded) ? DiagnosticStatus::kWarn
            : DiagnosticStatus::kOk, "safety");
        diagnostics_.UpdateStr("safety.last_fault", "Last Fault",
            e.description, DiagnosticStatus::kWarn, "safety");
      });

  // ── Sensor/vehicle-state source (compile-time selection) ──────────────────
#ifdef FSM_DEMO_MODE
  {
    demo::SimulationConfig sim_cfg;
    sim_cfg.start_latitude  = 30.5728;   // 成都 (demo GPS)
    sim_cfg.start_longitude = 104.0668;
    sim_cfg.cruise_speed_mps = 3.0F;
    sim_cfg.update_rate_hz   = 20;
    sim_cfg.scenario = demo::DemoScenario::kStraightLine;

    sim_vehicle_ = std::make_unique<demo::SimulatedVehicle>(sim_cfg);
    // Bridge sim status → cached_vehicle_state_
    sim_vehicle_->set_status_callback(
        [this](const NormalizedVehicleStatus& st) {
          VehicleState vs;
          vs.longitudinal_velocity = st.speed_mps;
          vs.steering_tire_angle   = st.steering_rad;
          vs.gear                  = st.gear;
          vs.latitude              = st.latitude;
          vs.longitude             = st.longitude;
          vs.altitude              = 500.0f;
          vs.heading_rate          = 0.0f;
          vs.timestamp_ns          = st.timestamp_ns;
          vs.localization_valid    = true;
          std::lock_guard<std::mutex> lk(state_mutex_);
          cached_vehicle_state_ = vs;
        });
    FSM_LOG_INFO("Demo mode: SimulatedVehicle active (scenario=straight_line)");
  }
#else
  {
    data_collector_ = std::make_unique<DataCollector>(
        shared_from_this(), config_manager_);
    data_collector_->set_vehicle_state_callback(
        [this](const VehicleState& s) {
          std::lock_guard<std::mutex> lk(state_mutex_);
          cached_vehicle_state_ = s;
        });
    data_collector_->set_system_state_callback(
        [this](const SystemState& s) {
          {
            std::lock_guard<std::mutex> lk(state_mutex_);
            cached_system_state_ = s;
          }
          auto st = [](double v, double w, double e) -> DiagnosticStatus {
            return (v >= e) ? DiagnosticStatus::kError
                 : (v >= w) ? DiagnosticStatus::kWarn
                 : DiagnosticStatus::kOk;
          };
          diagnostics_.Update("sys.cpu",    "CPU",    s.cpu_usage,    "%", st(s.cpu_usage,    85,95), "sys");
          diagnostics_.Update("sys.memory", "Memory", s.memory_usage, "%", st(s.memory_usage, 80,90), "sys");
          diagnostics_.Update("sys.disk",   "Disk",   s.disk_usage,   "%", st(s.disk_usage,   80,95), "sys");
        });
    data_collector_->set_camera_frame_callback(
        [this](const CameraFrame& f) {
          if (webrtc_client_ && webrtc_client_->is_connected()) {
            webrtc_client_->SendVideoFrame(f.camera_id, f.image, f.timestamp_ns);
            frames_sent_count_.fetch_add(1U, std::memory_order_relaxed);
          }
        });
    FSM_LOG_INFO("Production mode: DataCollector active");
  }
#endif

  // ── Command executor ──────────────────────────────────────────────────────
  command_executor_ = std::make_unique<CommandExecutor>(
      shared_from_this(), config_manager_, &safety_monitor_);

  // ── WebRTC client ─────────────────────────────────────────────────────────
  const auto& rtc = config_manager_.webrtc_config();
  const auto& vp  = config_manager_.vehicle_params();
  WebRtcClientConfig rtc_cfg;
  rtc_cfg.vehicle_id             = config_manager_.vehicle_id();
  rtc_cfg.signaling_url          = rtc.signaling_url;
  rtc_cfg.stun_servers           = rtc.stun_servers;
  rtc_cfg.turn_servers           = rtc.turn_servers;
  rtc_cfg.turn_username          = rtc.turn_username;
  rtc_cfg.turn_password          = rtc.turn_password;
  rtc_cfg.max_steering_angle_deg = static_cast<float>(vp.max_steering_angle_deg);
  rtc_cfg.max_speed_kph          = static_cast<float>(vp.max_speed_kph);
  rtc_cfg.max_accel_mps2         = static_cast<float>(vp.max_acceleration_mps2);
  rtc_cfg.max_decel_mps2         = static_cast<float>(vp.max_deceleration_mps2);

  webrtc_client_ = std::make_unique<WebRtcClient>(rtc_cfg);
  webrtc_client_->set_connection_state_callback(
      std::bind(&VehicleNode::OnConnectionStateChange, this, std::placeholders::_1));
  webrtc_client_->set_control_command_callback(
      std::bind(&VehicleNode::OnControlCommand, this, std::placeholders::_1));
  webrtc_client_->set_latency_update_callback(
      std::bind(&VehicleNode::OnLatencyUpdate, this, std::placeholders::_1));

  // ── Optional MQTT ─────────────────────────────────────────────────────────
  const auto& mqtt_cfg = config_manager_.mqtt_config();
  if (mqtt_cfg.enabled) {
    mqtt_bridge_ = std::make_unique<MqttBridge>(
        mqtt_cfg, config_manager_.vehicle_id());
    mqtt_bridge_->set_control_callback([this](const std::string& payload) {
      try {
        auto j = nlohmann::json::parse(payload);
        const auto& d = j.contains("data") ? j["data"] : j;
        static constexpr float kD2R = 3.14159265F / 180.0F;
        const float max_steer = static_cast<float>(
            config_manager_.vehicle_params().max_steering_angle_deg) * kD2R;
        ControlCommand cmd;
        cmd.steering_tire_angle = std::clamp(d.value("steering",0.0F),-1.0F,1.0F) * max_steer;
        const float brk = d.value("brake", 0.0F);
        if (brk > 0.01F) {
          cmd.acceleration = -brk * static_cast<float>(
              config_manager_.vehicle_params().max_deceleration_mps2);
        } else {
          cmd.speed = d.value("throttle",0.0F) *
              static_cast<float>(config_manager_.vehicle_params().max_speed_kph)/3.6F;
        }
        cmd.emergency_stop = d.value("emergency", false);
        OnControlCommand(cmd);
      } catch (const std::exception& e) {
        FSM_LOG_ERROR("MQTT parse: {}", e.what());
      }
    });
  }

  // Initial diagnostics
  diagnostics_.UpdateStr("vehicle.id",     "Vehicle ID",    config_manager_.vehicle_id(), DiagnosticStatus::kOk, "vehicle");
  diagnostics_.UpdateStr("build.variant",  "Build Variant", fsm::kBuildVariant,           DiagnosticStatus::kOk, "build");
  diagnostics_.UpdateStr("safety.state",   "Safety State",  "NOMINAL",                    DiagnosticStatus::kOk, "safety");

  FSM_LOG_INFO("VehicleNode initialized OK");
  return true;
}

// ── Start / Stop ──────────────────────────────────────────────────────────

void VehicleNode::Start() {
  if (running_.exchange(true, std::memory_order_acq_rel)) return;

  safety_monitor_.Start();   // [VN-01] first

#ifdef FSM_DEMO_MODE
  if (sim_vehicle_) sim_vehicle_->Initialize();
#else
  if (data_collector_) data_collector_->Start();
#endif

  command_executor_->Start();

  if (!webrtc_client_->Connect()) {
    FSM_LOG_WARN("WebRTC connect initiation failed");
  }
  if (mqtt_bridge_ && !mqtt_bridge_->Start()) {
    FSM_LOG_WARN("MQTT bridge start failed; continuing without MQTT");
  }

  const int rate_hz = std::max(1, this->get_parameter("telemetry_rate_hz").as_int());
  telemetry_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / rate_hz),
      std::bind(&VehicleNode::TelemetryTimerCallback, this));

  health_check_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(
          this->get_parameter("health_check_interval_ms").as_int()),
      std::bind(&VehicleNode::HealthCheckCallback, this));

  FSM_LOG_INFO("VehicleNode started (build={})", fsm::kBuildVariant);
}

void VehicleNode::Stop() {
  if (!running_.exchange(false, std::memory_order_acq_rel)) return;

  if (telemetry_timer_)    { telemetry_timer_->cancel();    telemetry_timer_.reset(); }
  if (health_check_timer_) { health_check_timer_->cancel(); health_check_timer_.reset(); }

  if (webrtc_client_) webrtc_client_->Disconnect();
  if (mqtt_bridge_)   mqtt_bridge_->Stop();
  if (command_executor_) command_executor_->Stop();

#ifdef FSM_DEMO_MODE
  if (sim_vehicle_) sim_vehicle_->Shutdown();
#else
  if (data_collector_) data_collector_->Stop();
#endif

  safety_monitor_.Stop();   // [VN-01] last
  FSM_LOG_INFO("VehicleNode stopped");
}

// ── Accessors ─────────────────────────────────────────────────────────────

bool VehicleNode::is_running() const noexcept {
  return running_.load(std::memory_order_acquire);
}
const std::string& VehicleNode::vehicle_id() const noexcept {
  return config_manager_.vehicle_id();
}
WebRtcState VehicleNode::connection_state() const {
  return webrtc_client_ ? webrtc_client_->state() : WebRtcState::kDisconnected;
}
LatencyInfo VehicleNode::latency_info() const {
  std::lock_guard<std::mutex> lk(state_mutex_);
  return cached_latency_info_;
}
SafetyState VehicleNode::safety_state() const noexcept {
  return safety_monitor_.state();
}
nlohmann::json VehicleNode::GetDiagnosticsJson() const {
  return diagnostics_.ToJson();
}

// ── Callbacks ─────────────────────────────────────────────────────────────

void VehicleNode::OnConnectionStateChange(WebRtcState state) {
  connected_.store(state == WebRtcState::kConnected, std::memory_order_release);
  static const char* kNames[] = {"DISCONNECTED","CONNECTING","CONNECTED",
                                  "RECONNECTING","FAILED"};
  const uint8_t idx = static_cast<uint8_t>(state);
  const char* name = (idx < 5U) ? kNames[idx] : "UNKNOWN";
  FSM_LOG_INFO("WebRTC: {}", name);

  const bool ok = (state == WebRtcState::kConnected);
  diagnostics_.UpdateStr("webrtc.state", "WebRTC", name,
      ok ? DiagnosticStatus::kOk : DiagnosticStatus::kWarn, "webrtc");

  if (ok) {
    safety_monitor_.Feed();
    command_executor_->SetRemoteControlMode();
  } else {
    command_executor_->ExitRemoteControlMode();
    if (state == WebRtcState::kFailed) {
      ReportSubsystemFault(FaultCode::kCommLinkLost, AsilLevel::kAsilB,
                            "webrtc", "WebRTC connection failed");
    }
  }

#ifdef FSM_DEMO_MODE
  // In demo mode, announce vehicle capabilities on connect.
  if (ok && sim_vehicle_) {
    const auto caps = sim_vehicle_->GetCapabilities();
    FSM_LOG_INFO("Demo vehicle caps: max_speed={:.0f}kph cameras={}",
                 static_cast<double>(caps.max_speed_kph), caps.num_cameras);
  }
#endif
}

void VehicleNode::OnControlCommand(const ControlCommand& cmd) {
  if (!running_.load(std::memory_order_acquire)) return;
  safety_monitor_.Feed();

  // In demo mode, also forward to SimulatedVehicle for closed-loop response.
#ifdef FSM_DEMO_MODE
  if (sim_vehicle_) {
    NormalizedControlCmd ncmd;
    const float max_steer = static_cast<float>(
        utils::DegToRad(config_manager_.vehicle_params().max_steering_angle_deg));
    ncmd.steering_normalized = (max_steer > 0.0F)
        ? std::clamp(cmd.steering_tire_angle / max_steer, -1.0F, 1.0F)
        : 0.0F;
    const float max_speed = static_cast<float>(
        config_manager_.vehicle_params().max_speed_kph / 3.6);
    ncmd.throttle_normalized = (max_speed > 0.0F && cmd.speed > 0.0F)
        ? std::clamp(cmd.speed / max_speed, 0.0F, 1.0F)
        : 0.0F;
    ncmd.brake_normalized  = (cmd.acceleration < -0.1F) ? 0.5F : 0.0F;
    ncmd.emergency_stop    = cmd.emergency_stop;
    ncmd.timestamp_ns      = cmd.timestamp_ns;
    ncmd.sequence_number   = cmd.sequence_number;
    sim_vehicle_->ApplyCommand(ncmd);
  }
#endif

  if (!command_executor_->Execute(cmd)) {
    FSM_LOG_WARN("Command rejected seq={}", cmd.sequence_number);
  }
  commands_received_count_.fetch_add(1U, std::memory_order_relaxed);
}

void VehicleNode::OnLatencyUpdate(const LatencyInfo& info) {
  { std::lock_guard<std::mutex> lk(state_mutex_); cached_latency_info_ = info; }
  const double rtt = static_cast<double>(info.rtt_ms);
  diagnostics_.Update("webrtc.rtt_ms", "RTT", rtt, "ms",
      (rtt > 500.0) ? DiagnosticStatus::kError
      : (rtt > 200.0) ? DiagnosticStatus::kWarn
      : DiagnosticStatus::kOk, "webrtc");
  if (rtt > 500.0) {
    ReportSubsystemFault(FaultCode::kCommLatencyExceeded, AsilLevel::kAsilA,
                          "webrtc", "RTT > 500ms");
  }
}

// ── Timer callbacks ────────────────────────────────────────────────────────

void VehicleNode::TelemetryTimerCallback() {
  if (!webrtc_client_ || !webrtc_client_->is_connected()) return;

  VehicleState vs;
  SystemState  ss;
  { std::lock_guard<std::mutex> lk(state_mutex_); vs = cached_vehicle_state_; ss = cached_system_state_; }

  const auto telem = SerializeVehicleState(vs);
  webrtc_client_->SendTelemetry(telem);
  if (mqtt_bridge_) mqtt_bridge_->PublishTelemetry(telem);

  static int sys_tick = 0;
  if (++sys_tick >= 10) {
    const auto sys = SerializeSystemState(ss);
    webrtc_client_->SendSystemStatus(sys);
    if (mqtt_bridge_) mqtt_bridge_->PublishStatus(sys);
    sys_tick = 0;
  }
  telemetry_sent_count_.fetch_add(1U, std::memory_order_relaxed);
}

void VehicleNode::HealthCheckCallback() {
  const bool safety_ok = safety_monitor_.is_remote_control_permitted();
  const bool estop     = command_executor_->is_emergency_active();

#ifdef FSM_DEMO_MODE
  const bool sensor_ok = (sim_vehicle_ != nullptr);
  diagnostics_.UpdateStr("sensor.mode", "Sensor Mode", "SIMULATED",
                          DiagnosticStatus::kOk, "sensor");
  if (sim_vehicle_) {
    diagnostics_.Update("sim.speed_mps", "Sim Speed",
        static_cast<double>(sim_vehicle_->sim_speed_mps()),
        "m/s", DiagnosticStatus::kOk, "sim");
    diagnostics_.UpdateStr("sim.scenario", "Scenario",
        demo::DemoScenarioName(sim_vehicle_->scenario()),
        DiagnosticStatus::kOk, "sim");
  }
#else
  const bool sensor_ok = data_collector_ && data_collector_->IsHealthy();
  diagnostics_.UpdateStr("sensor.mode", "Sensor Mode", "HARDWARE",
      sensor_ok ? DiagnosticStatus::kOk : DiagnosticStatus::kError, "sensor");
  if (!sensor_ok) {
    ReportSubsystemFault(FaultCode::kSensorDataStale, AsilLevel::kAsilA,
                          "data_collector", "Sensor data stale");
  }
#endif

  diagnostics_.UpdateStr("control.estop",  "E-Stop",  estop ? "ACTIVE" : "inactive",
      estop ? DiagnosticStatus::kError : DiagnosticStatus::kOk, "control");
  diagnostics_.Update("safety.watchdog_ms", "Watchdog",
      static_cast<double>(safety_monitor_.ms_since_last_feed()),
      "ms", safety_ok ? DiagnosticStatus::kOk : DiagnosticStatus::kError, "safety");
  diagnostics_.Update("stats.telem",   "Telem Sent",
      static_cast<double>(telemetry_sent_count_.load()),   "pkt", DiagnosticStatus::kOk, "stats");
  diagnostics_.Update("stats.cmds",    "Cmds Recv",
      static_cast<double>(commands_received_count_.load()), "pkt", DiagnosticStatus::kOk, "stats");

  const auto lat = webrtc_client_ ? webrtc_client_->latency_info() : LatencyInfo{};
  FSM_LOG_DEBUG("health: sensor={} estop={} safety_ok={} rtt={:.0f}ms telem={} cmds={}",
                sensor_ok, estop, safety_ok,
                static_cast<double>(lat.rtt_ms),
                telemetry_sent_count_.load(), commands_received_count_.load());
}

// ── Serialisation ─────────────────────────────────────────────────────────

std::vector<uint8_t> VehicleNode::SerializeVehicleState(const VehicleState& s) const {
  fsm::proto::TelemetryData p;
  auto* stamp = p.mutable_stamp();
  stamp->set_seconds(s.timestamp_ns / 1'000'000'000LL);
  stamp->set_nanos(static_cast<int32_t>(s.timestamp_ns % 1'000'000'000LL));
  p.set_vehicle_id(config_manager_.vehicle_id());

  auto* vs = p.mutable_vehicle_status();
  vs->mutable_stamp()->CopyFrom(*stamp);
  vs->set_vehicle_id(config_manager_.vehicle_id());
  vs->mutable_velocity()->set_longitudinal_velocity(s.longitudinal_velocity);
  vs->mutable_velocity()->set_lateral_velocity(s.lateral_velocity);
  vs->mutable_velocity()->set_heading_rate(s.heading_rate);
  vs->mutable_steering()->set_steering_tire_angle(s.steering_tire_angle);
  vs->set_gear(static_cast<fsm::proto::GearPosition>(s.gear));

  auto* loc = p.mutable_localization();
  loc->mutable_pose()->mutable_position()->set_x(s.pose_x);
  loc->mutable_pose()->mutable_position()->set_y(s.pose_y);
  loc->mutable_pose()->mutable_position()->set_z(s.pose_z);
  loc->mutable_geo_point()->set_latitude(s.latitude);
  loc->mutable_geo_point()->set_longitude(s.longitude);
  loc->mutable_geo_point()->set_altitude(s.altitude);

  std::string buf; p.SerializeToString(&buf);
  return std::vector<uint8_t>(buf.begin(), buf.end());
}

std::vector<uint8_t> VehicleNode::SerializeSystemState(const SystemState& s) const {
  fsm::proto::SystemStatus p;
  auto* stamp = p.mutable_stamp();
  stamp->set_seconds(s.timestamp_ns / 1'000'000'000LL);
  stamp->set_nanos(static_cast<int32_t>(s.timestamp_ns % 1'000'000'000LL));
  p.set_vehicle_id(config_manager_.vehicle_id());
  p.set_cpu_usage(s.cpu_usage);
  p.set_memory_usage(s.memory_usage);
  p.set_disk_usage(s.disk_usage);
  std::string buf; p.SerializeToString(&buf);
  return std::vector<uint8_t>(buf.begin(), buf.end());
}

// ── Safety helpers ────────────────────────────────────────────────────────

void VehicleNode::ReportSubsystemFault(FaultCode code, AsilLevel level,
                                        const std::string& sub,
                                        const std::string& desc) {
  safety_monitor_.ReportFault(code, level, sub, desc);
  diagnostics_.UpdateStr("safety.last_fault", "Last Fault",
                          desc, DiagnosticStatus::kWarn, "safety");
}

}  // namespace vehicle
}  // namespace fsm

// ── main() ────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
  fsm::Logger::Init("fsm_vehicle", "/tmp/fsm_vehicle.log",
                    fsm::Logger::Level::kInfo);
  rclcpp::init(argc, argv);

  FSM_LOG_INFO("=== FSM Vehicle Node === build={} version={}",
               fsm::kBuildVariant, fsm::kFsmVersion);

  auto node = std::make_shared<fsm::vehicle::VehicleNode>();
  const std::string cfg = (argc > 1) ? argv[1] : "config/vehicle_config.yaml";

  if (!node->Initialize(cfg)) {
    FSM_LOG_CRITICAL("Init failed — abort");
    return 1;
  }

  node->Start();
  FSM_LOG_INFO("Running... (Ctrl-C to stop)");
  rclcpp::spin(node);

  node->Stop();
  rclcpp::shutdown();
  fsm::Logger::Shutdown();
  return 0;
}
