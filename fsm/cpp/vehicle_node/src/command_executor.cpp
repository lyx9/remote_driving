#include "fsm/vehicle/command_executor.hpp"
#include "fsm/utils.hpp"

namespace fsm {
namespace vehicle {

// Build rclcpp::QoS from config.
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

// ── Constructor / Destructor ─────────────────────────────────────────────────

CommandExecutor::CommandExecutor(
    rclcpp::Node::SharedPtr             node,
    const config::VehicleConfigManager& config,
    SafetyMonitor*                      safety_monitor)
    : node_(node), config_(config), safety_monitor_(safety_monitor) {
  const auto& topics = config_.control_topics();
  const auto& cqos   = config_.control_qos();

#ifdef HAVE_AUTOWARE_MSGS
  ackermann_pub_ = node_->create_publisher<
      autoware_auto_control_msgs::msg::AckermannControlCommand>(
      topics.steering_cmd, BuildQos(cqos.steering_cmd));
  gear_pub_ = node_->create_publisher<
      autoware_auto_vehicle_msgs::msg::GearCommand>(
      topics.gear_cmd, BuildQos(cqos.gear_cmd));
  turn_signal_pub_ = node_->create_publisher<
      autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
      topics.turn_signal_cmd, BuildQos(cqos.turn_signal));
  emergency_pub_ = node_->create_publisher<
      tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
      topics.emergency_cmd, BuildQos(cqos.emergency_cmd));
  FSM_LOG_INFO("CommandExecutor: Autoware topics (reliability={})",
               cqos.steering_cmd.reliability);
#else
  twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      topics.velocity_cmd, BuildQos(cqos.steering_cmd));
  emergency_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      topics.emergency_cmd, BuildQos(cqos.emergency_cmd));
  FSM_LOG_INFO("CommandExecutor: generic topics (reliability={})",
               cqos.steering_cmd.reliability);
#endif

  const int32_t rate = config_.safety_config().command_publish_rate_hz;
  min_publish_interval_ms_ = (rate > 0)
      ? std::chrono::milliseconds(1000 / rate)
      : std::chrono::milliseconds(0);

  FSM_LOG_INFO("CommandExecutor initialized: {} (max_rate={}Hz, safety_monitor={})",
               config_.vehicle_id(), rate,
               (safety_monitor_ != nullptr) ? "enabled" : "disabled");
}

CommandExecutor::~CommandExecutor() {
  Stop();
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

void CommandExecutor::Start() {
  if (running_.exchange(true, std::memory_order_acq_rel)) {
    FSM_LOG_WARN("CommandExecutor already running");
    return;
  }

  watchdog_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(config_.safety_config().watchdog_timeout_ms),
      std::bind(&CommandExecutor::WatchdogCallback, this));

  last_command_tp_ = std::chrono::steady_clock::now();
  FSM_LOG_INFO("CommandExecutor started");
}

void CommandExecutor::Stop() {
  if (!running_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }

  // Send a zero command to zero out actuators cleanly.
  (void)Execute(ControlCommand{});

  if (watchdog_timer_) {
    watchdog_timer_->cancel();
    watchdog_timer_.reset();
  }
  FSM_LOG_INFO("CommandExecutor stopped");
}

// ── Command execution ─────────────────────────────────────────────────────────

bool CommandExecutor::Execute(const ControlCommand& cmd) {
  std::lock_guard<std::mutex> lock(mu_);

  if (!running_.load(std::memory_order_acquire)) {
    FSM_LOG_WARN("CommandExecutor not running, ignoring command");
    return false;
  }

  if (cmd.emergency_stop) {
    TriggerEmergencyStop("Remote emergency command");
    return true;  // E-stop executed — not a rejection
  }

  if (emergency_active_.load(std::memory_order_acquire)) {
    FSM_LOG_WARN("Emergency active, ignoring control command (seq={})",
                 cmd.sequence_number);
    return false;
  }

  ControlCommand safe_cmd = cmd;
  const bool safety_ok = PassesSafetyCheck(safe_cmd);
  if (!safety_ok) {
    FSM_LOG_WARN("Command failed safety check — applying soft limits (seq={})",
                 cmd.sequence_number);
    if (safety_monitor_ != nullptr) {
      safety_monitor_->ReportFault(
          FaultCode::kControlCommandOutOfRange,
          AsilLevel::kAsilA,
          "command_executor",
          "Control command exceeded vehicle limits; soft-limited");
    }
  }
  ApplySoftLimits(safe_cmd);

  // Sequence gap tracking — [CE-MISRA] explicit arithmetic with no overflow.
  if (cmd.sequence_number > 0U) {
    if (cmd.sequence_number > expected_sequence_) {
      const uint64_t gap = cmd.sequence_number - expected_sequence_;
      missed_commands_.fetch_add(gap, std::memory_order_relaxed);
      FSM_LOG_WARN("Missed {} commands (expected={}, got={})",
                   gap, expected_sequence_, cmd.sequence_number);
      // ISO 26262: large sequence gap → report fault.
      static constexpr uint64_t kMaxGap = 5U;
      if (gap > kMaxGap && safety_monitor_ != nullptr) {
        safety_monitor_->ReportFault(
            FaultCode::kControlSequenceGap,
            AsilLevel::kAsilA,
            "command_executor",
            "Large sequence gap in control commands");
      }
    }
    expected_sequence_ = cmd.sequence_number + 1U;
  }

  // Rate-limit publishing.
  const auto now = std::chrono::steady_clock::now();
  if (min_publish_interval_ms_.count() > 0) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_publish_tp_);
    if (elapsed < min_publish_interval_ms_) {
      last_command_    = safe_cmd;
      last_command_ns_.store(utils::NowNanos(), std::memory_order_relaxed);
      return true;  // Stored for next publish window — not a rejection
    }
  }
  last_publish_tp_ = now;

  PublishAckermannCommand(safe_cmd);

  if (safe_cmd.gear != last_command_.gear) {
    PublishGearCommand(safe_cmd.gear);
  }
  if (safe_cmd.turn_signal != last_command_.turn_signal) {
    PublishTurnSignalCommand(safe_cmd.turn_signal);
  }

  last_command_    = safe_cmd;
  last_command_tp_ = now;
  last_command_ns_.store(utils::NowNanos(), std::memory_order_relaxed);
  return true;
}

// ── Emergency control ─────────────────────────────────────────────────────────

void CommandExecutor::TriggerEmergencyStop(const std::string& reason) {
  if (emergency_active_.exchange(true, std::memory_order_acq_rel)) {
    return;  // Already engaged
  }
  FSM_LOG_CRITICAL("EMERGENCY STOP: {}", reason);

  PublishEmergencyCommand(true);

  ControlCommand ecmd;
  ecmd.speed        = 0.0F;
  ecmd.acceleration = -config_.safety_config().emergency_decel_mps2;
  PublishAckermannCommand(ecmd);

  if (safety_monitor_ != nullptr) {
    safety_monitor_->ReportFault(
        FaultCode::kSysEmergencyActivated,
        AsilLevel::kAsilB,
        "command_executor",
        "Emergency stop activated: " + reason);
  }
}

bool CommandExecutor::ReleaseEmergencyStop() {
  if (!emergency_active_.exchange(false, std::memory_order_acq_rel)) {
    return false;  // Was not active
  }
  FSM_LOG_INFO("Emergency stop released");
  PublishEmergencyCommand(false);
  return true;
}

// ── Remote control mode ───────────────────────────────────────────────────────

void CommandExecutor::SetRemoteControlMode() {
  if (remote_control_active_.exchange(true, std::memory_order_acq_rel)) {
    return;
  }
  last_command_tp_ = std::chrono::steady_clock::now();
  // Feed safety monitor watchdog on connect.
  if (safety_monitor_ != nullptr) {
    safety_monitor_->Feed();
  }
  FSM_LOG_INFO("Remote control mode activated");
}

void CommandExecutor::ExitRemoteControlMode() {
  if (!remote_control_active_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  (void)Execute(ControlCommand{});
  FSM_LOG_INFO("Remote control mode deactivated");
}

// ── State accessors ───────────────────────────────────────────────────────────

bool CommandExecutor::is_emergency_active() const noexcept {
  return emergency_active_.load(std::memory_order_acquire);
}

bool CommandExecutor::is_remote_control_active() const noexcept {
  return remote_control_active_.load(std::memory_order_acquire);
}

int64_t CommandExecutor::last_command_time_ns() const noexcept {
  return last_command_ns_.load(std::memory_order_relaxed);
}

uint64_t CommandExecutor::missed_commands() const noexcept {
  return missed_commands_.load(std::memory_order_relaxed);
}

// ── Private helpers ───────────────────────────────────────────────────────────

bool CommandExecutor::PassesSafetyCheck(const ControlCommand& cmd) const {
  const auto& params = config_.vehicle_params();
  bool passed = true;

  const double max_steer_rad =
      utils::DegToRad(params.max_steering_angle_deg);
  if (std::abs(static_cast<double>(cmd.steering_tire_angle)) > max_steer_rad) {
    FSM_LOG_WARN("Steering {:.2f}rad exceeds limit {:.2f}rad",
                 static_cast<double>(cmd.steering_tire_angle), max_steer_rad);
    passed = false;
  }

  const double max_speed_mps = params.max_speed_kph / 3.6;
  if (static_cast<double>(cmd.speed) > max_speed_mps) {
    FSM_LOG_WARN("Speed {:.1f}m/s exceeds limit {:.1f}m/s",
                 static_cast<double>(cmd.speed), max_speed_mps);
    passed = false;
  }

  if (static_cast<double>(cmd.acceleration) >
      params.max_acceleration_mps2) {
    FSM_LOG_WARN("Acceleration {:.2f} exceeds limit {:.2f}",
                 static_cast<double>(cmd.acceleration),
                 params.max_acceleration_mps2);
    passed = false;
  }
  if (static_cast<double>(cmd.acceleration) <
      -params.max_deceleration_mps2) {
    FSM_LOG_WARN("Deceleration {:.2f} exceeds limit {:.2f}",
                 -static_cast<double>(cmd.acceleration),
                 params.max_deceleration_mps2);
    passed = false;
  }

  return passed;
}

void CommandExecutor::ApplySoftLimits(ControlCommand& cmd) const {
  const auto& params = config_.vehicle_params();
  const auto& safety = config_.safety_config();
  if (!safety.soft_limits_enabled) {
    return;
  }

  const double max_steer_rad =
      utils::DegToRad(params.max_steering_angle_deg);
  cmd.steering_tire_angle = static_cast<float>(
      utils::Clamp(static_cast<double>(cmd.steering_tire_angle),
                   -max_steer_rad, max_steer_rad));

  const double max_speed_mps = params.max_speed_kph / 3.6;
  cmd.speed = static_cast<float>(
      utils::Clamp(static_cast<double>(cmd.speed), 0.0, max_speed_mps));

  cmd.acceleration = static_cast<float>(
      utils::Clamp(static_cast<double>(cmd.acceleration),
                   -params.max_deceleration_mps2,
                   params.max_acceleration_mps2));

  // Steering rate limit.
  const auto now = std::chrono::steady_clock::now();
  const double dt = std::chrono::duration<double>(
      now - last_command_tp_).count();
  if (dt > 0.0 && dt < 1.0) {
    const double max_delta =
        utils::DegToRad(safety.max_steering_rate_dps) * dt;
    double delta = static_cast<double>(cmd.steering_tire_angle) -
                   static_cast<double>(last_command_.steering_tire_angle);
    delta = utils::Clamp(delta, -max_delta, max_delta);
    cmd.steering_tire_angle = static_cast<float>(
        static_cast<double>(last_command_.steering_tire_angle) + delta);
  }
}

void CommandExecutor::WatchdogCallback() {
  if (!running_.load(std::memory_order_acquire) ||
      !remote_control_active_.load(std::memory_order_acquire)) {
    return;
  }

  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_command_tp_).count();
  const auto& safety = config_.safety_config();

  if (elapsed_ms > static_cast<int64_t>(safety.watchdog_timeout_ms) * 2) {
    FSM_LOG_CRITICAL("Watchdog: no command for {}ms — triggering E-stop",
                     elapsed_ms);
    TriggerEmergencyStop("Watchdog timeout");
    if (safety_monitor_ != nullptr) {
      safety_monitor_->ReportFault(
          FaultCode::kControlWatchdogTimeout,
          AsilLevel::kAsilB,
          "command_executor",
          "No control command received within watchdog window");
    }
  } else if (elapsed_ms > static_cast<int64_t>(safety.watchdog_timeout_ms)) {
    FSM_LOG_WARN("Watchdog: no command for {}ms — sending zero command",
                 elapsed_ms);
    PublishAckermannCommand(ControlCommand{});
  }
}

// ── Low-level publish helpers ─────────────────────────────────────────────────

void CommandExecutor::PublishAckermannCommand(const ControlCommand& cmd) {
#ifdef HAVE_AUTOWARE_MSGS
  auto msg = autoware_auto_control_msgs::msg::AckermannControlCommand();
  msg.stamp = node_->now();
  msg.lateral.steering_tire_angle          = cmd.steering_tire_angle;
  msg.lateral.steering_tire_rotation_rate  = cmd.steering_tire_rotation_rate;
  msg.longitudinal.speed                   = cmd.speed;
  msg.longitudinal.acceleration            = cmd.acceleration;
  msg.longitudinal.jerk                    = cmd.jerk;
  ackermann_pub_->publish(msg);
#else
  auto msg = geometry_msgs::msg::TwistStamped();
  msg.header.stamp    = node_->now();
  msg.header.frame_id = "base_link";
  msg.twist.linear.x  = static_cast<double>(cmd.speed);
  msg.twist.angular.z = static_cast<double>(cmd.steering_tire_angle);
  twist_pub_->publish(msg);
#endif
}

void CommandExecutor::PublishGearCommand(int32_t gear) {
#ifdef HAVE_AUTOWARE_MSGS
  auto msg = autoware_auto_vehicle_msgs::msg::GearCommand();
  msg.stamp   = node_->now();
  msg.command = static_cast<uint8_t>(gear);
  gear_pub_->publish(msg);
#else
  (void)gear;
#endif
}

void CommandExecutor::PublishTurnSignalCommand(int32_t signal) {
#ifdef HAVE_AUTOWARE_MSGS
  auto msg = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand();
  msg.stamp   = node_->now();
  msg.command = static_cast<uint8_t>(signal);
  turn_signal_pub_->publish(msg);
#else
  (void)signal;
#endif
}

void CommandExecutor::PublishEmergencyCommand(bool emergency) {
#ifdef HAVE_AUTOWARE_MSGS
  auto msg = tier4_vehicle_msgs::msg::VehicleEmergencyStamped();
  msg.stamp     = node_->now();
  msg.emergency = emergency;
  emergency_pub_->publish(msg);
#else
  auto msg = std_msgs::msg::Bool();
  msg.data = emergency;
  emergency_pub_->publish(msg);
#endif
}

}  // namespace vehicle
}  // namespace fsm
