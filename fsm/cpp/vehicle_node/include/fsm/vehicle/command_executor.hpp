#pragma once

// ============================================================================
// FSM-Pilot  |  CommandExecutor
// ============================================================================
// Validates, rate-limits, and publishes vehicle control commands to ROS2.
//
// ISO 26262 / MISRA C++ compliance:
//   [CE-01] Execute() returns [[nodiscard]] bool — caller must check.
//   [CE-02] All safety-critical bounds violations are reported through the
//           optional SafetyMonitor* (non-owning pointer, may be nullptr).
//   [CE-03] Watchdog fires -> TriggerEmergencyStop + SafetyMonitor::ReportFault.
//   [CE-04] No implicit narrowing conversions in safety-critical checks.
//   [CE-05] All member variables initialised at point of declaration.
// ============================================================================

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"
#include "fsm/safety_monitor.hpp"

#ifdef HAVE_AUTOWARE_MSGS
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#endif

namespace fsm {
namespace vehicle {

// ----------------------------------------------------------------------------
// ControlCommand — physical (SI) units from operator after denormalisation.
// ----------------------------------------------------------------------------
struct ControlCommand {
  float    steering_tire_angle        = 0.0F;  // rad
  float    steering_tire_rotation_rate = 0.0F; // rad/s
  float    speed                      = 0.0F;  // m/s
  float    acceleration               = 0.0F;  // m/s²
  float    jerk                       = 0.0F;  // m/s³
  int32_t  gear                       = 3;     // 0:P 1:R 2:N 3:D
  int32_t  turn_signal                = 0;     // 0:NONE 1:LEFT 2:RIGHT
  bool     hazard_lights              = false;
  bool     emergency_stop             = false;
  int64_t  timestamp_ns               = 0;
  uint64_t sequence_number            = 0U;
};

// ----------------------------------------------------------------------------
// CommandExecutor
// ----------------------------------------------------------------------------
class CommandExecutor {
 public:
  // safety_monitor may be nullptr (no fault reporting, watchdog still fires).
  CommandExecutor(rclcpp::Node::SharedPtr          node,
                  const config::VehicleConfigManager& config,
                  SafetyMonitor*                   safety_monitor = nullptr);
  ~CommandExecutor();

  CommandExecutor(const CommandExecutor&)            = delete;
  CommandExecutor& operator=(const CommandExecutor&) = delete;

  // ── Lifecycle ──────────────────────────────────────────────────────────────

  void Start();
  void Stop();

  // ── Command execution ─────────────────────────────────────────────────────

  // Validate, rate-limit, and publish a control command.
  // Returns false if the command was rejected (out-of-range, emergency active,
  // executor not running) — caller MUST check the return value.
  // [[nodiscard]] — MISRA C++:2023 Rule 0-1-7 (result of function call used).
  [[nodiscard]] bool Execute(const ControlCommand& cmd);

  // ── Emergency control ─────────────────────────────────────────────────────

  // Engage emergency stop: publishes full-brake command and sets E-stop flag.
  // Thread-safe; idempotent.
  void TriggerEmergencyStop(const std::string& reason = "Remote emergency");

  // Release emergency stop (requires explicit operator action).
  // Returns false if no emergency was active.
  [[nodiscard]] bool ReleaseEmergencyStop();

  // ── Remote control mode ───────────────────────────────────────────────────

  // Called by VehicleNode when WebRTC connects.
  void SetRemoteControlMode();

  // Called by VehicleNode when WebRTC disconnects.
  void ExitRemoteControlMode();

  // ── State accessors ───────────────────────────────────────────────────────

  [[nodiscard]] bool    is_emergency_active()       const noexcept;
  [[nodiscard]] bool    is_remote_control_active()  const noexcept;
  [[nodiscard]] int64_t last_command_time_ns()      const noexcept;
  [[nodiscard]] uint64_t missed_commands()          const noexcept;

 private:
  // Returns true if the command is within safe operating limits.
  // Does NOT modify the command.
  [[nodiscard]] bool PassesSafetyCheck(const ControlCommand& cmd) const;

  // Clamps the command to safe operating limits (soft limits).
  // [CE-04] All clamp operations use explicit static_cast.
  void ApplySoftLimits(ControlCommand& cmd) const;

  // ROS2 timer callback — fires every watchdog_timeout_ms.
  void WatchdogCallback();

  // Low-level publish helpers.
  void PublishAckermannCommand(const ControlCommand& cmd);
  void PublishGearCommand(int32_t gear);
  void PublishTurnSignalCommand(int32_t signal);
  void PublishEmergencyCommand(bool emergency);

  // ── Dependencies ─────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr              node_;
  const config::VehicleConfigManager&  config_;
  SafetyMonitor*                       safety_monitor_ = nullptr;  // Non-owning

  // ── ROS2 publishers (conditional on Autoware msg availability) ────────────
#ifdef HAVE_AUTOWARE_MSGS
  rclcpp::Publisher<
      autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr ackermann_pub_;
  rclcpp::Publisher<
      autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_pub_;
  rclcpp::Publisher<
      autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_signal_pub_;
  rclcpp::Publisher<
      autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_pub_;
  rclcpp::Publisher<
      tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_pub_;
#else
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              emergency_pub_;
#endif

  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // ── Mutable state (thread-safe) ───────────────────────────────────────────
  mutable std::mutex mu_;

  std::atomic<bool>     running_{false};
  std::atomic<bool>     emergency_active_{false};
  std::atomic<bool>     remote_control_active_{false};
  std::atomic<int64_t>  last_command_ns_{0};
  std::atomic<uint64_t> missed_commands_{0U};

  ControlCommand last_command_{};  // Zero-initialised
  std::chrono::steady_clock::time_point last_command_tp_{};
  std::chrono::steady_clock::time_point last_publish_tp_{};
  std::chrono::milliseconds             min_publish_interval_ms_{0};

  uint64_t expected_sequence_ = 0U;
};

}  // namespace vehicle
}  // namespace fsm
