#pragma once

// ============================================================================
// FSM-Pilot  |  VehicleNode
// ============================================================================
// Top-level ROS2 node orchestrating the remote driving vehicle stack.
//
// ISO 26262 / MISRA C++ compliance:
//   [VN-01] SafetyMonitor is initialised before all other subsystems and
//           shut down after them (reverse initialisation order).
//   [VN-02] Fail-safe action registered with SafetyMonitor calls
//           CommandExecutor::TriggerEmergencyStop — guaranteed to execute
//           even if other subsystems are unresponsive.
//   [VN-03] HealthCheckCallback updates DiagnosticsManager with structured
//           items — enables remote debug without ROS2 tooling.
//   [VN-04] In FSM_DEMO_MODE the ROS2 hardware stack is replaced by
//           SimulatedVehicle — no ifdef branching in safety-critical paths.
// ============================================================================

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "fsm/config_manager.hpp"
#include "fsm/safety_monitor.hpp"
#include "fsm/diagnostics.hpp"
#include "fsm/vehicle/data_collector.hpp"
#include "fsm/vehicle/command_executor.hpp"
#include "fsm/vehicle/webrtc_client.hpp"
#include "fsm/vehicle/mqtt_bridge.hpp"
#include "fsm/logger.hpp"

namespace fsm {
namespace vehicle {

class VehicleNode : public rclcpp::Node {
 public:
  explicit VehicleNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~VehicleNode() override;

  VehicleNode(const VehicleNode&)            = delete;
  VehicleNode& operator=(const VehicleNode&) = delete;

  // ── Lifecycle ──────────────────────────────────────────────────────────────

  // Load config, initialise sub-modules (safety first), return false on error.
  // [[nodiscard]] — caller must check; failure = unsafe to call Start().
  [[nodiscard]] bool Initialize(const std::string& config_path);

  void Start();
  void Stop();

  // ── Accessors ─────────────────────────────────────────────────────────────

  [[nodiscard]] bool              is_running()       const noexcept;
  [[nodiscard]] const std::string& vehicle_id()      const noexcept;
  [[nodiscard]] WebRtcState        connection_state() const;
  [[nodiscard]] LatencyInfo        latency_info()     const;

  // Current safety state (for REST health endpoint).
  [[nodiscard]] SafetyState safety_state() const noexcept;

  // Diagnostics snapshot (for REST diagnostics endpoint).
  [[nodiscard]] nlohmann::json GetDiagnosticsJson() const;

 private:
  // ── Subsystem callbacks ───────────────────────────────────────────────────
  void OnVehicleStateUpdate(const VehicleState& state);
  void OnSystemStateUpdate(const SystemState& state);
  void OnCameraFrame(const CameraFrame& frame);
  void OnConnectionStateChange(WebRtcState state);
  void OnControlCommand(const ControlCommand& cmd);
  void OnLatencyUpdate(const LatencyInfo& info);

  // ── Timer callbacks ───────────────────────────────────────────────────────
  void TelemetryTimerCallback();

  // Health check: updates DiagnosticsManager, feeds SafetyMonitor.
  void HealthCheckCallback();

  // ── Serialisation ─────────────────────────────────────────────────────────
  [[nodiscard]] std::vector<uint8_t> SerializeVehicleState(
      const VehicleState& state) const;
  [[nodiscard]] std::vector<uint8_t> SerializeSystemState(
      const SystemState& state) const;

  // ── Safety monitor helper ─────────────────────────────────────────────────
  // Reports a fault and propagates to DiagnosticsManager.
  void ReportSubsystemFault(FaultCode code, AsilLevel level,
                             const std::string& subsystem,
                             const std::string& description);

  // ── Configuration ─────────────────────────────────────────────────────────
  config::VehicleConfigManager config_manager_;

  // ── Safety & diagnostics (initialised first, destroyed last) ─────────────
  SafetyMonitor       safety_monitor_;
  DiagnosticsManager  diagnostics_;

  // ── Subsystems ────────────────────────────────────────────────────────────
  std::unique_ptr<DataCollector>   data_collector_;
  std::unique_ptr<CommandExecutor> command_executor_;
  std::unique_ptr<WebRtcClient>    webrtc_client_;
  std::unique_ptr<MqttBridge>      mqtt_bridge_;  // null if disabled

  // ── ROS2 timers ───────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp::TimerBase::SharedPtr health_check_timer_;

  // ── Node state ────────────────────────────────────────────────────────────
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};

  mutable std::mutex state_mutex_;
  VehicleState  cached_vehicle_state_;
  SystemState   cached_system_state_;
  LatencyInfo   cached_latency_info_;

  // Statistics counters (relaxed ordering — diagnostic use only).
  std::atomic<uint64_t> telemetry_sent_count_{0U};
  std::atomic<uint64_t> commands_received_count_{0U};
  std::atomic<uint64_t> frames_sent_count_{0U};
};

}  // namespace vehicle
}  // namespace fsm
