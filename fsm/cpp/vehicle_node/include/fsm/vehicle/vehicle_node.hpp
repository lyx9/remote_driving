#pragma once

// ============================================================================
// FSM-Pilot  |  VehicleNode
// ============================================================================
// Top-level ROS2 node orchestrating the remote driving vehicle stack.
//
// Build variants (set via CMake option):
//   PRODUCTION  (-DFSM_DEMO_MODE=OFF)  Uses real hardware via DataCollector
//   DEMO        (-DFSM_DEMO_MODE=ON)   Uses SimulatedVehicle, no real sensors
//
// ISO 26262 / MISRA C++ compliance:
//   [VN-01] SafetyMonitor initialised before all subsystems; stopped last.
//   [VN-02] Fail-safe action → CommandExecutor::TriggerEmergencyStop.
//   [VN-03] HealthCheckCallback feeds DiagnosticsManager for remote debug.
//   [VN-04] Demo/production split is compile-time, not runtime branching.
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
#include "fsm/build_mode.hpp"
#include "fsm/vehicle/command_executor.hpp"
#include "fsm/vehicle/webrtc_client.hpp"
#include "fsm/vehicle/mqtt_bridge.hpp"
#include "fsm/logger.hpp"

// Production-only: real sensor collection
#ifndef FSM_DEMO_MODE
#include "fsm/vehicle/data_collector.hpp"
#endif

// Demo-only: simulated vehicle
#ifdef FSM_DEMO_MODE
#include "demo/simulated_vehicle.hpp"
#endif

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

  [[nodiscard]] bool Initialize(const std::string& config_path);
  void Start();
  void Stop();

  // ── Accessors ─────────────────────────────────────────────────────────────

  [[nodiscard]] bool               is_running()       const noexcept;
  [[nodiscard]] const std::string& vehicle_id()       const noexcept;
  [[nodiscard]] WebRtcState        connection_state()  const;
  [[nodiscard]] LatencyInfo        latency_info()      const;
  [[nodiscard]] SafetyState        safety_state()      const noexcept;
  [[nodiscard]] nlohmann::json     GetDiagnosticsJson() const;

 private:
  // ── Subsystem callbacks ───────────────────────────────────────────────────
  void OnConnectionStateChange(WebRtcState state);
  void OnControlCommand(const ControlCommand& cmd);
  void OnLatencyUpdate(const LatencyInfo& info);

  // ── Timer callbacks ───────────────────────────────────────────────────────
  void TelemetryTimerCallback();
  void HealthCheckCallback();

  // ── Serialisation ─────────────────────────────────────────────────────────
  [[nodiscard]] std::vector<uint8_t> SerializeVehicleState(
      const VehicleState& state) const;
  [[nodiscard]] std::vector<uint8_t> SerializeSystemState(
      const SystemState& state) const;

  // ── Safety helpers ─────────────────────────────────────────────────────────
  void ReportSubsystemFault(FaultCode code, AsilLevel level,
                             const std::string& subsystem,
                             const std::string& description);

  // ── Configuration ─────────────────────────────────────────────────────────
  config::VehicleConfigManager config_manager_;

  // ── Safety & diagnostics (first in, last out) ─────────────────────────────
  SafetyMonitor      safety_monitor_;
  DiagnosticsManager diagnostics_;

  // ── Core subsystems (common to both modes) ────────────────────────────────
  std::unique_ptr<CommandExecutor> command_executor_;
  std::unique_ptr<WebRtcClient>    webrtc_client_;
  std::unique_ptr<MqttBridge>      mqtt_bridge_;  // null if disabled

#ifdef FSM_DEMO_MODE
  // ── Demo mode: simulated vehicle (replaces DataCollector) ─────────────────
  std::unique_ptr<demo::SimulatedVehicle> sim_vehicle_;
#else
  // ── Production mode: real sensor collection ───────────────────────────────
  std::unique_ptr<DataCollector> data_collector_;
#endif

  // ── ROS2 timers ───────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp::TimerBase::SharedPtr health_check_timer_;

  // ── Shared state ──────────────────────────────────────────────────────────
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};

  mutable std::mutex state_mutex_;
  VehicleState  cached_vehicle_state_{};
  SystemState   cached_system_state_{};
  LatencyInfo   cached_latency_info_{};

  std::atomic<uint64_t> telemetry_sent_count_{0U};
  std::atomic<uint64_t> commands_received_count_{0U};
  std::atomic<uint64_t> frames_sent_count_{0U};
};

}  // namespace vehicle
}  // namespace fsm
