#pragma once

// ============================================================================
// FSM-Pilot  |  Simulated Vehicle  [DEMO MODE ONLY]
// ============================================================================
// A software-only PlatformAdapter that generates synthetic vehicle data.
// Used for:
//   - Indoor lab demos without a real vehicle
//   - CI/CD integration tests
//   - Operator training scenarios
//
// Compile guard: this header is only included when FSM_DEMO_MODE is defined.
// The production build never links against this code.
//
// SimulatedVehicle implements PlatformAdapter and exposes:
//   - Sinusoidal speed / steering (mimics constant-radius turn)
//   - Configurable GPS start position + simulated movement
//   - Injected command passthrough (for closed-loop testing)
//   - Scenario selection: kStraightLine, kRoundAbout, kEmergencyBrake
//
// Thread safety: thread-safe.
// ============================================================================

#ifndef FSM_DEMO_MODE
  #error "simulated_vehicle.hpp must only be included in FSM_DEMO_MODE builds"
#endif

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "fsm/vehicle/platform_adapter.hpp"
#include "fsm/config_manager.hpp"

namespace fsm {
namespace vehicle {
namespace demo {

// ----------------------------------------------------------------------------
// Scenarios — pre-canned simulation behaviours for demos.
// ----------------------------------------------------------------------------
enum class DemoScenario : uint8_t {
  kStraightLine    = 0U,  // Accelerate to cruise, hold, brake to stop
  kRoundAbout      = 1U,  // Constant-radius circular loop
  kEmergencyBrake  = 2U,  // Cruise → sudden full brake → stop
  kSlalom          = 3U,  // Lane-change weave pattern
  kIdle            = 4U,  // Stationary; responds only to operator commands
};

inline const char* DemoScenarioName(DemoScenario s) noexcept {
  switch (s) {
    case DemoScenario::kStraightLine:   return "straight_line";
    case DemoScenario::kRoundAbout:     return "round_about";
    case DemoScenario::kEmergencyBrake: return "emergency_brake";
    case DemoScenario::kSlalom:         return "slalom";
    case DemoScenario::kIdle:           return "idle";
    default:                            return "unknown";
  }
}

// ----------------------------------------------------------------------------
// SimulationConfig — tune the virtual vehicle.
// ----------------------------------------------------------------------------
struct SimulationConfig {
  double start_latitude   = 37.3861;    // Waymo HQ, Mountain View, CA
  double start_longitude  = -122.0839;
  float  cruise_speed_mps = 5.0F;       // m/s (~18 km/h)
  float  loop_radius_m    = 20.0F;      // m  (for kRoundAbout)
  float  battery_pct      = 85.0F;      // Initial battery
  int    update_rate_hz   = 20;         // Status publish frequency
  DemoScenario scenario   = DemoScenario::kStraightLine;
};

// ----------------------------------------------------------------------------
// SimulatedVehicle
// ----------------------------------------------------------------------------
class SimulatedVehicle : public PlatformAdapter {
 public:
  explicit SimulatedVehicle(const SimulationConfig& cfg = SimulationConfig{});
  ~SimulatedVehicle() override;

  // ── PlatformAdapter interface ─────────────────────────────────────────────

  // Always succeeds (no real hardware).
  [[nodiscard]] bool Initialize() override;

  // Stop simulation thread.
  void Shutdown() override;

  // Accept operator command; blend with autonomous scenario motion.
  void ApplyCommand(const NormalizedControlCmd& cmd) override;

  // Return pre-configured capabilities for a simulated vehicle.
  [[nodiscard]] VehicleCapabilities GetCapabilities() const override;

  // ── Demo-specific API ─────────────────────────────────────────────────────

  // Change the active scenario at runtime (thread-safe).
  void SetScenario(DemoScenario s);

  // Inject a fault (e.g. simulate sensor dropout for training).
  void InjectFault(const std::string& subsystem);

  // Current simulated speed (m/s) — for test assertions.
  [[nodiscard]] float sim_speed_mps() const noexcept;

  // Current scenario.
  [[nodiscard]] DemoScenario scenario() const noexcept;

 private:
  void SimulationLoop();

  NormalizedVehicleStatus GenerateStatus(double elapsed_s) const;

  // Scenario generators (return [speed_mps, steering_rad]).
  std::pair<float, float> ScenarioStraightLine(double t) const;
  std::pair<float, float> ScenarioRoundAbout(double t) const;
  std::pair<float, float> ScenarioEmergencyBrake(double t) const;
  std::pair<float, float> ScenarioSlalom(double t) const;

  SimulationConfig cfg_;

  mutable std::mutex mu_;
  std::atomic<bool>         running_{false};
  std::atomic<DemoScenario> scenario_{DemoScenario::kStraightLine};
  std::thread               sim_thread_;

  // Simulated physical state (protected by mu_).
  double sim_lat_          = 0.0;
  double sim_lon_          = 0.0;
  float  sim_speed_mps_    = 0.0F;
  float  sim_steering_rad_ = 0.0F;
  float  sim_heading_rad_  = 0.0F;
  float  sim_battery_pct_  = 100.0F;

  // Operator override (blended with scenario output).
  NormalizedControlCmd last_operator_cmd_;
  std::chrono::steady_clock::time_point last_cmd_tp_;
  bool operator_has_control_ = false;

  // Scenario start time.
  std::chrono::steady_clock::time_point start_tp_;
};

}  // namespace demo
}  // namespace vehicle
}  // namespace fsm
