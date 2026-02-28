#include "demo/simulated_vehicle.hpp"

#include "fsm/logger.hpp"
#include "fsm/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace fsm {
namespace vehicle {
namespace demo {

// Metres per degree of latitude (approximate, sufficient for demo GPS).
static constexpr double kMetresPerDegreeLat = 111'320.0;

// ── Constructor / Destructor ─────────────────────────────────────────────────

SimulatedVehicle::SimulatedVehicle(const SimulationConfig& cfg)
    : cfg_(cfg) {
  sim_lat_         = cfg_.start_latitude;
  sim_lon_         = cfg_.start_longitude;
  sim_battery_pct_ = cfg_.battery_pct;
  scenario_.store(cfg_.scenario, std::memory_order_relaxed);
  FSM_LOG_INFO("[SimVehicle] Created — scenario='{}' cruise={:.1f}m/s",
               DemoScenarioName(cfg_.scenario), static_cast<double>(cfg_.cruise_speed_mps));
}

SimulatedVehicle::~SimulatedVehicle() {
  Shutdown();
}

// ── PlatformAdapter interface ─────────────────────────────────────────────────

bool SimulatedVehicle::Initialize() {
  if (running_.exchange(true, std::memory_order_acq_rel)) {
    return true;  // Already running
  }
  sim_thread_ = std::thread(&SimulatedVehicle::SimulationLoop, this);
  FSM_LOG_INFO("[SimVehicle] Initialized — simulation thread started (DEMO MODE)");
  return true;
}

void SimulatedVehicle::Shutdown() {
  if (!running_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  if (sim_thread_.joinable()) {
    sim_thread_.join();
  }
  FSM_LOG_INFO("[SimVehicle] Shutdown");
}

void SimulatedVehicle::ApplyCommand(const NormalizedControlCmd& cmd) {
  std::lock_guard<std::mutex> lock(mu_);
  last_operator_cmd_     = cmd;
  last_cmd_tp_           = std::chrono::steady_clock::now();
  operator_has_control_  = true;
}

VehicleCapabilities SimulatedVehicle::GetCapabilities() const {
  VehicleCapabilities caps;
  caps.vehicle_id             = "SIM-DEMO-01";
  caps.platform_type          = PlatformType::kCarlaSim;
  caps.has_steer_by_wire      = true;
  caps.has_brake_by_wire      = true;
  caps.has_throttle_by_wire   = true;
  caps.has_gear_control        = true;
  caps.has_turn_signal_control = true;
  caps.has_emergency_stop      = true;
  caps.has_lidar              = true;
  caps.has_gps                = true;
  caps.num_cameras            = 3U;
  caps.max_speed_kph          = 50.0F;
  caps.max_steering_angle_deg = 35.0F;
  caps.wheelbase_m            = 2.7F;
  caps.firmware_version       = "sim-" + std::string(fsm::kFsmVersion);
  caps.supported_codecs       = {"h264"};
  return caps;
}

// ── Demo-specific API ─────────────────────────────────────────────────────────

void SimulatedVehicle::SetScenario(DemoScenario s) {
  scenario_.store(s, std::memory_order_relaxed);
  // Reset scenario clock.
  std::lock_guard<std::mutex> lock(mu_);
  start_tp_ = std::chrono::steady_clock::now();
  FSM_LOG_INFO("[SimVehicle] Scenario → '{}'", DemoScenarioName(s));
}

void SimulatedVehicle::InjectFault(const std::string& subsystem) {
  FSM_LOG_WARN("[SimVehicle] Fault injected in subsystem '{}'", subsystem);
  // Subsystem-specific fault injection can be extended here.
  // Currently just logs; future: set a flag read in SimulationLoop.
  (void)subsystem;
}

float SimulatedVehicle::sim_speed_mps() const noexcept {
  std::lock_guard<std::mutex> lock(mu_);
  return sim_speed_mps_;
}

DemoScenario SimulatedVehicle::scenario() const noexcept {
  return scenario_.load(std::memory_order_relaxed);
}

// ── Simulation loop ───────────────────────────────────────────────────────────

void SimulatedVehicle::SimulationLoop() {
  const int    rate_hz   = std::max(1, cfg_.update_rate_hz);
  const auto   period    = std::chrono::milliseconds(1000 / rate_hz);

  {
    std::lock_guard<std::mutex> lock(mu_);
    start_tp_ = std::chrono::steady_clock::now();
  }

  FSM_LOG_INFO("[SimVehicle] Simulation loop running at {}Hz", rate_hz);

  while (running_.load(std::memory_order_acquire)) {
    const auto loop_start = std::chrono::steady_clock::now();

    double elapsed_s = 0.0;
    {
      std::lock_guard<std::mutex> lock(mu_);
      elapsed_s = std::chrono::duration<double>(loop_start - start_tp_).count();
    }

    // Pick scenario output.
    const DemoScenario scen = scenario_.load(std::memory_order_relaxed);
    float tgt_speed_mps   = 0.0F;
    float tgt_steering_rad = 0.0F;

    switch (scen) {
      case DemoScenario::kStraightLine: {
        auto [s, st] = ScenarioStraightLine(elapsed_s);
        tgt_speed_mps = s; tgt_steering_rad = st;
        break;
      }
      case DemoScenario::kRoundAbout: {
        auto [s, st] = ScenarioRoundAbout(elapsed_s);
        tgt_speed_mps = s; tgt_steering_rad = st;
        break;
      }
      case DemoScenario::kEmergencyBrake: {
        auto [s, st] = ScenarioEmergencyBrake(elapsed_s);
        tgt_speed_mps = s; tgt_steering_rad = st;
        break;
      }
      case DemoScenario::kSlalom: {
        auto [s, st] = ScenarioSlalom(elapsed_s);
        tgt_speed_mps = s; tgt_steering_rad = st;
        break;
      }
      case DemoScenario::kIdle:
      default:
        tgt_speed_mps = 0.0F;
        tgt_steering_rad = 0.0F;
        break;
    }

    // Check if operator has recently issued a command (blend if within 2 s).
    {
      std::lock_guard<std::mutex> lock(mu_);
      const auto now = std::chrono::steady_clock::now();
      const auto cmd_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - last_cmd_tp_).count();

      if (operator_has_control_ && cmd_age_ms < 2000) {
        // Operator override: use their inputs directly.
        const float max_steer = utils::DegToRad(35.0);
        tgt_steering_rad = last_operator_cmd_.steering_normalized *
                           static_cast<float>(max_steer);
        tgt_speed_mps    = last_operator_cmd_.throttle_normalized *
                           cfg_.cruise_speed_mps;
        if (last_operator_cmd_.brake_normalized > 0.01F) {
          tgt_speed_mps = 0.0F;
        }
      } else {
        operator_has_control_ = false;
      }

      // Simple first-order lag (slew rate limiter).
      const float dt    = 1.0F / static_cast<float>(rate_hz);
      const float accel = 2.0F;   // m/s²
      const float steer_rate = static_cast<float>(utils::DegToRad(30.0));  // rad/s

      const float dv    = utils::Clamp(tgt_speed_mps    - sim_speed_mps_,
                                       -accel * dt, accel * dt);
      const float dst   = utils::Clamp(tgt_steering_rad - sim_steering_rad_,
                                       -steer_rate * dt, steer_rate * dt);

      sim_speed_mps_    += dv;
      sim_steering_rad_ += dst;

      // Update heading from steering (bicycle model).
      const double wb = 2.7;  // wheelbase_m
      const double angular_vel = (wb > 0.0)
          ? static_cast<double>(sim_speed_mps_) *
            std::tan(static_cast<double>(sim_steering_rad_)) / wb
          : 0.0;
      sim_heading_rad_ += static_cast<float>(angular_vel * static_cast<double>(dt));

      // Update GPS position.
      const double dx = static_cast<double>(sim_speed_mps_) *
                        std::cos(static_cast<double>(sim_heading_rad_)) *
                        static_cast<double>(dt);
      const double dy = static_cast<double>(sim_speed_mps_) *
                        std::sin(static_cast<double>(sim_heading_rad_)) *
                        static_cast<double>(dt);

      // Approximate degree conversion (sufficient for demo).
      sim_lat_ += dy / kMetresPerDegreeLat;
      sim_lon_ += dx / (kMetresPerDegreeLat *
                        std::cos(sim_lat_ * M_PI / 180.0));

      // Discharge battery slowly (1% per 60 s at cruise).
      sim_battery_pct_ -= (static_cast<float>(sim_speed_mps_) / cfg_.cruise_speed_mps)
                          * dt / 60.0F;
      sim_battery_pct_ = utils::Clamp(sim_battery_pct_, 0.0F, 100.0F);
    }

    // Publish status via callback.
    if (status_cb_) {
      std::lock_guard<std::mutex> lock(mu_);
      NormalizedVehicleStatus st;
      st.speed_mps         = sim_speed_mps_;
      st.steering_rad      = sim_steering_rad_;
      st.gear              = 3;  // Drive
      st.battery_pct       = sim_battery_pct_;
      st.localization_valid = true;
      st.latitude           = sim_lat_;
      st.longitude          = sim_lon_;
      st.heading_rad        = sim_heading_rad_;
      st.timestamp_ns       = utils::NowNanos();
      status_cb_(st);
    }

    // Sleep until next tick.
    const auto elapsed = std::chrono::steady_clock::now() - loop_start;
    if (elapsed < period) {
      std::this_thread::sleep_for(period - elapsed);
    }
  }
}

// ── Scenario generators ───────────────────────────────────────────────────────

std::pair<float, float> SimulatedVehicle::ScenarioStraightLine(
    double t) const {
  // 0–3 s: accelerate, 3–15 s: cruise, 15–18 s: brake.
  const float cruise = cfg_.cruise_speed_mps;
  float speed = 0.0F;
  if (t < 3.0) {
    speed = cruise * static_cast<float>(t / 3.0);
  } else if (t < 15.0) {
    speed = cruise;
  } else if (t < 18.0) {
    speed = cruise * static_cast<float>(1.0 - (t - 15.0) / 3.0);
  }
  return {speed, 0.0F};
}

std::pair<float, float> SimulatedVehicle::ScenarioRoundAbout(
    double t) const {
  const float cruise = cfg_.cruise_speed_mps;
  const float radius = cfg_.loop_radius_m;
  const float steer  = (radius > 0.0F)
      ? std::atan2(2.7F, radius)   // Ackermann angle for given radius
      : 0.0F;
  // Accelerate into circle after 2 s ramp.
  const float speed = (t < 2.0)
      ? cruise * static_cast<float>(t / 2.0)
      : cruise;
  return {speed, steer};
}

std::pair<float, float> SimulatedVehicle::ScenarioEmergencyBrake(
    double t) const {
  const float cruise = cfg_.cruise_speed_mps;
  // 0–3 s: accelerate, 3–7 s: cruise, 7 s: emergency brake (handled externally).
  float speed = 0.0F;
  if (t < 3.0) {
    speed = cruise * static_cast<float>(t / 3.0);
  } else if (t < 7.0) {
    speed = cruise;
  }
  // After 7 s the vehicle is fully stopped (speed slew limiter in SimLoop handles decel).
  return {speed, 0.0F};
}

std::pair<float, float> SimulatedVehicle::ScenarioSlalom(double t) const {
  const float cruise = cfg_.cruise_speed_mps * 0.7F;
  const float max_steer = static_cast<float>(utils::DegToRad(20.0));
  const float steer = max_steer * static_cast<float>(
      std::sin(2.0 * M_PI * t / 4.0));  // 4-second period slalom
  return {cruise, steer};
}

}  // namespace demo
}  // namespace vehicle
}  // namespace fsm
