#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "fsm/config_manager.hpp"

namespace fsm {
namespace vehicle {

// ---------------------------------------------------------------------------
// Supported platform types
// ---------------------------------------------------------------------------

enum class PlatformType {
  kUnknown,
  kAutoware,       // Autoware.universe (ROS2 Humble)
  kGenericRos2,    // Any ROS2 vehicle with configurable topics
  kApollo,         // Baidu Apollo (future)
  kCarlaSim,       // CARLA simulator
  kLgsvlSim,       // LGSVL simulator
  kCustom,
};

inline PlatformType PlatformTypeFromString(const std::string& s) {
  if (s == "autoware")     return PlatformType::kAutoware;
  if (s == "generic_ros2") return PlatformType::kGenericRos2;
  if (s == "apollo")       return PlatformType::kApollo;
  if (s == "carla_sim")    return PlatformType::kCarlaSim;
  if (s == "lgsvl_sim")    return PlatformType::kLgsvlSim;
  if (s == "custom")       return PlatformType::kCustom;
  return PlatformType::kUnknown;
}

// ---------------------------------------------------------------------------
// Normalized (platform-independent) data structures
// ---------------------------------------------------------------------------

// Operator → Vehicle: steering/throttle/brake in normalized [-1,1] / [0,1] range.
struct NormalizedControlCmd {
  float    steering_normalized = 0.0f;   // -1.0 (left) to +1.0 (right)
  float    throttle_normalized = 0.0f;   //  0.0 to 1.0
  float    brake_normalized    = 0.0f;   //  0.0 to 1.0
  int      gear                = 3;      // 0=P, 1=R, 2=N, 3=D
  int      turn_signal         = 0;      // 0=none, 1=left, 2=right, 3=hazard
  bool     emergency_stop      = false;
  int64_t  timestamp_ns        = 0;
  uint64_t sequence_number     = 0;
};

// Vehicle → Cloud: vehicle status snapshot in SI units.
struct NormalizedVehicleStatus {
  float   speed_mps          = 0.0f;
  float   steering_rad       = 0.0f;
  int     gear               = 3;
  float   battery_pct        = 100.0f;
  bool    localization_valid = false;
  double  latitude           = 0.0;
  double  longitude          = 0.0;
  float   heading_rad        = 0.0f;
  int64_t timestamp_ns       = 0;
};

// Vehicle capabilities advertised at connect time.
struct VehicleCapabilities {
  std::string  vehicle_id;
  PlatformType platform_type           = PlatformType::kUnknown;
  bool         has_steer_by_wire       = false;
  bool         has_brake_by_wire       = false;
  bool         has_throttle_by_wire    = false;
  bool         has_gear_control        = false;
  bool         has_turn_signal_control = false;
  bool         has_emergency_stop      = true;
  bool         has_lidar               = false;
  bool         has_radar               = false;
  bool         has_gps                 = true;
  uint32_t     num_cameras             = 0;
  float        max_speed_kph           = 60.0f;
  float        max_steering_angle_deg  = 35.0f;
  float        wheelbase_m             = 2.7f;
  std::string  firmware_version        = "1.0.0";
  std::vector<std::string> supported_codecs = {"h264"};
};

// ---------------------------------------------------------------------------
// Abstract PlatformAdapter interface
// ---------------------------------------------------------------------------

using VehicleStatusCallback = std::function<void(const NormalizedVehicleStatus&)>;

class PlatformAdapter {
 public:
  virtual ~PlatformAdapter() = default;

  PlatformAdapter(const PlatformAdapter&) = delete;
  PlatformAdapter& operator=(const PlatformAdapter&) = delete;

  // Initialize ROS2 publishers/subscribers. Returns false on error.
  virtual bool Initialize() = 0;

  // Clean shutdown.
  virtual void Shutdown() = 0;

  // Apply a normalized control command to the vehicle platform.
  virtual void ApplyCommand(const NormalizedControlCmd& cmd) = 0;

  // Return the static capability set of this vehicle.
  virtual VehicleCapabilities GetCapabilities() const = 0;

  // Register callback invoked every time vehicle status is updated from ROS2.
  void set_status_callback(VehicleStatusCallback cb) {
    status_cb_ = std::move(cb);
  }

  // Factory: create the correct implementation for the given platform.
  static std::unique_ptr<PlatformAdapter> Create(
      PlatformType type,
      rclcpp::Node::SharedPtr node,
      const config::VehicleConfigManager& config);

 protected:
  PlatformAdapter() = default;

  VehicleStatusCallback status_cb_;
};

}  // namespace vehicle
}  // namespace fsm
