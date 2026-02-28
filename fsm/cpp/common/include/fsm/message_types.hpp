#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace fsm {
namespace common {

enum class VehicleStatus {
  kUnknown = 0,
  kIdle = 1,
  kActive = 2,
  kEmergency = 3,
  kOffline = 4,
};

enum class VehicleType {
  kRoboTaxi = 0,
  kLogistics = 1,
  kSecurity = 2,
};

enum class Gear {
  kPark = 0,
  kReverse = 1,
  kNeutral = 2,
  kDrive = 3,
};

enum class NetworkQuality {
  kExcellent = 0,
  kGood = 1,
  kFair = 2,
  kPoor = 3,
};

enum class AlertSeverity {
  kInfo = 0,
  kWarning = 1,
  kCritical = 2,
  kEmergency = 3,
};

enum class WsEventType {
  kVehicleStatus,
  kControlCommand,
  kAlert,
  kSchedulingUpdate,
  kEmergency,
  kLatencyUpdate,
  kPing,
  kPong,
};

struct Location {
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  double heading = 0.0;

  nlohmann::json ToJson() const;
  static Location FromJson(const nlohmann::json& j);
};

struct TelemetryData {
  float cpu_usage = 0.0f;
  float gpu_usage = 0.0f;
  float memory_usage = 0.0f;
  NetworkQuality network_quality = NetworkQuality::kGood;
  int signal_strength = 0;

  nlohmann::json ToJson() const;
};

struct SensorStatus {
  int cameras_online = 0;
  int cameras_total = 0;
  bool lidar_online = false;
  bool gps_fix = false;

  nlohmann::json ToJson() const;
};

struct VehicleStatusMessage {
  std::string vehicle_id;
  VehicleStatus status = VehicleStatus::kUnknown;
  VehicleType type = VehicleType::kRoboTaxi;
  Location location;
  float speed = 0.0f;
  float steering = 0.0f;
  Gear gear = Gear::kPark;
  int battery_level = 0;
  int emergency_level = 0;
  float latency_ms = 0.0f;
  TelemetryData telemetry;
  SensorStatus sensors;
  uint64_t timestamp = 0;

  nlohmann::json ToJson() const;
};

struct ControlCommand {
  std::string vehicle_id;
  float steering = 0.0f;   // -1.0 to 1.0
  float throttle = 0.0f;   // 0.0 to 1.0
  float brake = 0.0f;      // 0.0 to 1.0
  Gear gear = Gear::kDrive;
  int turn_signal = 0;     // -1: left, 0: off, 1: right
  bool emergency = false;
  uint64_t sequence = 0;
  uint64_t timestamp = 0;

  nlohmann::json ToJson() const;
  static ControlCommand FromJson(const nlohmann::json& j);
};

struct AlertMessage {
  std::string id;
  std::string vehicle_id;
  AlertSeverity severity = AlertSeverity::kInfo;
  std::string message;
  std::string category;
  uint64_t timestamp = 0;
  bool acknowledged = false;

  nlohmann::json ToJson() const;
};

struct SchedulingQueueItem {
  std::string vehicle_id;
  float priority_score = 0.0f;
  int emergency_level = 0;
  float latency_ms = 0.0f;
  float distance_km = 0.0f;

  nlohmann::json ToJson() const;
};

struct PingMessage {
  uint64_t sequence = 0;
  uint64_t timestamp = 0;

  nlohmann::json ToJson() const;
};

struct PongMessage {
  uint64_t sequence = 0;
  uint64_t client_timestamp = 0;
  uint64_t server_timestamp = 0;

  nlohmann::json ToJson() const;
};

}  // namespace common
}  // namespace fsm
