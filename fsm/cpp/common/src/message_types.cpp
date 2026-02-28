#include "fsm/message_types.hpp"
#include <chrono>
#include <sstream>

namespace fsm {
namespace common {

uint64_t NowMs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

uint64_t NowUs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

nlohmann::json Location::ToJson() const {
  return {{"lat", latitude}, {"lng", longitude},
          {"alt", altitude},  {"heading", heading}};
}

Location Location::FromJson(const nlohmann::json& j) {
  Location loc;
  loc.latitude  = j.value("lat", 0.0);
  loc.longitude = j.value("lng", 0.0);
  loc.altitude  = j.value("alt", 0.0);
  loc.heading   = j.value("heading", 0.0);
  return loc;
}

nlohmann::json TelemetryData::ToJson() const {
  return {{"cpu_usage", cpu_usage},
          {"gpu_usage", gpu_usage},
          {"memory_usage", memory_usage},
          {"network_quality", static_cast<int>(network_quality)},
          {"signal_strength", signal_strength}};
}

nlohmann::json SensorStatus::ToJson() const {
  return {{"cameras_online", cameras_online},
          {"cameras_total", cameras_total},
          {"lidar_online", lidar_online},
          {"gps_fix", gps_fix}};
}

nlohmann::json VehicleStatusMessage::ToJson() const {
  return {{"vehicle_id", vehicle_id},
          {"status", static_cast<int>(status)},
          {"type", static_cast<int>(type)},
          {"location", location.ToJson()},
          {"speed", speed},
          {"steering", steering},
          {"gear", static_cast<int>(gear)},
          {"battery_level", battery_level},
          {"emergency_level", emergency_level},
          {"latency_ms", latency_ms},
          {"telemetry", telemetry.ToJson()},
          {"sensors", sensors.ToJson()},
          {"timestamp", timestamp}};
}

nlohmann::json ControlCommand::ToJson() const {
  return {{"vehicle_id", vehicle_id},
          {"steering", steering},
          {"throttle", throttle},
          {"brake", brake},
          {"gear", static_cast<int>(gear)},
          {"turn_signal", turn_signal},
          {"emergency", emergency},
          {"sequence", sequence},
          {"timestamp", timestamp}};
}

ControlCommand ControlCommand::FromJson(const nlohmann::json& j) {
  ControlCommand cmd;
  cmd.vehicle_id  = j.value("vehicle_id", "");
  cmd.steering    = j.value("steering", 0.0f);
  cmd.throttle    = j.value("throttle", 0.0f);
  cmd.brake       = j.value("brake", 0.0f);
  cmd.gear        = static_cast<Gear>(j.value("gear", 0));
  cmd.turn_signal = j.value("turn_signal", 0);
  cmd.emergency   = j.value("emergency", false);
  cmd.sequence    = j.value("sequence", uint64_t{0});
  cmd.timestamp   = j.value("timestamp", uint64_t{0});
  return cmd;
}

nlohmann::json AlertMessage::ToJson() const {
  return {{"id", id},
          {"vehicle_id", vehicle_id},
          {"severity", static_cast<int>(severity)},
          {"message", message},
          {"category", category},
          {"timestamp", timestamp},
          {"acknowledged", acknowledged}};
}

nlohmann::json SchedulingQueueItem::ToJson() const {
  return {{"vehicle_id", vehicle_id},
          {"priority", priority_score},
          {"emergency_level", emergency_level},
          {"latency_ms", latency_ms},
          {"distance_km", distance_km}};
}

nlohmann::json PingMessage::ToJson() const {
  return {{"type", "ping"}, {"sequence", sequence}, {"timestamp", timestamp}};
}

nlohmann::json PongMessage::ToJson() const {
  return {{"type", "pong"},
          {"sequence", sequence},
          {"client_timestamp", client_timestamp},
          {"server_timestamp", server_timestamp}};
}

}  // namespace common
}  // namespace fsm
