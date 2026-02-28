#pragma once

#include <functional>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>
#include "fsm/config_manager.hpp"

namespace fsm {
namespace cloud {

class WebSocketSession;

// Callback invoked whenever a vehicle posts a telemetry message.
// Parameters: vehicle_id, parsed JSON payload.
using TelemetryRelayCallback =
    std::function<void(const std::string& vehicle_id,
                       const nlohmann::json& payload)>;

// Callbacks for vehicle connect / disconnect events.
using VehicleEventCallback = std::function<void(const std::string& vehicle_id)>;

class SignalingServer {
 public:
  explicit SignalingServer(const config::CloudConfigManager& config);
  ~SignalingServer();

  SignalingServer(const SignalingServer&) = delete;
  SignalingServer& operator=(const SignalingServer&) = delete;

  [[nodiscard]] bool Start();
  void Stop();

  // Register a callback that fires on every relayed telemetry message.
  void set_telemetry_callback(TelemetryRelayCallback cb);

  // Register callbacks for vehicle connect/disconnect events.
  void set_connect_callback(VehicleEventCallback cb);
  void set_disconnect_callback(VehicleEventCallback cb);

  void AddSession(const std::string& id,
                  std::shared_ptr<WebSocketSession> session);
  void RemoveSession(const std::string& id);
  std::shared_ptr<WebSocketSession> GetSession(const std::string& id) const;
  std::shared_ptr<WebSocketSession> FindVehicleSession(
      const std::string& vehicle_id) const;

  void NotifyVehicleOnline(const std::string& vehicle_id);
  void BroadcastToOperators(const std::string& message);

  // Called by WebSocketSession on telemetry relay.
  void OnVehicleTelemetry(const std::string& vehicle_id,
                          const nlohmann::json& payload);
  void OnVehicleConnect(const std::string& vehicle_id);
  void OnVehicleDisconnect(const std::string& vehicle_id);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace cloud
}  // namespace fsm
