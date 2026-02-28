#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"

struct mosquitto;

namespace fsm {
namespace cloud {

// ---------------------------------------------------------------------------
// MqttRelay — cloud-side MQTT bridge.
//
// Receives vehicle telemetry (protobuf bytes) from the SignalingServer via its
// TelemetryRelayCallback, then re-publishes the bytes to the MQTT broker so
// that external consumers (dashboards, analytics, BI tools) can subscribe.
//
// Topic layout:
//   fsm/telemetry/{vehicle_id}  — protobuf TelemetryData (serialised bytes)
//   fsm/status/{vehicle_id}     — protobuf SystemStatus  (serialised bytes)
//
// The class can also forward incoming MQTT operator commands back to the
// cloud WebSocket layer (future extension; hook is left as a comment below).
// ---------------------------------------------------------------------------
class MqttRelay {
 public:
  explicit MqttRelay(const config::CloudMqttConfig& config);
  ~MqttRelay();

  MqttRelay(const MqttRelay&) = delete;
  MqttRelay& operator=(const MqttRelay&) = delete;

  [[nodiscard]] bool Start();
  void Stop();

  // Called by the SignalingServer telemetry callback.
  void PublishTelemetry(const std::string& vehicle_id,
                        const std::vector<uint8_t>& proto_data);
  void PublishStatus(const std::string& vehicle_id,
                     const std::vector<uint8_t>& proto_data);

  bool is_connected() const { return connected_.load(); }

 private:
  static void OnConnect   (struct mosquitto*, void* self, int rc);
  static void OnDisconnect(struct mosquitto*, void* self, int rc);

  void DoPublish(const std::string& topic, const void* data, int len);

  config::CloudMqttConfig cfg_;
  struct mosquitto*        mosq_{nullptr};
  std::atomic<bool>        connected_{false};
};

}  // namespace cloud
}  // namespace fsm
