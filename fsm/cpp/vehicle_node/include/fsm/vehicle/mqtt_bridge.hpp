#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"

// Forward-declare the mosquitto struct so we do not expose its header here.
struct mosquitto;

namespace fsm {
namespace vehicle {

// Invoked with raw JSON payload when a control message arrives via MQTT.
using MqttControlCallback = std::function<void(const std::string& payload)>;
// Invoked when the MQTT connection state changes (true = connected).
using MqttConnectCallback = std::function<void(bool connected)>;

// ---------------------------------------------------------------------------
// MqttBridge — enterprise MQTT transport layer (vehicle side).
//
// Runs alongside the WebSocket client.  Vehicle telemetry (protobuf bytes) is
// published to fsm/telemetry/{vehicle_id}; control commands arrive on
// fsm/control/{vehicle_id}.
//
// Authentication modes
//  • Plain TCP / TLS: set broker_url, username, password directly.
//  • Alibaba Cloud IoT Platform: set aliyun_mode=true and provide
//    aliyun_product_key / aliyun_device_name / aliyun_device_secret.
//    The broker URL, client-id, username and HMAC-SHA256 password are
//    derived automatically per the Alibaba IoT MQTT spec.
// ---------------------------------------------------------------------------
class MqttBridge {
 public:
  explicit MqttBridge(const config::MqttConfig& config,
                      const std::string& vehicle_id);
  ~MqttBridge();

  MqttBridge(const MqttBridge&) = delete;
  MqttBridge& operator=(const MqttBridge&) = delete;

  [[nodiscard]] bool Start();
  void Stop();

  // Publish serialised protobuf bytes to the configured topic.
  void PublishTelemetry(const std::vector<uint8_t>& data);
  void PublishStatus(const std::vector<uint8_t>& data);

  bool is_connected() const { return connected_.load(); }

  void set_control_callback(MqttControlCallback cb);
  void set_connect_callback(MqttConnectCallback cb);

 private:
  // Resolve Alibaba Cloud IoT credentials into a concrete MqttConfig.
  static config::MqttConfig ResolveAliyunAuth(const config::MqttConfig& cfg,
                                               const std::string& vehicle_id);

  // mosquitto static callback shims.
  static void OnConnect   (struct mosquitto*, void* self, int rc);
  static void OnDisconnect(struct mosquitto*, void* self, int rc);
  static void OnMessage   (struct mosquitto*, void* self,
                            const struct mosquitto_message* msg);

  void DoPublish(const std::string& topic, const void* data, int len);

  config::MqttConfig  cfg_;
  std::string         vehicle_id_;
  std::string         telemetry_topic_;
  std::string         status_topic_;
  std::string         control_topic_;

  struct mosquitto*   mosq_{nullptr};
  std::atomic<bool>   connected_{false};
  std::atomic<bool>   stop_{false};

  mutable std::mutex  cb_mutex_;
  MqttControlCallback control_cb_;
  MqttConnectCallback connect_cb_;
};

}  // namespace vehicle
}  // namespace fsm
