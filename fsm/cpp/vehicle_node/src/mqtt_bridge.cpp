#include "fsm/vehicle/mqtt_bridge.hpp"

#include <mosquitto.h>
#include <openssl/hmac.h>
#include <openssl/evp.h>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace fsm {
namespace vehicle {

// ============================================================================
// Internal helpers
// ============================================================================

static std::string SubstitutePlaceholder(const std::string& tmpl,
                                         const std::string& vehicle_id) {
  std::string result = tmpl;
  const std::string kTag = "{vehicle_id}";
  for (auto pos = result.find(kTag); pos != std::string::npos;
       pos = result.find(kTag, pos)) {
    result.replace(pos, kTag.size(), vehicle_id);
  }
  return result;
}

static std::string HexEncode(const uint8_t* data, size_t len) {
  std::ostringstream oss;
  for (size_t i = 0; i < len; ++i)
    oss << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(data[i]);
  return oss.str();
}

static std::string HmacSha256Hex(const std::string& key,
                                  const std::string& data) {
  uint8_t digest[32];
  unsigned int dlen = 32;
  HMAC(EVP_sha256(),
       key.data(),  static_cast<int>(key.size()),
       reinterpret_cast<const uint8_t*>(data.data()), data.size(),
       digest, &dlen);
  return HexEncode(digest, dlen);
}

// Parse "tcp://host:port" or "ssl://host:port" into host / port.
static void ParseBrokerUrl(const std::string& url,
                            std::string& host, int& port) {
  std::string rest = url;
  if (rest.rfind("tcp://", 0) == 0) {
    rest = rest.substr(6); port = 1883;
  } else if (rest.rfind("ssl://", 0) == 0) {
    rest = rest.substr(6); port = 8883;
  } else {
    port = 1883;
  }
  const auto colon = rest.rfind(':');
  if (colon != std::string::npos) {
    host = rest.substr(0, colon);
    try { port = std::stoi(rest.substr(colon + 1)); } catch (...) {}
  } else {
    host = rest;
  }
}

// ============================================================================
// Alibaba Cloud IoT Platform authentication
//
// Ref: https://help.aliyun.com/document_detail/73742.html
//   clientId  = "{dn}|securemode=2,signmethod=hmacsha256,timestamp={ts}|"
//   username  = "{dn}&{pk}"
//   sign_str  = "clientId{dn}deviceName{dn}productKey{pk}timestamp{ts}"
//   password  = HMAC-SHA256(sign_str, deviceSecret)   [lower-case hex]
//   broker    = tcp://{pk}.iot-as-mqtt.{region}.aliyuncs.com:1883
// ============================================================================

config::MqttConfig MqttBridge::ResolveAliyunAuth(const config::MqttConfig& in,
                                                   const std::string& vehicle_id) {
  config::MqttConfig cfg = in;
  const std::string& pk = cfg.aliyun_product_key;
  const std::string  dn = cfg.aliyun_device_name.empty()
                               ? vehicle_id : cfg.aliyun_device_name;
  const std::string& ds = cfg.aliyun_device_secret;

  if (pk.empty() || ds.empty()) {
    FSM_LOG_ERROR("Aliyun mode requires product_key and device_secret");
    return cfg;
  }

  const auto ts = std::to_string(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());

  cfg.client_id  = dn + "|securemode=2,signmethod=hmacsha256,timestamp=" + ts + "|";
  cfg.username   = dn + "&" + pk;
  cfg.password   = HmacSha256Hex(ds,
      "clientId"   + dn +
      "deviceName" + dn +
      "productKey" + pk +
      "timestamp"  + ts);
  cfg.broker_url = "tcp://" + pk + ".iot-as-mqtt." +
                   cfg.aliyun_region + ".aliyuncs.com:1883";
  FSM_LOG_INFO("Aliyun IoT: broker={} clientId={}", cfg.broker_url, cfg.client_id);
  return cfg;
}

// ============================================================================
// Static mosquitto callbacks
// ============================================================================

void MqttBridge::OnConnect(struct mosquitto* /*mosq*/, void* self, int rc) {
  auto* b = static_cast<MqttBridge*>(self);
  if (rc == 0) {
    b->connected_ = true;
    FSM_LOG_INFO("MQTT connected: {}", b->cfg_.broker_url);
    // Subscribe to control topic after (re)connect.
    mosquitto_subscribe(b->mosq_, nullptr,
                        b->control_topic_.c_str(), b->cfg_.qos);
    std::lock_guard<std::mutex> lk(b->cb_mutex_);
    if (b->connect_cb_) b->connect_cb_(true);
  } else {
    FSM_LOG_ERROR("MQTT connect rejected (rc={}): {}",
                  rc, mosquitto_connack_string(rc));
    std::lock_guard<std::mutex> lk(b->cb_mutex_);
    if (b->connect_cb_) b->connect_cb_(false);
  }
}

void MqttBridge::OnDisconnect(struct mosquitto* /*mosq*/, void* self,
                               int /*rc*/) {
  auto* b = static_cast<MqttBridge*>(self);
  b->connected_ = false;
  FSM_LOG_WARN("MQTT disconnected — mosquitto will auto-reconnect");
  std::lock_guard<std::mutex> lk(b->cb_mutex_);
  if (b->connect_cb_) b->connect_cb_(false);
}

void MqttBridge::OnMessage(struct mosquitto* /*mosq*/, void* self,
                            const struct mosquitto_message* msg) {
  if (!msg || !msg->payload) return;
  auto* b = static_cast<MqttBridge*>(self);
  const std::string payload(static_cast<const char*>(msg->payload),
                             static_cast<size_t>(msg->payloadlen));
  std::lock_guard<std::mutex> lk(b->cb_mutex_);
  if (b->control_cb_) b->control_cb_(payload);
}

// ============================================================================
// MqttBridge public interface
// ============================================================================

MqttBridge::MqttBridge(const config::MqttConfig& config,
                        const std::string& vehicle_id)
    : cfg_(config), vehicle_id_(vehicle_id) {
  if (cfg_.aliyun_mode)
    cfg_ = ResolveAliyunAuth(cfg_, vehicle_id_);

  telemetry_topic_ = SubstitutePlaceholder(cfg_.telemetry_topic, vehicle_id_);
  status_topic_    = SubstitutePlaceholder(cfg_.status_topic,    vehicle_id_);
  control_topic_   = SubstitutePlaceholder(cfg_.control_topic,   vehicle_id_);
}

MqttBridge::~MqttBridge() {
  Stop();
}

bool MqttBridge::Start() {
  mosquitto_lib_init();

  const std::string client_id = cfg_.client_id.empty()
      ? ("fsm_vehicle_" + vehicle_id_) : cfg_.client_id;

  mosq_ = mosquitto_new(client_id.c_str(), /*clean_session=*/true, this);
  if (!mosq_) {
    FSM_LOG_ERROR("mosquitto_new failed (out of memory?)");
    return false;
  }

  mosquitto_connect_callback_set   (mosq_, &MqttBridge::OnConnect);
  mosquitto_disconnect_callback_set(mosq_, &MqttBridge::OnDisconnect);
  mosquitto_message_callback_set   (mosq_, &MqttBridge::OnMessage);

  if (!cfg_.username.empty())
    mosquitto_username_pw_set(mosq_, cfg_.username.c_str(),
                               cfg_.password.c_str());

  if (cfg_.use_tls && !cfg_.ca_cert_path.empty()) {
    const int rc = mosquitto_tls_set(mosq_, cfg_.ca_cert_path.c_str(),
                                      nullptr, nullptr, nullptr, nullptr);
    if (rc != MOSQ_ERR_SUCCESS)
      FSM_LOG_ERROR("MQTT TLS setup failed: {}", mosquitto_strerror(rc));
  }

  // mosquitto_reconnect_delay_set provides exponential back-off on failures.
  mosquitto_reconnect_delay_set(mosq_, 2, 30, true);

  std::string host; int port = 1883;
  ParseBrokerUrl(cfg_.broker_url, host, port);

  const int rc = mosquitto_connect_async(mosq_, host.c_str(), port,
                                          cfg_.keepalive_s);
  if (rc != MOSQ_ERR_SUCCESS) {
    FSM_LOG_ERROR("mosquitto_connect_async failed: {}", mosquitto_strerror(rc));
    mosquitto_destroy(mosq_);
    mosq_ = nullptr;
    mosquitto_lib_cleanup();
    return false;
  }

  // Loop runs in a background thread managed by mosquitto.
  mosquitto_loop_start(mosq_);

  FSM_LOG_INFO("MqttBridge started: {} | telemetry={} control={}",
               cfg_.broker_url, telemetry_topic_, control_topic_);
  return true;
}

void MqttBridge::Stop() {
  if (!mosq_) return;
  stop_ = true;
  mosquitto_loop_stop(mosq_, /*force=*/true);
  mosquitto_disconnect(mosq_);
  mosquitto_destroy(mosq_);
  mosq_ = nullptr;
  mosquitto_lib_cleanup();
  connected_ = false;
  FSM_LOG_INFO("MqttBridge stopped");
}

void MqttBridge::DoPublish(const std::string& topic,
                            const void* data, int len) {
  if (!mosq_ || !connected_.load()) return;
  const int rc = mosquitto_publish(mosq_, nullptr, topic.c_str(),
                                   len, data, cfg_.qos, cfg_.retain);
  if (rc != MOSQ_ERR_SUCCESS)
    FSM_LOG_WARN("MQTT publish failed ({}): {}", topic, mosquitto_strerror(rc));
}

void MqttBridge::PublishTelemetry(const std::vector<uint8_t>& data) {
  DoPublish(telemetry_topic_, data.data(), static_cast<int>(data.size()));
}

void MqttBridge::PublishStatus(const std::vector<uint8_t>& data) {
  DoPublish(status_topic_, data.data(), static_cast<int>(data.size()));
}

void MqttBridge::set_control_callback(MqttControlCallback cb) {
  std::lock_guard<std::mutex> lk(cb_mutex_);
  control_cb_ = std::move(cb);
}

void MqttBridge::set_connect_callback(MqttConnectCallback cb) {
  std::lock_guard<std::mutex> lk(cb_mutex_);
  connect_cb_ = std::move(cb);
}

}  // namespace vehicle
}  // namespace fsm
