#include "fsm/cloud/mqtt_relay.hpp"

#include <mosquitto.h>

#include <string>

namespace fsm {
namespace cloud {

// ── helpers ─────────────────────────────────────────────────────────────────

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

// ── static callbacks ─────────────────────────────────────────────────────────

void MqttRelay::OnConnect(struct mosquitto* /*mosq*/, void* self, int rc) {
  auto* r = static_cast<MqttRelay*>(self);
  if (rc == 0) {
    r->connected_ = true;
    FSM_LOG_INFO("Cloud MQTT relay connected: {}", r->cfg_.broker_url);
  } else {
    FSM_LOG_ERROR("Cloud MQTT relay connect rejected (rc={}): {}",
                  rc, mosquitto_connack_string(rc));
  }
}

void MqttRelay::OnDisconnect(struct mosquitto* /*mosq*/, void* self,
                               int /*rc*/) {
  auto* r = static_cast<MqttRelay*>(self);
  r->connected_ = false;
  FSM_LOG_WARN("Cloud MQTT relay disconnected — will reconnect");
}

// ── MqttRelay ────────────────────────────────────────────────────────────────

MqttRelay::MqttRelay(const config::CloudMqttConfig& config)
    : cfg_(config) {}

MqttRelay::~MqttRelay() {
  Stop();
}

bool MqttRelay::Start() {
  mosquitto_lib_init();

  mosq_ = mosquitto_new(cfg_.client_id.c_str(), true, this);
  if (!mosq_) {
    FSM_LOG_ERROR("MqttRelay: mosquitto_new failed");
    return false;
  }

  mosquitto_connect_callback_set   (mosq_, &MqttRelay::OnConnect);
  mosquitto_disconnect_callback_set(mosq_, &MqttRelay::OnDisconnect);

  if (!cfg_.username.empty())
    mosquitto_username_pw_set(mosq_, cfg_.username.c_str(),
                               cfg_.password.c_str());

  if (cfg_.use_tls && !cfg_.ca_cert_path.empty())
    mosquitto_tls_set(mosq_, cfg_.ca_cert_path.c_str(),
                      nullptr, nullptr, nullptr, nullptr);

  mosquitto_reconnect_delay_set(mosq_, 2, 30, true);

  std::string host; int port = 1883;
  ParseBrokerUrl(cfg_.broker_url, host, port);

  const int rc = mosquitto_connect_async(mosq_, host.c_str(), port,
                                          cfg_.keepalive_s);
  if (rc != MOSQ_ERR_SUCCESS) {
    FSM_LOG_ERROR("MqttRelay connect_async failed: {}", mosquitto_strerror(rc));
    mosquitto_destroy(mosq_);
    mosq_ = nullptr;
    mosquitto_lib_cleanup();
    return false;
  }

  mosquitto_loop_start(mosq_);
  FSM_LOG_INFO("MqttRelay started: {}", cfg_.broker_url);
  return true;
}

void MqttRelay::Stop() {
  if (!mosq_) return;
  mosquitto_loop_stop(mosq_, true);
  mosquitto_disconnect(mosq_);
  mosquitto_destroy(mosq_);
  mosq_ = nullptr;
  mosquitto_lib_cleanup();
  connected_ = false;
  FSM_LOG_INFO("MqttRelay stopped");
}

void MqttRelay::DoPublish(const std::string& topic,
                           const void* data, int len) {
  if (!mosq_ || !connected_.load()) return;
  const int rc = mosquitto_publish(mosq_, nullptr, topic.c_str(),
                                   len, data, cfg_.qos, false);
  if (rc != MOSQ_ERR_SUCCESS)
    FSM_LOG_WARN("MqttRelay publish failed ({}): {}", topic,
                 mosquitto_strerror(rc));
}

void MqttRelay::PublishTelemetry(const std::string& vehicle_id,
                                  const std::vector<uint8_t>& data) {
  DoPublish(cfg_.telemetry_pub_prefix + "/" + vehicle_id,
            data.data(), static_cast<int>(data.size()));
}

void MqttRelay::PublishStatus(const std::string& vehicle_id,
                               const std::vector<uint8_t>& data) {
  DoPublish(cfg_.status_pub_prefix + "/" + vehicle_id,
            data.data(), static_cast<int>(data.size()));
}

}  // namespace cloud
}  // namespace fsm
