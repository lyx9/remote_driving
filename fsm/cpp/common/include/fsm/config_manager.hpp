#pragma once

#include <optional>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace fsm {
namespace config {

// ROS2 QoS profile for a single topic.
struct QosConfig {
  std::string reliability = "best_effort";  // "reliable" | "best_effort"
  std::string durability  = "volatile";     // "volatile"  | "transient_local"
  int         depth       = 10;             // message queue depth
};

struct CameraConfig {
  std::string id;
  std::string topic;
  int fps = 30;
  int width = 1920;
  int height = 1080;
  std::string encoding = "h264";
  int bitrate_bps = 4000000;
  bool enabled = true;
  // Per-camera QoS (default: best_effort/volatile for high-frequency image streams)
  QosConfig qos{"best_effort", "volatile", 1};
};

struct VehicleParams {
  double wheelbase_m = 2.7;
  double max_steering_angle_deg = 35.0;
  double max_speed_kph = 60.0;
  double max_acceleration_mps2 = 2.0;
  double max_deceleration_mps2 = 5.0;
  double track_width_m = 1.5;
};

struct WebRtcConfig {
  std::vector<std::string> stun_servers;
  std::vector<std::string> turn_servers;
  std::string turn_username;
  std::string turn_password;
  std::string signaling_url;
  int reconnect_interval_ms = 5000;
  int ice_timeout_ms = 10000;
};

struct ControlTopics {
  std::string steering_cmd = "/control/command/steering_cmd";
  std::string velocity_cmd = "/control/command/velocity_cmd";
  std::string gear_cmd = "/vehicle/command/gear_cmd";
  std::string turn_signal_cmd = "/vehicle/command/turn_signal_cmd";
  std::string emergency_cmd = "/system/emergency/emergency_cmd";
};

struct SensorTopics {
  std::string velocity_status = "/vehicle/status/velocity_status";
  std::string steering_status = "/vehicle/status/steering_status";
  std::string gear_status = "/vehicle/status/gear_status";
  std::string control_mode = "/vehicle/status/control_mode";
  std::string localization = "/localization/kinematic_state";
  std::string diagnostics = "/diagnostics_agg";
};

struct SafetyConfig {
  double max_steering_rate_dps = 30.0;
  double emergency_decel_mps2 = 9.0;
  int watchdog_timeout_ms = 200;
  bool soft_limits_enabled = true;
  bool emergency_stop_enabled = true;
  // Max rate (Hz) at which ROS2 control commands are published.
  // 0 = unlimited (publish on every incoming command).
  int command_publish_rate_hz = 50;
};

// Per-group QoS for sensor subscriptions.
struct SensorQosConfig {
  QosConfig cameras    {"best_effort", "volatile", 1};   // image streams
  QosConfig vehicle    {"best_effort", "volatile", 10};  // velocity/steering/gear
  QosConfig localization{"best_effort", "volatile", 10}; // odometry
};

// QoS for control command publishers + optional publish-rate cap.
struct ControlQosConfig {
  QosConfig steering_cmd  {"reliable", "volatile", 10};
  QosConfig gear_cmd      {"reliable", "volatile", 10};
  QosConfig turn_signal   {"reliable", "volatile", 10};
  QosConfig emergency_cmd {"reliable", "volatile", 10};
};

// MQTT transport configuration.
// When mqtt.enabled=true the vehicle also publishes telemetry to and
// receives control commands from an MQTT broker (alongside WebSocket).
struct MqttConfig {
  bool        enabled     = false;
  // Full broker URL, e.g. "tcp://localhost:1883" or "ssl://host:8883".
  std::string broker_url  = "tcp://localhost:1883";
  std::string client_id;           // auto-generated from vehicle_id if empty
  std::string username;
  std::string password;
  bool        use_tls     = false;
  std::string ca_cert_path;        // path to PEM CA file for TLS verification
  int         keepalive_s = 60;
  int         qos         = 1;     // MQTT QoS level: 0 / 1 / 2
  bool        retain      = false;

  // Topic templates — {vehicle_id} will be substituted at runtime.
  std::string telemetry_topic = "fsm/telemetry/{vehicle_id}";
  std::string control_topic   = "fsm/control/{vehicle_id}";
  std::string status_topic    = "fsm/status/{vehicle_id}";

  // ── Alibaba Cloud IoT Platform ─────────────────────────────────────────
  // Set aliyun_mode=true to derive broker_url, client_id, username and
  // password automatically from the product/device triple-tuple.
  bool        aliyun_mode          = false;
  std::string aliyun_product_key;
  std::string aliyun_device_name;
  std::string aliyun_device_secret;
  std::string aliyun_region        = "cn-shanghai";
};

// Cloud-side MQTT relay: the cloud server re-publishes received vehicle
// telemetry to an MQTT broker for downstream analytics / monitoring tools.
struct CloudMqttConfig {
  bool        enabled              = false;
  std::string broker_url           = "tcp://localhost:1883";
  std::string client_id            = "fsm_cloud_relay";
  std::string username;
  std::string password;
  bool        use_tls              = false;
  std::string ca_cert_path;
  int         keepalive_s          = 60;
  int         qos                  = 1;
  // Cloud subscribes to these prefixes from vehicles and re-publishes locally.
  std::string telemetry_pub_prefix = "fsm/telemetry";  // /{vehicle_id}
  std::string status_pub_prefix    = "fsm/status";     // /{vehicle_id}
};

struct SchedulingWeights {
  double emergency = 0.35;
  double latency = 0.25;
  double distance = 0.20;
  double battery = 0.10;
  double task_priority = 0.10;
};

struct SchedulingConfig {
  bool enabled = true;
  std::string algorithm = "weighted_priority";
  SchedulingWeights weights;
  int max_latency_ms = 200;
  int critical_latency_ms = 500;
  double min_battery_pct = 20.0;
};

struct PredictionConfig {
  bool enabled = false;
  std::string model_endpoint;
  bool compensation_enabled = false;
  double prediction_horizon_s = 2.0;
  int update_rate_hz = 10;
};

struct AlertRuleConfig {
  std::string name;
  std::string condition;
  std::string severity;
  bool enabled = true;
};

// ---------------------------------------------------------------------------

class VehicleConfigManager {
 public:
  VehicleConfigManager() = default;

  [[nodiscard]] bool LoadFromFile(const std::string& path);
  [[nodiscard]] bool SaveToFile(const std::string& path) const;

  const std::string& vehicle_id() const { return vehicle_id_; }
  const std::string& vehicle_type() const { return vehicle_type_; }
  const std::string& vehicle_name() const { return vehicle_name_; }
  const VehicleParams& vehicle_params() const { return vehicle_params_; }
  const std::vector<CameraConfig>& cameras() const { return cameras_; }
  const WebRtcConfig& webrtc_config() const { return webrtc_config_; }
  const ControlTopics& control_topics() const { return control_topics_; }
  const SensorTopics& sensor_topics() const { return sensor_topics_; }
  const SafetyConfig& safety_config() const { return safety_config_; }
  const SensorQosConfig&  sensor_qos()  const { return sensor_qos_; }
  const ControlQosConfig& control_qos() const { return control_qos_; }
  const MqttConfig&       mqtt_config() const { return mqtt_config_; }

  void set_vehicle_id(const std::string& id) { vehicle_id_ = id; }
  void set_webrtc_config(const WebRtcConfig& cfg) { webrtc_config_ = cfg; }

  // Returns nullopt if no camera with the given id exists.
  std::optional<CameraConfig> FindCameraById(const std::string& id) const;

 private:
  void ParseVehicle(const YAML::Node& node);
  void ParseSensors(const YAML::Node& node);
  void ParseWebRtc(const YAML::Node& node);
  void ParseControl(const YAML::Node& node);
  void ParseSafety(const YAML::Node& node);
  void ParseMqtt(const YAML::Node& node);

  std::string vehicle_id_;
  std::string vehicle_type_;
  std::string vehicle_name_;
  VehicleParams vehicle_params_;
  std::vector<CameraConfig> cameras_;
  WebRtcConfig webrtc_config_;
  ControlTopics control_topics_;
  SensorTopics sensor_topics_;
  SafetyConfig safety_config_;
  SensorQosConfig  sensor_qos_;
  ControlQosConfig control_qos_;
  MqttConfig       mqtt_config_;
};

// ---------------------------------------------------------------------------

class CloudConfigManager {
 public:
  CloudConfigManager() = default;

  [[nodiscard]] bool LoadFromFile(const std::string& path);
  [[nodiscard]] bool SaveToFile(const std::string& path) const;

  int signaling_port() const { return signaling_port_; }
  int api_port() const { return api_port_; }
  const SchedulingConfig& scheduling_config() const { return scheduling_config_; }
  const std::vector<AlertRuleConfig>& alert_rules() const { return alert_rules_; }
  const PredictionConfig& prediction_config() const { return prediction_config_; }
  const CloudMqttConfig&  cloud_mqtt_config() const { return cloud_mqtt_config_; }

  // Returns JWT secret for REST API authentication.
  // Empty string means authentication is disabled.
  const std::string& api_jwt_secret() const { return api_jwt_secret_; }

  // Session timeout in seconds (0 = use default 300 s).
  int session_timeout_s() const { return session_timeout_s_; }

  void set_scheduling_enabled(bool enabled) { scheduling_config_.enabled = enabled; }
  void set_prediction_enabled(bool enabled) { prediction_config_.enabled = enabled; }

 private:
  void ParseServer(const YAML::Node& node);
  void ParseScheduling(const YAML::Node& node);
  void ParseAlerts(const YAML::Node& node);
  void ParsePrediction(const YAML::Node& node);
  void ParseCloudMqtt(const YAML::Node& node);
  void ParseApi(const YAML::Node& node);

  int signaling_port_ = 8080;
  int api_port_ = 8081;
  SchedulingConfig scheduling_config_;
  std::vector<AlertRuleConfig> alert_rules_;
  PredictionConfig prediction_config_;
  CloudMqttConfig  cloud_mqtt_config_;
  std::string      api_jwt_secret_;    // empty = auth disabled
  int              session_timeout_s_ = 300;
};

}  // namespace config
}  // namespace fsm
