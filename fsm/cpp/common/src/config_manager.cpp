#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"
#include <fstream>

namespace fsm {
namespace config {

// ---------------------------------------------------------------------------
// VehicleConfigManager
// ---------------------------------------------------------------------------

bool VehicleConfigManager::LoadFromFile(const std::string& path) {
  try {
    YAML::Node cfg = YAML::LoadFile(path);
    if (cfg["vehicle"]) ParseVehicle(cfg["vehicle"]);
    if (cfg["sensors"]) ParseSensors(cfg["sensors"]);
    if (cfg["webrtc"])  ParseWebRtc(cfg["webrtc"]);
    if (cfg["control"]) ParseControl(cfg["control"]);
    if (cfg["mqtt"])    ParseMqtt(cfg["mqtt"]);
    FSM_LOG_INFO("Vehicle config loaded: {}", path);
    return true;
  } catch (const YAML::Exception& e) {
    FSM_LOG_ERROR("Failed to load vehicle config: {}", e.what());
    return false;
  }
}

bool VehicleConfigManager::SaveToFile(const std::string& path) const {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;

    out << YAML::Key << "vehicle" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "id"   << YAML::Value << vehicle_id_;
    out << YAML::Key << "type" << YAML::Value << vehicle_type_;
    out << YAML::Key << "name" << YAML::Value << vehicle_name_;
    out << YAML::Key << "parameters" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "wheelbase"          << YAML::Value << vehicle_params_.wheelbase_m;
    out << YAML::Key << "max_steering_angle" << YAML::Value << vehicle_params_.max_steering_angle_deg;
    out << YAML::Key << "max_speed"          << YAML::Value << vehicle_params_.max_speed_kph;
    out << YAML::EndMap;
    out << YAML::EndMap;

    out << YAML::Key << "sensors" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "cameras" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "list" << YAML::Value << YAML::BeginSeq;
    for (const auto& cam : cameras_) {
      out << YAML::BeginMap;
      out << YAML::Key << "id"      << YAML::Value << cam.id;
      out << YAML::Key << "topic"   << YAML::Value << cam.topic;
      out << YAML::Key << "fps"     << YAML::Value << cam.fps;
      out << YAML::Key << "width"   << YAML::Value << cam.width;
      out << YAML::Key << "height"  << YAML::Value << cam.height;
      out << YAML::Key << "bitrate" << YAML::Value << cam.bitrate_bps;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
    out << YAML::EndMap;

    out << YAML::EndMap;

    std::ofstream fout(path);
    fout << out.c_str();
    FSM_LOG_INFO("Vehicle config saved: {}", path);
    return true;
  } catch (const std::exception& e) {
    FSM_LOG_ERROR("Failed to save vehicle config: {}", e.what());
    return false;
  }
}

std::optional<CameraConfig> VehicleConfigManager::FindCameraById(
    const std::string& id) const {
  for (const auto& cam : cameras_) {
    if (cam.id == id) return cam;
  }
  return std::nullopt;
}

void VehicleConfigManager::ParseVehicle(const YAML::Node& node) {
  if (node["id"])   vehicle_id_   = node["id"].as<std::string>();
  if (node["type"]) vehicle_type_ = node["type"].as<std::string>();
  if (node["name"]) vehicle_name_ = node["name"].as<std::string>();

  if (node["parameters"]) {
    const auto& p = node["parameters"];
    if (p["wheelbase"])          vehicle_params_.wheelbase_m            = p["wheelbase"].as<double>();
    if (p["max_steering_angle"]) vehicle_params_.max_steering_angle_deg = p["max_steering_angle"].as<double>();
    if (p["max_speed"])          vehicle_params_.max_speed_kph          = p["max_speed"].as<double>();
    if (p["max_acceleration"])   vehicle_params_.max_acceleration_mps2  = p["max_acceleration"].as<double>();
    if (p["max_deceleration"])   vehicle_params_.max_deceleration_mps2  = p["max_deceleration"].as<double>();
    if (p["track_width"])        vehicle_params_.track_width_m          = p["track_width"].as<double>();
  }
}

void VehicleConfigManager::ParseSensors(const YAML::Node& node) {
  // Helper: parse a QosConfig node.
  auto parseQos = [](const YAML::Node& q, QosConfig& out) {
    if (q["reliability"]) out.reliability = q["reliability"].as<std::string>();
    if (q["durability"])  out.durability  = q["durability"].as<std::string>();
    if (q["depth"])       out.depth       = q["depth"].as<int>();
  };

  if (node["cameras"] && node["cameras"]["list"]) {
    cameras_.clear();
    for (const auto& n : node["cameras"]["list"]) {
      CameraConfig cam;
      if (n["id"])       cam.id          = n["id"].as<std::string>();
      if (n["topic"])    cam.topic       = n["topic"].as<std::string>();
      if (n["fps"])      cam.fps         = n["fps"].as<int>();
      if (n["width"])    cam.width       = n["width"].as<int>();
      if (n["height"])   cam.height      = n["height"].as<int>();
      if (n["encoding"]) cam.encoding    = n["encoding"].as<std::string>();
      if (n["bitrate"])  cam.bitrate_bps = n["bitrate"].as<int>();
      if (n["enabled"])  cam.enabled     = n["enabled"].as<bool>();
      if (n["qos"])      parseQos(n["qos"], cam.qos);
      cameras_.push_back(cam);
    }
  }

  if (node["topics"]) {
    const auto& t = node["topics"];
    if (t["velocity_status"]) sensor_topics_.velocity_status = t["velocity_status"].as<std::string>();
    if (t["steering_status"]) sensor_topics_.steering_status = t["steering_status"].as<std::string>();
    if (t["gear_status"])     sensor_topics_.gear_status     = t["gear_status"].as<std::string>();
    if (t["control_mode"])    sensor_topics_.control_mode    = t["control_mode"].as<std::string>();
    if (t["localization"])    sensor_topics_.localization    = t["localization"].as<std::string>();
    if (t["diagnostics"])     sensor_topics_.diagnostics     = t["diagnostics"].as<std::string>();
  }

  if (node["topics_qos"]) {
    const auto& tq = node["topics_qos"];
    if (tq["cameras"])      parseQos(tq["cameras"],      sensor_qos_.cameras);
    if (tq["vehicle"])      parseQos(tq["vehicle"],      sensor_qos_.vehicle);
    if (tq["localization"]) parseQos(tq["localization"], sensor_qos_.localization);
  }
}

void VehicleConfigManager::ParseWebRtc(const YAML::Node& node) {
  if (node["stun_servers"]) {
    webrtc_config_.stun_servers.clear();
    for (const auto& s : node["stun_servers"])
      webrtc_config_.stun_servers.push_back(s.as<std::string>());
  }

  if (node["turn_servers"]) {
    webrtc_config_.turn_servers.clear();
    for (const auto& s : node["turn_servers"]) {
      if (s["url"])        webrtc_config_.turn_servers.push_back(s["url"].as<std::string>());
      if (s["username"])   webrtc_config_.turn_username = s["username"].as<std::string>();
      if (s["credential"]) webrtc_config_.turn_password = s["credential"].as<std::string>();
    }
  }

  if (node["signaling"]) {
    const auto& sig = node["signaling"];
    if (sig["url"])                webrtc_config_.signaling_url         = sig["url"].as<std::string>();
    if (sig["reconnect_interval"]) webrtc_config_.reconnect_interval_ms = sig["reconnect_interval"].as<int>();
  }
}

void VehicleConfigManager::ParseControl(const YAML::Node& node) {
  auto parseQos = [](const YAML::Node& q, QosConfig& out) {
    if (q["reliability"]) out.reliability = q["reliability"].as<std::string>();
    if (q["durability"])  out.durability  = q["durability"].as<std::string>();
    if (q["depth"])       out.depth       = q["depth"].as<int>();
  };

  if (node["topics"]) {
    const auto& t = node["topics"];
    if (t["steering_cmd"])    control_topics_.steering_cmd    = t["steering_cmd"].as<std::string>();
    if (t["velocity_cmd"])    control_topics_.velocity_cmd    = t["velocity_cmd"].as<std::string>();
    if (t["gear_cmd"])        control_topics_.gear_cmd        = t["gear_cmd"].as<std::string>();
    if (t["turn_signal_cmd"]) control_topics_.turn_signal_cmd = t["turn_signal_cmd"].as<std::string>();
    if (t["emergency"])       control_topics_.emergency_cmd   = t["emergency"].as<std::string>();
  }

  if (node["topics_qos"]) {
    const auto& tq = node["topics_qos"];
    if (tq["steering_cmd"])   parseQos(tq["steering_cmd"],  control_qos_.steering_cmd);
    if (tq["gear_cmd"])       parseQos(tq["gear_cmd"],      control_qos_.gear_cmd);
    if (tq["turn_signal"])    parseQos(tq["turn_signal"],   control_qos_.turn_signal);
    if (tq["emergency_cmd"])  parseQos(tq["emergency_cmd"], control_qos_.emergency_cmd);
  }

  if (node["safety"]) ParseSafety(node["safety"]);
}

void VehicleConfigManager::ParseSafety(const YAML::Node& node) {
  if (node["max_steering_rate"])       safety_config_.max_steering_rate_dps    = node["max_steering_rate"].as<double>();
  if (node["emergency_decel"])         safety_config_.emergency_decel_mps2     = node["emergency_decel"].as<double>();
  if (node["watchdog_timeout_ms"])     safety_config_.watchdog_timeout_ms      = node["watchdog_timeout_ms"].as<int>();
  if (node["enable_soft_limits"])      safety_config_.soft_limits_enabled      = node["enable_soft_limits"].as<bool>();
  if (node["enable_emergency_stop"])   safety_config_.emergency_stop_enabled   = node["enable_emergency_stop"].as<bool>();
  if (node["command_publish_rate_hz"]) safety_config_.command_publish_rate_hz  = node["command_publish_rate_hz"].as<int>();
}

void VehicleConfigManager::ParseMqtt(const YAML::Node& node) {
  if (node["enabled"])         mqtt_config_.enabled       = node["enabled"].as<bool>();
  if (node["broker_url"])      mqtt_config_.broker_url    = node["broker_url"].as<std::string>();
  if (node["client_id"])       mqtt_config_.client_id     = node["client_id"].as<std::string>();
  if (node["username"])        mqtt_config_.username      = node["username"].as<std::string>();
  if (node["password"])        mqtt_config_.password      = node["password"].as<std::string>();
  if (node["use_tls"])         mqtt_config_.use_tls       = node["use_tls"].as<bool>();
  if (node["ca_cert_path"])    mqtt_config_.ca_cert_path  = node["ca_cert_path"].as<std::string>();
  if (node["keepalive_s"])     mqtt_config_.keepalive_s   = node["keepalive_s"].as<int>();
  if (node["qos"])             mqtt_config_.qos           = node["qos"].as<int>();
  if (node["retain"])          mqtt_config_.retain        = node["retain"].as<bool>();
  if (node["telemetry_topic"]) mqtt_config_.telemetry_topic = node["telemetry_topic"].as<std::string>();
  if (node["control_topic"])   mqtt_config_.control_topic   = node["control_topic"].as<std::string>();
  if (node["status_topic"])    mqtt_config_.status_topic    = node["status_topic"].as<std::string>();

  if (node["aliyun"]) {
    const auto& a = node["aliyun"];
    if (a["mode"])           mqtt_config_.aliyun_mode          = a["mode"].as<bool>();
    if (a["product_key"])    mqtt_config_.aliyun_product_key   = a["product_key"].as<std::string>();
    if (a["device_name"])    mqtt_config_.aliyun_device_name   = a["device_name"].as<std::string>();
    if (a["device_secret"])  mqtt_config_.aliyun_device_secret = a["device_secret"].as<std::string>();
    if (a["region"])         mqtt_config_.aliyun_region        = a["region"].as<std::string>();
  }
}

// ---------------------------------------------------------------------------
// CloudConfigManager
// ---------------------------------------------------------------------------

bool CloudConfigManager::LoadFromFile(const std::string& path) {
  try {
    YAML::Node cfg = YAML::LoadFile(path);
    if (cfg["server"])     ParseServer(cfg["server"]);
    if (cfg["scheduling"]) ParseScheduling(cfg["scheduling"]);
    if (cfg["alerts"])     ParseAlerts(cfg["alerts"]);
    if (cfg["prediction"]) ParsePrediction(cfg["prediction"]);
    if (cfg["mqtt"])       ParseCloudMqtt(cfg["mqtt"]);
    if (cfg["api"])        ParseApi(cfg["api"]);
    FSM_LOG_INFO("Cloud config loaded: {}", path);
    return true;
  } catch (const YAML::Exception& e) {
    FSM_LOG_ERROR("Failed to load cloud config: {}", e.what());
    return false;
  }
}

bool CloudConfigManager::SaveToFile(const std::string& path) const {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;

    out << YAML::Key << "server" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "signaling_port" << YAML::Value << signaling_port_;
    out << YAML::Key << "api_port"       << YAML::Value << api_port_;
    out << YAML::EndMap;

    out << YAML::Key << "scheduling" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "enabled"   << YAML::Value << scheduling_config_.enabled;
    out << YAML::Key << "algorithm" << YAML::Value << scheduling_config_.algorithm;
    out << YAML::Key << "weights" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "emergency"     << YAML::Value << scheduling_config_.weights.emergency;
    out << YAML::Key << "latency"       << YAML::Value << scheduling_config_.weights.latency;
    out << YAML::Key << "distance"      << YAML::Value << scheduling_config_.weights.distance;
    out << YAML::Key << "battery"       << YAML::Value << scheduling_config_.weights.battery;
    out << YAML::Key << "task_priority" << YAML::Value << scheduling_config_.weights.task_priority;
    out << YAML::EndMap;
    out << YAML::EndMap;

    out << YAML::Key << "prediction" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "enabled"              << YAML::Value << prediction_config_.enabled;
    out << YAML::Key << "compensation_enabled" << YAML::Value << prediction_config_.compensation_enabled;
    out << YAML::EndMap;

    out << YAML::EndMap;

    std::ofstream fout(path);
    fout << out.c_str();
    return true;
  } catch (const std::exception& e) {
    FSM_LOG_ERROR("Failed to save cloud config: {}", e.what());
    return false;
  }
}

void CloudConfigManager::ParseServer(const YAML::Node& node) {
  if (node["signaling_port"]) signaling_port_ = node["signaling_port"].as<int>();
  if (node["api_port"])       api_port_       = node["api_port"].as<int>();
}

void CloudConfigManager::ParseScheduling(const YAML::Node& node) {
  if (node["enabled"])   scheduling_config_.enabled   = node["enabled"].as<bool>();
  if (node["algorithm"]) scheduling_config_.algorithm = node["algorithm"].as<std::string>();

  if (node["weights"]) {
    const auto& w = node["weights"];
    if (w["emergency"])     scheduling_config_.weights.emergency     = w["emergency"].as<double>();
    if (w["latency"])       scheduling_config_.weights.latency       = w["latency"].as<double>();
    if (w["distance"])      scheduling_config_.weights.distance      = w["distance"].as<double>();
    if (w["battery"])       scheduling_config_.weights.battery       = w["battery"].as<double>();
    if (w["task_priority"]) scheduling_config_.weights.task_priority = w["task_priority"].as<double>();
  }

  if (node["thresholds"]) {
    const auto& th = node["thresholds"];
    if (th["max_latency_ms"])      scheduling_config_.max_latency_ms      = th["max_latency_ms"].as<int>();
    if (th["critical_latency_ms"]) scheduling_config_.critical_latency_ms = th["critical_latency_ms"].as<int>();
    if (th["min_battery_percent"]) scheduling_config_.min_battery_pct     = th["min_battery_percent"].as<double>();
  }
}

void CloudConfigManager::ParseAlerts(const YAML::Node& node) {
  if (node["rules"]) {
    alert_rules_.clear();
    for (const auto& r : node["rules"]) {
      AlertRuleConfig rule;
      if (r["name"])      rule.name      = r["name"].as<std::string>();
      if (r["condition"]) rule.condition = r["condition"].as<std::string>();
      if (r["severity"])  rule.severity  = r["severity"].as<std::string>();
      if (r["enabled"])   rule.enabled   = r["enabled"].as<bool>();
      alert_rules_.push_back(rule);
    }
  }
}

void CloudConfigManager::ParsePrediction(const YAML::Node& node) {
  if (node["enabled"])               prediction_config_.enabled              = node["enabled"].as<bool>();
  if (node["model_endpoint"])        prediction_config_.model_endpoint       = node["model_endpoint"].as<std::string>();
  if (node["compensation_enabled"])  prediction_config_.compensation_enabled = node["compensation_enabled"].as<bool>();
  if (node["prediction_horizon"])    prediction_config_.prediction_horizon_s = node["prediction_horizon"].as<double>();
  if (node["update_rate_hz"])        prediction_config_.update_rate_hz       = node["update_rate_hz"].as<int>();
}

void CloudConfigManager::ParseCloudMqtt(const YAML::Node& node) {
  if (node["enabled"])              cloud_mqtt_config_.enabled              = node["enabled"].as<bool>();
  if (node["broker_url"])           cloud_mqtt_config_.broker_url           = node["broker_url"].as<std::string>();
  if (node["client_id"])            cloud_mqtt_config_.client_id            = node["client_id"].as<std::string>();
  if (node["username"])             cloud_mqtt_config_.username             = node["username"].as<std::string>();
  if (node["password"])             cloud_mqtt_config_.password             = node["password"].as<std::string>();
  if (node["use_tls"])              cloud_mqtt_config_.use_tls              = node["use_tls"].as<bool>();
  if (node["ca_cert_path"])         cloud_mqtt_config_.ca_cert_path         = node["ca_cert_path"].as<std::string>();
  if (node["keepalive_s"])          cloud_mqtt_config_.keepalive_s          = node["keepalive_s"].as<int>();
  if (node["qos"])                  cloud_mqtt_config_.qos                  = node["qos"].as<int>();
  if (node["telemetry_pub_prefix"]) cloud_mqtt_config_.telemetry_pub_prefix = node["telemetry_pub_prefix"].as<std::string>();
  if (node["status_pub_prefix"])    cloud_mqtt_config_.status_pub_prefix    = node["status_pub_prefix"].as<std::string>();
}

void CloudConfigManager::ParseApi(const YAML::Node& node) {
  if (node["auth"]) {
    const auto& a = node["auth"];
    if (a["jwt_secret"]) api_jwt_secret_ = a["jwt_secret"].as<std::string>();
  }
  if (node["session_timeout_s"])
    session_timeout_s_ = node["session_timeout_s"].as<int>();
}

}  // namespace config
}  // namespace fsm
