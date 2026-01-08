#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"
#include <fstream>

namespace fsm {
namespace config {

bool VehicleConfigManager::loadFromFile(const std::string& config_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_path);

        if (config["vehicle"]) {
            parseVehicle(config["vehicle"]);
        }

        if (config["sensors"]) {
            parseSensors(config["sensors"]);
        }

        if (config["webrtc"]) {
            parseWebRTC(config["webrtc"]);
        }

        if (config["control"]) {
            parseControl(config["control"]);
        }

        FSM_LOG_INFO("Vehicle config loaded from: {}", config_path);
        return true;

    } catch (const YAML::Exception& e) {
        FSM_LOG_ERROR("Failed to load vehicle config: {}", e.what());
        return false;
    }
}

void VehicleConfigManager::parseVehicle(const YAML::Node& node) {
    if (node["id"]) vehicle_id_ = node["id"].as<std::string>();
    if (node["type"]) vehicle_type_ = node["type"].as<std::string>();
    if (node["name"]) vehicle_name_ = node["name"].as<std::string>();

    if (node["parameters"]) {
        auto params = node["parameters"];
        if (params["wheelbase"]) vehicle_params_.wheelbase = params["wheelbase"].as<double>();
        if (params["max_steering_angle"]) vehicle_params_.max_steering_angle = params["max_steering_angle"].as<double>();
        if (params["max_speed"]) vehicle_params_.max_speed = params["max_speed"].as<double>();
        if (params["max_acceleration"]) vehicle_params_.max_acceleration = params["max_acceleration"].as<double>();
        if (params["max_deceleration"]) vehicle_params_.max_deceleration = params["max_deceleration"].as<double>();
        if (params["track_width"]) vehicle_params_.track_width = params["track_width"].as<double>();
    }
}

void VehicleConfigManager::parseSensors(const YAML::Node& node) {
    if (node["cameras"] && node["cameras"]["list"]) {
        cameras_.clear();
        for (const auto& cam_node : node["cameras"]["list"]) {
            CameraConfig cam;
            if (cam_node["id"]) cam.id = cam_node["id"].as<std::string>();
            if (cam_node["topic"]) cam.topic = cam_node["topic"].as<std::string>();
            if (cam_node["fps"]) cam.fps = cam_node["fps"].as<int>();
            if (cam_node["width"]) cam.width = cam_node["width"].as<int>();
            if (cam_node["height"]) cam.height = cam_node["height"].as<int>();
            if (cam_node["encoding"]) cam.encoding = cam_node["encoding"].as<std::string>();
            if (cam_node["bitrate"]) cam.bitrate = cam_node["bitrate"].as<int>();
            if (cam_node["enabled"]) cam.enabled = cam_node["enabled"].as<bool>();
            cameras_.push_back(cam);
        }
    }

    // 解析传感器话题配置
    if (node["topics"]) {
        auto topics = node["topics"];
        if (topics["velocity_status"]) sensor_topics_.velocity_status = topics["velocity_status"].as<std::string>();
        if (topics["steering_status"]) sensor_topics_.steering_status = topics["steering_status"].as<std::string>();
        if (topics["gear_status"]) sensor_topics_.gear_status = topics["gear_status"].as<std::string>();
        if (topics["control_mode"]) sensor_topics_.control_mode = topics["control_mode"].as<std::string>();
        if (topics["localization"]) sensor_topics_.localization = topics["localization"].as<std::string>();
        if (topics["diagnostics"]) sensor_topics_.diagnostics = topics["diagnostics"].as<std::string>();
    }
}

void VehicleConfigManager::parseWebRTC(const YAML::Node& node) {
    if (node["stun_servers"]) {
        webrtc_config_.stun_servers.clear();
        for (const auto& server : node["stun_servers"]) {
            webrtc_config_.stun_servers.push_back(server.as<std::string>());
        }
    }

    if (node["turn_servers"]) {
        webrtc_config_.turn_servers.clear();
        for (const auto& server : node["turn_servers"]) {
            if (server["url"]) {
                webrtc_config_.turn_servers.push_back(server["url"].as<std::string>());
            }
            if (server["username"]) webrtc_config_.turn_username = server["username"].as<std::string>();
            if (server["credential"]) webrtc_config_.turn_password = server["credential"].as<std::string>();
        }
    }

    if (node["signaling"]) {
        auto sig = node["signaling"];
        if (sig["url"]) webrtc_config_.signaling_url = sig["url"].as<std::string>();
        if (sig["reconnect_interval"]) webrtc_config_.reconnect_interval_ms = sig["reconnect_interval"].as<int>();
    }
}

void VehicleConfigManager::parseControl(const YAML::Node& node) {
    if (node["topics"]) {
        auto topics = node["topics"];
        if (topics["steering_cmd"]) control_topics_.steering_cmd = topics["steering_cmd"].as<std::string>();
        if (topics["velocity_cmd"]) control_topics_.velocity_cmd = topics["velocity_cmd"].as<std::string>();
        if (topics["gear_cmd"]) control_topics_.gear_cmd = topics["gear_cmd"].as<std::string>();
        if (topics["turn_signal_cmd"]) control_topics_.turn_signal_cmd = topics["turn_signal_cmd"].as<std::string>();
        if (topics["emergency"]) control_topics_.emergency_cmd = topics["emergency"].as<std::string>();
    }

    if (node["safety"]) {
        parseSafety(node["safety"]);
    }
}

void VehicleConfigManager::parseSafety(const YAML::Node& node) {
    if (node["max_steering_rate"]) safety_config_.max_steering_rate = node["max_steering_rate"].as<double>();
    if (node["emergency_decel"]) safety_config_.emergency_decel = node["emergency_decel"].as<double>();
    if (node["watchdog_timeout_ms"]) safety_config_.watchdog_timeout_ms = node["watchdog_timeout_ms"].as<int>();
    if (node["enable_soft_limits"]) safety_config_.enable_soft_limits = node["enable_soft_limits"].as<bool>();
    if (node["enable_emergency_stop"]) safety_config_.enable_emergency_stop = node["enable_emergency_stop"].as<bool>();
}

std::optional<CameraConfig> VehicleConfigManager::getCameraById(const std::string& id) const {
    for (const auto& cam : cameras_) {
        if (cam.id == id) {
            return cam;
        }
    }
    return std::nullopt;
}

bool VehicleConfigManager::saveToFile(const std::string& config_path) const {
    try {
        YAML::Emitter out;
        out << YAML::BeginMap;

        // Vehicle section
        out << YAML::Key << "vehicle" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "id" << YAML::Value << vehicle_id_;
        out << YAML::Key << "type" << YAML::Value << vehicle_type_;
        out << YAML::Key << "name" << YAML::Value << vehicle_name_;
        out << YAML::Key << "parameters" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "wheelbase" << YAML::Value << vehicle_params_.wheelbase;
        out << YAML::Key << "max_steering_angle" << YAML::Value << vehicle_params_.max_steering_angle;
        out << YAML::Key << "max_speed" << YAML::Value << vehicle_params_.max_speed;
        out << YAML::EndMap;
        out << YAML::EndMap;

        // Sensors section
        out << YAML::Key << "sensors" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "cameras" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "list" << YAML::Value << YAML::BeginSeq;
        for (const auto& cam : cameras_) {
            out << YAML::BeginMap;
            out << YAML::Key << "id" << YAML::Value << cam.id;
            out << YAML::Key << "topic" << YAML::Value << cam.topic;
            out << YAML::Key << "fps" << YAML::Value << cam.fps;
            out << YAML::Key << "width" << YAML::Value << cam.width;
            out << YAML::Key << "height" << YAML::Value << cam.height;
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;
        out << YAML::EndMap;
        out << YAML::EndMap;

        out << YAML::EndMap;

        std::ofstream fout(config_path);
        fout << out.c_str();
        fout.close();

        FSM_LOG_INFO("Vehicle config saved to: {}", config_path);
        return true;

    } catch (const std::exception& e) {
        FSM_LOG_ERROR("Failed to save vehicle config: {}", e.what());
        return false;
    }
}

// ============================================================================
// CloudConfigManager Implementation
// ============================================================================

bool CloudConfigManager::loadFromFile(const std::string& config_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_path);

        if (config["server"]) {
            parseServer(config["server"]);
        }

        if (config["scheduling"]) {
            parseScheduling(config["scheduling"]);
        }

        if (config["alerts"]) {
            parseAlerts(config["alerts"]);
        }

        if (config["prediction"]) {
            parsePrediction(config["prediction"]);
        }

        FSM_LOG_INFO("Cloud config loaded from: {}", config_path);
        return true;

    } catch (const YAML::Exception& e) {
        FSM_LOG_ERROR("Failed to load cloud config: {}", e.what());
        return false;
    }
}

void CloudConfigManager::parseServer(const YAML::Node& node) {
    if (node["signaling_port"]) signaling_port_ = node["signaling_port"].as<int>();
    if (node["api_port"]) api_port_ = node["api_port"].as<int>();
}

void CloudConfigManager::parseScheduling(const YAML::Node& node) {
    if (node["enabled"]) scheduling_config_.enabled = node["enabled"].as<bool>();
    if (node["algorithm"]) scheduling_config_.algorithm = node["algorithm"].as<std::string>();

    if (node["weights"]) {
        auto weights = node["weights"];
        if (weights["emergency"]) scheduling_config_.weights.emergency = weights["emergency"].as<double>();
        if (weights["latency"]) scheduling_config_.weights.latency = weights["latency"].as<double>();
        if (weights["distance"]) scheduling_config_.weights.distance = weights["distance"].as<double>();
        if (weights["battery"]) scheduling_config_.weights.battery = weights["battery"].as<double>();
        if (weights["task_priority"]) scheduling_config_.weights.task_priority = weights["task_priority"].as<double>();
    }

    if (node["thresholds"]) {
        auto th = node["thresholds"];
        if (th["max_latency_ms"]) scheduling_config_.max_latency_ms = th["max_latency_ms"].as<int>();
        if (th["critical_latency_ms"]) scheduling_config_.critical_latency_ms = th["critical_latency_ms"].as<int>();
        if (th["min_battery_percent"]) scheduling_config_.min_battery_percent = th["min_battery_percent"].as<double>();
    }
}

void CloudConfigManager::parseAlerts(const YAML::Node& node) {
    if (node["rules"]) {
        alert_rules_.clear();
        for (const auto& rule_node : node["rules"]) {
            AlertRule rule;
            if (rule_node["name"]) rule.name = rule_node["name"].as<std::string>();
            if (rule_node["condition"]) rule.condition = rule_node["condition"].as<std::string>();
            if (rule_node["severity"]) rule.severity = rule_node["severity"].as<std::string>();
            if (rule_node["enabled"]) rule.enabled = rule_node["enabled"].as<bool>();
            alert_rules_.push_back(rule);
        }
    }
}

void CloudConfigManager::parsePrediction(const YAML::Node& node) {
    if (node["enabled"]) prediction_config_.enabled = node["enabled"].as<bool>();
    if (node["model_endpoint"]) prediction_config_.model_endpoint = node["model_endpoint"].as<std::string>();
    if (node["compensation_enabled"]) prediction_config_.compensation_enabled = node["compensation_enabled"].as<bool>();
    if (node["prediction_horizon"]) prediction_config_.prediction_horizon = node["prediction_horizon"].as<double>();
    if (node["update_rate_hz"]) prediction_config_.update_rate_hz = node["update_rate_hz"].as<int>();
}

bool CloudConfigManager::saveToFile(const std::string& config_path) const {
    try {
        YAML::Emitter out;
        out << YAML::BeginMap;

        // Server
        out << YAML::Key << "server" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "signaling_port" << YAML::Value << signaling_port_;
        out << YAML::Key << "api_port" << YAML::Value << api_port_;
        out << YAML::EndMap;

        // Scheduling
        out << YAML::Key << "scheduling" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "enabled" << YAML::Value << scheduling_config_.enabled;
        out << YAML::Key << "algorithm" << YAML::Value << scheduling_config_.algorithm;
        out << YAML::Key << "weights" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "emergency" << YAML::Value << scheduling_config_.weights.emergency;
        out << YAML::Key << "latency" << YAML::Value << scheduling_config_.weights.latency;
        out << YAML::Key << "distance" << YAML::Value << scheduling_config_.weights.distance;
        out << YAML::Key << "battery" << YAML::Value << scheduling_config_.weights.battery;
        out << YAML::Key << "task_priority" << YAML::Value << scheduling_config_.weights.task_priority;
        out << YAML::EndMap;
        out << YAML::EndMap;

        // Prediction
        out << YAML::Key << "prediction" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "enabled" << YAML::Value << prediction_config_.enabled;
        out << YAML::Key << "compensation_enabled" << YAML::Value << prediction_config_.compensation_enabled;
        out << YAML::EndMap;

        out << YAML::EndMap;

        std::ofstream fout(config_path);
        fout << out.c_str();
        fout.close();

        return true;

    } catch (const std::exception& e) {
        FSM_LOG_ERROR("Failed to save cloud config: {}", e.what());
        return false;
    }
}

}  // namespace config
}  // namespace fsm
