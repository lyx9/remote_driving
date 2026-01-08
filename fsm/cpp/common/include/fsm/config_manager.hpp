#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <optional>
#include <yaml-cpp/yaml.h>

namespace fsm {
namespace config {

/**
 * @brief 摄像头配置
 */
struct CameraConfig {
    std::string id;
    std::string topic;
    int fps = 30;
    int width = 1920;
    int height = 1080;
    std::string encoding = "h264";
    int bitrate = 4000000;
    bool enabled = true;
};

/**
 * @brief 车辆参数配置
 */
struct VehicleParameters {
    double wheelbase = 2.7;           // 轴距 (m)
    double max_steering_angle = 35.0;  // 最大转向角 (度)
    double max_speed = 60.0;           // 最大速度 (km/h)
    double max_acceleration = 2.0;     // 最大加速度 (m/s²)
    double max_deceleration = 5.0;     // 最大减速度 (m/s²)
    double track_width = 1.5;          // 轮距 (m)
};

/**
 * @brief WebRTC配置
 */
struct WebRTCConfig {
    std::vector<std::string> stun_servers;
    std::vector<std::string> turn_servers;
    std::string turn_username;
    std::string turn_password;
    std::string signaling_url;
    int reconnect_interval_ms = 5000;
    int ice_timeout_ms = 10000;
};

/**
 * @brief 控制话题配置
 */
struct ControlTopicsConfig {
    std::string steering_cmd = "/control/command/steering_cmd";
    std::string velocity_cmd = "/control/command/velocity_cmd";
    std::string gear_cmd = "/vehicle/command/gear_cmd";
    std::string turn_signal_cmd = "/vehicle/command/turn_signal_cmd";
    std::string emergency_cmd = "/system/emergency/emergency_cmd";
};

/**
 * @brief 传感器话题配置
 */
struct SensorTopicsConfig {
    std::string velocity_status = "/vehicle/status/velocity_status";
    std::string steering_status = "/vehicle/status/steering_status";
    std::string gear_status = "/vehicle/status/gear_status";
    std::string control_mode = "/vehicle/status/control_mode";
    std::string localization = "/localization/kinematic_state";
    std::string diagnostics = "/diagnostics_agg";
};

/**
 * @brief 安全配置
 */
struct SafetyConfig {
    double max_steering_rate = 30.0;   // 度/秒
    double emergency_decel = 9.0;      // m/s²
    int watchdog_timeout_ms = 200;     // 看门狗超时
    bool enable_soft_limits = true;
    bool enable_emergency_stop = true;
};

/**
 * @brief 调度算法权重配置
 */
struct SchedulingWeights {
    double emergency = 0.35;
    double latency = 0.25;
    double distance = 0.20;
    double battery = 0.10;
    double task_priority = 0.10;
};

/**
 * @brief 调度配置
 */
struct SchedulingConfig {
    bool enabled = true;
    std::string algorithm = "weighted_priority";
    SchedulingWeights weights;
    int max_latency_ms = 200;
    int critical_latency_ms = 500;
    double min_battery_percent = 20.0;
};

/**
 * @brief 预测模型配置
 */
struct PredictionConfig {
    bool enabled = false;
    std::string model_endpoint;
    bool compensation_enabled = false;
    double prediction_horizon = 2.0;   // 预测时间范围 (秒)
    int update_rate_hz = 10;
};

/**
 * @brief 告警规则配置
 */
struct AlertRule {
    std::string name;
    std::string condition;
    std::string severity;
    bool enabled = true;
};

/**
 * @brief 车端配置管理器
 */
class VehicleConfigManager {
public:
    VehicleConfigManager() = default;
    ~VehicleConfigManager() = default;

    /**
     * @brief 从YAML文件加载配置
     * @param config_path 配置文件路径
     * @return 是否加载成功
     */
    bool loadFromFile(const std::string& config_path);

    /**
     * @brief 保存配置到YAML文件
     * @param config_path 配置文件路径
     * @return 是否保存成功
     */
    bool saveToFile(const std::string& config_path) const;

    // Getters
    const std::string& getVehicleId() const { return vehicle_id_; }
    const std::string& getVehicleType() const { return vehicle_type_; }
    const std::string& getVehicleName() const { return vehicle_name_; }
    const VehicleParameters& getVehicleParams() const { return vehicle_params_; }
    const std::vector<CameraConfig>& getCameras() const { return cameras_; }
    const WebRTCConfig& getWebRTCConfig() const { return webrtc_config_; }
    const ControlTopicsConfig& getControlTopics() const { return control_topics_; }
    const SensorTopicsConfig& getSensorTopics() const { return sensor_topics_; }
    const SafetyConfig& getSafetyConfig() const { return safety_config_; }

    // Setters
    void setVehicleId(const std::string& id) { vehicle_id_ = id; }
    void setWebRTCConfig(const WebRTCConfig& config) { webrtc_config_ = config; }

    /**
     * @brief 获取指定ID的摄像头配置
     */
    std::optional<CameraConfig> getCameraById(const std::string& id) const;

private:
    void parseVehicle(const YAML::Node& node);
    void parseSensors(const YAML::Node& node);
    void parseWebRTC(const YAML::Node& node);
    void parseControl(const YAML::Node& node);
    void parseSafety(const YAML::Node& node);

    std::string vehicle_id_;
    std::string vehicle_type_;
    std::string vehicle_name_;
    VehicleParameters vehicle_params_;
    std::vector<CameraConfig> cameras_;
    WebRTCConfig webrtc_config_;
    ControlTopicsConfig control_topics_;
    SensorTopicsConfig sensor_topics_;
    SafetyConfig safety_config_;
};

/**
 * @brief 云端配置管理器
 */
class CloudConfigManager {
public:
    CloudConfigManager() = default;
    ~CloudConfigManager() = default;

    bool loadFromFile(const std::string& config_path);
    bool saveToFile(const std::string& config_path) const;

    // Getters
    int getSignalingPort() const { return signaling_port_; }
    int getApiPort() const { return api_port_; }
    const SchedulingConfig& getSchedulingConfig() const { return scheduling_config_; }
    const std::vector<AlertRule>& getAlertRules() const { return alert_rules_; }
    const PredictionConfig& getPredictionConfig() const { return prediction_config_; }

    // Setters
    void setSchedulingEnabled(bool enabled) { scheduling_config_.enabled = enabled; }
    void setPredictionEnabled(bool enabled) { prediction_config_.enabled = enabled; }

private:
    void parseServer(const YAML::Node& node);
    void parseScheduling(const YAML::Node& node);
    void parseAlerts(const YAML::Node& node);
    void parsePrediction(const YAML::Node& node);

    int signaling_port_ = 8080;
    int api_port_ = 8081;
    SchedulingConfig scheduling_config_;
    std::vector<AlertRule> alert_rules_;
    PredictionConfig prediction_config_;
};

}  // namespace config
}  // namespace fsm
