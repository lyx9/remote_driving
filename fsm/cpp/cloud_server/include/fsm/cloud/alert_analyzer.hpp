/**
 * FSM-Pilot 告警分析器头文件
 */

#pragma once

#include "fsm/config_manager.hpp"
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <nlohmann/json.hpp>

namespace fsm {
namespace cloud {

/**
 * 告警严重级别
 */
enum class AlertSeverity {
    INFO,
    WARNING,
    CRITICAL
};

/**
 * 车辆状态 (用于告警分析)
 */
struct VehicleState {
    double latency_ms = 0;
    int battery_level = 100;
    bool emergency_active = false;
    int cameras_online = 0;
    int cameras_total = 0;
    bool lidar_online = true;
    bool gps_fix = true;
    double cpu_usage = 0;
    double gpu_usage = 0;
    double memory_usage = 0;
    int64_t last_update_time = 0;
};

/**
 * 告警信息
 */
struct Alert {
    std::string id;
    std::string vehicle_id;
    std::string type;
    AlertSeverity severity;
    std::string message;
    int64_t timestamp;
    bool acknowledged = false;
    bool resolved = false;
    int64_t resolved_time = 0;
    nlohmann::json details;
};

/**
 * 告警规则
 */
struct AlertRule {
    std::string name;
    std::function<bool(const VehicleState&)> condition;
    AlertSeverity severity;
    std::string message;
};

/**
 * 告警统计
 */
struct AlertStatistics {
    size_t total_alerts;
    size_t critical_count;
    size_t warning_count;
    size_t info_count;
    size_t acknowledged_count;
};

using AlertCallback = std::function<void(const Alert&)>;

/**
 * 告警分析器
 * 分析车辆状态并生成告警
 */
class AlertAnalyzer {
public:
    explicit AlertAnalyzer(const config::CloudConfigManager& config);

    /**
     * 分析车辆状态，返回新生成的告警
     */
    std::vector<Alert> analyze(const std::string& vehicle_id, const VehicleState& state);

    /**
     * 获取所有活动告警
     */
    std::vector<Alert> getActiveAlerts() const;

    /**
     * 获取指定车辆的活动告警
     */
    std::vector<Alert> getActiveAlertsForVehicle(const std::string& vehicle_id) const;

    /**
     * 确认告警
     */
    bool acknowledgeAlert(const std::string& alert_id);

    /**
     * 清除车辆的所有告警
     */
    void clearAlertsForVehicle(const std::string& vehicle_id);

    /**
     * 注册告警回调
     */
    void onAlert(AlertCallback callback);

    /**
     * 注册告警恢复回调
     */
    void onAlertResolved(AlertCallback callback);

    /**
     * 获取告警统计
     */
    AlertStatistics getStatistics() const;

private:
    void loadAlertRules();

private:
    const config::CloudConfigManager& config_;
    std::vector<AlertRule> rules_;
    std::map<std::string, Alert> active_alerts_;  // key: vehicle_id + "_" + alert_type
    uint64_t alert_id_counter_;

    AlertCallback alert_callback_;
    AlertCallback resolved_callback_;
};

}  // namespace cloud
}  // namespace fsm
