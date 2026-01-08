/**
 * FSM-Pilot 告警分析器实现
 * 分析车辆状态并生成告警
 */

#include "fsm/cloud/alert_analyzer.hpp"
#include "fsm/logger.hpp"
#include <chrono>

namespace fsm {
namespace cloud {

AlertAnalyzer::AlertAnalyzer(const config::CloudConfigManager& config)
    : config_(config)
    , alert_id_counter_(0)
{
    // 加载告警规则
    loadAlertRules();
}

void AlertAnalyzer::loadAlertRules() {
    const auto& alert_config = config_.getAlertConfig();

    if (!alert_config.enabled) {
        LOG_INFO("[AlertAnalyzer] Alerts disabled");
        return;
    }

    // 内置规则
    rules_.push_back({
        "high_latency",
        [](const VehicleState& state) {
            return state.latency_ms > 300;
        },
        AlertSeverity::WARNING,
        "Communication latency exceeds threshold"
    });

    rules_.push_back({
        "critical_latency",
        [](const VehicleState& state) {
            return state.latency_ms > 500;
        },
        AlertSeverity::CRITICAL,
        "Communication latency critical - consider emergency stop"
    });

    rules_.push_back({
        "low_battery",
        [](const VehicleState& state) {
            return state.battery_level < 20;
        },
        AlertSeverity::WARNING,
        "Low battery level"
    });

    rules_.push_back({
        "critical_battery",
        [](const VehicleState& state) {
            return state.battery_level < 10;
        },
        AlertSeverity::CRITICAL,
        "Critical battery level - return to base"
    });

    rules_.push_back({
        "connection_lost",
        [](const VehicleState& state) {
            auto now = std::chrono::system_clock::now();
            auto last_update = std::chrono::system_clock::from_time_t(
                static_cast<time_t>(state.last_update_time / 1000));
            auto diff = std::chrono::duration_cast<std::chrono::seconds>(
                now - last_update).count();
            return diff > 10;  // 10秒无更新
        },
        AlertSeverity::CRITICAL,
        "Vehicle connection lost"
    });

    rules_.push_back({
        "emergency_active",
        [](const VehicleState& state) {
            return state.emergency_active;
        },
        AlertSeverity::CRITICAL,
        "Emergency state active"
    });

    rules_.push_back({
        "camera_offline",
        [](const VehicleState& state) {
            return state.cameras_online < state.cameras_total;
        },
        AlertSeverity::WARNING,
        "One or more cameras offline"
    });

    rules_.push_back({
        "high_cpu",
        [](const VehicleState& state) {
            return state.cpu_usage > 90;
        },
        AlertSeverity::WARNING,
        "High CPU usage"
    });

    rules_.push_back({
        "lidar_offline",
        [](const VehicleState& state) {
            return !state.lidar_online;
        },
        AlertSeverity::WARNING,
        "LiDAR sensor offline"
    });

    rules_.push_back({
        "gps_lost",
        [](const VehicleState& state) {
            return !state.gps_fix;
        },
        AlertSeverity::WARNING,
        "GPS signal lost"
    });

    LOG_INFO("[AlertAnalyzer] Loaded " + std::to_string(rules_.size()) + " alert rules");
}

std::vector<Alert> AlertAnalyzer::analyze(const std::string& vehicle_id,
                                           const VehicleState& state) {
    std::vector<Alert> new_alerts;
    auto now = std::chrono::system_clock::now();

    for (const auto& rule : rules_) {
        bool triggered = rule.condition(state);
        std::string alert_key = vehicle_id + "_" + rule.name;

        auto it = active_alerts_.find(alert_key);
        bool was_active = (it != active_alerts_.end());

        if (triggered && !was_active) {
            // 新告警
            Alert alert;
            alert.id = "alert_" + std::to_string(++alert_id_counter_);
            alert.vehicle_id = vehicle_id;
            alert.type = rule.name;
            alert.severity = rule.severity;
            alert.message = rule.message;
            alert.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count();
            alert.acknowledged = false;

            // 添加详细信息
            alert.details["latency_ms"] = state.latency_ms;
            alert.details["battery_level"] = state.battery_level;
            alert.details["cpu_usage"] = state.cpu_usage;

            active_alerts_[alert_key] = alert;
            new_alerts.push_back(alert);

            LOG_WARNING("[AlertAnalyzer] New alert: " + alert.type +
                       " for vehicle " + vehicle_id);

            // 触发回调
            if (alert_callback_) {
                alert_callback_(alert);
            }

        } else if (!triggered && was_active) {
            // 告警恢复
            Alert& alert = it->second;
            alert.resolved = true;
            alert.resolved_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count();

            LOG_INFO("[AlertAnalyzer] Alert resolved: " + alert.type +
                    " for vehicle " + vehicle_id);

            // 触发恢复回调
            if (resolved_callback_) {
                resolved_callback_(alert);
            }

            active_alerts_.erase(it);
        }
    }

    return new_alerts;
}

std::vector<Alert> AlertAnalyzer::getActiveAlerts() const {
    std::vector<Alert> alerts;
    for (const auto& [key, alert] : active_alerts_) {
        if (!alert.resolved) {
            alerts.push_back(alert);
        }
    }
    return alerts;
}

std::vector<Alert> AlertAnalyzer::getActiveAlertsForVehicle(const std::string& vehicle_id) const {
    std::vector<Alert> alerts;
    for (const auto& [key, alert] : active_alerts_) {
        if (alert.vehicle_id == vehicle_id && !alert.resolved) {
            alerts.push_back(alert);
        }
    }
    return alerts;
}

bool AlertAnalyzer::acknowledgeAlert(const std::string& alert_id) {
    for (auto& [key, alert] : active_alerts_) {
        if (alert.id == alert_id) {
            alert.acknowledged = true;
            LOG_INFO("[AlertAnalyzer] Alert acknowledged: " + alert_id);
            return true;
        }
    }
    return false;
}

void AlertAnalyzer::clearAlertsForVehicle(const std::string& vehicle_id) {
    for (auto it = active_alerts_.begin(); it != active_alerts_.end();) {
        if (it->second.vehicle_id == vehicle_id) {
            it = active_alerts_.erase(it);
        } else {
            ++it;
        }
    }
}

void AlertAnalyzer::onAlert(AlertCallback callback) {
    alert_callback_ = callback;
}

void AlertAnalyzer::onAlertResolved(AlertCallback callback) {
    resolved_callback_ = callback;
}

AlertStatistics AlertAnalyzer::getStatistics() const {
    AlertStatistics stats;
    stats.total_alerts = active_alerts_.size();
    stats.critical_count = 0;
    stats.warning_count = 0;
    stats.info_count = 0;
    stats.acknowledged_count = 0;

    for (const auto& [key, alert] : active_alerts_) {
        switch (alert.severity) {
            case AlertSeverity::CRITICAL:
                stats.critical_count++;
                break;
            case AlertSeverity::WARNING:
                stats.warning_count++;
                break;
            case AlertSeverity::INFO:
                stats.info_count++;
                break;
        }

        if (alert.acknowledged) {
            stats.acknowledged_count++;
        }
    }

    return stats;
}

}  // namespace cloud
}  // namespace fsm
