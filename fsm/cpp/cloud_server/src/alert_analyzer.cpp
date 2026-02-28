#include "fsm/cloud/alert_analyzer.hpp"
#include "fsm/logger.hpp"
#include <chrono>

namespace fsm {
namespace cloud {

AlertAnalyzer::AlertAnalyzer(const config::CloudConfigManager& config)
    : config_(config) {
  LoadRules();
}

void AlertAnalyzer::LoadRules() {
  rules_.push_back({
    "high_latency",
    [](const VehicleHealthSnapshot& s) { return s.latency_ms > 300; },
    AlertSeverity::kWarning,
    "Communication latency exceeds threshold"
  });

  rules_.push_back({
    "critical_latency",
    [](const VehicleHealthSnapshot& s) { return s.latency_ms > 500; },
    AlertSeverity::kCritical,
    "Communication latency critical"
  });

  rules_.push_back({
    "low_battery",
    [](const VehicleHealthSnapshot& s) { return s.battery_pct < 20; },
    AlertSeverity::kWarning,
    "Low battery level"
  });

  rules_.push_back({
    "critical_battery",
    [](const VehicleHealthSnapshot& s) { return s.battery_pct < 10; },
    AlertSeverity::kCritical,
    "Critical battery level - return to base"
  });

  rules_.push_back({
    "connection_lost",
    [](const VehicleHealthSnapshot& s) {
      const auto now  = std::chrono::system_clock::now();
      const auto last = std::chrono::system_clock::from_time_t(
          static_cast<time_t>(s.last_update_ns / 1000000000LL));
      return std::chrono::duration_cast<std::chrono::seconds>(
          now - last).count() > 10;
    },
    AlertSeverity::kCritical,
    "Vehicle connection lost"
  });

  rules_.push_back({
    "emergency_active",
    [](const VehicleHealthSnapshot& s) { return s.emergency_active; },
    AlertSeverity::kCritical,
    "Emergency state active"
  });

  rules_.push_back({
    "camera_offline",
    [](const VehicleHealthSnapshot& s) {
      return s.cameras_online < s.cameras_total;
    },
    AlertSeverity::kWarning,
    "One or more cameras offline"
  });

  rules_.push_back({
    "high_cpu",
    [](const VehicleHealthSnapshot& s) { return s.cpu_usage > 90; },
    AlertSeverity::kWarning,
    "High CPU usage"
  });

  rules_.push_back({
    "lidar_offline",
    [](const VehicleHealthSnapshot& s) { return !s.lidar_online; },
    AlertSeverity::kWarning,
    "LiDAR sensor offline"
  });

  rules_.push_back({
    "gps_lost",
    [](const VehicleHealthSnapshot& s) { return !s.gps_fix; },
    AlertSeverity::kWarning,
    "GPS signal lost"
  });

  FSM_LOG_INFO("AlertAnalyzer: {} rules loaded", rules_.size());
}

std::vector<Alert> AlertAnalyzer::Analyze(const std::string& vehicle_id,
                                           const VehicleHealthSnapshot& snapshot) {
  std::vector<Alert> new_alerts;
  const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

  for (const auto& rule : rules_) {
    const bool triggered = rule.condition(snapshot);
    const std::string key = vehicle_id + "_" + rule.name;
    const auto it = active_alerts_.find(key);
    const bool was_active = (it != active_alerts_.end());

    if (triggered && !was_active) {
      Alert alert;
      alert.id           = "alert_" + std::to_string(++id_counter_);
      alert.vehicle_id   = vehicle_id;
      alert.type         = rule.name;
      alert.severity     = rule.severity;
      alert.message      = rule.message;
      alert.timestamp_ns = now_ns;
      alert.details["latency_ms"]  = snapshot.latency_ms;
      alert.details["battery_pct"] = snapshot.battery_pct;
      alert.details["cpu_usage"]   = snapshot.cpu_usage;

      active_alerts_[key] = alert;
      new_alerts.push_back(alert);

      FSM_LOG_WARN("Alert: {} vehicle={}", alert.type, vehicle_id);
      if (alert_cb_) alert_cb_(alert);

    } else if (!triggered && was_active) {
      Alert& alert      = it->second;
      alert.resolved    = true;
      alert.resolved_ns = now_ns;

      FSM_LOG_INFO("Alert resolved: {} vehicle={}", alert.type, vehicle_id);
      if (resolved_cb_) resolved_cb_(alert);
      active_alerts_.erase(it);
    }
  }

  return new_alerts;
}

std::vector<Alert> AlertAnalyzer::GetActiveAlerts() const {
  std::vector<Alert> result;
  for (const auto& [key, alert] : active_alerts_) {
    if (!alert.resolved) result.push_back(alert);
  }
  return result;
}

std::vector<Alert> AlertAnalyzer::GetActiveAlertsForVehicle(
    const std::string& vehicle_id) const {
  std::vector<Alert> result;
  for (const auto& [key, alert] : active_alerts_) {
    if (alert.vehicle_id == vehicle_id && !alert.resolved)
      result.push_back(alert);
  }
  return result;
}

bool AlertAnalyzer::AcknowledgeAlert(const std::string& alert_id) {
  for (auto& [key, alert] : active_alerts_) {
    if (alert.id == alert_id) {
      alert.acknowledged = true;
      FSM_LOG_INFO("Alert acknowledged: {}", alert_id);
      return true;
    }
  }
  return false;
}

void AlertAnalyzer::ClearAlertsForVehicle(const std::string& vehicle_id) {
  for (auto it = active_alerts_.begin(); it != active_alerts_.end(); ) {
    it = (it->second.vehicle_id == vehicle_id)
        ? active_alerts_.erase(it)
        : std::next(it);
  }
}

void AlertAnalyzer::set_alert_callback(AlertCallback cb) {
  alert_cb_ = std::move(cb);
}

void AlertAnalyzer::set_resolved_callback(AlertCallback cb) {
  resolved_cb_ = std::move(cb);
}

AlertStatistics AlertAnalyzer::GetStatistics() const {
  AlertStatistics stats;
  stats.total = active_alerts_.size();
  for (const auto& [key, alert] : active_alerts_) {
    switch (alert.severity) {
      case AlertSeverity::kCritical: ++stats.critical; break;
      case AlertSeverity::kWarning:  ++stats.warning;  break;
      case AlertSeverity::kInfo:     ++stats.info;     break;
    }
    if (alert.acknowledged) ++stats.acknowledged;
  }
  return stats;
}

}  // namespace cloud
}  // namespace fsm
