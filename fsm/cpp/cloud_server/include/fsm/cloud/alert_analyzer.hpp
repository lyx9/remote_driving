#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include "fsm/config_manager.hpp"

namespace fsm {
namespace cloud {

enum class AlertSeverity {
  kInfo,
  kWarning,
  kCritical,
};

struct VehicleHealthSnapshot {
  double latency_ms = 0.0;
  int battery_pct = 100;
  bool emergency_active = false;
  int cameras_online = 0;
  int cameras_total = 0;
  bool lidar_online = true;
  bool gps_fix = true;
  double cpu_usage = 0.0;
  double gpu_usage = 0.0;
  double memory_usage = 0.0;
  int64_t last_update_ns = 0;
};

struct Alert {
  std::string id;
  std::string vehicle_id;
  std::string type;
  AlertSeverity severity = AlertSeverity::kInfo;
  std::string message;
  int64_t timestamp_ns = 0;
  bool acknowledged = false;
  bool resolved = false;
  int64_t resolved_ns = 0;
  nlohmann::json details;
};

struct AlertStatistics {
  size_t total = 0;
  size_t critical = 0;
  size_t warning = 0;
  size_t info = 0;
  size_t acknowledged = 0;
};

using AlertCallback = std::function<void(const Alert&)>;

class AlertAnalyzer {
 public:
  explicit AlertAnalyzer(const config::CloudConfigManager& config);

  AlertAnalyzer(const AlertAnalyzer&) = delete;
  AlertAnalyzer& operator=(const AlertAnalyzer&) = delete;

  // Evaluates rules against the snapshot and returns newly triggered alerts.
  std::vector<Alert> Analyze(const std::string& vehicle_id,
                              const VehicleHealthSnapshot& snapshot);

  std::vector<Alert> GetActiveAlerts() const;
  std::vector<Alert> GetActiveAlertsForVehicle(const std::string& vehicle_id) const;

  bool AcknowledgeAlert(const std::string& alert_id);
  void ClearAlertsForVehicle(const std::string& vehicle_id);

  void set_alert_callback(AlertCallback cb);
  void set_resolved_callback(AlertCallback cb);

  AlertStatistics GetStatistics() const;

 private:
  struct AlertRule {
    std::string name;
    std::function<bool(const VehicleHealthSnapshot&)> condition;
    AlertSeverity severity;
    std::string message;
  };

  void LoadRules();

  const config::CloudConfigManager& config_;
  std::vector<AlertRule> rules_;
  std::map<std::string, Alert> active_alerts_;  // key: vehicle_id + "_" + type
  uint64_t id_counter_ = 0;

  AlertCallback alert_cb_;
  AlertCallback resolved_cb_;
};

}  // namespace cloud
}  // namespace fsm
