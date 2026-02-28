#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"

namespace fsm {
namespace cloud {

struct VehicleSchedulingInfo {
  std::string vehicle_id;
  std::string task_status;      // IDLE | ACTIVE | PAUSED | COMPLETED | FAILED
  int emergency_level = 0;      // 0–4; 0 = none, 4 = highest
  float priority_score = 0.0f;
  double latitude = 0.0;
  double longitude = 0.0;
  float latency_ms = 0.0f;
  float battery_pct = 100.0f;
  std::string current_task;
  int64_t last_update_ns = 0;
  bool is_connected = false;
  float speed_mps = 0.0f;
  int control_mode = 0;         // 0:MANUAL 1:AUTONOMOUS 2:REMOTE
};

enum class SchedulingAlgorithm {
  kWeightedPriority,
  kRoundRobin,
  kEmergencyFirst,
  kLatencyBased,
};

using SchedulingUpdateCallback   = std::function<void(const std::vector<VehicleSchedulingInfo>&)>;
using VehicleStateChangeCallback = std::function<void(const std::string&, const VehicleSchedulingInfo&)>;

class SchedulingService {
 public:
  explicit SchedulingService(const config::CloudConfigManager& config);
  ~SchedulingService();

  SchedulingService(const SchedulingService&) = delete;
  SchedulingService& operator=(const SchedulingService&) = delete;

  void Start();
  void Stop();

  void UpdateVehicleState(const VehicleSchedulingInfo& info);
  void RemoveVehicle(const std::string& vehicle_id);

  std::vector<VehicleSchedulingInfo> GetSchedulingQueue() const;
  std::optional<VehicleSchedulingInfo> GetVehicleInfo(const std::string& vehicle_id) const;
  size_t connected_vehicle_count() const;

  void SetAlgorithm(SchedulingAlgorithm algorithm);
  void SetWeights(const config::SchedulingWeights& weights);
  void SetSchedulingEnabled(bool enabled);
  bool is_scheduling_enabled() const { return scheduling_enabled_.load(); }

  void set_scheduling_update_callback(SchedulingUpdateCallback cb);
  void set_vehicle_state_change_callback(VehicleStateChangeCallback cb);

  void RecalculateNow();

 private:
  void SchedulingLoop();
  float CalculatePriorityScore(const VehicleSchedulingInfo& info) const;
  void SortByWeightedPriority(std::vector<VehicleSchedulingInfo>& vehicles) const;
  void SortByEmergencyFirst(std::vector<VehicleSchedulingInfo>& vehicles) const;
  void SortByLatency(std::vector<VehicleSchedulingInfo>& vehicles) const;
  void CheckVehicleTimeouts();
  void GenerateAlerts();

  const config::CloudConfigManager& config_;

  mutable std::mutex vehicles_mutex_;
  std::map<std::string, VehicleSchedulingInfo> vehicles_;

  mutable std::mutex queue_mutex_;
  std::vector<VehicleSchedulingInfo> scheduling_queue_;

  std::atomic<bool> scheduling_enabled_{true};
  SchedulingAlgorithm algorithm_{SchedulingAlgorithm::kWeightedPriority};
  config::SchedulingWeights weights_;

  SchedulingUpdateCallback   scheduling_cb_;
  VehicleStateChangeCallback state_change_cb_;

  std::unique_ptr<std::thread> scheduling_thread_;
  std::atomic<bool> running_{false};
  std::condition_variable cv_;
  std::mutex cv_mutex_;

  static constexpr int kSchedulingIntervalMs = 100;
  static constexpr int kVehicleTimeoutMs     = 10000;
};

}  // namespace cloud
}  // namespace fsm
