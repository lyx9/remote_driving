#include "fsm/cloud/scheduling_service.hpp"
#include "fsm/utils.hpp"
#include <algorithm>
#include <condition_variable>

namespace fsm {
namespace cloud {

SchedulingService::SchedulingService(const config::CloudConfigManager& config)
    : config_(config) {
  const auto& sched = config_.scheduling_config();
  scheduling_enabled_ = sched.enabled;
  weights_            = sched.weights;

  if (sched.algorithm == "weighted_priority") {
    algorithm_ = SchedulingAlgorithm::kWeightedPriority;
  } else if (sched.algorithm == "emergency_first") {
    algorithm_ = SchedulingAlgorithm::kEmergencyFirst;
  } else if (sched.algorithm == "latency_based") {
    algorithm_ = SchedulingAlgorithm::kLatencyBased;
  }

  FSM_LOG_INFO("SchedulingService initialized");
}

SchedulingService::~SchedulingService() {
  Stop();
}

void SchedulingService::Start() {
  if (running_) {
    FSM_LOG_WARN("SchedulingService already running");
    return;
  }
  running_ = true;
  scheduling_thread_ = std::make_unique<std::thread>(
      &SchedulingService::SchedulingLoop, this);
  FSM_LOG_INFO("SchedulingService started");
}

void SchedulingService::Stop() {
  if (!running_) return;
  running_ = false;
  cv_.notify_all();
  if (scheduling_thread_ && scheduling_thread_->joinable()) {
    scheduling_thread_->join();
  }
  FSM_LOG_INFO("SchedulingService stopped");
}

void SchedulingService::UpdateVehicleState(const VehicleSchedulingInfo& info) {
  bool is_new = false;
  {
    std::lock_guard<std::mutex> lock(vehicles_mutex_);
    is_new = (vehicles_.find(info.vehicle_id) == vehicles_.end());
    vehicles_[info.vehicle_id] = info;
    vehicles_[info.vehicle_id].last_update_ns = utils::NowMillis() * 1000000LL;
  }

  if (is_new) FSM_LOG_INFO("Vehicle connected: {}", info.vehicle_id);
  if (state_change_cb_) state_change_cb_(info.vehicle_id, info);
  cv_.notify_one();
}

void SchedulingService::RemoveVehicle(const std::string& vehicle_id) {
  {
    std::lock_guard<std::mutex> lock(vehicles_mutex_);
    vehicles_.erase(vehicle_id);
  }
  FSM_LOG_INFO("Vehicle removed: {}", vehicle_id);
  cv_.notify_one();
}

std::vector<VehicleSchedulingInfo> SchedulingService::GetSchedulingQueue() const {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return scheduling_queue_;
}

std::optional<VehicleSchedulingInfo> SchedulingService::GetVehicleInfo(
    const std::string& vehicle_id) const {
  std::lock_guard<std::mutex> lock(vehicles_mutex_);
  const auto it = vehicles_.find(vehicle_id);
  if (it != vehicles_.end()) return it->second;
  return std::nullopt;
}

size_t SchedulingService::connected_vehicle_count() const {
  std::lock_guard<std::mutex> lock(vehicles_mutex_);
  size_t count = 0;
  for (const auto& [id, info] : vehicles_) {
    if (info.is_connected) ++count;
  }
  return count;
}

void SchedulingService::SetAlgorithm(SchedulingAlgorithm algorithm) {
  algorithm_ = algorithm;
  cv_.notify_one();
}

void SchedulingService::SetWeights(const config::SchedulingWeights& weights) {
  weights_ = weights;
  cv_.notify_one();
}

void SchedulingService::SetSchedulingEnabled(bool enabled) {
  scheduling_enabled_ = enabled;
  FSM_LOG_INFO("Scheduling {}", enabled ? "enabled" : "disabled");
}

void SchedulingService::set_scheduling_update_callback(SchedulingUpdateCallback cb) {
  scheduling_cb_ = std::move(cb);
}

void SchedulingService::set_vehicle_state_change_callback(VehicleStateChangeCallback cb) {
  state_change_cb_ = std::move(cb);
}

void SchedulingService::RecalculateNow() {
  cv_.notify_one();
}

void SchedulingService::SchedulingLoop() {
  FSM_LOG_DEBUG("Scheduling thread started");

  while (running_) {
    {
      std::unique_lock<std::mutex> lock(cv_mutex_);
      cv_.wait_for(lock, std::chrono::milliseconds(kSchedulingIntervalMs));
    }
    if (!running_) break;

    CheckVehicleTimeouts();
    if (!scheduling_enabled_) continue;

    std::vector<VehicleSchedulingInfo> vehicles;
    {
      std::lock_guard<std::mutex> lock(vehicles_mutex_);
      vehicles.reserve(vehicles_.size());
      for (const auto& [id, info] : vehicles_) {
        if (info.is_connected) vehicles.push_back(info);
      }
    }
    if (vehicles.empty()) continue;

    for (auto& v : vehicles) {
      v.priority_score = CalculatePriorityScore(v);
    }

    switch (algorithm_) {
      case SchedulingAlgorithm::kWeightedPriority: SortByWeightedPriority(vehicles); break;
      case SchedulingAlgorithm::kEmergencyFirst:   SortByEmergencyFirst(vehicles);   break;
      case SchedulingAlgorithm::kLatencyBased:     SortByLatency(vehicles);          break;
      default:                                     SortByWeightedPriority(vehicles); break;
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      scheduling_queue_ = std::move(vehicles);
    }

    if (scheduling_cb_) {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      scheduling_cb_(scheduling_queue_);
    }

    GenerateAlerts();
  }

  FSM_LOG_DEBUG("Scheduling thread stopped");
}

float SchedulingService::CalculatePriorityScore(
    const VehicleSchedulingInfo& info) const {
  const auto& sched = config_.scheduling_config();

  const float emergency_score = info.emergency_level * 25.0f;

  float latency_score = 0.0f;
  if (info.latency_ms > sched.critical_latency_ms) {
    latency_score = 100.0f;
  } else if (info.latency_ms > sched.max_latency_ms) {
    latency_score = 50.0f + 50.0f *
        (info.latency_ms - sched.max_latency_ms) /
        (sched.critical_latency_ms - sched.max_latency_ms);
  } else {
    latency_score = 50.0f * info.latency_ms / sched.max_latency_ms;
  }

  float battery_score = 0.0f;
  if (info.battery_pct < sched.min_battery_pct) {
    battery_score = 100.0f;
  } else {
    battery_score = 100.0f * (1.0f - info.battery_pct / 100.0f);
  }

  float task_score = 0.0f;
  if      (info.task_status == "FAILED") task_score = 100.0f;
  else if (info.task_status == "PAUSED") task_score = 75.0f;
  else if (info.task_status == "ACTIVE") task_score = 50.0f;

  return weights_.emergency     * emergency_score +
         weights_.latency       * latency_score   +
         weights_.battery       * battery_score   +
         weights_.task_priority * task_score      +
         weights_.distance      * 50.0f;
}

void SchedulingService::SortByWeightedPriority(
    std::vector<VehicleSchedulingInfo>& vehicles) const {
  std::sort(vehicles.begin(), vehicles.end(),
      [](const VehicleSchedulingInfo& a, const VehicleSchedulingInfo& b) {
        return a.priority_score > b.priority_score;
      });
}

void SchedulingService::SortByEmergencyFirst(
    std::vector<VehicleSchedulingInfo>& vehicles) const {
  std::sort(vehicles.begin(), vehicles.end(),
      [](const VehicleSchedulingInfo& a, const VehicleSchedulingInfo& b) {
        if (a.emergency_level != b.emergency_level)
          return a.emergency_level > b.emergency_level;
        return a.priority_score > b.priority_score;
      });
}

void SchedulingService::SortByLatency(
    std::vector<VehicleSchedulingInfo>& vehicles) const {
  std::sort(vehicles.begin(), vehicles.end(),
      [](const VehicleSchedulingInfo& a, const VehicleSchedulingInfo& b) {
        if (std::abs(a.latency_ms - b.latency_ms) > 50.0f)
          return a.latency_ms > b.latency_ms;
        return a.priority_score > b.priority_score;
      });
}

void SchedulingService::CheckVehicleTimeouts() {
  const int64_t now_ms = utils::NowMillis();
  std::vector<std::string> timed_out;

  {
    std::lock_guard<std::mutex> lock(vehicles_mutex_);
    for (auto& [id, info] : vehicles_) {
      if (!info.is_connected) continue;
      const int64_t elapsed_ms = now_ms - info.last_update_ns / 1000000LL;
      if (elapsed_ms > kVehicleTimeoutMs) {
        info.is_connected = false;
        timed_out.push_back(id);
      }
    }
  }

  for (const auto& id : timed_out) {
    FSM_LOG_WARN("Vehicle {} timed out", id);
  }
}

void SchedulingService::GenerateAlerts() {
  std::lock_guard<std::mutex> lock(vehicles_mutex_);
  const auto& sched = config_.scheduling_config();

  for (const auto& [id, info] : vehicles_) {
    if (!info.is_connected) continue;

    if (info.latency_ms > sched.critical_latency_ms) {
      FSM_LOG_WARN("Vehicle {} critical latency: {:.0f}ms", id, info.latency_ms);
    } else if (info.latency_ms > sched.max_latency_ms) {
      FSM_LOG_WARN("Vehicle {} high latency: {:.0f}ms", id, info.latency_ms);
    }

    if (info.battery_pct < sched.min_battery_pct) {
      FSM_LOG_WARN("Vehicle {} low battery: {:.0f}%", id, info.battery_pct);
    }

    if (info.emergency_level >= 3) {
      FSM_LOG_CRITICAL("Vehicle {} emergency level {}", id, info.emergency_level);
    }
  }
}

}  // namespace cloud
}  // namespace fsm
