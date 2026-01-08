#include "fsm/cloud/scheduling_service.hpp"
#include "fsm/utils.hpp"
#include <algorithm>

namespace fsm {
namespace cloud {

SchedulingService::SchedulingService(const config::CloudConfigManager& config)
    : config_(config) {

    // 从配置加载参数
    const auto& sched_config = config_.getSchedulingConfig();
    scheduling_enabled_ = sched_config.enabled;
    weights_ = sched_config.weights;

    if (sched_config.algorithm == "weighted_priority") {
        algorithm_ = SchedulingAlgorithm::WEIGHTED_PRIORITY;
    } else if (sched_config.algorithm == "emergency_first") {
        algorithm_ = SchedulingAlgorithm::EMERGENCY_FIRST;
    } else if (sched_config.algorithm == "latency_based") {
        algorithm_ = SchedulingAlgorithm::LATENCY_BASED;
    }

    FSM_LOG_INFO("SchedulingService initialized");
}

SchedulingService::~SchedulingService() {
    stop();
}

void SchedulingService::start() {
    if (running_) {
        FSM_LOG_WARN("SchedulingService already running");
        return;
    }

    running_ = true;

    // 启动调度线程
    scheduling_thread_ = std::make_unique<std::thread>(
        &SchedulingService::schedulingThread, this);

    FSM_LOG_INFO("SchedulingService started");
}

void SchedulingService::stop() {
    if (!running_) {
        return;
    }

    running_ = false;

    // 唤醒线程
    cv_.notify_all();

    if (scheduling_thread_ && scheduling_thread_->joinable()) {
        scheduling_thread_->join();
    }

    FSM_LOG_INFO("SchedulingService stopped");
}

void SchedulingService::updateVehicleState(const VehicleSchedulingInfo& info) {
    bool is_new_vehicle = false;

    {
        std::lock_guard<std::mutex> lock(vehicles_mutex_);

        auto it = vehicles_.find(info.vehicle_id);
        if (it == vehicles_.end()) {
            is_new_vehicle = true;
        }

        vehicles_[info.vehicle_id] = info;
        vehicles_[info.vehicle_id].last_update_time = utils::TimeUtils::nowMillis();
    }

    if (is_new_vehicle) {
        FSM_LOG_INFO("New vehicle connected: {}", info.vehicle_id);
    }

    // 触发状态变化回调
    if (state_change_callback_) {
        state_change_callback_(info.vehicle_id, info);
    }

    // 唤醒调度线程重新计算
    cv_.notify_one();
}

void SchedulingService::removeVehicle(const std::string& vehicle_id) {
    {
        std::lock_guard<std::mutex> lock(vehicles_mutex_);
        vehicles_.erase(vehicle_id);
    }

    FSM_LOG_INFO("Vehicle removed: {}", vehicle_id);

    cv_.notify_one();
}

std::vector<VehicleSchedulingInfo> SchedulingService::getSchedulingQueue() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return scheduling_queue_;
}

std::optional<VehicleSchedulingInfo> SchedulingService::getVehicleInfo(
    const std::string& vehicle_id) const {

    std::lock_guard<std::mutex> lock(vehicles_mutex_);

    auto it = vehicles_.find(vehicle_id);
    if (it != vehicles_.end()) {
        return it->second;
    }

    return std::nullopt;
}

size_t SchedulingService::getConnectedVehicleCount() const {
    std::lock_guard<std::mutex> lock(vehicles_mutex_);

    size_t count = 0;
    for (const auto& [id, info] : vehicles_) {
        if (info.is_connected) {
            ++count;
        }
    }

    return count;
}

void SchedulingService::setAlgorithm(SchedulingAlgorithm algorithm) {
    algorithm_ = algorithm;
    cv_.notify_one();
}

void SchedulingService::setWeights(const config::SchedulingWeights& weights) {
    weights_ = weights;
    cv_.notify_one();
}

void SchedulingService::setSchedulingEnabled(bool enabled) {
    scheduling_enabled_ = enabled;

    if (enabled) {
        FSM_LOG_INFO("Scheduling enabled");
    } else {
        FSM_LOG_INFO("Scheduling disabled");
    }
}

void SchedulingService::setSchedulingUpdateCallback(SchedulingUpdateCallback callback) {
    scheduling_callback_ = std::move(callback);
}

void SchedulingService::setVehicleStateChangeCallback(VehicleStateChangeCallback callback) {
    state_change_callback_ = std::move(callback);
}

void SchedulingService::recalculateScheduling() {
    cv_.notify_one();
}

void SchedulingService::schedulingThread() {
    FSM_LOG_DEBUG("Scheduling thread started");

    while (running_) {
        // 等待更新或超时
        {
            std::unique_lock<std::mutex> lock(cv_mutex_);
            cv_.wait_for(lock, std::chrono::milliseconds(scheduling_interval_ms_));
        }

        if (!running_) break;

        // 检查车辆超时
        checkVehicleTimeouts();

        // 如果调度未启用，跳过计算
        if (!scheduling_enabled_) {
            continue;
        }

        // 复制车辆列表
        std::vector<VehicleSchedulingInfo> vehicles;
        {
            std::lock_guard<std::mutex> lock(vehicles_mutex_);
            vehicles.reserve(vehicles_.size());
            for (const auto& [id, info] : vehicles_) {
                if (info.is_connected) {
                    vehicles.push_back(info);
                }
            }
        }

        if (vehicles.empty()) {
            continue;
        }

        // 计算优先级分数
        for (auto& v : vehicles) {
            v.priority_score = calculatePriorityScore(v);
        }

        // 根据算法排序
        switch (algorithm_) {
            case SchedulingAlgorithm::WEIGHTED_PRIORITY:
                weightedPrioritySort(vehicles);
                break;
            case SchedulingAlgorithm::EMERGENCY_FIRST:
                emergencyFirstSort(vehicles);
                break;
            case SchedulingAlgorithm::LATENCY_BASED:
                latencyBasedSort(vehicles);
                break;
            default:
                weightedPrioritySort(vehicles);
                break;
        }

        // 更新调度队列
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            scheduling_queue_ = std::move(vehicles);
        }

        // 触发回调
        if (scheduling_callback_) {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            scheduling_callback_(scheduling_queue_);
        }

        // 生成告警
        generateAlerts();
    }

    FSM_LOG_DEBUG("Scheduling thread stopped");
}

float SchedulingService::calculatePriorityScore(const VehicleSchedulingInfo& info) const {
    float score = 0.0f;

    // 紧急程度分数 (0-100)
    float emergency_score = info.emergency_level * 25.0f;  // 0, 25, 50, 75, 100

    // 延迟分数 (延迟越高分数越高)
    float latency_score = 0.0f;
    const auto& sched_config = config_.getSchedulingConfig();
    if (info.latency_ms > sched_config.critical_latency_ms) {
        latency_score = 100.0f;
    } else if (info.latency_ms > sched_config.max_latency_ms) {
        latency_score = 50.0f + 50.0f * (info.latency_ms - sched_config.max_latency_ms) /
                        (sched_config.critical_latency_ms - sched_config.max_latency_ms);
    } else {
        latency_score = 50.0f * info.latency_ms / sched_config.max_latency_ms;
    }

    // 电量分数 (电量越低分数越高)
    float battery_score = 0.0f;
    if (info.battery_level < sched_config.min_battery_percent) {
        battery_score = 100.0f;
    } else {
        battery_score = 100.0f * (1.0f - info.battery_level / 100.0f);
    }

    // 任务优先级分数
    float task_score = 0.0f;
    if (info.task_status == "ACTIVE") {
        task_score = 50.0f;
    } else if (info.task_status == "PAUSED") {
        task_score = 75.0f;  // 暂停的任务优先处理
    } else if (info.task_status == "FAILED") {
        task_score = 100.0f;
    }

    // 距离分数 (简化: 可以根据到目的地的距离计算)
    float distance_score = 50.0f;  // 暂时使用固定值

    // 加权求和
    score = weights_.emergency * emergency_score +
            weights_.latency * latency_score +
            weights_.battery * battery_score +
            weights_.task_priority * task_score +
            weights_.distance * distance_score;

    return score;
}

void SchedulingService::weightedPrioritySort(std::vector<VehicleSchedulingInfo>& vehicles) {
    std::sort(vehicles.begin(), vehicles.end(),
        [](const VehicleSchedulingInfo& a, const VehicleSchedulingInfo& b) {
            return a.priority_score > b.priority_score;  // 降序
        });
}

void SchedulingService::emergencyFirstSort(std::vector<VehicleSchedulingInfo>& vehicles) {
    std::sort(vehicles.begin(), vehicles.end(),
        [](const VehicleSchedulingInfo& a, const VehicleSchedulingInfo& b) {
            if (a.emergency_level != b.emergency_level) {
                return a.emergency_level > b.emergency_level;  // 紧急级别高的优先
            }
            return a.priority_score > b.priority_score;
        });
}

void SchedulingService::latencyBasedSort(std::vector<VehicleSchedulingInfo>& vehicles) {
    std::sort(vehicles.begin(), vehicles.end(),
        [](const VehicleSchedulingInfo& a, const VehicleSchedulingInfo& b) {
            // 延迟高的优先 (需要更多关注)
            if (std::abs(a.latency_ms - b.latency_ms) > 50.0f) {
                return a.latency_ms > b.latency_ms;
            }
            return a.priority_score > b.priority_score;
        });
}

void SchedulingService::checkVehicleTimeouts() {
    int64_t now = utils::TimeUtils::nowMillis();
    std::vector<std::string> timed_out;

    {
        std::lock_guard<std::mutex> lock(vehicles_mutex_);

        for (auto& [id, info] : vehicles_) {
            if (info.is_connected) {
                int64_t elapsed = now - info.last_update_time;
                if (elapsed > vehicle_timeout_ms_) {
                    info.is_connected = false;
                    timed_out.push_back(id);
                }
            }
        }
    }

    for (const auto& id : timed_out) {
        FSM_LOG_WARN("Vehicle {} connection timed out", id);
    }
}

void SchedulingService::generateAlerts() {
    // 检查各种告警条件
    std::lock_guard<std::mutex> lock(vehicles_mutex_);

    const auto& sched_config = config_.getSchedulingConfig();

    for (const auto& [id, info] : vehicles_) {
        if (!info.is_connected) continue;

        // 高延迟告警
        if (info.latency_ms > sched_config.critical_latency_ms) {
            FSM_LOG_WARN("ALERT: Vehicle {} has critical latency: {}ms", id, info.latency_ms);
        } else if (info.latency_ms > sched_config.max_latency_ms) {
            FSM_LOG_WARN("ALERT: Vehicle {} has high latency: {}ms", id, info.latency_ms);
        }

        // 低电量告警
        if (info.battery_level < sched_config.min_battery_percent) {
            FSM_LOG_WARN("ALERT: Vehicle {} has low battery: {}%", id, info.battery_level);
        }

        // 紧急状态告警
        if (info.emergency_level >= 3) {
            FSM_LOG_CRITICAL("ALERT: Vehicle {} in emergency state: level {}", id, info.emergency_level);
        }
    }
}

}  // namespace cloud
}  // namespace fsm
