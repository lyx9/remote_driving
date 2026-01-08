#pragma once

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <memory>
#include <functional>
#include <atomic>
#include <thread>
#include <chrono>

#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"

namespace fsm {
namespace cloud {

/**
 * @brief 车辆调度信息
 */
struct VehicleSchedulingInfo {
    std::string vehicle_id;
    std::string task_status;        // IDLE, ACTIVE, PAUSED, COMPLETED, FAILED
    int emergency_level = 0;         // 0-4, 0=无紧急, 4=最高紧急
    float priority_score = 0.0f;     // 计算后的优先级分数
    double latitude = 0.0;
    double longitude = 0.0;
    float latency_ms = 0.0f;
    float battery_level = 100.0f;    // 0-100
    std::string current_task;
    int64_t last_update_time = 0;
    bool is_connected = false;

    // 额外指标
    float speed = 0.0f;
    int control_mode = 0;            // 0:MANUAL, 1:AUTONOMOUS, 2:REMOTE
};

/**
 * @brief 调度队列更新回调
 */
using SchedulingUpdateCallback = std::function<void(const std::vector<VehicleSchedulingInfo>&)>;

/**
 * @brief 车辆状态变化回调
 */
using VehicleStateChangeCallback = std::function<void(const std::string& vehicle_id, const VehicleSchedulingInfo&)>;

/**
 * @brief 调度算法类型
 */
enum class SchedulingAlgorithm {
    WEIGHTED_PRIORITY,    // 加权优先级
    ROUND_ROBIN,          // 轮询
    EMERGENCY_FIRST,      // 紧急优先
    LATENCY_BASED,        // 基于延迟
    CUSTOM                // 自定义
};

/**
 * @brief 多车辆调度服务
 *
 * 负责:
 * - 聚合多车辆状态
 * - 计算调度优先级
 * - 管理调度队列
 * - 生成告警
 */
class SchedulingService {
public:
    /**
     * @brief 构造函数
     * @param config 云端配置
     */
    explicit SchedulingService(const config::CloudConfigManager& config);

    ~SchedulingService();

    /**
     * @brief 启动调度服务
     */
    void start();

    /**
     * @brief 停止调度服务
     */
    void stop();

    /**
     * @brief 更新车辆状态
     * @param info 车辆调度信息
     */
    void updateVehicleState(const VehicleSchedulingInfo& info);

    /**
     * @brief 移除车辆
     * @param vehicle_id 车辆ID
     */
    void removeVehicle(const std::string& vehicle_id);

    /**
     * @brief 获取当前调度队列
     */
    std::vector<VehicleSchedulingInfo> getSchedulingQueue() const;

    /**
     * @brief 获取指定车辆的信息
     */
    std::optional<VehicleSchedulingInfo> getVehicleInfo(const std::string& vehicle_id) const;

    /**
     * @brief 获取已连接的车辆数量
     */
    size_t getConnectedVehicleCount() const;

    /**
     * @brief 设置调度算法
     */
    void setAlgorithm(SchedulingAlgorithm algorithm);

    /**
     * @brief 设置调度算法权重
     */
    void setWeights(const config::SchedulingWeights& weights);

    /**
     * @brief 启用/禁用调度
     */
    void setSchedulingEnabled(bool enabled);

    /**
     * @brief 检查调度是否启用
     */
    bool isSchedulingEnabled() const { return scheduling_enabled_; }

    /**
     * @brief 设置调度队列更新回调
     */
    void setSchedulingUpdateCallback(SchedulingUpdateCallback callback);

    /**
     * @brief 设置车辆状态变化回调
     */
    void setVehicleStateChangeCallback(VehicleStateChangeCallback callback);

    /**
     * @brief 强制重新计算调度
     */
    void recalculateScheduling();

private:
    /**
     * @brief 调度计算线程
     */
    void schedulingThread();

    /**
     * @brief 计算单个车辆的优先级分数
     */
    float calculatePriorityScore(const VehicleSchedulingInfo& info) const;

    /**
     * @brief 加权优先级算法
     */
    void weightedPrioritySort(std::vector<VehicleSchedulingInfo>& vehicles);

    /**
     * @brief 紧急优先算法
     */
    void emergencyFirstSort(std::vector<VehicleSchedulingInfo>& vehicles);

    /**
     * @brief 延迟优先算法
     */
    void latencyBasedSort(std::vector<VehicleSchedulingInfo>& vehicles);

    /**
     * @brief 检查车辆超时
     */
    void checkVehicleTimeouts();

    /**
     * @brief 生成告警
     */
    void generateAlerts();

    const config::CloudConfigManager& config_;

    // 车辆状态
    mutable std::mutex vehicles_mutex_;
    std::map<std::string, VehicleSchedulingInfo> vehicles_;

    // 调度队列
    mutable std::mutex queue_mutex_;
    std::vector<VehicleSchedulingInfo> scheduling_queue_;

    // 配置
    std::atomic<bool> scheduling_enabled_{true};
    SchedulingAlgorithm algorithm_{SchedulingAlgorithm::WEIGHTED_PRIORITY};
    config::SchedulingWeights weights_;

    // 回调
    SchedulingUpdateCallback scheduling_callback_;
    VehicleStateChangeCallback state_change_callback_;

    // 线程
    std::unique_ptr<std::thread> scheduling_thread_;
    std::atomic<bool> running_{false};
    std::condition_variable cv_;
    std::mutex cv_mutex_;

    // 参数
    int scheduling_interval_ms_ = 100;   // 调度间隔
    int vehicle_timeout_ms_ = 10000;     // 车辆超时
};

}  // namespace cloud
}  // namespace fsm
