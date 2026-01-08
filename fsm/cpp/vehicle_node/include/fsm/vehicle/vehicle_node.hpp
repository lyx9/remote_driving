#pragma once

#include <rclcpp/rclcpp.hpp>
#include "fsm/config_manager.hpp"
#include "fsm/vehicle/data_collector.hpp"
#include "fsm/vehicle/command_executor.hpp"
#include "fsm/vehicle/webrtc_client.hpp"
#include "fsm/logger.hpp"

#include <memory>
#include <string>
#include <atomic>

namespace fsm {
namespace vehicle {

/**
 * @brief FSM车端节点
 *
 * 主节点类，整合数据采集、WebRTC通信和指令执行
 */
class VehicleNode : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * @param options ROS2节点选项
     */
    explicit VehicleNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    ~VehicleNode();

    /**
     * @brief 初始化节点
     * @param config_path 配置文件路径
     * @return 是否成功初始化
     */
    bool initialize(const std::string& config_path);

    /**
     * @brief 启动节点
     */
    void start();

    /**
     * @brief 停止节点
     */
    void stop();

    /**
     * @brief 检查节点是否正在运行
     */
    bool isRunning() const { return running_; }

    /**
     * @brief 获取车辆ID
     */
    const std::string& getVehicleId() const { return config_manager_.getVehicleId(); }

    /**
     * @brief 获取WebRTC连接状态
     */
    WebRTCState getConnectionState() const;

    /**
     * @brief 获取延迟信息
     */
    LatencyInfo getLatencyInfo() const;

private:
    // 回调处理
    void onVehicleStateUpdate(const VehicleState& state);
    void onSystemStateUpdate(const SystemState& state);
    void onCameraFrame(const CameraFrame& frame);
    void onConnectionStateChange(WebRTCState state);
    void onControlCommand(const ControlCommand& cmd);
    void onLatencyUpdate(const LatencyInfo& info);

    // 定时发送遥测数据
    void telemetryTimerCallback();

    // 状态检查
    void healthCheckCallback();

    // 序列化辅助
    std::vector<uint8_t> serializeVehicleState(const VehicleState& state);
    std::vector<uint8_t> serializeSystemState(const SystemState& state);

    // 配置管理
    config::VehicleConfigManager config_manager_;

    // 子模块
    std::unique_ptr<DataCollector> data_collector_;
    std::unique_ptr<CommandExecutor> command_executor_;
    std::unique_ptr<WebRTCClient> webrtc_client_;

    // 定时器
    rclcpp::TimerBase::SharedPtr telemetry_timer_;
    rclcpp::TimerBase::SharedPtr health_check_timer_;

    // 状态
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    // 缓存的状态数据
    std::mutex state_mutex_;
    VehicleState cached_vehicle_state_;
    SystemState cached_system_state_;
    LatencyInfo cached_latency_info_;

    // 统计
    uint64_t telemetry_sent_count_ = 0;
    uint64_t commands_received_count_ = 0;
    uint64_t frames_sent_count_ = 0;
};

}  // namespace vehicle
}  // namespace fsm
