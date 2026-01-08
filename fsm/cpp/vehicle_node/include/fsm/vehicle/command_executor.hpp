#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"

#include <mutex>
#include <atomic>
#include <chrono>

#ifdef HAVE_AUTOWARE_MSGS
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#endif

namespace fsm {
namespace vehicle {

/**
 * @brief 控制指令数据结构
 */
struct ControlCommand {
    // 转向控制
    float steering_tire_angle = 0.0;      // rad
    float steering_tire_rotation_rate = 0.0;  // rad/s

    // 纵向控制
    float speed = 0.0;            // m/s
    float acceleration = 0.0;     // m/s²
    float jerk = 0.0;             // m/s³

    // 档位
    int gear = 3;  // 0:P, 1:R, 2:N, 3:D

    // 信号灯
    int turn_signal = 0;  // 0:NONE, 1:LEFT, 2:RIGHT
    bool hazard_lights = false;

    // 紧急停车
    bool emergency_stop = false;

    // 时间戳和序列号
    int64_t timestamp_ns = 0;
    uint64_t sequence_number = 0;
};

/**
 * @brief 指令执行器
 *
 * 接收远程控制指令并发布到Autoware控制话题
 */
class CommandExecutor {
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     * @param config 车辆配置
     */
    CommandExecutor(
        rclcpp::Node::SharedPtr node,
        const config::VehicleConfigManager& config);

    ~CommandExecutor();

    /**
     * @brief 启动指令执行器
     */
    void start();

    /**
     * @brief 停止指令执行器
     */
    void stop();

    /**
     * @brief 执行控制指令
     * @param cmd 控制指令
     */
    void executeCommand(const ControlCommand& cmd);

    /**
     * @brief 触发紧急停车
     * @param reason 紧急停车原因
     */
    void triggerEmergencyStop(const std::string& reason = "Remote emergency");

    /**
     * @brief 解除紧急停车
     */
    void releaseEmergencyStop();

    /**
     * @brief 检查是否处于紧急状态
     */
    bool isEmergencyActive() const { return emergency_active_; }

    /**
     * @brief 设置控制模式为远程
     */
    void setRemoteControlMode();

    /**
     * @brief 退出远程控制模式
     */
    void exitRemoteControlMode();

    /**
     * @brief 检查是否处于远程控制模式
     */
    bool isRemoteControlActive() const { return remote_control_active_; }

    /**
     * @brief 获取最后一次指令的时间戳
     */
    int64_t getLastCommandTime() const { return last_command_time_; }

private:
    /**
     * @brief 安全检查
     * @param cmd 待执行的指令
     * @return 是否通过安全检查
     */
    bool safetyCheck(const ControlCommand& cmd);

    /**
     * @brief 应用软限制
     * @param cmd 控制指令 (会被修改)
     */
    void applySoftLimits(ControlCommand& cmd);

    /**
     * @brief 看门狗定时器回调
     */
    void watchdogCallback();

    /**
     * @brief 发布Ackermann控制指令
     */
    void publishAckermannCommand(const ControlCommand& cmd);

    /**
     * @brief 发布档位指令
     */
    void publishGearCommand(int gear);

    /**
     * @brief 发布转向灯指令
     */
    void publishTurnSignalCommand(int signal);

    /**
     * @brief 发布紧急停车指令
     */
    void publishEmergencyCommand(bool emergency);

    rclcpp::Node::SharedPtr node_;
    const config::VehicleConfigManager& config_;

    // 发布者
#ifdef HAVE_AUTOWARE_MSGS
    rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr ackermann_pub_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr gear_pub_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_signal_pub_;
    rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr hazard_pub_;
    rclcpp::Publisher<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr emergency_pub_;
#else
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
#endif

    // 看门狗定时器
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // 状态
    mutable std::mutex mutex_;
    std::atomic<bool> running_{false};
    std::atomic<bool> emergency_active_{false};
    std::atomic<bool> remote_control_active_{false};
    std::atomic<int64_t> last_command_time_{0};

    // 上一次的指令 (用于限制变化率)
    ControlCommand last_command_;
    std::chrono::steady_clock::time_point last_command_tp_;

    // 序列号跟踪
    uint64_t expected_sequence_ = 0;
    uint64_t missed_commands_ = 0;
};

}  // namespace vehicle
}  // namespace fsm
