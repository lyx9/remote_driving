#include "fsm/vehicle/command_executor.hpp"
#include "fsm/utils.hpp"

namespace fsm {
namespace vehicle {

CommandExecutor::CommandExecutor(
    rclcpp::Node::SharedPtr node,
    const config::VehicleConfigManager& config)
    : node_(node), config_(config) {

    const auto& topics = config_.getControlTopics();

#ifdef HAVE_AUTOWARE_MSGS
    // Autoware控制话题发布者
    ackermann_pub_ = node_->create_publisher<
        autoware_auto_control_msgs::msg::AckermannControlCommand>(
        topics.steering_cmd, 10);

    gear_pub_ = node_->create_publisher<
        autoware_auto_vehicle_msgs::msg::GearCommand>(
        topics.gear_cmd, 10);

    turn_signal_pub_ = node_->create_publisher<
        autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>(
        topics.turn_signal_cmd, 10);

    emergency_pub_ = node_->create_publisher<
        tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
        topics.emergency_cmd, 10);

    FSM_LOG_INFO("CommandExecutor initialized with Autoware topics");

#else
    // 通用话题发布者
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        topics.velocity_cmd, 10);

    emergency_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
        topics.emergency_cmd, 10);

    FSM_LOG_INFO("CommandExecutor initialized with generic topics");
#endif

    FSM_LOG_INFO("CommandExecutor initialized for vehicle: {}", config_.getVehicleId());
}

CommandExecutor::~CommandExecutor() {
    stop();
}

void CommandExecutor::start() {
    if (running_) {
        FSM_LOG_WARN("CommandExecutor already running");
        return;
    }

    running_ = true;

    // 启动看门狗定时器
    const auto& safety = config_.getSafetyConfig();
    watchdog_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(safety.watchdog_timeout_ms),
        std::bind(&CommandExecutor::watchdogCallback, this));

    last_command_tp_ = std::chrono::steady_clock::now();

    FSM_LOG_INFO("CommandExecutor started");
}

void CommandExecutor::stop() {
    if (!running_) {
        return;
    }

    running_ = false;

    // 发送零指令
    ControlCommand zero_cmd;
    zero_cmd.emergency_stop = false;
    executeCommand(zero_cmd);

    // 停止看门狗
    if (watchdog_timer_) {
        watchdog_timer_->cancel();
        watchdog_timer_.reset();
    }

    FSM_LOG_INFO("CommandExecutor stopped");
}

void CommandExecutor::executeCommand(const ControlCommand& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!running_) {
        FSM_LOG_WARN("CommandExecutor not running, ignoring command");
        return;
    }

    // 紧急停车优先
    if (cmd.emergency_stop) {
        triggerEmergencyStop("Remote emergency command");
        return;
    }

    // 如果处于紧急状态，忽略非紧急指令
    if (emergency_active_) {
        FSM_LOG_WARN("Emergency active, ignoring control command");
        return;
    }

    // 安全检查
    ControlCommand safe_cmd = cmd;
    if (!safetyCheck(safe_cmd)) {
        FSM_LOG_WARN("Command failed safety check, applying limits");
    }

    // 应用软限制
    applySoftLimits(safe_cmd);

    // 检查序列号
    if (cmd.sequence_number > 0) {
        if (cmd.sequence_number != expected_sequence_) {
            missed_commands_ += (cmd.sequence_number - expected_sequence_);
            FSM_LOG_WARN("Missed {} commands", cmd.sequence_number - expected_sequence_);
        }
        expected_sequence_ = cmd.sequence_number + 1;
    }

    // 发布控制指令
    publishAckermannCommand(safe_cmd);

    // 发布档位指令 (如果变化)
    if (safe_cmd.gear != last_command_.gear) {
        publishGearCommand(safe_cmd.gear);
    }

    // 发布转向灯指令 (如果变化)
    if (safe_cmd.turn_signal != last_command_.turn_signal) {
        publishTurnSignalCommand(safe_cmd.turn_signal);
    }

    // 更新状态
    last_command_ = safe_cmd;
    last_command_tp_ = std::chrono::steady_clock::now();
    last_command_time_ = utils::TimeUtils::nowNanos();
}

bool CommandExecutor::safetyCheck(const ControlCommand& cmd) {
    const auto& params = config_.getVehicleParams();
    const auto& safety = config_.getSafetyConfig();

    bool passed = true;

    // 检查转向角范围
    double max_steering_rad = utils::MathUtils::degToRad(params.max_steering_angle);
    if (std::abs(cmd.steering_tire_angle) > max_steering_rad) {
        FSM_LOG_WARN("Steering angle {} exceeds limit {}",
                     utils::MathUtils::radToDeg(cmd.steering_tire_angle),
                     params.max_steering_angle);
        passed = false;
    }

    // 检查速度范围
    double max_speed_ms = params.max_speed / 3.6;  // km/h -> m/s
    if (cmd.speed > max_speed_ms) {
        FSM_LOG_WARN("Speed {} m/s exceeds limit {} m/s", cmd.speed, max_speed_ms);
        passed = false;
    }

    // 检查加速度范围
    if (cmd.acceleration > params.max_acceleration) {
        FSM_LOG_WARN("Acceleration {} exceeds limit {}", cmd.acceleration, params.max_acceleration);
        passed = false;
    }

    if (cmd.acceleration < -params.max_deceleration) {
        FSM_LOG_WARN("Deceleration {} exceeds limit {}", -cmd.acceleration, params.max_deceleration);
        passed = false;
    }

    return passed;
}

void CommandExecutor::applySoftLimits(ControlCommand& cmd) {
    const auto& params = config_.getVehicleParams();
    const auto& safety = config_.getSafetyConfig();

    if (!safety.enable_soft_limits) {
        return;
    }

    // 限制转向角
    double max_steering_rad = utils::MathUtils::degToRad(params.max_steering_angle);
    cmd.steering_tire_angle = utils::MathUtils::clamp(
        static_cast<double>(cmd.steering_tire_angle), -max_steering_rad, max_steering_rad);

    // 限制速度
    double max_speed_ms = params.max_speed / 3.6;
    cmd.speed = utils::MathUtils::clamp(cmd.speed, 0.0f, static_cast<float>(max_speed_ms));

    // 限制加速度
    cmd.acceleration = utils::MathUtils::clamp(
        cmd.acceleration,
        static_cast<float>(-params.max_deceleration),
        static_cast<float>(params.max_acceleration));

    // 限制转向变化率
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - last_command_tp_).count();
    if (dt > 0 && dt < 1.0) {
        double max_delta = utils::MathUtils::degToRad(safety.max_steering_rate) * dt;
        double delta = cmd.steering_tire_angle - last_command_.steering_tire_angle;
        delta = utils::MathUtils::clamp(delta, -max_delta, max_delta);
        cmd.steering_tire_angle = last_command_.steering_tire_angle + delta;
    }
}

void CommandExecutor::watchdogCallback() {
    if (!running_ || !remote_control_active_) {
        return;
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_command_tp_).count();

    const auto& safety = config_.getSafetyConfig();

    if (elapsed > safety.watchdog_timeout_ms * 2) {
        // 超时两倍，触发紧急停车
        FSM_LOG_ERROR("Watchdog timeout! No command for {} ms", elapsed);
        if (safety.enable_emergency_stop) {
            triggerEmergencyStop("Watchdog timeout");
        }
    } else if (elapsed > safety.watchdog_timeout_ms) {
        // 超时一倍，发送零指令
        FSM_LOG_WARN("Command timeout! Sending zero command");
        ControlCommand zero_cmd;
        publishAckermannCommand(zero_cmd);
    }
}

void CommandExecutor::publishAckermannCommand(const ControlCommand& cmd) {
#ifdef HAVE_AUTOWARE_MSGS
    auto msg = autoware_auto_control_msgs::msg::AckermannControlCommand();
    msg.stamp = node_->now();

    msg.lateral.steering_tire_angle = cmd.steering_tire_angle;
    msg.lateral.steering_tire_rotation_rate = cmd.steering_tire_rotation_rate;

    msg.longitudinal.speed = cmd.speed;
    msg.longitudinal.acceleration = cmd.acceleration;
    msg.longitudinal.jerk = cmd.jerk;

    ackermann_pub_->publish(msg);

#else
    auto msg = geometry_msgs::msg::TwistStamped();
    msg.header.stamp = node_->now();
    msg.header.frame_id = "base_link";

    msg.twist.linear.x = cmd.speed;
    msg.twist.angular.z = cmd.steering_tire_angle;

    twist_pub_->publish(msg);
#endif
}

void CommandExecutor::publishGearCommand(int gear) {
#ifdef HAVE_AUTOWARE_MSGS
    auto msg = autoware_auto_vehicle_msgs::msg::GearCommand();
    msg.stamp = node_->now();
    msg.command = static_cast<uint8_t>(gear);
    gear_pub_->publish(msg);

    FSM_LOG_DEBUG("Gear command: {}", gear);
#endif
}

void CommandExecutor::publishTurnSignalCommand(int signal) {
#ifdef HAVE_AUTOWARE_MSGS
    auto msg = autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand();
    msg.stamp = node_->now();
    msg.command = static_cast<uint8_t>(signal);
    turn_signal_pub_->publish(msg);

    FSM_LOG_DEBUG("Turn signal command: {}", signal);
#endif
}

void CommandExecutor::publishEmergencyCommand(bool emergency) {
#ifdef HAVE_AUTOWARE_MSGS
    auto msg = tier4_vehicle_msgs::msg::VehicleEmergencyStamped();
    msg.stamp = node_->now();
    msg.emergency = emergency;
    emergency_pub_->publish(msg);

#else
    auto msg = std_msgs::msg::Bool();
    msg.data = emergency;
    emergency_pub_->publish(msg);
#endif
}

void CommandExecutor::triggerEmergencyStop(const std::string& reason) {
    if (emergency_active_) {
        return;
    }

    emergency_active_ = true;
    FSM_LOG_CRITICAL("EMERGENCY STOP triggered: {}", reason);

    // 发布紧急停车指令
    publishEmergencyCommand(true);

    // 发送零速度指令
    ControlCommand emergency_cmd;
    emergency_cmd.speed = 0;
    emergency_cmd.acceleration = -config_.getSafetyConfig().emergency_decel;
    publishAckermannCommand(emergency_cmd);
}

void CommandExecutor::releaseEmergencyStop() {
    if (!emergency_active_) {
        return;
    }

    FSM_LOG_INFO("Emergency stop released");
    emergency_active_ = false;

    publishEmergencyCommand(false);
}

void CommandExecutor::setRemoteControlMode() {
    if (remote_control_active_) {
        return;
    }

    remote_control_active_ = true;
    last_command_tp_ = std::chrono::steady_clock::now();

    FSM_LOG_INFO("Remote control mode activated");
}

void CommandExecutor::exitRemoteControlMode() {
    if (!remote_control_active_) {
        return;
    }

    // 发送零指令
    ControlCommand zero_cmd;
    executeCommand(zero_cmd);

    remote_control_active_ = false;
    FSM_LOG_INFO("Remote control mode deactivated");
}

}  // namespace vehicle
}  // namespace fsm
