/**
 * FSM-Pilot 操作端客户端实现
 * 整合方向盘输入和 WebRTC 通信
 */

#include "fsm/operator/operator_client.hpp"
#include "fsm/logger.hpp"
#include <chrono>
#include <cmath>

namespace fsm {
namespace operator_client {

OperatorClient::OperatorClient(const OperatorConfig& config)
    : config_(config)
    , running_(false)
    , current_gear_(3)  // D档
    , turn_signal_(0)
{
}

OperatorClient::~OperatorClient() {
    stop();
}

bool OperatorClient::initialize() {
    LOG_INFO("[Operator] Initializing operator client...");

    // 初始化方向盘控制器
    wheel_controller_ = std::make_unique<WheelController>();
    if (!wheel_controller_->open()) {
        LOG_WARNING("[Operator] No steering wheel detected, using keyboard mode");
    } else {
        LOG_INFO("[Operator] Steering wheel connected: " + wheel_controller_->getDeviceName());

        // 设置力反馈
        wheel_controller_->setForceFeedback(config_.force_feedback_strength, 0);
    }

    // 初始化 WebRTC 连接 (将在连接时创建)
    webrtc_connected_ = false;

    LOG_INFO("[Operator] Operator client initialized");
    return true;
}

void OperatorClient::start() {
    if (running_) return;

    running_ = true;

    // 启动输入处理线程
    input_thread_ = std::thread([this]() {
        inputLoop();
    });

    LOG_INFO("[Operator] Input processing started");
}

void OperatorClient::stop() {
    running_ = false;

    if (input_thread_.joinable()) {
        input_thread_.join();
    }

    disconnect();

    LOG_INFO("[Operator] Operator client stopped");
}

bool OperatorClient::connect(const std::string& vehicle_id) {
    LOG_INFO("[Operator] Connecting to vehicle: " + vehicle_id);
    current_vehicle_id_ = vehicle_id;

    // 创建 WebRTC 连接
    // 实际实现需要 WebRTC 库
    webrtc_connected_ = true;  // 占位

    return true;
}

void OperatorClient::disconnect() {
    webrtc_connected_ = false;
    current_vehicle_id_.clear();
    LOG_INFO("[Operator] Disconnected from vehicle");
}

void OperatorClient::inputLoop() {
    auto last_time = std::chrono::steady_clock::now();

    while (running_) {
        auto now = std::chrono::steady_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

        if (dt < config_.input_poll_interval_ms) {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                config_.input_poll_interval_ms - dt));
            continue;
        }

        last_time = now;

        // 读取方向盘输入
        if (wheel_controller_ && wheel_controller_->isConnected()) {
            processWheelInput();
        }
    }
}

void OperatorClient::processWheelInput() {
    auto input = wheel_controller_->getInput();

    // 构建控制指令
    ControlCommand cmd;
    cmd.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // 转向 (应用死区和曲线)
    cmd.steering = applyDeadzone(input.steering, config_.steering_deadzone);
    cmd.steering = applyCurve(cmd.steering, config_.steering_curve);

    // 油门和刹车
    cmd.throttle = input.throttle;
    cmd.brake = input.brake;

    // 档位
    cmd.gear = current_gear_;

    // 转向灯
    cmd.turn_signal = turn_signal_;

    // 紧急停车
    cmd.emergency = false;

    // 处理按钮
    if (input.emergency_button) {
        cmd.emergency = true;
        LOG_WARNING("[Operator] EMERGENCY STOP ACTIVATED!");
    }

    // 档位切换
    if (input.gear_up_button && !last_input_.gear_up_button) {
        shiftGearUp();
    }
    if (input.gear_down_button && !last_input_.gear_down_button) {
        shiftGearDown();
    }

    // 转向灯
    if (input.left_turn_button && !last_input_.left_turn_button) {
        turn_signal_ = (turn_signal_ == 1) ? 0 : 1;
        LOG_INFO("[Operator] Left turn signal: " + std::string(turn_signal_ == 1 ? "ON" : "OFF"));
    }
    if (input.right_turn_button && !last_input_.right_turn_button) {
        turn_signal_ = (turn_signal_ == 2) ? 0 : 2;
        LOG_INFO("[Operator] Right turn signal: " + std::string(turn_signal_ == 2 ? "ON" : "OFF"));
    }

    last_input_ = input;

    // 发送控制指令
    sendControlCommand(cmd);

    // 触发回调
    if (control_callback_) {
        control_callback_(cmd);
    }
}

void OperatorClient::sendControlCommand(const ControlCommand& cmd) {
    if (!webrtc_connected_ || current_vehicle_id_.empty()) {
        return;
    }

    // 通过 WebRTC 发送控制指令
    // 实际实现需要 WebRTC 数据通道
}

float OperatorClient::applyDeadzone(float value, float deadzone) {
    if (std::abs(value) < deadzone) {
        return 0.0f;
    }
    float sign = value > 0 ? 1.0f : -1.0f;
    return sign * (std::abs(value) - deadzone) / (1.0f - deadzone);
}

float OperatorClient::applyCurve(float value, const std::string& curve) {
    if (curve == "exponential") {
        float sign = value > 0 ? 1.0f : -1.0f;
        return sign * std::pow(std::abs(value), 2.0f);
    }
    // 默认线性
    return value;
}

void OperatorClient::shiftGearUp() {
    if (current_gear_ < 3) {
        current_gear_++;
        LOG_INFO("[Operator] Gear: " + getGearName(current_gear_));
    }
}

void OperatorClient::shiftGearDown() {
    if (current_gear_ > 0) {
        current_gear_--;
        LOG_INFO("[Operator] Gear: " + getGearName(current_gear_));
    }
}

std::string OperatorClient::getGearName(int gear) {
    switch (gear) {
        case 0: return "P";
        case 1: return "R";
        case 2: return "N";
        case 3: return "D";
        default: return "?";
    }
}

void OperatorClient::setControlCallback(ControlCallback callback) {
    control_callback_ = callback;
}

void OperatorClient::setTelemetryCallback(TelemetryCallback callback) {
    telemetry_callback_ = callback;
}

void OperatorClient::setVideoCallback(VideoCallback callback) {
    video_callback_ = callback;
}

void OperatorClient::triggerEmergencyStop() {
    ControlCommand cmd;
    cmd.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    cmd.steering = 0;
    cmd.throttle = 0;
    cmd.brake = 1.0f;
    cmd.emergency = true;

    sendControlCommand(cmd);
    LOG_WARNING("[Operator] Emergency stop triggered");

    // 力反馈震动
    if (wheel_controller_) {
        wheel_controller_->setForceFeedback(1.0f, 500);
    }
}

OperatorStatus OperatorClient::getStatus() const {
    OperatorStatus status;
    status.connected = webrtc_connected_;
    status.vehicle_id = current_vehicle_id_;
    status.wheel_connected = wheel_controller_ && wheel_controller_->isConnected();
    status.current_gear = current_gear_;
    status.turn_signal = turn_signal_;
    return status;
}

}  // namespace operator_client
}  // namespace fsm
