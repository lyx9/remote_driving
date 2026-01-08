/**
 * FSM-Pilot 操作端客户端头文件
 */

#pragma once

#include "fsm/operator/wheel_controller.hpp"
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <functional>

namespace fsm {
namespace operator_client {

/**
 * 操作端配置
 */
struct OperatorConfig {
    std::string signaling_url;
    int input_poll_interval_ms = 10;
    float steering_deadzone = 0.02f;
    std::string steering_curve = "linear";
    float force_feedback_strength = 0.5f;
};

/**
 * 控制指令
 */
struct ControlCommand {
    int64_t timestamp;
    float steering;
    float throttle;
    float brake;
    int gear;
    int turn_signal;
    bool emergency;
};

/**
 * 遥测数据
 */
struct TelemetryData {
    double speed;
    double steering_angle;
    int gear;
    double latitude;
    double longitude;
    double heading;
    int battery_level;
    int64_t timestamp;
};

/**
 * 视频帧
 */
struct VideoFrame {
    std::string camera_id;
    std::vector<uint8_t> data;
    int width;
    int height;
    int64_t timestamp;
};

/**
 * 操作端状态
 */
struct OperatorStatus {
    bool connected;
    std::string vehicle_id;
    bool wheel_connected;
    int current_gear;
    int turn_signal;
};

// 回调类型
using ControlCallback = std::function<void(const ControlCommand&)>;
using TelemetryCallback = std::function<void(const TelemetryData&)>;
using VideoCallback = std::function<void(const VideoFrame&)>;

/**
 * 操作端客户端
 * 整合方向盘输入和远程通信
 */
class OperatorClient {
public:
    explicit OperatorClient(const OperatorConfig& config);
    ~OperatorClient();

    /**
     * 初始化客户端
     */
    bool initialize();

    /**
     * 启动输入处理
     */
    void start();

    /**
     * 停止客户端
     */
    void stop();

    /**
     * 连接到车辆
     */
    bool connect(const std::string& vehicle_id);

    /**
     * 断开连接
     */
    void disconnect();

    /**
     * 设置控制指令回调
     */
    void setControlCallback(ControlCallback callback);

    /**
     * 设置遥测数据回调
     */
    void setTelemetryCallback(TelemetryCallback callback);

    /**
     * 设置视频帧回调
     */
    void setVideoCallback(VideoCallback callback);

    /**
     * 触发紧急停车
     */
    void triggerEmergencyStop();

    /**
     * 获取状态
     */
    OperatorStatus getStatus() const;

private:
    void inputLoop();
    void processWheelInput();
    void sendControlCommand(const ControlCommand& cmd);

    float applyDeadzone(float value, float deadzone);
    float applyCurve(float value, const std::string& curve);

    void shiftGearUp();
    void shiftGearDown();
    std::string getGearName(int gear);

private:
    OperatorConfig config_;
    std::unique_ptr<WheelController> wheel_controller_;

    std::thread input_thread_;
    std::atomic<bool> running_;

    std::string current_vehicle_id_;
    bool webrtc_connected_ = false;

    int current_gear_;
    int turn_signal_;
    WheelInput last_input_;

    ControlCallback control_callback_;
    TelemetryCallback telemetry_callback_;
    VideoCallback video_callback_;
};

}  // namespace operator_client
}  // namespace fsm
