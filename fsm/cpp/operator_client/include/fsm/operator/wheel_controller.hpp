#pragma once

#include <string>
#include <functional>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <linux/joystick.h>

#include "fsm/logger.hpp"

namespace fsm {
namespace operator_client {

/**
 * @brief 方向盘输入数据
 */
struct WheelInput {
    // 轴输入 (归一化到 -1.0 ~ 1.0 或 0.0 ~ 1.0)
    float steering = 0.0f;        // 方向盘 (-1.0 左, 1.0 右)
    float throttle = 0.0f;        // 油门 (0.0 ~ 1.0)
    float brake = 0.0f;           // 刹车 (0.0 ~ 1.0)
    float clutch = 0.0f;          // 离合器 (0.0 ~ 1.0)

    // 按钮状态
    bool button_emergency = false;   // 紧急停车按钮
    bool button_horn = false;        // 喇叭
    bool button_left_signal = false; // 左转向灯
    bool button_right_signal = false;// 右转向灯
    bool button_gear_up = false;     // 升档
    bool button_gear_down = false;   // 降档

    // 时间戳
    int64_t timestamp_ns = 0;
};

/**
 * @brief 方向盘配置
 */
struct WheelConfig {
    // 设备路径
    std::string device_path = "/dev/input/js0";

    // 轴映射 (Logitech G29/G920)
    int axis_steering = 0;
    int axis_throttle = 2;
    int axis_brake = 3;
    int axis_clutch = 1;

    // 按钮映射
    int button_emergency = 0;      // X 按钮
    int button_horn = 1;           // □ 按钮
    int button_left_signal = 4;    // L1
    int button_right_signal = 5;   // R1
    int button_gear_up = 6;        // 右拨片
    int button_gear_down = 7;      // 左拨片

    // 转换参数
    float steering_range = 900.0f;  // 方向盘总转角范围 (度)
    float steering_deadzone = 0.02f;
    float throttle_deadzone = 0.05f;
    float brake_deadzone = 0.05f;

    // 力反馈
    bool force_feedback_enabled = true;
    float force_feedback_strength = 0.5f;  // 0.0 ~ 1.0
};

/**
 * @brief 方向盘输入回调
 */
using WheelInputCallback = std::function<void(const WheelInput&)>;

/**
 * @brief 罗技方向盘控制器
 *
 * 支持 Logitech G29, G920, G27 等方向盘
 */
class WheelController {
public:
    /**
     * @brief 构造函数
     * @param config 方向盘配置
     */
    explicit WheelController(const WheelConfig& config = WheelConfig());

    ~WheelController();

    /**
     * @brief 打开方向盘设备
     * @return 是否成功打开
     */
    bool open();

    /**
     * @brief 关闭方向盘设备
     */
    void close();

    /**
     * @brief 检查方向盘是否连接
     */
    bool isConnected() const { return connected_; }

    /**
     * @brief 获取当前输入状态
     */
    WheelInput getInput() const;

    /**
     * @brief 设置输入回调
     */
    void setInputCallback(WheelInputCallback callback);

    /**
     * @brief 设置力反馈
     * @param strength 力度 (0.0 ~ 1.0)
     * @param duration_ms 持续时间 (毫秒)
     */
    void setForceFeedback(float strength, int duration_ms = 100);

    /**
     * @brief 设置恒定力 (模拟路面反馈)
     * @param force 力度 (-1.0 ~ 1.0, 负为左, 正为右)
     */
    void setConstantForce(float force);

    /**
     * @brief 停止所有力反馈
     */
    void stopForceFeedback();

    /**
     * @brief 设置中心弹簧效果
     * @param strength 弹簧强度 (0.0 ~ 1.0)
     */
    void setCenterSpring(float strength);

    /**
     * @brief 获取设备名称
     */
    std::string getDeviceName() const { return device_name_; }

    /**
     * @brief 获取轴数量
     */
    int getAxisCount() const { return axis_count_; }

    /**
     * @brief 获取按钮数量
     */
    int getButtonCount() const { return button_count_; }

private:
    /**
     * @brief 读取线程
     */
    void readThread();

    /**
     * @brief 处理摇杆事件
     */
    void processEvent(const js_event& event);

    /**
     * @brief 应用死区
     */
    float applyDeadzone(float value, float deadzone) const;

    /**
     * @brief 归一化轴值
     */
    float normalizeAxis(int16_t value) const;

    WheelConfig config_;

    // 设备信息
    int fd_ = -1;
    std::string device_name_;
    int axis_count_ = 0;
    int button_count_ = 0;

    // 状态
    std::atomic<bool> connected_{false};
    std::atomic<bool> running_{false};

    // 输入数据
    mutable std::mutex input_mutex_;
    WheelInput current_input_;
    std::vector<int16_t> axis_values_;
    std::vector<bool> button_values_;

    // 线程
    std::unique_ptr<std::thread> read_thread_;

    // 回调
    WheelInputCallback input_callback_;
};

/**
 * @brief 自动检测方向盘设备
 * @return 设备路径列表
 */
std::vector<std::string> detectWheelDevices();

/**
 * @brief 检测方向盘类型
 * @param device_path 设备路径
 * @return 方向盘型号名称
 */
std::string detectWheelModel(const std::string& device_path);

}  // namespace operator_client
}  // namespace fsm
