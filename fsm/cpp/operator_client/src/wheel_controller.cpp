#include "fsm/operator/wheel_controller.hpp"
#include "fsm/utils.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <cstring>
#include <fstream>
#include <algorithm>

namespace fsm {
namespace operator_client {

WheelController::WheelController(const WheelConfig& config)
    : config_(config) {
}

WheelController::~WheelController() {
    close();
}

bool WheelController::open() {
    if (connected_) {
        FSM_LOG_WARN("Wheel already connected");
        return true;
    }

    // 打开设备
    fd_ = ::open(config_.device_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) {
        FSM_LOG_ERROR("Failed to open joystick device: {}", config_.device_path);
        return false;
    }

    // 获取设备信息
    char name[128] = "Unknown";
    if (ioctl(fd_, JSIOCGNAME(sizeof(name)), name) >= 0) {
        device_name_ = name;
    }

    // 获取轴和按钮数量
    uint8_t axes = 0, buttons = 0;
    ioctl(fd_, JSIOCGAXES, &axes);
    ioctl(fd_, JSIOCGBUTTONS, &buttons);
    axis_count_ = axes;
    button_count_ = buttons;

    // 初始化状态数组
    axis_values_.resize(axis_count_, 0);
    button_values_.resize(button_count_, false);

    FSM_LOG_INFO("Wheel connected: {} ({} axes, {} buttons)",
                 device_name_, axis_count_, button_count_);

    connected_ = true;
    running_ = true;

    // 启动读取线程
    read_thread_ = std::make_unique<std::thread>(&WheelController::readThread, this);

    return true;
}

void WheelController::close() {
    if (!connected_) {
        return;
    }

    running_ = false;

    if (read_thread_ && read_thread_->joinable()) {
        read_thread_->join();
    }

    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }

    connected_ = false;

    FSM_LOG_INFO("Wheel disconnected");
}

WheelInput WheelController::getInput() const {
    std::lock_guard<std::mutex> lock(input_mutex_);
    return current_input_;
}

void WheelController::setInputCallback(WheelInputCallback callback) {
    input_callback_ = std::move(callback);
}

void WheelController::readThread() {
    FSM_LOG_DEBUG("Wheel read thread started");

    js_event event;

    while (running_) {
        // 读取事件
        ssize_t bytes = read(fd_, &event, sizeof(event));

        if (bytes == sizeof(event)) {
            processEvent(event);
        } else if (bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // 非阻塞模式下没有数据，等待一下
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } else {
                FSM_LOG_ERROR("Error reading from joystick: {}", strerror(errno));
                connected_ = false;
                break;
            }
        }
    }

    FSM_LOG_DEBUG("Wheel read thread stopped");
}

void WheelController::processEvent(const js_event& event) {
    // 过滤初始化事件
    bool is_init = (event.type & JS_EVENT_INIT) != 0;
    uint8_t type = event.type & ~JS_EVENT_INIT;

    std::lock_guard<std::mutex> lock(input_mutex_);

    if (type == JS_EVENT_AXIS) {
        if (event.number < axis_values_.size()) {
            axis_values_[event.number] = event.value;

            // 更新对应的输入值
            if (event.number == config_.axis_steering) {
                float raw = normalizeAxis(event.value);
                current_input_.steering = applyDeadzone(raw, config_.steering_deadzone);
            } else if (event.number == config_.axis_throttle) {
                // 油门通常是 32767 (松开) 到 -32767 (踩下)
                float raw = (-normalizeAxis(event.value) + 1.0f) / 2.0f;
                current_input_.throttle = applyDeadzone(raw, config_.throttle_deadzone);
            } else if (event.number == config_.axis_brake) {
                float raw = (-normalizeAxis(event.value) + 1.0f) / 2.0f;
                current_input_.brake = applyDeadzone(raw, config_.brake_deadzone);
            } else if (event.number == config_.axis_clutch) {
                float raw = (-normalizeAxis(event.value) + 1.0f) / 2.0f;
                current_input_.clutch = raw;
            }
        }
    } else if (type == JS_EVENT_BUTTON) {
        if (event.number < button_values_.size()) {
            bool pressed = (event.value != 0);
            button_values_[event.number] = pressed;

            // 更新对应的按钮状态
            if (event.number == config_.button_emergency) {
                current_input_.button_emergency = pressed;
            } else if (event.number == config_.button_horn) {
                current_input_.button_horn = pressed;
            } else if (event.number == config_.button_left_signal) {
                current_input_.button_left_signal = pressed;
            } else if (event.number == config_.button_right_signal) {
                current_input_.button_right_signal = pressed;
            } else if (event.number == config_.button_gear_up) {
                current_input_.button_gear_up = pressed;
            } else if (event.number == config_.button_gear_down) {
                current_input_.button_gear_down = pressed;
            }
        }
    }

    current_input_.timestamp_ns = utils::TimeUtils::nowNanos();

    // 触发回调 (仅非初始化事件)
    if (!is_init && input_callback_) {
        input_callback_(current_input_);
    }
}

float WheelController::applyDeadzone(float value, float deadzone) const {
    if (std::abs(value) < deadzone) {
        return 0.0f;
    }

    // 重新映射死区外的值到完整范围
    float sign = (value > 0) ? 1.0f : -1.0f;
    return sign * (std::abs(value) - deadzone) / (1.0f - deadzone);
}

float WheelController::normalizeAxis(int16_t value) const {
    return static_cast<float>(value) / 32767.0f;
}

void WheelController::setForceFeedback(float strength, int duration_ms) {
    if (!config_.force_feedback_enabled || fd_ < 0) {
        return;
    }

    // 查找力反馈设备
    // 注意: js设备不支持力反馈，需要使用对应的event设备
    // 这里是简化实现，实际需要打开 /dev/input/eventX 设备

    FSM_LOG_DEBUG("Force feedback: strength={}, duration={}ms", strength, duration_ms);
}

void WheelController::setConstantForce(float force) {
    if (!config_.force_feedback_enabled || fd_ < 0) {
        return;
    }

    // 设置恒定力 (模拟路面反馈或方向阻力)
    // 实际实现需要使用 FF_CONSTANT 效果

    FSM_LOG_DEBUG("Constant force: {}", force);
}

void WheelController::stopForceFeedback() {
    if (fd_ < 0) {
        return;
    }

    // 停止所有力反馈效果
    FSM_LOG_DEBUG("Stopping all force feedback effects");
}

void WheelController::setCenterSpring(float strength) {
    if (!config_.force_feedback_enabled || fd_ < 0) {
        return;
    }

    // 设置中心回弹弹簧效果
    // 实际实现需要使用 FF_SPRING 效果

    FSM_LOG_DEBUG("Center spring: strength={}", strength);
}

std::vector<std::string> detectWheelDevices() {
    std::vector<std::string> devices;

    // 扫描 /dev/input/ 目录
    DIR* dir = opendir("/dev/input");
    if (!dir) {
        return devices;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        if (name.find("js") == 0) {
            std::string path = "/dev/input/" + name;

            // 尝试打开设备
            int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
            if (fd >= 0) {
                char device_name[128] = "";
                ioctl(fd, JSIOCGNAME(sizeof(device_name)), device_name);

                // 检查是否是方向盘设备 (通过名称判断)
                std::string name_lower = device_name;
                std::transform(name_lower.begin(), name_lower.end(),
                               name_lower.begin(), ::tolower);

                if (name_lower.find("wheel") != std::string::npos ||
                    name_lower.find("logitech") != std::string::npos ||
                    name_lower.find("thrustmaster") != std::string::npos ||
                    name_lower.find("fanatec") != std::string::npos ||
                    name_lower.find("g29") != std::string::npos ||
                    name_lower.find("g920") != std::string::npos ||
                    name_lower.find("g27") != std::string::npos) {

                    devices.push_back(path);
                    FSM_LOG_INFO("Detected wheel device: {} ({})", path, device_name);
                }

                ::close(fd);
            }
        }
    }

    closedir(dir);
    return devices;
}

std::string detectWheelModel(const std::string& device_path) {
    int fd = ::open(device_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        return "Unknown";
    }

    char name[128] = "Unknown";
    ioctl(fd, JSIOCGNAME(sizeof(name)), name);
    ::close(fd);

    return std::string(name);
}

}  // namespace operator_client
}  // namespace fsm
