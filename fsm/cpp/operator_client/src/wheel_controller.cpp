#include "fsm/operator/wheel_controller.hpp"
#include "fsm/utils.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <linux/input.h>
#include <cstring>
#include <algorithm>

namespace fsm {
namespace operator_client {

WheelController::WheelController(const WheelConfig& config)
    : config_(config) {}

WheelController::~WheelController() {
  Close();
}

bool WheelController::Open() {
  if (connected_) {
    FSM_LOG_WARN("Wheel already connected");
    return true;
  }

  fd_ = ::open(config_.device_path.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd_ < 0) {
    FSM_LOG_ERROR("Failed to open joystick: {}", config_.device_path);
    return false;
  }

  char name[128] = "Unknown";
  if (ioctl(fd_, JSIOCGNAME(sizeof(name)), name) >= 0) {
    device_name_ = name;
  }

  uint8_t axes = 0, buttons = 0;
  ioctl(fd_, JSIOCGAXES,   &axes);
  ioctl(fd_, JSIOCGBUTTONS, &buttons);
  axis_count_   = axes;
  button_count_ = buttons;

  axis_values_.resize(axis_count_, 0);
  button_values_.resize(button_count_, false);

  FSM_LOG_INFO("Wheel connected: {} ({} axes, {} buttons)",
               device_name_, axis_count_, button_count_);

  connected_   = true;
  running_     = true;
  read_thread_ = std::make_unique<std::thread>(&WheelController::ReadLoop, this);
  return true;
}

void WheelController::Close() {
  if (!connected_) return;
  running_ = false;

  if (read_thread_ && read_thread_->joinable()) read_thread_->join();

  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  connected_ = false;
  FSM_LOG_INFO("Wheel disconnected");
}

WheelInput WheelController::GetInput() const {
  std::lock_guard<std::mutex> lock(input_mutex_);
  return current_input_;
}

void WheelController::set_input_callback(WheelInputCallback cb) {
  input_cb_ = std::move(cb);
}

void WheelController::SetForceFeedback(float strength, int duration_ms) {
  if (!config_.force_feedback_enabled || fd_ < 0) return;
  FSM_LOG_DEBUG("Force feedback: strength={:.2f} duration={}ms", strength, duration_ms);
}

void WheelController::SetConstantForce(float force) {
  if (!config_.force_feedback_enabled || fd_ < 0) return;
  FSM_LOG_DEBUG("Constant force: {:.2f}", force);
}

void WheelController::StopForceFeedback() {
  if (fd_ < 0) return;
  FSM_LOG_DEBUG("Force feedback stopped");
}

void WheelController::SetCenterSpring(float strength) {
  if (!config_.force_feedback_enabled || fd_ < 0) return;
  FSM_LOG_DEBUG("Center spring: {:.2f}", strength);
}

void WheelController::ReadLoop() {
  js_event event;
  while (running_) {
    const ssize_t bytes = read(fd_, &event, sizeof(event));
    if (bytes == sizeof(event)) {
      ProcessEvent(event);
    } else if (bytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } else {
        FSM_LOG_ERROR("Joystick read error: {}", strerror(errno));
        connected_ = false;
        break;
      }
    }
  }
}

void WheelController::ProcessEvent(const js_event& event) {
  const bool is_init = (event.type & JS_EVENT_INIT) != 0;
  const uint8_t type = event.type & ~JS_EVENT_INIT;

  std::lock_guard<std::mutex> lock(input_mutex_);

  if (type == JS_EVENT_AXIS && event.number < axis_values_.size()) {
    axis_values_[event.number] = event.value;

    if (event.number == config_.axis_steering) {
      current_input_.steering = ApplyDeadzone(
          NormalizeAxis(event.value), config_.steering_deadzone);
    } else if (event.number == config_.axis_throttle) {
      const float raw = (-NormalizeAxis(event.value) + 1.0f) / 2.0f;
      current_input_.throttle = ApplyDeadzone(raw, config_.throttle_deadzone);
    } else if (event.number == config_.axis_brake) {
      const float raw = (-NormalizeAxis(event.value) + 1.0f) / 2.0f;
      current_input_.brake = ApplyDeadzone(raw, config_.brake_deadzone);
    } else if (event.number == config_.axis_clutch) {
      current_input_.clutch = (-NormalizeAxis(event.value) + 1.0f) / 2.0f;
    }

  } else if (type == JS_EVENT_BUTTON && event.number < button_values_.size()) {
    const bool pressed = (event.value != 0);
    button_values_[event.number] = pressed;

    if      (event.number == config_.btn_emergency)    current_input_.btn_emergency    = pressed;
    else if (event.number == config_.btn_horn)         current_input_.btn_horn         = pressed;
    else if (event.number == config_.btn_left_signal)  current_input_.btn_left_signal  = pressed;
    else if (event.number == config_.btn_right_signal) current_input_.btn_right_signal = pressed;
    else if (event.number == config_.btn_gear_up)      current_input_.btn_gear_up      = pressed;
    else if (event.number == config_.btn_gear_down)    current_input_.btn_gear_down    = pressed;
  }

  current_input_.timestamp_ns = utils::NowNanos();

  if (!is_init && input_cb_) input_cb_(current_input_);
}

float WheelController::ApplyDeadzone(float value, float deadzone) const {
  if (std::abs(value) < deadzone) return 0.0f;
  const float sign = (value > 0) ? 1.0f : -1.0f;
  return sign * (std::abs(value) - deadzone) / (1.0f - deadzone);
}

float WheelController::NormalizeAxis(int16_t raw) const {
  return static_cast<float>(raw) / 32767.0f;
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

std::vector<std::string> DetectWheelDevices() {
  std::vector<std::string> devices;

  DIR* dir = opendir("/dev/input");
  if (!dir) return devices;

  struct dirent* entry;
  while ((entry = readdir(dir)) != nullptr) {
    const std::string name = entry->d_name;
    if (name.find("js") != 0) continue;

    const std::string path = "/dev/input/" + name;
    const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) continue;

    char dev_name[128] = "";
    ioctl(fd, JSIOCGNAME(sizeof(dev_name)), dev_name);
    ::close(fd);

    std::string lower = dev_name;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

    if (lower.find("wheel")       != std::string::npos ||
        lower.find("logitech")    != std::string::npos ||
        lower.find("thrustmaster") != std::string::npos ||
        lower.find("fanatec")     != std::string::npos ||
        lower.find("g29")         != std::string::npos ||
        lower.find("g920")        != std::string::npos ||
        lower.find("g27")         != std::string::npos) {
      devices.push_back(path);
      FSM_LOG_INFO("Detected wheel: {} ({})", path, dev_name);
    }
  }

  closedir(dir);
  return devices;
}

std::string DetectWheelModel(const std::string& device_path) {
  const int fd = ::open(device_path.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd < 0) return "Unknown";
  char name[128] = "Unknown";
  ioctl(fd, JSIOCGNAME(sizeof(name)), name);
  ::close(fd);
  return std::string(name);
}

}  // namespace operator_client
}  // namespace fsm
