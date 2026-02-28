#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <linux/joystick.h>
#include "fsm/logger.hpp"

namespace fsm {
namespace operator_client {

struct WheelInput {
  float steering = 0.0f;            // -1.0 (left) to 1.0 (right)
  float throttle = 0.0f;            // 0.0 to 1.0
  float brake = 0.0f;               // 0.0 to 1.0
  float clutch = 0.0f;              // 0.0 to 1.0
  bool btn_emergency = false;
  bool btn_horn = false;
  bool btn_left_signal = false;
  bool btn_right_signal = false;
  bool btn_gear_up = false;
  bool btn_gear_down = false;
  int64_t timestamp_ns = 0;
};

struct WheelConfig {
  std::string device_path = "/dev/input/js0";

  // Axis indices (Logitech G29/G920 defaults)
  int axis_steering = 0;
  int axis_throttle = 2;
  int axis_brake = 3;
  int axis_clutch = 1;

  // Button indices
  int btn_emergency = 0;      // X
  int btn_horn = 1;           // □
  int btn_left_signal = 4;    // L1
  int btn_right_signal = 5;   // R1
  int btn_gear_up = 6;        // right paddle
  int btn_gear_down = 7;      // left paddle

  float steering_range_deg = 900.0f;
  float steering_deadzone = 0.02f;
  float throttle_deadzone = 0.05f;
  float brake_deadzone = 0.05f;

  bool force_feedback_enabled = true;
  float force_feedback_strength = 0.5f;  // 0.0 to 1.0
};

using WheelInputCallback = std::function<void(const WheelInput&)>;

class WheelController {
 public:
  explicit WheelController(const WheelConfig& config = WheelConfig());
  ~WheelController();

  WheelController(const WheelController&) = delete;
  WheelController& operator=(const WheelController&) = delete;

  [[nodiscard]] bool Open();
  void Close();

  bool is_connected() const { return connected_.load(); }
  WheelInput GetInput() const;
  void set_input_callback(WheelInputCallback cb);

  void SetForceFeedback(float strength, int duration_ms = 100);
  void SetConstantForce(float force);   // -1.0 to 1.0
  void StopForceFeedback();
  void SetCenterSpring(float strength); // 0.0 to 1.0

  std::string device_name() const { return device_name_; }
  int axis_count() const { return axis_count_; }
  int button_count() const { return button_count_; }

 private:
  void ReadLoop();
  void ProcessEvent(const js_event& event);
  float ApplyDeadzone(float value, float deadzone) const;
  float NormalizeAxis(int16_t raw) const;

  WheelConfig config_;

  int fd_ = -1;
  std::string device_name_;
  int axis_count_ = 0;
  int button_count_ = 0;

  std::atomic<bool> connected_{false};
  std::atomic<bool> running_{false};

  mutable std::mutex input_mutex_;
  WheelInput current_input_;
  std::vector<int16_t> axis_values_;
  std::vector<bool> button_values_;

  std::unique_ptr<std::thread> read_thread_;
  WheelInputCallback input_cb_;
};

// Returns paths of detected joystick devices (e.g. /dev/input/js0).
std::vector<std::string> DetectWheelDevices();

// Returns a human-readable model name for the device at the given path.
std::string DetectWheelModel(const std::string& device_path);

}  // namespace operator_client
}  // namespace fsm
