#pragma once

#include "fsm/operator/wheel_controller.hpp"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace fsm {
namespace operator_client {

struct OperatorConfig {
  std::string signaling_url;
  int   input_poll_interval_ms  = 10;
  float steering_deadzone       = 0.02f;
  std::string steering_curve    = "linear";
  float force_feedback_strength = 0.5f;
  int reconnect_interval_ms     = 5000;
};

struct ControlCommand {
  int64_t  timestamp_ns = 0;
  float    steering     = 0.0f;   // -1.0 to 1.0
  float    throttle     = 0.0f;   //  0.0 to 1.0
  float    brake        = 0.0f;   //  0.0 to 1.0
  int      gear         = 3;      // 0=P,1=R,2=N,3=D
  int      turn_signal  = 0;
  bool     emergency    = false;
  uint64_t sequence     = 0;
};

struct TelemetryData {
  double  speed_mps    = 0.0;
  double  steering_rad = 0.0;
  int     gear         = 0;
  double  latitude     = 0.0;
  double  longitude    = 0.0;
  double  heading_rad  = 0.0;
  int     battery_pct  = 0;
  float   latency_ms   = 0.0f;
  int64_t timestamp_ns = 0;
};

struct VideoFrame {
  std::string          camera_id;
  std::vector<uint8_t> data;       // JPEG bytes
  int64_t              timestamp_ns = 0;
  uint64_t             frame_number = 0;
};

struct OperatorStatus {
  bool        connected      = false;
  bool        vehicle_ready  = false;
  std::string vehicle_id;
  bool        wheel_connected = false;
  int         current_gear   = 3;
  int         turn_signal    = 0;
  float       latency_ms     = 0.0f;
};

using ControlCallback   = std::function<void(const ControlCommand&)>;
using TelemetryCallback = std::function<void(const TelemetryData&)>;
using VideoCallback     = std::function<void(const VideoFrame&)>;

class OperatorClient {
 public:
  explicit OperatorClient(const OperatorConfig& config);
  ~OperatorClient();

  OperatorClient(const OperatorClient&) = delete;
  OperatorClient& operator=(const OperatorClient&) = delete;

  [[nodiscard]] bool Initialize();
  void Start();
  void Stop();

  // Connect to a specific vehicle via the signaling server.
  [[nodiscard]] bool Connect(const std::string& vehicle_id);
  void Disconnect();

  void set_control_callback(ControlCallback cb);
  void set_telemetry_callback(TelemetryCallback cb);
  void set_video_callback(VideoCallback cb);

  void TriggerEmergencyStop();
  OperatorStatus GetStatus() const;

 private:
  void InputLoop();
  void ProcessWheelInput();
  void SendControlCommand(const ControlCommand& cmd);

  static float ApplyDeadzone(float value, float deadzone);
  static float ApplyCurve(float value, const std::string& curve);

  void ShiftGearUp();
  void ShiftGearDown();
  static const char* GearName(int gear);

  OperatorConfig                   config_;
  std::unique_ptr<WheelController> wheel_controller_;

  std::thread       input_thread_;
  std::atomic<bool> running_{false};

  // WebSocket implementation (Pimpl)
  class WsImpl;
  std::unique_ptr<WsImpl> ws_;

  std::string current_vehicle_id_;
  int         current_gear_  = 3;
  int         turn_signal_   = 0;
  WheelInput  last_input_;
  uint64_t    cmd_seq_       = 0;

  ControlCallback   control_cb_;
  TelemetryCallback telemetry_cb_;
  VideoCallback     video_cb_;
};

}  // namespace operator_client
}  // namespace fsm
