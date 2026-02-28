#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"

namespace fsm {
namespace vehicle {

// Forward declaration — defined in command_executor.hpp.
struct ControlCommand;

enum class WebRtcState {
  kDisconnected,
  kConnecting,
  kConnected,
  kReconnecting,
  kFailed,
};

struct LatencyInfo {
  float   rtt_ms             = 0.0f;
  float   video_latency_ms   = 0.0f;
  float   control_latency_ms = 0.0f;
  float   jitter_ms          = 0.0f;
  float   packet_loss_rate   = 0.0f;
  int64_t timestamp_ns       = 0;
};

struct WebRtcClientConfig {
  std::string              vehicle_id;
  std::string              signaling_url;
  std::vector<std::string> stun_servers;
  std::vector<std::string> turn_servers;
  std::string              turn_username;
  std::string              turn_password;
  int reconnect_interval_ms  = 5000;
  int ice_timeout_ms         = 10000;
  int heartbeat_interval_ms  = 1000;
  std::string video_codec    = "h264";
  int video_bitrate_bps      = 4000000;
  int video_fps              = 30;

  // Physics parameters: operator sends normalised values (steering ±1,
  // throttle/brake 0–1); these convert them into physical units for
  // ControlCommand before the callback fires.
  float max_steering_angle_deg = 35.0f;  // tire angle at normalised ±1
  float max_speed_kph          = 60.0f;  // speed at throttle = 1
  float max_accel_mps2         = 2.0f;   // acceleration at throttle = 1
  float max_decel_mps2         = 5.0f;   // |deceleration| at brake = 1
};

using ConnectionStateCallback = std::function<void(WebRtcState)>;
using ControlCommandCallback  = std::function<void(const ControlCommand&)>;
using LatencyUpdateCallback   = std::function<void(const LatencyInfo&)>;

// ---------------------------------------------------------------------------
// WebRtcClient — relay-mode WebSocket client to FSM signaling server.
//
// In relay mode all vehicle↔operator data flows through the cloud
// SignalingServer via JSON/base64 messages.  A P2P WebRTC upgrade path can
// be layered on top without changing this interface.
// ---------------------------------------------------------------------------
class WebRtcClient {
 public:
  explicit WebRtcClient(const WebRtcClientConfig& config);
  ~WebRtcClient();

  WebRtcClient(const WebRtcClient&) = delete;
  WebRtcClient& operator=(const WebRtcClient&) = delete;

  // Asynchronously connect to the signaling server.  Returns immediately;
  // actual state changes are reported via ConnectionStateCallback.
  [[nodiscard]] bool Connect();
  void Disconnect();

  // Send serialised protobuf TelemetryData / SystemStatus bytes.
  void SendTelemetry(const std::vector<uint8_t>& data);
  void SendSystemStatus(const std::vector<uint8_t>& data);

  // Encode and relay a camera frame as JPEG-over-WebSocket.
  void SendVideoFrame(const std::string& camera_id,
                      const cv::Mat& frame,
                      int64_t timestamp_ns);

  // Send a latency-ping to the signaling server.
  void SendHeartbeat();

  WebRtcState state() const;
  LatencyInfo latency_info() const;
  bool        is_connected() const;

  void set_connection_state_callback(ConnectionStateCallback cb);
  void set_control_command_callback(ControlCommandCallback cb);
  void set_latency_update_callback(LatencyUpdateCallback cb);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace vehicle
}  // namespace fsm
