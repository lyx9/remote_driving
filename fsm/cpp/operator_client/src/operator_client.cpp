#include "fsm/operator/operator_client.hpp"
#include "fsm/logger.hpp"

#include <boost/asio.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/ssl.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <mutex>
#include <thread>
#include <variant>
#include <vector>

namespace beast     = boost::beast;
namespace websocket = beast::websocket;
namespace net       = boost::asio;
namespace ssl       = boost::asio::ssl;
using tcp           = boost::asio::ip::tcp;

namespace fsm {
namespace operator_client {

// ============================================================================
// Base64 decoder (for JPEG video frames from vehicle)
// ============================================================================

static std::vector<uint8_t> Base64Decode(const std::string& s) {
  static const int kD[256] = {
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,62,-1,-1,-1,63,
    52,53,54,55,56,57,58,59,60,61,-1,-1,-1,-1,-1,-1,
    -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,
    15,16,17,18,19,20,21,22,23,24,25,-1,-1,-1,-1,-1,
    -1,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,
    41,42,43,44,45,46,47,48,49,50,51,-1,-1,-1,-1,-1,
  };
  std::vector<uint8_t> out;
  out.reserve(s.size() * 3 / 4);
  uint32_t acc = 0; int bits = 0;
  for (unsigned char c : s) {
    if (c == '=') break;
    const int v = (c < 128) ? kD[c] : -1;
    if (v < 0) continue;
    acc = (acc << 6) | static_cast<uint32_t>(v);
    bits += 6;
    if (bits >= 8) { bits -= 8; out.push_back(static_cast<uint8_t>(acc >> bits)); }
  }
  return out;
}

// ============================================================================
// URL parser (same as in webrtc_client.cpp)
// ============================================================================

struct ParsedUrl {
  bool        use_ssl = false;
  std::string host;
  std::string port;
  std::string path  = "/";
  bool        valid = false;
};

static ParsedUrl ParseWsUrl(const std::string& url) {
  ParsedUrl r;
  std::string rest;
  if (url.size() > 6 && url.substr(0, 6) == "wss://") {
    r.use_ssl = true;  rest = url.substr(6);
  } else if (url.size() > 5 && url.substr(0, 5) == "ws://") {
    r.use_ssl = false; rest = url.substr(5);
  } else {
    return r;
  }
  const auto slash = rest.find('/');
  const std::string hp = (slash != std::string::npos) ? rest.substr(0, slash) : rest;
  r.path = (slash != std::string::npos) ? rest.substr(slash) : "/";
  const auto colon = hp.find(':');
  if (colon != std::string::npos) {
    r.host = hp.substr(0, colon);
    r.port = hp.substr(colon + 1);
  } else {
    r.host = hp;
    r.port = r.use_ssl ? "443" : "80";
  }
  r.valid = !r.host.empty();
  return r;
}

// ============================================================================
// OperatorClient::WsImpl  —  WebSocket relay client
// ============================================================================

class OperatorClient::WsImpl {
 public:
  using PlainWs = websocket::stream<beast::tcp_stream>;
  using SslWs   = websocket::stream<beast::ssl_stream<beast::tcp_stream>>;
  using WsVar   = std::variant<std::monostate, PlainWs, SslWs>;

  explicit WsImpl(OperatorClient& parent)
      : parent_(parent), ssl_ctx_(ssl::context::tlsv12_client) {
    ssl_ctx_.set_verify_mode(ssl::verify_none);
  }

  // State
  std::atomic<bool>  connected{false};
  std::atomic<bool>  vehicle_ready{false};
  std::atomic<float> latency_ms{0.0f};
  std::string        client_id;
  std::string        target_vehicle_id;

  // IO
  net::io_context ioc{1};
  ssl::context    ssl_ctx_;
  WsVar           ws_var{std::monostate{}};
  beast::flat_buffer       buffer_;
  std::deque<std::string>  write_queue_;
  bool                     writing_ = false;

  std::thread       io_thread_;
  std::thread       hb_thread_;
  std::atomic<bool> stop_flag{false};

  ParsedUrl   parsed_url_;
  std::shared_ptr<net::steady_timer> reconnect_timer_;

  std::atomic<uint64_t> ping_seq{0};
  uint64_t              cmd_seq = 0;

  // ---- helpers ----

  void QueueSend(std::string msg) {
    write_queue_.push_back(std::move(msg));
    if (!writing_) DoWrite();
  }

  void PostSend(std::string msg) {
    net::post(ioc, [this, m = std::move(msg)]() mutable { QueueSend(std::move(m)); });
  }

  void DoWrite() {
    if (write_queue_.empty()) { writing_ = false; return; }
    writing_ = true;
    std::visit([&](auto& ws) {
      using T = std::decay_t<decltype(ws)>;
      if constexpr (!std::is_same_v<T, std::monostate>) {
        ws.async_write(net::buffer(write_queue_.front()),
            [this](beast::error_code ec, std::size_t) {
              if (ec) { write_queue_.clear(); writing_ = false; HandleDisconnect(); return; }
              write_queue_.pop_front();
              DoWrite();
            });
      }
    }, ws_var);
  }

  void DoRead() {
    std::visit([&](auto& ws) {
      using T = std::decay_t<decltype(ws)>;
      if constexpr (!std::is_same_v<T, std::monostate>) {
        ws.async_read(buffer_,
            [this](beast::error_code ec, std::size_t) {
              if (ec) {
                if (!stop_flag.load()) {
                  FSM_LOG_WARN("Operator WS read closed: {}", ec.message());
                  HandleDisconnect();
                }
                return;
              }
              OnMessage(beast::buffers_to_string(buffer_.data()));
              buffer_.consume(buffer_.size());
              DoRead();
            });
      }
    }, ws_var);
  }

  void OnMessage(const std::string& raw) {
    try {
      auto j = nlohmann::json::parse(raw);
      const std::string type = j.value("type", "");

      if (type == "registered") {
        client_id = j.value("client_id", "");
        FSM_LOG_INFO("Operator registered: client_id={}", client_id);
        connected.store(true);
        // Immediately request connection to target vehicle
        if (!target_vehicle_id.empty()) SendConnect();

      } else if (type == "vehicle_relay_ready" || type == "relay_ready") {
        vehicle_ready.store(true);
        FSM_LOG_INFO("Vehicle relay ready: {}", target_vehicle_id);

      } else if (type == "vehicle_online") {
        FSM_LOG_INFO("Vehicle online: {}", j.value("vehicle_id", ""));

      } else if (type == "telemetry") {
        if (parent_.telemetry_cb_) {
          TelemetryData td;
          const auto& d = j.value("data", nlohmann::json::object());
          td.speed_mps   = d.value("speed_mps",   0.0);
          td.steering_rad= d.value("steering_rad", 0.0);
          td.gear        = d.value("gear",         3);
          td.latitude    = d.value("latitude",     0.0);
          td.longitude   = d.value("longitude",    0.0);
          td.heading_rad = d.value("heading_rad",  0.0);
          td.battery_pct = d.value("battery_pct",  100);
          td.latency_ms  = latency_ms.load();
          td.timestamp_ns= d.value("timestamp_ns", int64_t{0});
          parent_.telemetry_cb_(td);
        }

      } else if (type == "video_frame") {
        if (parent_.video_cb_) {
          VideoFrame vf;
          vf.camera_id    = j.value("camera_id",   "");
          vf.timestamp_ns = j.value("timestamp_ns", int64_t{0});
          vf.frame_number = j.value("frame_number", uint64_t{0});
          const std::string b64 = j.value("data", "");
          vf.data = Base64Decode(b64);
          parent_.video_cb_(vf);
        }

      } else if (type == "pong") {
        const int64_t orig_t = j.value("timestamp", int64_t{0});
        const int64_t now_ns = static_cast<int64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        const float rtt = static_cast<float>(now_ns - orig_t) / 1.0e6f;
        latency_ms.store(rtt);
        FSM_LOG_DEBUG("Operator RTT: {:.1f}ms", rtt);

      } else if (type == "peer_disconnected") {
        vehicle_ready.store(false);
        FSM_LOG_INFO("Vehicle disconnected");

      } else if (type == "error") {
        FSM_LOG_ERROR("Signaling error: {}", j.value("message", ""));
      }
    } catch (const std::exception& e) {
      FSM_LOG_ERROR("Operator WS parse error: {}", e.what());
    }
  }

  void SendConnect() {
    nlohmann::json j;
    j["type"]       = "connect";
    j["vehicle_id"] = target_vehicle_id;
    QueueSend(j.dump());
    FSM_LOG_INFO("Requesting connection to vehicle: {}", target_vehicle_id);
  }

  void SendRegister() {
    nlohmann::json j;
    j["type"]        = "register";
    j["client_type"] = "operator";
    QueueSend(j.dump());
    FSM_LOG_INFO("Operator registering with signaling server");
  }

  void HandleDisconnect() {
    if (stop_flag.load()) return;
    connected.store(false);
    vehicle_ready.store(false);
    write_queue_.clear();
    writing_ = false;
    buffer_.consume(buffer_.size());
    ScheduleReconnect();
  }

  void ScheduleReconnect() {
    reconnect_timer_ = std::make_shared<net::steady_timer>(
        ioc, std::chrono::milliseconds(parent_.config_.reconnect_interval_ms));
    reconnect_timer_->async_wait([this](beast::error_code ec) {
      if (ec || stop_flag.load()) return;
      FSM_LOG_INFO("Operator reconnecting...");
      ws_var = std::monostate{};
      auto resolver = std::make_shared<tcp::resolver>(ioc);
      resolver->async_resolve(parsed_url_.host, parsed_url_.port,
          [this, resolver](beast::error_code ec2, tcp::resolver::results_type res) {
            if (ec2) { ScheduleReconnect(); return; }
            StartConnect(res);
          });
    });
  }

  void StartConnect(tcp::resolver::results_type results) {
    if (parsed_url_.use_ssl) {
      ws_var = SslWs{net::make_strand(ioc), ssl_ctx_};
      auto& ws = std::get<SslWs>(ws_var);
      beast::get_lowest_layer(ws).expires_after(std::chrono::seconds(30));
      beast::get_lowest_layer(ws).async_connect(results,
          [this](beast::error_code ec, tcp::resolver::results_type::endpoint_type) {
            if (ec) { ScheduleReconnect(); return; }
            auto& ws = std::get<SslWs>(ws_var);
            beast::get_lowest_layer(ws).expires_after(std::chrono::seconds(30));
            ws.next_layer().async_handshake(ssl::stream_base::client,
                [this](beast::error_code ec2) {
                  if (ec2) { ScheduleReconnect(); return; }
                  FinishWsSsl();
                });
          });
    } else {
      ws_var = PlainWs{net::make_strand(ioc)};
      auto& ws = std::get<PlainWs>(ws_var);
      beast::get_lowest_layer(ws).expires_after(std::chrono::seconds(30));
      beast::get_lowest_layer(ws).async_connect(results,
          [this](beast::error_code ec, tcp::resolver::results_type::endpoint_type) {
            if (ec) { ScheduleReconnect(); return; }
            FinishWsPlain();
          });
    }
  }

  void FinishWsPlain() {
    auto& ws = std::get<PlainWs>(ws_var);
    beast::get_lowest_layer(ws).expires_never();
    ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));
    ws.set_option(websocket::stream_base::decorator([](websocket::request_type& req) {
      req.set(beast::http::field::user_agent, "FSM-Pilot-Operator/1.0");
    }));
    ws.async_handshake(parsed_url_.host, parsed_url_.path,
        [this](beast::error_code ec) {
          if (ec) { ScheduleReconnect(); return; }
          FSM_LOG_INFO("Operator WS connected (plain)");
          writing_ = false;
          SendRegister();
          DoRead();
        });
  }

  void FinishWsSsl() {
    auto& ws = std::get<SslWs>(ws_var);
    beast::get_lowest_layer(ws).expires_never();
    ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));
    ws.set_option(websocket::stream_base::decorator([](websocket::request_type& req) {
      req.set(beast::http::field::user_agent, "FSM-Pilot-Operator/1.0");
    }));
    ws.async_handshake(parsed_url_.host, parsed_url_.path,
        [this](beast::error_code ec) {
          if (ec) { ScheduleReconnect(); return; }
          FSM_LOG_INFO("Operator WS connected (TLS)");
          writing_ = false;
          SendRegister();
          DoRead();
        });
  }

  void CloseNow() {
    stop_flag = true;
    if (reconnect_timer_) reconnect_timer_->cancel();
    std::visit([](auto& ws) {
      using T = std::decay_t<decltype(ws)>;
      if constexpr (!std::is_same_v<T, std::monostate>) {
        beast::error_code ec;
        ws.close(websocket::close_code::normal, ec);
      }
    }, ws_var);
    ioc.stop();
  }

 private:
  OperatorClient& parent_;
};

// ============================================================================
// OperatorClient public interface
// ============================================================================

OperatorClient::OperatorClient(const OperatorConfig& config)
    : config_(config), ws_(std::make_unique<WsImpl>(*this)) {}

OperatorClient::~OperatorClient() {
  Stop();
}

bool OperatorClient::Initialize() {
  wheel_controller_ = std::make_unique<WheelController>();
  if (!wheel_controller_->Open()) {
    FSM_LOG_WARN("No steering wheel detected — keyboard-only mode");
    wheel_controller_.reset();
  } else {
    FSM_LOG_INFO("Steering wheel: {}", wheel_controller_->device_name());
    wheel_controller_->SetForceFeedback(config_.force_feedback_strength, 0);
  }
  FSM_LOG_INFO("OperatorClient initialized");
  return true;
}

void OperatorClient::Start() {
  if (running_) return;
  running_ = true;
  input_thread_ = std::thread([this]() { InputLoop(); });
  FSM_LOG_INFO("OperatorClient started");
}

void OperatorClient::Stop() {
  running_ = false;
  if (input_thread_.joinable()) input_thread_.join();
  Disconnect();
  FSM_LOG_INFO("OperatorClient stopped");
}

bool OperatorClient::Connect(const std::string& vehicle_id) {
  FSM_LOG_INFO("Connecting to vehicle: {}", vehicle_id);
  current_vehicle_id_     = vehicle_id;
  ws_->target_vehicle_id  = vehicle_id;

  ws_->parsed_url_ = ParseWsUrl(config_.signaling_url);
  if (!ws_->parsed_url_.valid) {
    FSM_LOG_ERROR("Invalid signaling URL: {}", config_.signaling_url);
    return false;
  }

  ws_->stop_flag = false;
  ws_->ioc.restart();

  net::post(ws_->ioc, [this]() {
    auto resolver = std::make_shared<tcp::resolver>(ws_->ioc);
    resolver->async_resolve(ws_->parsed_url_.host, ws_->parsed_url_.port,
        [this, resolver](beast::error_code ec, tcp::resolver::results_type results) {
          if (ec) { ws_->ScheduleReconnect(); return; }
          ws_->StartConnect(results);
        });
  });

  ws_->io_thread_ = std::thread([this]() { ws_->ioc.run(); });

  // Heartbeat thread — send ping every 1 second to measure RTT
  ws_->hb_thread_ = std::thread([this]() {
    while (!ws_->stop_flag.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      if (ws_->stop_flag.load() || !ws_->connected.load()) continue;
      nlohmann::json ping;
      ping["type"]      = "ping";
      ping["sequence"]  = ws_->ping_seq.fetch_add(1);
      ping["timestamp"] = static_cast<int64_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::steady_clock::now().time_since_epoch()).count());
      ws_->PostSend(ping.dump());
    }
  });

  return true;
}

void OperatorClient::Disconnect() {
  if (!current_vehicle_id_.empty()) {
    ws_->stop_flag = true;
    if (ws_->hb_thread_.joinable()) ws_->hb_thread_.join();
    net::post(ws_->ioc, [this]() { ws_->CloseNow(); });
    if (ws_->io_thread_.joinable()) ws_->io_thread_.join();
    ws_->connected.store(false);
    ws_->vehicle_ready.store(false);
    current_vehicle_id_.clear();
    FSM_LOG_INFO("Operator disconnected");
  }
}

void OperatorClient::set_control_callback(ControlCallback cb)    { control_cb_   = std::move(cb); }
void OperatorClient::set_telemetry_callback(TelemetryCallback cb){ telemetry_cb_ = std::move(cb); }
void OperatorClient::set_video_callback(VideoCallback cb)        { video_cb_     = std::move(cb); }

void OperatorClient::TriggerEmergencyStop() {
  ControlCommand cmd;
  cmd.timestamp_ns = static_cast<int64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
  cmd.brake     = 1.0f;
  cmd.emergency = true;
  cmd.sequence  = ++cmd_seq_;
  SendControlCommand(cmd);
  FSM_LOG_WARN("EMERGENCY STOP triggered");
  if (wheel_controller_) wheel_controller_->SetForceFeedback(1.0f, 500);
}

OperatorStatus OperatorClient::GetStatus() const {
  OperatorStatus s;
  s.connected      = ws_->connected.load();
  s.vehicle_ready  = ws_->vehicle_ready.load();
  s.vehicle_id     = current_vehicle_id_;
  s.wheel_connected= wheel_controller_ && wheel_controller_->is_connected();
  s.current_gear   = current_gear_;
  s.turn_signal    = turn_signal_;
  s.latency_ms     = ws_->latency_ms.load();
  return s;
}

void OperatorClient::InputLoop() {
  auto last_time = std::chrono::steady_clock::now();
  while (running_) {
    const auto now = std::chrono::steady_clock::now();
    const auto dt  = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_time).count();
    if (dt < config_.input_poll_interval_ms) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(config_.input_poll_interval_ms - dt));
      continue;
    }
    last_time = now;
    if (wheel_controller_ && wheel_controller_->is_connected()) {
      ProcessWheelInput();
    }
  }
}

void OperatorClient::ProcessWheelInput() {
  const auto input = wheel_controller_->GetInput();
  ControlCommand cmd;
  cmd.timestamp_ns = static_cast<int64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
  cmd.steering    = ApplyCurve(ApplyDeadzone(input.steering, config_.steering_deadzone),
                                config_.steering_curve);
  cmd.throttle    = input.throttle;
  cmd.brake       = input.brake;
  cmd.gear        = current_gear_;
  cmd.turn_signal = turn_signal_;
  cmd.sequence    = ++cmd_seq_;

  if (input.btn_emergency) { cmd.emergency = true; FSM_LOG_WARN("EMERGENCY from wheel"); }
  if (input.btn_gear_up   && !last_input_.btn_gear_up)   ShiftGearUp();
  if (input.btn_gear_down && !last_input_.btn_gear_down) ShiftGearDown();
  if (input.btn_left_signal  && !last_input_.btn_left_signal)
    turn_signal_ = (turn_signal_ == 1) ? 0 : 1;
  if (input.btn_right_signal && !last_input_.btn_right_signal)
    turn_signal_ = (turn_signal_ == 2) ? 0 : 2;

  last_input_ = input;
  SendControlCommand(cmd);
  if (control_cb_) control_cb_(cmd);
}

void OperatorClient::SendControlCommand(const ControlCommand& cmd) {
  if (!ws_->connected.load() || current_vehicle_id_.empty()) return;
  nlohmann::json j;
  j["type"]       = "control";
  j["vehicle_id"] = current_vehicle_id_;
  nlohmann::json d;
  d["steering"]     = cmd.steering;
  d["throttle"]     = cmd.throttle;
  d["brake"]        = cmd.brake;
  d["gear"]         = cmd.gear;
  d["turn_signal"]  = cmd.turn_signal;
  d["emergency"]    = cmd.emergency;
  d["timestamp_ns"] = cmd.timestamp_ns;
  d["sequence"]     = cmd.sequence;
  j["data"] = d;
  ws_->PostSend(j.dump());
}

float OperatorClient::ApplyDeadzone(float value, float deadzone) {
  if (std::abs(value) < deadzone) return 0.0f;
  const float sign = (value > 0) ? 1.0f : -1.0f;
  return sign * (std::abs(value) - deadzone) / (1.0f - deadzone);
}

float OperatorClient::ApplyCurve(float value, const std::string& curve) {
  if (curve == "exponential") {
    return (value > 0 ? 1.0f : -1.0f) * std::pow(std::abs(value), 2.0f);
  }
  return value;
}

void OperatorClient::ShiftGearUp() {
  if (current_gear_ < 3) { ++current_gear_; FSM_LOG_INFO("Gear: {}", GearName(current_gear_)); }
}
void OperatorClient::ShiftGearDown() {
  if (current_gear_ > 0) { --current_gear_; FSM_LOG_INFO("Gear: {}", GearName(current_gear_)); }
}
const char* OperatorClient::GearName(int gear) {
  switch (gear) { case 0: return "P"; case 1: return "R"; case 2: return "N"; case 3: return "D"; }
  return "?";
}

}  // namespace operator_client
}  // namespace fsm
