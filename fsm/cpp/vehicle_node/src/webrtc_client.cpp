#include "fsm/vehicle/webrtc_client.hpp"
#include "fsm/vehicle/command_executor.hpp"
#include "fsm/logger.hpp"
#include "fsm/utils.hpp"

#include <boost/asio.hpp>
#include <boost/asio/ssl.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/ssl.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <variant>

namespace beast     = boost::beast;
namespace websocket = beast::websocket;
namespace net       = boost::asio;
namespace ssl       = boost::asio::ssl;
using tcp           = boost::asio::ip::tcp;

namespace fsm {
namespace vehicle {

// ============================================================================
// URL parser
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
    FSM_LOG_ERROR("Unsupported WS URL scheme: {}", url);
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
// Base64 encode helper (no deps)
// ============================================================================

static std::string Base64Encode(const uint8_t* data, size_t len) {
  static const char* kTable =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((len + 2) / 3 * 4);
  for (size_t i = 0; i < len; i += 3) {
    const uint32_t v = (static_cast<uint32_t>(data[i]) << 16)
        | ((i + 1 < len ? static_cast<uint32_t>(data[i + 1]) : 0u) << 8)
        |  (i + 2 < len ? static_cast<uint32_t>(data[i + 2]) : 0u);
    out += kTable[(v >> 18) & 63];
    out += kTable[(v >> 12) & 63];
    out += (i + 1 < len) ? kTable[(v >> 6) & 63] : '=';
    out += (i + 2 < len) ? kTable[v & 63]        : '=';
  }
  return out;
}

// ============================================================================
// WebRtcClient::Impl  —  relay-mode Boost.Beast WebSocket client
// ============================================================================

class WebRtcClient::Impl {
 public:
  using PlainWs = websocket::stream<beast::tcp_stream>;
  using SslWs   = websocket::stream<beast::ssl_stream<beast::tcp_stream>>;
  using WsVar   = std::variant<std::monostate, PlainWs, SslWs>;

  explicit Impl() : ssl_ctx_(ssl::context::tlsv12_client) {
    ssl_ctx_.set_verify_mode(ssl::verify_none);
  }

  // ---- state accessible from WebRtcClient public methods ----
  std::atomic<WebRtcState>  state{WebRtcState::kDisconnected};
  mutable std::mutex         latency_mu;
  LatencyInfo                latency;
  std::deque<float>          rtt_samples;

  ConnectionStateCallback  state_cb;
  ControlCommandCallback   command_cb;
  LatencyUpdateCallback    latency_cb;

  WebRtcClientConfig       config;
  std::atomic<uint64_t>    telem_seq{0};
  std::atomic<uint64_t>    video_seq{0};
  std::atomic<uint64_t>    ping_seq{0};

  // ---- async IO ----
  net::io_context  ioc{1};
  ssl::context     ssl_ctx_;
  WsVar            ws_var{std::monostate{}};
  beast::flat_buffer       buffer_;
  std::deque<std::string>  write_queue_;
  bool                     writing_ = false;

  std::thread       io_thread_;
  std::thread       hb_thread_;
  std::atomic<bool> stop_flag{false};

  ParsedUrl   parsed_url_;
  std::string client_id_;
  std::string peer_id_;
  std::shared_ptr<net::steady_timer> reconnect_timer_;

  // ---- methods ----

  void SetState(WebRtcState s) {
    state.store(s);
    if (state_cb) state_cb(s);
  }

  // Must be called on io_context thread.
  void QueueSend(std::string msg) {
    write_queue_.push_back(std::move(msg));
    if (!writing_) DoWrite();
  }

  // Thread-safe post from any thread.
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
              if (ec) {
                FSM_LOG_WARN("WS write error: {}", ec.message());
                write_queue_.clear();
                writing_ = false;
                HandleDisconnect();
                return;
              }
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
                  FSM_LOG_WARN("WS read closed: {}", ec.message());
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

  void SendRegister() {
    nlohmann::json j;
    j["type"]        = "register";
    j["client_type"] = "vehicle";
    j["vehicle_id"]  = config.vehicle_id;
    QueueSend(j.dump());
    FSM_LOG_INFO("Registering vehicle_id={}", config.vehicle_id);
  }

  void OnMessage(const std::string& raw) {
    try {
      auto j = nlohmann::json::parse(raw);
      const std::string type = j.value("type", "");

      if (type == "registered") {
        client_id_ = j.value("client_id", "");
        FSM_LOG_INFO("Registered: client_id={}", client_id_);
        SetState(WebRtcState::kConnected);

      } else if (type == "create_offer") {
        peer_id_ = j.value("operator_id", "");
        FSM_LOG_INFO("Operator connected: peer={}", peer_id_);
        nlohmann::json ack;
        ack["type"]       = "relay_ready";
        ack["vehicle_id"] = config.vehicle_id;
        QueueSend(ack.dump());

      } else if (type == "control") {
        if (command_cb) {
          const auto& d = j.at("data");
          // Operator sends normalised values: steering ∈ [-1, 1],
          // throttle/brake ∈ [0, 1].  Convert to physical units here.
          const float steer_norm    = d.value("steering",  0.0f);
          const float throttle_norm = d.value("throttle",  0.0f);
          const float brake_norm    = d.value("brake",     0.0f);

          ControlCommand cmd;
          static constexpr float kDegToRad = 3.14159265f / 180.0f;
          cmd.steering_tire_angle =
              std::clamp(steer_norm, -1.0f, 1.0f) *
              (config.max_steering_angle_deg * kDegToRad);

          if (brake_norm > 0.01f) {
            cmd.speed        = 0.0f;
            cmd.acceleration = -brake_norm * config.max_decel_mps2;
          } else {
            cmd.speed        = throttle_norm * config.max_speed_kph / 3.6f;
            cmd.acceleration = throttle_norm * config.max_accel_mps2;
          }

          cmd.gear            = d.value("gear",         3);
          cmd.turn_signal     = d.value("turn_signal",  0);
          cmd.hazard_lights   = false;
          cmd.emergency_stop  = d.value("emergency",    false);
          cmd.timestamp_ns    = d.value("timestamp_ns", int64_t{0});
          cmd.sequence_number = d.value("sequence",     uint64_t{0});
          command_cb(cmd);
        }

      } else if (type == "pong") {
        const int64_t orig_t = j.value("timestamp", int64_t{0});
        const int64_t now_ns = NowNanos();
        const float   rtt_ms = static_cast<float>(now_ns - orig_t) / 1.0e6f;
        {
          std::lock_guard<std::mutex> lock(latency_mu);
          rtt_samples.push_back(rtt_ms);
          if (rtt_samples.size() > 10) rtt_samples.pop_front();
          float avg = 0.0f;
          for (float v : rtt_samples) avg += v;
          latency.rtt_ms             = avg / static_cast<float>(rtt_samples.size());
          latency.control_latency_ms = latency.rtt_ms / 2.0f;
          latency.timestamp_ns       = now_ns;
        }
        if (latency_cb) {
          std::lock_guard<std::mutex> lock(latency_mu);
          latency_cb(latency);
        }
        FSM_LOG_DEBUG("RTT {:.1f}ms", rtt_ms);

      } else if (type == "peer_disconnected") {
        peer_id_.clear();
        FSM_LOG_INFO("Operator disconnected");

      } else if (type == "error") {
        FSM_LOG_ERROR("Signaling error: {}", j.value("message", ""));
      }

    } catch (const std::exception& e) {
      FSM_LOG_ERROR("WS message error: {}", e.what());
    }
  }

  void HandleDisconnect() {
    if (stop_flag.load()) return;
    SetState(WebRtcState::kReconnecting);
    peer_id_.clear();
    write_queue_.clear();
    writing_ = false;
    buffer_.consume(buffer_.size());
    ScheduleReconnect();
  }

  void ScheduleReconnect() {
    reconnect_timer_ = std::make_shared<net::steady_timer>(
        ioc, std::chrono::milliseconds(config.reconnect_interval_ms));
    reconnect_timer_->async_wait([this](beast::error_code ec) {
      if (ec || stop_flag.load()) return;
      FSM_LOG_INFO("Reconnecting to {}:{}{}", parsed_url_.host,
                   parsed_url_.port, parsed_url_.path);
      ws_var = std::monostate{};
      auto resolver = std::make_shared<tcp::resolver>(ioc);
      resolver->async_resolve(parsed_url_.host, parsed_url_.port,
          [this, resolver](beast::error_code ec2, tcp::resolver::results_type res) {
            if (ec2) {
              FSM_LOG_ERROR("Resolve failed: {}", ec2.message());
              ScheduleReconnect();
              return;
            }
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
            if (ec) { FSM_LOG_ERROR("TCP connect failed: {}", ec.message()); ScheduleReconnect(); return; }
            auto& ws = std::get<SslWs>(ws_var);
            beast::get_lowest_layer(ws).expires_after(std::chrono::seconds(30));
            ws.next_layer().async_handshake(ssl::stream_base::client,
                [this](beast::error_code ec2) {
                  if (ec2) { FSM_LOG_ERROR("SSL handshake failed: {}", ec2.message()); ScheduleReconnect(); return; }
                  FinishWsHandshakeSsl();
                });
          });
    } else {
      ws_var = PlainWs{net::make_strand(ioc)};
      auto& ws = std::get<PlainWs>(ws_var);
      beast::get_lowest_layer(ws).expires_after(std::chrono::seconds(30));
      beast::get_lowest_layer(ws).async_connect(results,
          [this](beast::error_code ec, tcp::resolver::results_type::endpoint_type) {
            if (ec) { FSM_LOG_ERROR("TCP connect failed: {}", ec.message()); ScheduleReconnect(); return; }
            FinishWsHandshakePlain();
          });
    }
  }

  void FinishWsHandshakePlain() {
    auto& ws = std::get<PlainWs>(ws_var);
    beast::get_lowest_layer(ws).expires_never();
    ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));
    ws.set_option(websocket::stream_base::decorator([](websocket::request_type& req) {
      req.set(beast::http::field::user_agent, "FSM-Pilot-Vehicle/1.0");
    }));
    ws.async_handshake(parsed_url_.host, parsed_url_.path,
        [this](beast::error_code ec) {
          if (ec) { FSM_LOG_ERROR("WS handshake failed: {}", ec.message()); ScheduleReconnect(); return; }
          FSM_LOG_INFO("WS connected (plain): {}:{}{}", parsed_url_.host, parsed_url_.port, parsed_url_.path);
          writing_ = false;
          SendRegister();
          DoRead();
        });
  }

  void FinishWsHandshakeSsl() {
    auto& ws = std::get<SslWs>(ws_var);
    beast::get_lowest_layer(ws).expires_never();
    ws.set_option(websocket::stream_base::timeout::suggested(beast::role_type::client));
    ws.set_option(websocket::stream_base::decorator([](websocket::request_type& req) {
      req.set(beast::http::field::user_agent, "FSM-Pilot-Vehicle/1.0");
    }));
    ws.async_handshake(parsed_url_.host, parsed_url_.path,
        [this](beast::error_code ec) {
          if (ec) { FSM_LOG_ERROR("WS handshake failed: {}", ec.message()); ScheduleReconnect(); return; }
          FSM_LOG_INFO("WS connected (TLS): {}:{}{}", parsed_url_.host, parsed_url_.port, parsed_url_.path);
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
};

// ============================================================================
// WebRtcClient public interface
// ============================================================================

WebRtcClient::WebRtcClient(const WebRtcClientConfig& config)
    : impl_(std::make_unique<Impl>()) {
  impl_->config = config;
  FSM_LOG_INFO("WebRtcClient created: vehicle_id={}", config.vehicle_id);
}

WebRtcClient::~WebRtcClient() {
  Disconnect();
}

bool WebRtcClient::Connect() {
  if (impl_->state.load() == WebRtcState::kConnected ||
      impl_->state.load() == WebRtcState::kConnecting) return true;

  impl_->parsed_url_ = ParseWsUrl(impl_->config.signaling_url);
  if (!impl_->parsed_url_.valid) {
    FSM_LOG_ERROR("Invalid signaling URL: {}", impl_->config.signaling_url);
    impl_->SetState(WebRtcState::kFailed);
    return false;
  }

  impl_->SetState(WebRtcState::kConnecting);
  impl_->stop_flag = false;
  impl_->ioc.restart();

  net::post(impl_->ioc, [this]() {
    auto resolver = std::make_shared<tcp::resolver>(impl_->ioc);
    resolver->async_resolve(impl_->parsed_url_.host, impl_->parsed_url_.port,
        [this, resolver](beast::error_code ec, tcp::resolver::results_type results) {
          if (ec) {
            FSM_LOG_ERROR("DNS resolve failed: {}", ec.message());
            impl_->ScheduleReconnect();
            return;
          }
          impl_->StartConnect(results);
        });
  });

  impl_->io_thread_ = std::thread([this]() { impl_->ioc.run(); });

  impl_->hb_thread_ = std::thread([this]() {
    while (!impl_->stop_flag.load()) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(impl_->config.heartbeat_interval_ms));
      if (!impl_->stop_flag.load()) SendHeartbeat();
    }
  });

  FSM_LOG_INFO("Connecting to signaling: {}", impl_->config.signaling_url);
  return true;
}

void WebRtcClient::Disconnect() {
  if (impl_->stop_flag.load()) return;

  if (impl_->hb_thread_.joinable()) {
    impl_->stop_flag = true;
    impl_->hb_thread_.join();
  }

  net::post(impl_->ioc, [this]() { impl_->CloseNow(); });
  if (impl_->io_thread_.joinable()) impl_->io_thread_.join();

  impl_->SetState(WebRtcState::kDisconnected);
  FSM_LOG_INFO("WebRtcClient disconnected");
}

void WebRtcClient::SendTelemetry(const std::vector<uint8_t>& data) {
  if (!is_connected() || data.empty()) return;
  nlohmann::json j;
  j["type"]       = "telemetry";
  j["vehicle_id"] = impl_->config.vehicle_id;
  j["sequence"]   = impl_->telem_seq.fetch_add(1);
  j["proto_b64"]  = Base64Encode(data.data(), data.size());
  impl_->PostSend(j.dump());
}

void WebRtcClient::SendSystemStatus(const std::vector<uint8_t>& data) {
  if (!is_connected() || data.empty()) return;
  nlohmann::json j;
  j["type"]       = "system_status";
  j["vehicle_id"] = impl_->config.vehicle_id;
  j["proto_b64"]  = Base64Encode(data.data(), data.size());
  impl_->PostSend(j.dump());
}

void WebRtcClient::SendVideoFrame(const std::string& camera_id,
                                   const cv::Mat& frame,
                                   int64_t timestamp_ns) {
  if (!is_connected() || frame.empty()) return;
  std::vector<uint8_t> buf;
  cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 70});
  nlohmann::json j;
  j["type"]        = "video_frame";
  j["camera_id"]   = camera_id;
  j["timestamp_ns"]= timestamp_ns;
  j["frame_number"]= impl_->video_seq.fetch_add(1);
  j["data"]        = Base64Encode(buf.data(), buf.size());
  impl_->PostSend(j.dump());
}

void WebRtcClient::SendHeartbeat() {
  if (!is_connected()) return;
  nlohmann::json j;
  j["type"]       = "ping";
  j["sequence"]   = impl_->ping_seq.fetch_add(1);
  j["timestamp"]  = NowNanos();
  j["vehicle_id"] = impl_->config.vehicle_id;
  impl_->PostSend(j.dump());
}

WebRtcState WebRtcClient::state() const         { return impl_->state.load(); }
bool        WebRtcClient::is_connected() const  { return impl_->state.load() == WebRtcState::kConnected; }
LatencyInfo WebRtcClient::latency_info() const  {
  std::lock_guard<std::mutex> lock(impl_->latency_mu);
  return impl_->latency;
}

void WebRtcClient::set_connection_state_callback(ConnectionStateCallback cb) {
  impl_->state_cb = std::move(cb);
}
void WebRtcClient::set_control_command_callback(ControlCommandCallback cb) {
  impl_->command_cb = std::move(cb);
}
void WebRtcClient::set_latency_update_callback(LatencyUpdateCallback cb) {
  impl_->latency_cb = std::move(cb);
}

}  // namespace vehicle
}  // namespace fsm
