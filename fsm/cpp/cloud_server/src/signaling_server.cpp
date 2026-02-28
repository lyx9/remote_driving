#include "fsm/cloud/signaling_server.hpp"
#include "fsm/logger.hpp"
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>
#include <atomic>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace beast     = boost::beast;
namespace websocket = beast::websocket;
namespace net       = boost::asio;
using tcp           = boost::asio::ip::tcp;

namespace fsm {
namespace cloud {

// ---------------------------------------------------------------------------
// WebSocketSession
// ---------------------------------------------------------------------------

class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
 public:
  explicit WebSocketSession(tcp::socket&& socket, SignalingServer* server)
      : ws_(std::move(socket)), server_(server) {}

  ~WebSocketSession() {
    if (!client_id_.empty()) {
      if (client_type_ == "vehicle" && !vehicle_id_.empty()) {
        server_->OnVehicleDisconnect(vehicle_id_);
      }
      server_->RemoveSession(client_id_);
    }
  }

  void Run() {
    ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
    ws_.set_option(websocket::stream_base::decorator(
        [](websocket::response_type& res) {
          res.set(beast::http::field::server, "FSM-Pilot Signaling");
        }));
    ws_.async_accept(beast::bind_front_handler(
        &WebSocketSession::OnAccept, shared_from_this()));
  }

  void Send(const std::string& message) {
    net::post(ws_.get_executor(), [self = shared_from_this(), message]() {
      self->DoSend(message);
    });
  }

  const std::string& client_id()   const { return client_id_; }
  const std::string& vehicle_id()  const { return vehicle_id_; }
  const std::string& client_type() const { return client_type_; }

  void set_peer_id(const std::string& id) { peer_id_ = id; }
  void clear_peer_id()                    { peer_id_.clear(); }

 private:
  void OnAccept(beast::error_code ec) {
    if (ec) { FSM_LOG_ERROR("WS accept error: {}", ec.message()); return; }
    DoRead();
  }

  void DoRead() {
    ws_.async_read(buffer_,
        beast::bind_front_handler(&WebSocketSession::OnRead, shared_from_this()));
  }

  void OnRead(beast::error_code ec, std::size_t /*bytes*/) {
    if (ec == websocket::error::closed) return;
    if (ec) { FSM_LOG_ERROR("WS read error: {}", ec.message()); return; }

    HandleMessage(beast::buffers_to_string(buffer_.data()));
    buffer_.consume(buffer_.size());
    DoRead();
  }

  void DoSend(const std::string& message) {
    send_queue_.push_back(message);
    if (send_queue_.size() > 1) return;
    ws_.async_write(net::buffer(send_queue_.front()),
        beast::bind_front_handler(&WebSocketSession::OnWrite, shared_from_this()));
  }

  void OnWrite(beast::error_code ec, std::size_t /*bytes*/) {
    if (ec) { FSM_LOG_ERROR("WS write error: {}", ec.message()); return; }
    send_queue_.erase(send_queue_.begin());
    if (!send_queue_.empty()) {
      ws_.async_write(net::buffer(send_queue_.front()),
          beast::bind_front_handler(&WebSocketSession::OnWrite, shared_from_this()));
    }
  }

  void HandleMessage(const std::string& message) {
    try {
      auto j = nlohmann::json::parse(message);
      const std::string type = j.value("type", "");

      if      (type == "register")        HandleRegister(j);
      else if (type == "connect")         HandleConnect(j);
      else if (type == "offer"         ||
               type == "answer"        ||
               type == "ice-candidate") HandleSignaling(j);
      else if (type == "disconnect")      HandleDisconnect(j);
      else if (type == "ping")            HandlePing(j);
      else if (type == "telemetry"     ||
               type == "video_frame"  ||
               type == "system_status") HandleTelemetryRelay(j);
      else if (type == "control")         HandleControlRelay(j);
      else if (type == "relay_ready")     HandleRelayReady(j);
      else                                HandleRelay(j);  // generic passthrough

    } catch (const std::exception& e) {
      FSM_LOG_ERROR("WS message parse error: {}", e.what());
      SendError("Invalid message format");
    }
  }

  void HandleRegister(const nlohmann::json& j) {
    client_id_   = j.value("client_id",   "");
    client_type_ = j.value("client_type", "operator");
    vehicle_id_  = j.value("vehicle_id",  "");

    if (client_id_.empty()) {
      static std::atomic<uint64_t> counter{0};
      client_id_ = "client_" + std::to_string(++counter);
    }

    server_->AddSession(client_id_, shared_from_this());
    FSM_LOG_INFO("Client registered: {} type={} vehicle={}",
                 client_id_, client_type_, vehicle_id_);

    nlohmann::json resp;
    resp["type"]      = "registered";
    resp["client_id"] = client_id_;
    Send(resp.dump());

    if (client_type_ == "vehicle" && !vehicle_id_.empty()) {
      server_->NotifyVehicleOnline(vehicle_id_);
      server_->OnVehicleConnect(vehicle_id_);
    }
  }

  void HandleConnect(const nlohmann::json& j) {
    const std::string target = j["vehicle_id"];
    auto vehicle_session = server_->FindVehicleSession(target);
    if (!vehicle_session) {
      SendError("Vehicle not found: " + target);
      return;
    }

    peer_id_ = vehicle_session->client_id();
    vehicle_session->set_peer_id(client_id_);

    FSM_LOG_INFO("Connecting operator {} to vehicle {}", client_id_, target);

    nlohmann::json req;
    req["type"]        = "create_offer";
    req["operator_id"] = client_id_;
    vehicle_session->Send(req.dump());
  }

  void HandleSignaling(const nlohmann::json& j) {
    if (peer_id_.empty()) { SendError("No peer connected"); return; }
    auto peer = server_->GetSession(peer_id_);
    if (!peer) {
      SendError("Peer disconnected");
      peer_id_.clear();
      return;
    }
    nlohmann::json fwd = j;
    fwd["from"] = client_id_;
    peer->Send(fwd.dump());
  }

  void HandleDisconnect(const nlohmann::json& /*j*/) {
    if (peer_id_.empty()) return;
    auto peer = server_->GetSession(peer_id_);
    if (peer) {
      nlohmann::json notify;
      notify["type"]    = "peer_disconnected";
      notify["peer_id"] = client_id_;
      peer->Send(notify.dump());
      peer->clear_peer_id();
    }
    peer_id_.clear();
  }

  // Respond to a ping with a pong (for latency measurement).
  void HandlePing(const nlohmann::json& j) {
    nlohmann::json pong;
    pong["type"]      = "pong";
    pong["sequence"]  = j.value("sequence",  uint64_t{0});
    pong["timestamp"] = j.value("timestamp", int64_t{0});
    Send(pong.dump());
  }

  // Relay telemetry (vehicle → operator) and notify server for scheduling.
  void HandleTelemetryRelay(const nlohmann::json& j) {
    if (!peer_id_.empty()) {
      auto peer = server_->GetSession(peer_id_);
      if (peer) {
        nlohmann::json fwd = j;
        fwd["from"] = client_id_;
        peer->Send(fwd.dump());
      }
    }
    // Notify server of telemetry for scheduling / alert updates.
    const std::string vid = j.value("vehicle_id", vehicle_id_);
    server_->OnVehicleTelemetry(vid, j);
  }

  // Relay control command (operator → vehicle).
  void HandleControlRelay(const nlohmann::json& j) {
    if (peer_id_.empty()) { SendError("Not connected to a vehicle"); return; }
    auto peer = server_->GetSession(peer_id_);
    if (!peer) { SendError("Vehicle disconnected"); peer_id_.clear(); return; }
    nlohmann::json fwd = j;
    fwd["from"] = client_id_;
    peer->Send(fwd.dump());
  }

  // Vehicle acknowledges relay mode is ready.
  void HandleRelayReady(const nlohmann::json& j) {
    FSM_LOG_INFO("Vehicle {} relay ready", j.value("vehicle_id", ""));
    if (!peer_id_.empty()) {
      auto peer = server_->GetSession(peer_id_);
      if (peer) {
        nlohmann::json notify;
        notify["type"]       = "vehicle_relay_ready";
        notify["vehicle_id"] = j.value("vehicle_id", vehicle_id_);
        peer->Send(notify.dump());
      }
    }
  }

  // Generic passthrough — forward any message to the peer unchanged.
  void HandleRelay(const nlohmann::json& j) {
    if (peer_id_.empty()) return;
    auto peer = server_->GetSession(peer_id_);
    if (!peer) { peer_id_.clear(); return; }
    nlohmann::json fwd = j;
    fwd["from"] = client_id_;
    peer->Send(fwd.dump());
  }

  void SendError(const std::string& message) {
    nlohmann::json err;
    err["type"]    = "error";
    err["message"] = message;
    Send(err.dump());
  }

  websocket::stream<beast::tcp_stream> ws_;
  SignalingServer*                     server_;
  beast::flat_buffer                   buffer_;
  std::vector<std::string>             send_queue_;

  std::string client_id_;
  std::string client_type_;
  std::string vehicle_id_;
  std::string peer_id_;
};

// ---------------------------------------------------------------------------
// SignalingServer::Impl
// ---------------------------------------------------------------------------

class SignalingServer::Impl {
 public:
  explicit Impl(const config::CloudConfigManager& config)
      : config_(config), ioc_(1), acceptor_(ioc_) {}

  bool Start(SignalingServer* server) {
    server_ = server;
    try {
      const auto addr = net::ip::make_address("0.0.0.0");
      const auto port = static_cast<unsigned short>(config_.signaling_port());
      tcp::endpoint ep(addr, port);

      acceptor_.open(ep.protocol());
      acceptor_.set_option(net::socket_base::reuse_address(true));
      acceptor_.bind(ep);
      acceptor_.listen(net::socket_base::max_listen_connections);

      FSM_LOG_INFO("Signaling server on port {}", port);
      running_ = true;
      DoAccept();

      io_thread_ = std::thread([this]() { ioc_.run(); });
      return true;
    } catch (const std::exception& e) {
      FSM_LOG_ERROR("Failed to start signaling server: {}", e.what());
      return false;
    }
  }

  void Stop() {
    running_ = false;
    ioc_.stop();
    if (io_thread_.joinable()) io_thread_.join();
    FSM_LOG_INFO("Signaling server stopped");
  }

  void AddSession(const std::string& id, std::shared_ptr<WebSocketSession> session) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_[id] = std::move(session);
  }

  void RemoveSession(const std::string& id) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    sessions_.erase(id);
  }

  std::shared_ptr<WebSocketSession> GetSession(const std::string& id) const {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    const auto it = sessions_.find(id);
    return (it != sessions_.end()) ? it->second : nullptr;
  }

  std::shared_ptr<WebSocketSession> FindVehicleSession(
      const std::string& vehicle_id) const {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    for (const auto& [id, session] : sessions_) {
      if (session->client_type() == "vehicle" &&
          session->vehicle_id()  == vehicle_id) {
        return session;
      }
    }
    return nullptr;
  }

  void NotifyVehicleOnline(const std::string& vehicle_id) {
    nlohmann::json msg;
    msg["type"]       = "vehicle_online";
    msg["vehicle_id"] = vehicle_id;
    const std::string payload = msg.dump();

    std::lock_guard<std::mutex> lock(sessions_mutex_);
    for (const auto& [id, session] : sessions_) {
      if (session->client_type() == "operator") session->Send(payload);
    }
  }

  void BroadcastToOperators(const std::string& message) {
    std::lock_guard<std::mutex> lock(sessions_mutex_);
    for (const auto& [id, session] : sessions_) {
      if (session->client_type() == "operator") session->Send(message);
    }
  }

  void SetTelemetryCallback(TelemetryRelayCallback cb) {
    std::lock_guard<std::mutex> lock(telemetry_cb_mutex_);
    telemetry_cb_ = std::move(cb);
  }

  void SetConnectCallback(VehicleEventCallback cb) {
    std::lock_guard<std::mutex> lock(event_cb_mutex_);
    connect_cb_ = std::move(cb);
  }

  void SetDisconnectCallback(VehicleEventCallback cb) {
    std::lock_guard<std::mutex> lock(event_cb_mutex_);
    disconnect_cb_ = std::move(cb);
  }

  void OnVehicleTelemetry(const std::string& vehicle_id,
                           const nlohmann::json& payload) {
    TelemetryRelayCallback cb;
    {
      std::lock_guard<std::mutex> lock(telemetry_cb_mutex_);
      cb = telemetry_cb_;
    }
    if (cb) cb(vehicle_id, payload);
  }

  void OnVehicleConnect(const std::string& vehicle_id) {
    VehicleEventCallback cb;
    {
      std::lock_guard<std::mutex> lock(event_cb_mutex_);
      cb = connect_cb_;
    }
    if (cb) cb(vehicle_id);
  }

  void OnVehicleDisconnect(const std::string& vehicle_id) {
    VehicleEventCallback cb;
    {
      std::lock_guard<std::mutex> lock(event_cb_mutex_);
      cb = disconnect_cb_;
    }
    if (cb) cb(vehicle_id);
  }

 private:
  void DoAccept() {
    acceptor_.async_accept(
        net::make_strand(ioc_),
        beast::bind_front_handler(&Impl::OnAccept, this));
  }

  void OnAccept(beast::error_code ec, tcp::socket socket) {
    if (!ec) {
      // Pass the public SignalingServer pointer so sessions can call AddSession etc.
      // The Impl is owned by SignalingServer so the cast is safe.
      std::make_shared<WebSocketSession>(
          std::move(socket),
          server_)->Run();
    } else if (running_) {
      FSM_LOG_ERROR("Accept error: {}", ec.message());
    }
    if (running_) DoAccept();
  }

  const config::CloudConfigManager& config_;
  net::io_context                   ioc_;
  tcp::acceptor                     acceptor_;
  std::thread                       io_thread_;
  std::atomic<bool>                 running_{false};
  SignalingServer*                  server_ = nullptr;

  mutable std::mutex sessions_mutex_;
  std::unordered_map<std::string, std::shared_ptr<WebSocketSession>> sessions_;

  mutable std::mutex       telemetry_cb_mutex_;
  TelemetryRelayCallback   telemetry_cb_;

  mutable std::mutex       event_cb_mutex_;
  VehicleEventCallback     connect_cb_;
  VehicleEventCallback     disconnect_cb_;
};

// ---------------------------------------------------------------------------
// SignalingServer public interface
// ---------------------------------------------------------------------------

SignalingServer::SignalingServer(const config::CloudConfigManager& config)
    : impl_(std::make_unique<Impl>(config)) {}

SignalingServer::~SignalingServer() = default;

bool SignalingServer::Start() { return impl_->Start(this); }
void SignalingServer::Stop()  { impl_->Stop(); }

void SignalingServer::AddSession(
    const std::string& id, std::shared_ptr<WebSocketSession> session) {
  impl_->AddSession(id, std::move(session));
}

void SignalingServer::RemoveSession(const std::string& id) {
  impl_->RemoveSession(id);
}

std::shared_ptr<WebSocketSession> SignalingServer::GetSession(
    const std::string& id) const {
  return impl_->GetSession(id);
}

std::shared_ptr<WebSocketSession> SignalingServer::FindVehicleSession(
    const std::string& vehicle_id) const {
  return impl_->FindVehicleSession(vehicle_id);
}

void SignalingServer::NotifyVehicleOnline(const std::string& vehicle_id) {
  impl_->NotifyVehicleOnline(vehicle_id);
}

void SignalingServer::BroadcastToOperators(const std::string& message) {
  impl_->BroadcastToOperators(message);
}

void SignalingServer::set_telemetry_callback(TelemetryRelayCallback cb) {
  impl_->SetTelemetryCallback(std::move(cb));
}

void SignalingServer::set_connect_callback(VehicleEventCallback cb) {
  impl_->SetConnectCallback(std::move(cb));
}

void SignalingServer::set_disconnect_callback(VehicleEventCallback cb) {
  impl_->SetDisconnectCallback(std::move(cb));
}

void SignalingServer::OnVehicleTelemetry(const std::string& vehicle_id,
                                          const nlohmann::json& payload) {
  impl_->OnVehicleTelemetry(vehicle_id, payload);
}

void SignalingServer::OnVehicleConnect(const std::string& vehicle_id) {
  impl_->OnVehicleConnect(vehicle_id);
}

void SignalingServer::OnVehicleDisconnect(const std::string& vehicle_id) {
  impl_->OnVehicleDisconnect(vehicle_id);
}

}  // namespace cloud
}  // namespace fsm
