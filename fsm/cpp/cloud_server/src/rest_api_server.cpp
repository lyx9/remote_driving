#include "fsm/cloud/rest_api_server.hpp"
#include "fsm/logger.hpp"

#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <nlohmann/json.hpp>
#include <openssl/hmac.h>
#include <openssl/evp.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <sstream>
#include <string>
#include <thread>

namespace beast = boost::beast;
namespace http  = beast::http;
namespace net   = boost::asio;
using tcp       = boost::asio::ip::tcp;

namespace fsm {
namespace cloud {

// ============================================================================
// JWT utilities — HS256 only; no external library needed.
// ============================================================================
namespace jwt {

static std::string Base64UrlDecode(const std::string& in) {
  std::string s = in;
  for (char& c : s) {
    if (c == '-') c = '+';
    else if (c == '_') c = '/';
  }
  while (s.size() % 4) s += '=';
  std::string out;
  uint32_t val = 0; int bits = -8;
  for (unsigned char c : s) {
    // Simple base64 decode table
    int d = -1;
    if (c >= 'A' && c <= 'Z') d = c - 'A';
    else if (c >= 'a' && c <= 'z') d = c - 'a' + 26;
    else if (c >= '0' && c <= '9') d = c - '0' + 52;
    else if (c == '+') d = 62;
    else if (c == '/') d = 63;
    else if (c == '=') continue;
    if (d < 0) continue;
    val = (val << 6) + static_cast<uint32_t>(d);
    bits += 6;
    if (bits >= 0) { out += static_cast<char>((val >> bits) & 0xFF); bits -= 8; }
  }
  return out;
}

static std::string HmacSha256(const std::string& key, const std::string& msg) {
  unsigned int len = 32;
  std::string out(32, '\0');
  HMAC(EVP_sha256(),
       key.data(), static_cast<int>(key.size()),
       reinterpret_cast<const unsigned char*>(msg.data()), msg.size(),
       reinterpret_cast<unsigned char*>(&out[0]), &len);
  return out;
}

// Returns JWT payload claims on success; throws std::runtime_error on failure.
static nlohmann::json Verify(const std::string& token,
                              const std::string& secret) {
  const auto d1 = token.find('.');
  const auto d2 = (d1 != std::string::npos) ? token.find('.', d1 + 1)
                                             : std::string::npos;
  if (d1 == std::string::npos || d2 == std::string::npos)
    throw std::runtime_error("malformed JWT");

  const std::string hdr_b64 = token.substr(0, d1);
  const std::string pay_b64 = token.substr(d1 + 1, d2 - d1 - 1);
  const std::string sig_b64 = token.substr(d2 + 1);

  const auto hdr = nlohmann::json::parse(Base64UrlDecode(hdr_b64),
                                         nullptr, false);
  if (hdr.is_discarded() || hdr.value("alg", "") != "HS256")
    throw std::runtime_error("unsupported JWT algorithm");

  const std::string expected = HmacSha256(secret, hdr_b64 + "." + pay_b64);
  if (expected != Base64UrlDecode(sig_b64))
    throw std::runtime_error("invalid JWT signature");

  const auto pay = nlohmann::json::parse(Base64UrlDecode(pay_b64),
                                         nullptr, false);
  if (pay.is_discarded()) throw std::runtime_error("malformed JWT payload");

  if (pay.contains("exp")) {
    const int64_t now = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    if (now > pay["exp"].get<int64_t>())
      throw std::runtime_error("JWT expired");
  }
  return pay;
}

}  // namespace jwt

// ============================================================================
// Atomic counters shared between HttpSession instances and the public API.
// ============================================================================
struct Counters {
  std::atomic<uint64_t> telemetry_received{0};
  std::atomic<uint64_t> commands_forwarded{0};
  std::atomic<uint64_t> alerts_fired{0};
  std::atomic<uint64_t> http_requests{0};
  std::atomic<uint64_t> http_4xx{0};
  std::atomic<uint64_t> http_auth_failures{0};
  std::atomic<uint64_t> sessions_created{0};
  std::atomic<uint64_t> sessions_terminated{0};
};

// ============================================================================
// HTTP session — single request/response lifecycle.
// ============================================================================
class HttpSession : public std::enable_shared_from_this<HttpSession> {
 public:
  HttpSession(tcp::socket&&      socket,
              SchedulingService& scheduler,
              AlertAnalyzer&     alerts,
              SessionManager&    sessions,
              const std::string& jwt_secret,
              Counters&          counters)
      : stream_(std::move(socket)),
        scheduler_(scheduler),
        alerts_(alerts),
        sessions_(sessions),
        jwt_secret_(jwt_secret),
        counters_(counters) {}

  void Run() {
    net::dispatch(stream_.get_executor(),
        beast::bind_front_handler(&HttpSession::DoRead, shared_from_this()));
  }

 private:
  void DoRead() {
    req_ = {};
    stream_.expires_after(std::chrono::seconds(10));
    http::async_read(stream_, buffer_, req_,
        beast::bind_front_handler(&HttpSession::OnRead, shared_from_this()));
  }

  void OnRead(beast::error_code ec, std::size_t) {
    if (ec == http::error::end_of_stream) { DoClose(); return; }
    if (ec) { FSM_LOG_WARN("HTTP read: {}", ec.message()); return; }
    counters_.http_requests.fetch_add(1, std::memory_order_relaxed);
    HandleRequest();
  }

  // Returns JWT payload if auth passes, or sends 401 and returns nullopt.
  std::optional<nlohmann::json> Authenticate() {
    if (jwt_secret_.empty()) return nlohmann::json::object();
    const auto hdr = req_[http::field::authorization];
    if (hdr.empty() || hdr.substr(0, 7) != "Bearer ") {
      counters_.http_auth_failures.fetch_add(1, std::memory_order_relaxed);
      SendError(http::status::unauthorized,
                "missing or malformed Authorization header");
      return std::nullopt;
    }
    try {
      return jwt::Verify(std::string(hdr.substr(7)), jwt_secret_);
    } catch (const std::exception& e) {
      counters_.http_auth_failures.fetch_add(1, std::memory_order_relaxed);
      SendError(http::status::unauthorized, e.what());
      return std::nullopt;
    }
  }

  void HandleRequest() {
    const std::string target(req_.target());
    const auto method = req_.method();

    // ── Unauthenticated ──────────────────────────────────────────────────
    if (target == "/api/v1/health" || target == "/api/v1/health/") {
      return SendJson(http::status::ok, BuildHealth());
    }
    if (target == "/metrics" || target == "/metrics/") {
      return SendText(http::status::ok, BuildMetrics());
    }

    // ── Authenticated ────────────────────────────────────────────────────
    auto claims = Authenticate();
    if (!claims) return;

    if (target == "/api/v1/vehicles" || target == "/api/v1/vehicles/") {
      return SendJson(http::status::ok, BuildVehicleList());
    }
    // /api/v1/vehicles/{id}/diagnostics — must be checked before generic {id}.
    if (target.rfind("/api/v1/vehicles/", 0) == 0 &&
        target.size() > 17U &&
        target.rfind("/diagnostics") == target.size() - 12U) {
      const std::string vid = target.substr(17, target.size() - 17U - 12U);
      return SendJson(http::status::ok, BuildVehicleDiagnostics(vid));
    }
    if (target.rfind("/api/v1/vehicles/", 0) == 0) {
      return SendJson(http::status::ok, BuildVehicleDetail(target.substr(17)));
    }
    if (target == "/api/v1/queue" || target == "/api/v1/queue/") {
      return SendJson(http::status::ok, BuildQueue());
    }
    if (target == "/api/v1/alerts" || target == "/api/v1/alerts/") {
      return SendJson(http::status::ok, BuildAlerts());
    }
    if (target.rfind("/api/v1/alerts/", 0) == 0 &&
        target.find("/ack") != std::string::npos) {
      const auto sl = target.rfind('/', target.size() - 5);
      alerts_.AcknowledgeAlert(target.substr(15, sl - 15));
      return SendJson(http::status::ok, R"({"status":"ok"})");
    }
    if (target == "/api/v1/sessions" || target == "/api/v1/sessions/") {
      if (method == http::verb::get)
        return SendJson(http::status::ok, BuildSessionList());
      if (method == http::verb::post)
        return HandleCreateSession(*claims);
    }
    if (target.rfind("/api/v1/sessions/", 0) == 0) {
      const std::string sid = target.substr(17);
      if (method == http::verb::get)
        return SendJson(http::status::ok, BuildSessionDetail(sid));
      if (method == http::verb::delete_)
        return HandleTerminateSession(sid);
    }

    counters_.http_4xx.fetch_add(1, std::memory_order_relaxed);
    SendJson(http::status::not_found, R"({"error":"not found"})");
  }

  // ── JSON builders ───────────────────────────────────────────────────────

  std::string BuildHealth() const {
    nlohmann::json j;
    j["status"]    = "ok";
    j["service"]   = "fsm-pilot-cloud";
    j["vehicles"]  = scheduler_.connected_vehicle_count();
    j["sessions"]  = sessions_.active_session_count();
    j["alerts"]    = alerts_.GetStatistics().total;
    j["timestamp"] = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return j.dump();
  }

  std::string BuildMetrics() const {
    std::ostringstream ss;
    auto G = [&](const char* n, const char* h, uint64_t v) {
      ss << "# HELP " << n << " " << h << "\n# TYPE " << n << " gauge\n"
         << n << " " << v << "\n";
    };
    auto C = [&](const char* n, const char* h, uint64_t v) {
      ss << "# HELP " << n << " " << h << "\n# TYPE " << n << " counter\n"
         << n << "_total " << v << "\n";
    };
    G("fsm_connected_vehicles",    "Connected vehicles",          scheduler_.connected_vehicle_count());
    G("fsm_active_sessions",       "Active operator sessions",    sessions_.active_session_count());
    const auto st = alerts_.GetStatistics();
    G("fsm_active_alerts",         "Active alerts",               st.total);
    G("fsm_active_alerts_critical","Critical alerts",             st.critical);
    G("fsm_active_alerts_warning", "Warning alerts",              st.warning);
    C("fsm_telemetry_received",    "Vehicle telemetry received",  counters_.telemetry_received.load(std::memory_order_relaxed));
    C("fsm_commands_forwarded",    "Control commands forwarded",  counters_.commands_forwarded.load(std::memory_order_relaxed));
    C("fsm_alerts_fired",          "Alerts fired total",          counters_.alerts_fired.load(std::memory_order_relaxed));
    C("fsm_http_requests",         "HTTP requests handled",       counters_.http_requests.load(std::memory_order_relaxed));
    C("fsm_http_4xx",              "HTTP 4xx responses",          counters_.http_4xx.load(std::memory_order_relaxed));
    C("fsm_http_auth_failures",    "JWT auth failures",           counters_.http_auth_failures.load(std::memory_order_relaxed));
    C("fsm_sessions_created",      "Sessions created",            counters_.sessions_created.load(std::memory_order_relaxed));
    C("fsm_sessions_terminated",   "Sessions terminated",         counters_.sessions_terminated.load(std::memory_order_relaxed));
    return ss.str();
  }

  std::string BuildVehicleList() const {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& v : scheduler_.GetSchedulingQueue()) {
      nlohmann::json item;
      item["vehicle_id"]      = v.vehicle_id;
      item["priority_score"]  = v.priority_score;
      item["latency_ms"]      = v.latency_ms;
      item["battery_pct"]     = v.battery_pct;
      item["emergency_level"] = v.emergency_level;
      item["task_status"]     = static_cast<int>(v.task_status);
      arr.push_back(item);
    }
    return nlohmann::json{{"vehicles", arr}, {"count", arr.size()}}.dump();
  }

  std::string BuildVehicleDiagnostics(const std::string& vid) const {
    // Aggregates all cloud-side knowledge about a vehicle's health.
    // Useful for remote debugging without ROS2 tooling on the vehicle.
    nlohmann::json result;
    result["vehicle_id"]  = vid;
    result["timestamp_ns"] = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    // ── Scheduling / motion state ──────────────────────────────────────────
    const auto info = scheduler_.GetVehicleInfo(vid);
    if (info) {
      nlohmann::json sched;
      sched["is_connected"]    = info->is_connected;
      sched["latency_ms"]      = info->latency_ms;
      sched["battery_pct"]     = info->battery_pct;
      sched["speed_mps"]       = info->speed_mps;
      sched["emergency_level"] = info->emergency_level;
      sched["priority_score"]  = info->priority_score;
      sched["task_status"]     = info->task_status;
      sched["last_update_ns"]  = info->last_update_ns;
      result["scheduling"]     = sched;
    } else {
      result["scheduling"] = nullptr;
    }

    // ── Session ────────────────────────────────────────────────────────────
    const auto sess = sessions_.GetActiveSessionForVehicle(vid);
    if (sess) {
      nlohmann::json s;
      s["session_id"]  = sess->session_id;
      s["operator_id"] = sess->operator_id;
      s["duration_s"]  = sess->duration_s();
      result["session"] = s;
    } else {
      result["session"] = nullptr;
    }

    // ── Active alerts ──────────────────────────────────────────────────────
    nlohmann::json alert_arr = nlohmann::json::array();
    for (const auto& a : alerts_.GetActiveAlertsForVehicle(vid)) {
      alert_arr.push_back(nlohmann::json{
          {"id",           a.id},
          {"type",         a.type},
          {"message",      a.message},
          {"severity",     static_cast<int>(a.severity)},
          {"acknowledged", a.acknowledged},
          {"timestamp_ns", a.timestamp_ns}});
    }
    result["active_alerts"] = alert_arr;
    result["alert_count"]   = alert_arr.size();

    // ── Summary health ─────────────────────────────────────────────────────
    const bool connected = info && info->is_connected;
    const bool has_crit  = [&]() -> bool {
      for (const auto& a : alerts_.GetActiveAlertsForVehicle(vid)) {
        if (a.severity == AlertSeverity::kCritical) return true;
      }
      return false;
    }();
    result["overall_status"] = has_crit  ? "critical"
                               : !connected ? "offline"
                               : "ok";

    return result.dump();
  }

  std::string BuildVehicleDetail(const std::string& vid) const {
    for (const auto& v : scheduler_.GetSchedulingQueue()) {
      if (v.vehicle_id != vid) continue;
      nlohmann::json item;
      item["vehicle_id"]      = v.vehicle_id;
      item["priority_score"]  = v.priority_score;
      item["latency_ms"]      = v.latency_ms;
      item["battery_pct"]     = v.battery_pct;
      item["emergency_level"] = v.emergency_level;
      item["task_status"]     = static_cast<int>(v.task_status);
      item["last_update_ns"]  = v.last_update_ns;
      item["speed_mps"]       = v.speed_mps;
      item["is_connected"]    = v.is_connected;
      const auto s = sessions_.GetActiveSessionForVehicle(vid);
      if (s) { item["session_id"] = s->session_id;
               item["operator_id"] = s->operator_id; }
      return item.dump();
    }
    return nlohmann::json{{"error", "vehicle not found: " + vid}}.dump();
  }

  std::string BuildQueue() const {
    nlohmann::json arr = nlohmann::json::array();
    int rank = 1;
    for (const auto& v : scheduler_.GetSchedulingQueue()) {
      arr.push_back(nlohmann::json{
          {"rank", rank++}, {"vehicle_id", v.vehicle_id},
          {"priority_score", v.priority_score},
          {"latency_ms", v.latency_ms}, {"battery_pct", v.battery_pct}});
    }
    return nlohmann::json{{"queue", arr}, {"count", arr.size()}}.dump();
  }

  std::string BuildAlerts() const {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& a : alerts_.GetActiveAlerts()) {
      arr.push_back(nlohmann::json{
          {"id", a.id}, {"vehicle_id", a.vehicle_id},
          {"type", a.type}, {"message", a.message},
          {"severity", static_cast<int>(a.severity)},
          {"acknowledged", a.acknowledged},
          {"timestamp_ns", a.timestamp_ns}});
    }
    return nlohmann::json{{"alerts", arr}, {"count", arr.size()}}.dump();
  }

  std::string BuildSessionList() const {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& s : sessions_.GetActiveSessions())
      arr.push_back(SessionJson(s));
    return nlohmann::json{{"sessions", arr}, {"count", arr.size()}}.dump();
  }

  std::string BuildSessionDetail(const std::string& sid) const {
    const auto s = sessions_.FindSession(sid);
    if (!s) return nlohmann::json{{"error", "session not found"}}.dump();
    return SessionJson(*s).dump();
  }

  void HandleCreateSession(const nlohmann::json& claims) {
    nlohmann::json body;
    try { body = nlohmann::json::parse(req_.body()); }
    catch (...) {
      return SendError(http::status::bad_request, "invalid JSON body");
    }
    const std::string vid = body.value("vehicle_id", "");
    if (vid.empty())
      return SendError(http::status::bad_request, "vehicle_id required");
    const std::string op_id = claims.value("sub",
        body.value("operator_id", "anonymous"));
    const auto s = sessions_.CreateSession(op_id, vid);
    if (!s)
      return SendError(http::status::conflict,
                       "vehicle already has an active session");
    counters_.sessions_created.fetch_add(1, std::memory_order_relaxed);
    SendJson(http::status::created, SessionJson(*s).dump());
  }

  void HandleTerminateSession(const std::string& sid) {
    if (!sessions_.TerminateSession(sid))
      return SendError(http::status::not_found,
                       "session not found or already terminated");
    counters_.sessions_terminated.fetch_add(1, std::memory_order_relaxed);
    SendJson(http::status::ok, R"({"status":"ok"})");
  }

  static nlohmann::json SessionJson(const Session& s) {
    static const char* kState[] = {"active", "terminated", "expired"};
    nlohmann::json j;
    j["session_id"]    = s.session_id;
    j["operator_id"]   = s.operator_id;
    j["vehicle_id"]    = s.vehicle_id;
    j["state"]         = kState[static_cast<int>(s.state)];
    j["started_at_ns"] = s.started_at_ns;
    j["last_ping_ns"]  = s.last_ping_ns;
    j["duration_s"]    = s.duration_s();
    if (s.ended_at_ns > 0) j["ended_at_ns"] = s.ended_at_ns;
    return j;
  }

  // ── Response helpers ────────────────────────────────────────────────────

  void SendError(http::status st, const std::string& msg) {
    counters_.http_4xx.fetch_add(1, std::memory_order_relaxed);
    SendJson(st, nlohmann::json{{"error", msg}}.dump());
  }

  void SendJson(http::status st, const std::string& body) {
    auto res = std::make_shared<http::response<http::string_body>>();
    res->result(st);
    res->version(req_.version());
    res->set(http::field::server, "FSM-Pilot/1.0");
    res->set(http::field::content_type, "application/json");
    res->set(http::field::access_control_allow_origin, "*");
    res->keep_alive(req_.keep_alive());
    res->body() = body;
    res->prepare_payload();
    DoWrite(res);
  }

  void SendText(http::status st, const std::string& body) {
    auto res = std::make_shared<http::response<http::string_body>>();
    res->result(st);
    res->version(req_.version());
    res->set(http::field::server, "FSM-Pilot/1.0");
    res->set(http::field::content_type, "text/plain; version=0.0.4");
    res->set(http::field::access_control_allow_origin, "*");
    res->keep_alive(req_.keep_alive());
    res->body() = body;
    res->prepare_payload();
    DoWrite(res);
  }

  void DoWrite(std::shared_ptr<http::response<http::string_body>> res) {
    http::async_write(stream_, *res,
        [self = shared_from_this(), res](beast::error_code ec, std::size_t) {
          if (ec) FSM_LOG_WARN("HTTP write: {}", ec.message());
          if (!res->keep_alive()) self->DoClose();
        });
  }

  void DoClose() {
    beast::error_code ec;
    stream_.socket().shutdown(tcp::socket::shutdown_send, ec);
  }

  beast::tcp_stream                    stream_;
  beast::flat_buffer                   buffer_;
  http::request<http::string_body>     req_;
  SchedulingService&                   scheduler_;
  AlertAnalyzer&                       alerts_;
  SessionManager&                      sessions_;
  const std::string&                   jwt_secret_;
  Counters&                            counters_;
};

// ============================================================================
// RestApiServer::Impl
// ============================================================================

class RestApiServer::Impl {
 public:
  Impl(const config::CloudConfigManager& cfg,
       SchedulingService& scheduler,
       AlertAnalyzer& alerts,
       SessionManager& sessions)
      : config_(cfg), scheduler_(scheduler), alerts_(alerts),
        sessions_(sessions), ioc_(1), acceptor_(ioc_),
        jwt_secret_(cfg.api_jwt_secret()) {}

  bool Start() {
    try {
      const auto addr = net::ip::make_address("0.0.0.0");
      const auto port = static_cast<unsigned short>(config_.api_port());
      tcp::endpoint ep(addr, port);
      acceptor_.open(ep.protocol());
      acceptor_.set_option(net::socket_base::reuse_address(true));
      acceptor_.bind(ep);
      acceptor_.listen(net::socket_base::max_listen_connections);
      FSM_LOG_INFO("REST API port {} (auth={})", port,
                   jwt_secret_.empty() ? "off" : "JWT-HS256");
      running_ = true;
      DoAccept();
      io_thread_ = std::thread([this]() { ioc_.run(); });
      return true;
    } catch (const std::exception& e) {
      FSM_LOG_ERROR("REST API start failed: {}", e.what());
      return false;
    }
  }

  void Stop() {
    running_ = false;
    ioc_.stop();
    if (io_thread_.joinable()) io_thread_.join();
    FSM_LOG_INFO("REST API stopped");
  }

  void RecordTelemetryReceived() {
    counters_.telemetry_received.fetch_add(1, std::memory_order_relaxed);
  }
  void RecordCommandForwarded() {
    counters_.commands_forwarded.fetch_add(1, std::memory_order_relaxed);
  }
  void RecordAlertFired() {
    counters_.alerts_fired.fetch_add(1, std::memory_order_relaxed);
  }

 private:
  void DoAccept() {
    acceptor_.async_accept(net::make_strand(ioc_),
        [this](beast::error_code ec, tcp::socket sock) {
          if (!ec) {
            std::make_shared<HttpSession>(
                std::move(sock), scheduler_, alerts_, sessions_,
                jwt_secret_, counters_)->Run();
          } else if (running_) {
            FSM_LOG_WARN("REST accept: {}", ec.message());
          }
          if (running_) DoAccept();
        });
  }

  const config::CloudConfigManager& config_;
  SchedulingService&                scheduler_;
  AlertAnalyzer&                    alerts_;
  SessionManager&                   sessions_;
  net::io_context                   ioc_;
  tcp::acceptor                     acceptor_;
  std::thread                       io_thread_;
  std::atomic<bool>                 running_{false};
  std::string                       jwt_secret_;
  Counters                          counters_;
};

// ============================================================================
// RestApiServer — public interface
// ============================================================================

RestApiServer::RestApiServer(const config::CloudConfigManager& cfg,
                              SchedulingService& scheduler,
                              AlertAnalyzer& alerts,
                              SessionManager& sessions)
    : impl_(std::make_unique<Impl>(cfg, scheduler, alerts, sessions)) {}

RestApiServer::~RestApiServer() = default;
bool RestApiServer::Start() { return impl_->Start(); }
void RestApiServer::Stop()  { impl_->Stop(); }
void RestApiServer::RecordTelemetryReceived() { impl_->RecordTelemetryReceived(); }
void RestApiServer::RecordCommandForwarded()  { impl_->RecordCommandForwarded(); }
void RestApiServer::RecordAlertFired()        { impl_->RecordAlertFired(); }

}  // namespace cloud
}  // namespace fsm
