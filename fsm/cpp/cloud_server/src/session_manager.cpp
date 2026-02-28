#include "fsm/cloud/session_manager.hpp"
#include "fsm/logger.hpp"

#include <algorithm>
#include <chrono>
#include <random>
#include <sstream>

namespace fsm {
namespace cloud {

// ---------------------------------------------------------------------------
// Session helpers
// ---------------------------------------------------------------------------

double Session::duration_s() const {
  const int64_t end = (ended_at_ns > 0) ? ended_at_ns : SessionManager::NowNs();
  return static_cast<double>(end - started_at_ns) / 1e9;
}

// ---------------------------------------------------------------------------
// SessionManager
// ---------------------------------------------------------------------------

SessionManager::SessionManager(int session_timeout_s)
    : session_timeout_s_(session_timeout_s) {}

SessionManager::~SessionManager() = default;

// static
int64_t SessionManager::NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

// static — generates a base-16 random session ID (128-bit entropy).
std::string SessionManager::GenerateSessionId() {
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dist;
  const uint64_t hi = dist(gen);
  const uint64_t lo = dist(gen);
  char buf[33];
  std::snprintf(buf, sizeof(buf), "%016llx%016llx",
      static_cast<unsigned long long>(hi),
      static_cast<unsigned long long>(lo));
  return std::string(buf);
}

std::optional<Session> SessionManager::CreateSession(
    const std::string& operator_id,
    const std::string& vehicle_id) {
  std::lock_guard<std::mutex> lk(mu_);

  // Enforce single-active-session-per-vehicle.
  for (const auto& [sid, s] : sessions_) {
    if (s.vehicle_id == vehicle_id && s.state == SessionState::kActive) {
      FSM_LOG_WARN("SessionManager: vehicle {} already has active session {}",
                   vehicle_id, sid);
      return std::nullopt;
    }
  }

  Session s;
  s.session_id   = GenerateSessionId();
  s.operator_id  = operator_id;
  s.vehicle_id   = vehicle_id;
  s.state        = SessionState::kActive;
  s.started_at_ns = NowNs();
  s.last_ping_ns  = s.started_at_ns;

  sessions_[s.session_id] = s;
  FSM_LOG_INFO("SessionManager: created session {} (op={} veh={})",
               s.session_id, operator_id, vehicle_id);

  if (on_created_) on_created_(s);
  return s;
}

bool SessionManager::Heartbeat(const std::string& session_id) {
  std::lock_guard<std::mutex> lk(mu_);
  auto it = sessions_.find(session_id);
  if (it == sessions_.end() || it->second.state != SessionState::kActive) {
    return false;
  }
  it->second.last_ping_ns = NowNs();
  return true;
}

bool SessionManager::TerminateSession(const std::string& session_id) {
  std::lock_guard<std::mutex> lk(mu_);
  auto it = sessions_.find(session_id);
  if (it == sessions_.end()) return false;
  if (it->second.state != SessionState::kActive) return false;

  it->second.state      = SessionState::kTerminated;
  it->second.ended_at_ns = NowNs();
  FSM_LOG_INFO("SessionManager: terminated session {} (op={} veh={} dur={:.1f}s)",
               session_id, it->second.operator_id, it->second.vehicle_id,
               it->second.duration_s());

  if (on_terminated_) on_terminated_(it->second);
  return true;
}

void SessionManager::TerminateVehicleSessions(const std::string& vehicle_id) {
  std::lock_guard<std::mutex> lk(mu_);
  for (auto& [sid, s] : sessions_) {
    if (s.vehicle_id == vehicle_id && s.state == SessionState::kActive) {
      s.state       = SessionState::kTerminated;
      s.ended_at_ns = NowNs();
      FSM_LOG_INFO("SessionManager: terminated session {} for disconnected vehicle {}",
                   sid, vehicle_id);
      if (on_terminated_) on_terminated_(s);
    }
  }
}

std::optional<Session> SessionManager::FindSession(
    const std::string& session_id) const {
  std::lock_guard<std::mutex> lk(mu_);
  auto it = sessions_.find(session_id);
  if (it == sessions_.end()) return std::nullopt;
  return it->second;
}

std::vector<Session> SessionManager::GetActiveSessions() const {
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<Session> result;
  result.reserve(sessions_.size());
  for (const auto& [sid, s] : sessions_) {
    if (s.state == SessionState::kActive) result.push_back(s);
  }
  return result;
}

std::vector<Session> SessionManager::GetAllSessions(size_t limit) const {
  std::lock_guard<std::mutex> lk(mu_);
  std::vector<Session> result;
  result.reserve(sessions_.size());
  for (const auto& [sid, s] : sessions_) result.push_back(s);
  std::sort(result.begin(), result.end(),
      [](const Session& a, const Session& b) {
        return a.started_at_ns > b.started_at_ns;
      });
  if (result.size() > limit) result.resize(limit);
  return result;
}

std::optional<Session> SessionManager::GetActiveSessionForVehicle(
    const std::string& vehicle_id) const {
  std::lock_guard<std::mutex> lk(mu_);
  for (const auto& [sid, s] : sessions_) {
    if (s.vehicle_id == vehicle_id && s.state == SessionState::kActive) {
      return s;
    }
  }
  return std::nullopt;
}

void SessionManager::ExpireStale() {
  const int64_t cutoff_ns =
      NowNs() - static_cast<int64_t>(session_timeout_s_) * 1'000'000'000LL;

  std::lock_guard<std::mutex> lk(mu_);
  for (auto& [sid, s] : sessions_) {
    if (s.state == SessionState::kActive && s.last_ping_ns < cutoff_ns) {
      s.state       = SessionState::kExpired;
      s.ended_at_ns = NowNs();
      FSM_LOG_WARN("SessionManager: session {} expired (op={} veh={})",
                   sid, s.operator_id, s.vehicle_id);
      if (on_expired_) on_expired_(s);
    }
  }
}

size_t SessionManager::active_session_count() const {
  std::lock_guard<std::mutex> lk(mu_);
  size_t n = 0;
  for (const auto& [sid, s] : sessions_) {
    if (s.state == SessionState::kActive) ++n;
  }
  return n;
}

}  // namespace cloud
}  // namespace fsm
