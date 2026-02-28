#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace fsm {
namespace cloud {

// ---------------------------------------------------------------------------
// Session lifecycle states.
// ---------------------------------------------------------------------------
enum class SessionState {
  kActive,      // operator and vehicle paired, control channel open
  kTerminated,  // cleanly closed
  kExpired,     // timed out (no heartbeat)
};

// ---------------------------------------------------------------------------
// A single operator–vehicle session.
// ---------------------------------------------------------------------------
struct Session {
  std::string session_id;    // UUID-like unique ID
  std::string operator_id;   // e.g. "op-0042" or authenticated user sub
  std::string vehicle_id;    // connected vehicle
  SessionState state = SessionState::kActive;
  int64_t started_at_ns  = 0;  // std::chrono::system_clock nanoseconds
  int64_t last_ping_ns   = 0;  // last heartbeat / activity
  int64_t ended_at_ns    = 0;  // set when state != kActive

  // Derived helpers
  double duration_s() const;
  bool is_active()    const { return state == SessionState::kActive; }
};

// ---------------------------------------------------------------------------
// Session event callbacks.
// ---------------------------------------------------------------------------
using SessionCallback = std::function<void(const Session&)>;

// ---------------------------------------------------------------------------
// SessionManager
//
// Thread-safe. Each vehicle may have AT MOST one active session at a time;
// trying to create a second session while one is active returns nullopt.
// ---------------------------------------------------------------------------
class SessionManager {
 public:
  // Session expires after this many seconds with no Heartbeat() call.
  static constexpr int kDefaultSessionTimeoutS = 300;  // 5 min

  explicit SessionManager(int session_timeout_s = kDefaultSessionTimeoutS);
  ~SessionManager();

  SessionManager(const SessionManager&) = delete;
  SessionManager& operator=(const SessionManager&) = delete;

  // Returns current time as nanoseconds since epoch (system clock).
  // Public so Session::duration_s() can call it without friendship.
  static int64_t NowNs();

  // Creates a new active session for the operator→vehicle pair.
  // Returns nullopt if vehicle_id already has an active session.
  [[nodiscard]] std::optional<Session> CreateSession(
      const std::string& operator_id,
      const std::string& vehicle_id);

  // Updates last_ping_ns to now; returns false if session not found / inactive.
  bool Heartbeat(const std::string& session_id);

  // Terminates a session by ID. Returns false if not found.
  bool TerminateSession(const std::string& session_id);

  // Terminates all active sessions for a vehicle (e.g. on disconnect).
  void TerminateVehicleSessions(const std::string& vehicle_id);

  // Returns a copy of a session by ID, or nullopt if not found.
  std::optional<Session> FindSession(const std::string& session_id) const;

  // Returns all currently active sessions.
  std::vector<Session> GetActiveSessions() const;

  // Returns all sessions (including terminated/expired), newest first.
  std::vector<Session> GetAllSessions(size_t limit = 100) const;

  // Returns the active session for a vehicle, if any.
  std::optional<Session> GetActiveSessionForVehicle(
      const std::string& vehicle_id) const;

  // Sweep expired sessions; call periodically (e.g. every 30 s).
  void ExpireStale();

  size_t active_session_count() const;

  // Event callbacks — fired under internal lock; keep them brief.
  void set_on_session_created(SessionCallback cb) { on_created_ = std::move(cb); }
  void set_on_session_terminated(SessionCallback cb) { on_terminated_ = std::move(cb); }
  void set_on_session_expired(SessionCallback cb) { on_expired_ = std::move(cb); }

 private:
  static std::string GenerateSessionId();

  int session_timeout_s_;

  mutable std::mutex mu_;
  std::unordered_map<std::string, Session> sessions_;  // keyed by session_id

  SessionCallback on_created_;
  SessionCallback on_terminated_;
  SessionCallback on_expired_;
};

}  // namespace cloud
}  // namespace fsm
