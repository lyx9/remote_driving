#pragma once

#include <chrono>
#include <functional>
#include <fstream>
#include <mutex>
#include <string>

#include <nlohmann/json.hpp>

namespace fsm {

// ---------------------------------------------------------------------------
// AuditLogger
//
// Writes structured JSON audit records to a dedicated file.
// Each record is a single JSON object on one line (NDJSON / JSON Lines).
//
// Thread-safe.  Designed to be a singleton (or per-service instance).
//
// Usage:
//   AuditLogger::Init("/var/log/fsm/audit.log");
//   AuditLogger::Log("session.created",
//       {{"operator_id", "op-1"}, {"vehicle_id", "v-001"}});
//   AuditLogger::Shutdown();
// ---------------------------------------------------------------------------
class AuditLogger {
 public:
  // Opens the audit log file.  Subsequent calls reopen (for log rotation).
  // If path is empty, records are discarded silently.
  static void Init(const std::string& path);
  static void Shutdown();

  // Write an audit record.
  //   event   — dot-separated event name, e.g. "session.created"
  //   details — arbitrary JSON object with event-specific fields
  //   actor   — who performed the action (operator_id, system, …)
  static void Log(const std::string& event,
                  const nlohmann::json& details = {},
                  const std::string& actor = "system");

  // Convenience overloads.
  static void LogSessionCreated(const std::string& session_id,
                                 const std::string& operator_id,
                                 const std::string& vehicle_id);
  static void LogSessionTerminated(const std::string& session_id,
                                    const std::string& reason,
                                    double duration_s);
  static void LogAuthFailure(const std::string& remote_addr,
                              const std::string& reason);
  static void LogAlertFired(const std::string& alert_id,
                             const std::string& vehicle_id,
                             const std::string& type,
                             const std::string& severity);
  static void LogVehicleConnected(const std::string& vehicle_id);
  static void LogVehicleDisconnected(const std::string& vehicle_id,
                                      const std::string& reason);
  static void LogCommandForwarded(const std::string& vehicle_id,
                                   const std::string& operator_id,
                                   const std::string& command_type);

 private:
  static int64_t NowMs();

  static std::mutex     mu_;
  static std::ofstream  file_;
};

}  // namespace fsm
