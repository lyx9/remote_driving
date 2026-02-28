#include "fsm/audit_logger.hpp"
#include "fsm/logger.hpp"

#include <chrono>

namespace fsm {

// Static member definitions.
std::mutex    AuditLogger::mu_;
std::ofstream AuditLogger::file_;

// static
int64_t AuditLogger::NowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

// static
void AuditLogger::Init(const std::string& path) {
  std::lock_guard<std::mutex> lk(mu_);
  if (path.empty()) return;
  file_.open(path, std::ios::app);
  if (!file_.is_open()) {
    FSM_LOG_WARN("AuditLogger: failed to open {}", path);
  } else {
    FSM_LOG_INFO("AuditLogger: writing to {}", path);
  }
}

// static
void AuditLogger::Shutdown() {
  std::lock_guard<std::mutex> lk(mu_);
  if (file_.is_open()) file_.close();
}

// static
void AuditLogger::Log(const std::string& event,
                      const nlohmann::json& details,
                      const std::string& actor) {
  nlohmann::json record;
  record["ts_ms"]  = NowMs();
  record["event"]  = event;
  record["actor"]  = actor;
  if (!details.is_null() && !details.empty()) {
    record["details"] = details;
  }

  const std::string line = record.dump() + "\n";

  std::lock_guard<std::mutex> lk(mu_);
  if (file_.is_open()) {
    file_ << line;
    file_.flush();
  }
}

// static
void AuditLogger::LogSessionCreated(const std::string& session_id,
                                     const std::string& operator_id,
                                     const std::string& vehicle_id) {
  Log("session.created",
      {{"session_id", session_id},
       {"vehicle_id", vehicle_id}},
      operator_id);
}

// static
void AuditLogger::LogSessionTerminated(const std::string& session_id,
                                        const std::string& reason,
                                        double duration_s) {
  Log("session.terminated",
      {{"session_id", session_id},
       {"reason", reason},
       {"duration_s", duration_s}});
}

// static
void AuditLogger::LogAuthFailure(const std::string& remote_addr,
                                  const std::string& reason) {
  Log("auth.failure",
      {{"remote_addr", remote_addr},
       {"reason", reason}});
}

// static
void AuditLogger::LogAlertFired(const std::string& alert_id,
                                 const std::string& vehicle_id,
                                 const std::string& type,
                                 const std::string& severity) {
  Log("alert.fired",
      {{"alert_id", alert_id},
       {"vehicle_id", vehicle_id},
       {"type", type},
       {"severity", severity}});
}

// static
void AuditLogger::LogVehicleConnected(const std::string& vehicle_id) {
  Log("vehicle.connected", {{"vehicle_id", vehicle_id}});
}

// static
void AuditLogger::LogVehicleDisconnected(const std::string& vehicle_id,
                                          const std::string& reason) {
  Log("vehicle.disconnected",
      {{"vehicle_id", vehicle_id}, {"reason", reason}});
}

// static
void AuditLogger::LogCommandForwarded(const std::string& vehicle_id,
                                       const std::string& operator_id,
                                       const std::string& command_type) {
  Log("command.forwarded",
      {{"vehicle_id", vehicle_id},
       {"command_type", command_type}},
      operator_id);
}

}  // namespace fsm
