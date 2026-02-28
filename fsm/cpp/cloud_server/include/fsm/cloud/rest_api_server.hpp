#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "fsm/config_manager.hpp"
#include "fsm/cloud/scheduling_service.hpp"
#include "fsm/cloud/alert_analyzer.hpp"
#include "fsm/cloud/session_manager.hpp"

namespace fsm {
namespace cloud {

// ---------------------------------------------------------------------------
// Boost.Beast HTTP REST API server.
//
// Endpoints:
//   GET  /api/v1/health                    — liveness probe (unauthenticated)
//   GET  /metrics                          — Prometheus text format (unauthenticated)
//   GET  /api/v1/vehicles                  — list all connected vehicles
//   GET  /api/v1/vehicles/{id}             — single vehicle status
//   GET  /api/v1/vehicles/{id}/diagnostics — vehicle debug dashboard (health, alerts, session)
//   GET  /api/v1/queue                     — scheduling priority queue
//   GET  /api/v1/alerts                    — active alerts
//   POST /api/v1/alerts/{id}/ack           — acknowledge alert
//   POST /api/v1/sessions                  — create operator–vehicle session
//   GET  /api/v1/sessions                  — list active sessions
//   GET  /api/v1/sessions/{id}             — single session detail
//   DELETE /api/v1/sessions/{id}           — terminate session
//
// Authentication:
//   When jwt_secret is non-empty all /api/v1/* endpoints (except /health)
//   require a valid "Authorization: Bearer <token>" header.
//   Tokens are validated by HMAC-SHA256 signature + exp claim.
// ---------------------------------------------------------------------------
class RestApiServer {
 public:
  RestApiServer(const config::CloudConfigManager& config,
                SchedulingService& scheduler,
                AlertAnalyzer&     alert_analyzer,
                SessionManager&    session_manager);
  ~RestApiServer();

  RestApiServer(const RestApiServer&) = delete;
  RestApiServer& operator=(const RestApiServer&) = delete;

  [[nodiscard]] bool Start();
  void Stop();

  // Increment counters from other components.
  void RecordTelemetryReceived();
  void RecordCommandForwarded();
  void RecordAlertFired();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace cloud
}  // namespace fsm
