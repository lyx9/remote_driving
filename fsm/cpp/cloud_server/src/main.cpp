#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"
#include "fsm/audit_logger.hpp"
#include "fsm/cloud/scheduling_service.hpp"
#include "fsm/cloud/signaling_server.hpp"
#include "fsm/cloud/alert_analyzer.hpp"
#include "fsm/cloud/session_manager.hpp"
#include "fsm/cloud/rest_api_server.hpp"
#include "fsm/cloud/mqtt_relay.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_running{true};

void SignalHandler(int /*signum*/) {
  g_running = false;
}
}  // namespace

static void PrintUsage(const char* program) {
  std::fprintf(stderr, "Usage: %s <config_file>\n"
               "  config_file  Path to cloud_config.yaml\n", program);
}

int main(int argc, char* argv[]) {
  if (argc < 2) {
    PrintUsage(argv[0]);
    return 1;
  }

  fsm::Logger::Init("fsm-cloud", "", fsm::Logger::Level::kInfo);
  FSM_LOG_INFO("FSM-Pilot Cloud Server starting");

  std::signal(SIGINT,  SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  fsm::config::CloudConfigManager config;
  if (!config.LoadFromFile(argv[1])) {
    FSM_LOG_ERROR("Failed to load config: {}", argv[1]);
    return 1;
  }

  // Audit log
  fsm::AuditLogger::Init("/var/log/fsm/audit.log");

  // Core services
  fsm::cloud::SchedulingService scheduler(config);
  scheduler.Start();

  fsm::cloud::AlertAnalyzer alert_analyzer(config);
  alert_analyzer.set_alert_callback([](const fsm::cloud::Alert& alert) {
    const char* sev =
        (alert.severity == fsm::cloud::AlertSeverity::kCritical) ? "CRITICAL" :
        (alert.severity == fsm::cloud::AlertSeverity::kWarning)  ? "WARNING"  : "INFO";
    FSM_LOG_WARN("[ALERT] {} - {}: {}", sev, alert.vehicle_id, alert.message);
    fsm::AuditLogger::LogAlertFired(
        alert.id, alert.vehicle_id, alert.type, sev);
  });
  alert_analyzer.set_resolved_callback([](const fsm::cloud::Alert& alert) {
    FSM_LOG_INFO("[ALERT RESOLVED] {}: {}", alert.vehicle_id, alert.type);
  });

  // Session manager
  const int session_timeout_s = (config.session_timeout_s() > 0)
      ? config.session_timeout_s() : 300;
  fsm::cloud::SessionManager session_manager(session_timeout_s);
  session_manager.set_on_session_created([](const fsm::cloud::Session& s) {
    fsm::AuditLogger::LogSessionCreated(
        s.session_id, s.operator_id, s.vehicle_id);
  });
  session_manager.set_on_session_terminated([](const fsm::cloud::Session& s) {
    fsm::AuditLogger::LogSessionTerminated(
        s.session_id, "operator_request", s.duration_s());
  });
  session_manager.set_on_session_expired([](const fsm::cloud::Session& s) {
    fsm::AuditLogger::LogSessionTerminated(
        s.session_id, "expired", s.duration_s());
  });

  // WebSocket signaling server
  fsm::cloud::SignalingServer signaling(config);

  // REST API server (also holds metrics counters)
  fsm::cloud::RestApiServer rest_api(config, scheduler, alert_analyzer,
                                     session_manager);

  // Wire telemetry relay into scheduling + alert pipeline
  signaling.set_telemetry_callback(
      [&scheduler, &alert_analyzer, &rest_api](
          const std::string& vehicle_id,
          const nlohmann::json& payload) {
        const auto& d = payload.value("data", nlohmann::json::object());

        fsm::cloud::VehicleSchedulingInfo vi;
        vi.vehicle_id     = vehicle_id;
        vi.latency_ms     = d.value("latency_ms",  0.0f);
        vi.battery_pct    = d.value("battery_pct", 100.0f);
        vi.speed_mps      = d.value("speed_mps",   0.0f);
        vi.is_connected   = true;
        vi.task_status    = "ACTIVE";
        vi.last_update_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        scheduler.UpdateVehicleState(vi);

        fsm::cloud::VehicleHealthSnapshot snap;
        snap.latency_ms     = vi.latency_ms;
        snap.battery_pct    = static_cast<int>(vi.battery_pct);
        snap.last_update_ns = vi.last_update_ns;
        alert_analyzer.Analyze(vehicle_id, snap);

        rest_api.RecordTelemetryReceived();
      });

  // Vehicle connect / disconnect audit events
  signaling.set_connect_callback([&session_manager](const std::string& vehicle_id) {
    FSM_LOG_INFO("Vehicle connected: {}", vehicle_id);
    fsm::AuditLogger::LogVehicleConnected(vehicle_id);
  });
  signaling.set_disconnect_callback(
      [&scheduler, &session_manager](const std::string& vehicle_id) {
        FSM_LOG_INFO("Vehicle disconnected: {}", vehicle_id);
        fsm::AuditLogger::LogVehicleDisconnected(vehicle_id, "connection_closed");
        scheduler.RemoveVehicle(vehicle_id);
        session_manager.TerminateVehicleSessions(vehicle_id);
      });

  // Optional MQTT relay
  std::unique_ptr<fsm::cloud::MqttRelay> mqtt_relay;
  if (config.cloud_mqtt_config().enabled) {
    mqtt_relay = std::make_unique<fsm::cloud::MqttRelay>(
        config.cloud_mqtt_config());
    if (mqtt_relay->Start()) {
      FSM_LOG_INFO("MQTT relay started: {}", config.cloud_mqtt_config().broker_url);
    } else {
      FSM_LOG_WARN("MQTT relay failed to start — continuing without MQTT");
      mqtt_relay.reset();
    }
  }

  if (!signaling.Start()) {
    FSM_LOG_ERROR("Failed to start signaling server on port {}",
                  config.signaling_port());
    return 1;
  }

  if (!rest_api.Start()) {
    FSM_LOG_WARN("REST API failed to start on port {}", config.api_port());
    // Non-fatal — continue without REST API
  }

  FSM_LOG_INFO("Signaling: ws://0.0.0.0:{}", config.signaling_port());
  FSM_LOG_INFO("REST API:  http://0.0.0.0:{}/api/v1", config.api_port());
  FSM_LOG_INFO("Metrics:   http://0.0.0.0:{}/metrics", config.api_port());
  FSM_LOG_INFO("All services started — press Ctrl+C to stop");

  // Status + session expiry monitor thread (30-second heartbeat)
  std::thread monitor([&]() {
    while (g_running) {
      std::this_thread::sleep_for(std::chrono::seconds(30));
      if (!g_running) break;
      session_manager.ExpireStale();
      const auto queue = scheduler.GetSchedulingQueue();
      const auto stats = alert_analyzer.GetStatistics();
      FSM_LOG_INFO("Status: vehicles={} sessions={} alerts={} (crit={} warn={})",
                   queue.size(), session_manager.active_session_count(),
                   stats.total, stats.critical, stats.warning);
      for (size_t i = 0; i < std::min(queue.size(), size_t{5}); ++i) {
        const auto& v = queue[i];
        FSM_LOG_DEBUG("  {}. {} priority={:.1f} latency={:.0f}ms",
                      i + 1, v.vehicle_id, v.priority_score, v.latency_ms);
      }
    }
  });

  while (g_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  FSM_LOG_INFO("Shutting down...");
  monitor.join();
  rest_api.Stop();
  signaling.Stop();
  if (mqtt_relay) mqtt_relay->Stop();
  scheduler.Stop();
  fsm::AuditLogger::Shutdown();
  fsm::Logger::Shutdown();
  return 0;
}
