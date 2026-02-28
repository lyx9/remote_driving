#include "fsm/operator/operator_client.hpp"
#include "fsm/logger.hpp"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

namespace {
std::atomic<bool> g_running{true};

void SignalHandler(int /*signum*/) {
  g_running = false;
}

void PrintUsage(const char* program) {
  std::fprintf(stderr,
      "Usage: %s [options]\n"
      "  -s, --signaling URL    Signaling server URL\n"
      "  -v, --vehicle  ID      Vehicle ID to connect\n"
      "  -h, --help             Show this help\n",
      program);
}
}  // namespace

int main(int argc, char* argv[]) {
  fsm::operator_client::OperatorConfig config;
  config.signaling_url = "wss://localhost:8080/signaling";
  std::string vehicle_id;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if ((arg == "-s" || arg == "--signaling") && i + 1 < argc) {
      config.signaling_url = argv[++i];
    } else if ((arg == "-v" || arg == "--vehicle") && i + 1 < argc) {
      vehicle_id = argv[++i];
    } else if (arg == "-h" || arg == "--help") {
      PrintUsage(argv[0]);
      return 0;
    }
  }

  fsm::Logger::Init("fsm-operator", "", fsm::Logger::Level::kInfo);
  FSM_LOG_INFO("FSM-Pilot Operator Client starting");

  std::signal(SIGINT,  SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  fsm::operator_client::OperatorClient client(config);

  if (!client.Initialize()) {
    FSM_LOG_ERROR("Failed to initialize operator client");
    return 1;
  }

  client.set_control_callback([](const fsm::operator_client::ControlCommand&) {});
  client.set_telemetry_callback([](const fsm::operator_client::TelemetryData& data) {
    FSM_LOG_DEBUG("Telemetry: speed={:.1f} gear={}", data.speed, data.gear);
  });

  client.Start();
  FSM_LOG_INFO("Operator client started — press Ctrl+C to exit");

  if (!vehicle_id.empty()) {
    if (client.Connect(vehicle_id)) {
      FSM_LOG_INFO("Connected to vehicle: {}", vehicle_id);
    } else {
      FSM_LOG_ERROR("Failed to connect to vehicle: {}", vehicle_id);
    }
  }

  while (g_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  FSM_LOG_INFO("Shutting down...");
  client.Stop();
  fsm::Logger::Shutdown();
  return 0;
}
