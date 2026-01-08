/**
 * FSM-Pilot 云端服务器
 * 主入口点
 */

#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"
#include "fsm/cloud/scheduling_service.hpp"
#include "fsm/cloud/signaling_server.hpp"
#include "fsm/cloud/alert_analyzer.hpp"
#include "fsm/cloud/prediction_interface.hpp"

#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

// 全局运行标志
std::atomic<bool> g_running{true};

// 信号处理
void signalHandler(int signum) {
    LOG_INFO("Received signal " + std::to_string(signum) + ", shutting down...");
    g_running = false;
}

// REST API 服务器 (简化实现)
class RestApiServer {
public:
    RestApiServer(int port,
                  fsm::cloud::SchedulingService& scheduler,
                  fsm::cloud::AlertAnalyzer& alert_analyzer,
                  fsm::cloud::PredictionInterface& prediction)
        : port_(port)
        , scheduler_(scheduler)
        , alert_analyzer_(alert_analyzer)
        , prediction_(prediction)
    {
    }

    bool start() {
        LOG_INFO("[REST API] Starting on port " + std::to_string(port_));
        // 实际实现需要使用 HTTP 库如 cpp-httplib
        // 这里是占位符
        return true;
    }

    void stop() {
        LOG_INFO("[REST API] Stopped");
    }

private:
    int port_;
    fsm::cloud::SchedulingService& scheduler_;
    fsm::cloud::AlertAnalyzer& alert_analyzer_;
    fsm::cloud::PredictionInterface& prediction_;
};

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " <config_file>\n"
              << "\n"
              << "FSM-Pilot Cloud Server\n"
              << "远程驾驶平台云端服务\n"
              << "\n"
              << "Arguments:\n"
              << "  config_file    Path to cloud_config.yaml\n"
              << "\n"
              << "Example:\n"
              << "  " << program << " config/cloud_config.yaml\n";
}

int main(int argc, char* argv[]) {
    // 参数检查
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string config_path = argv[1];

    // 初始化日志
    fsm::Logger::getInstance().setLevel(fsm::LogLevel::DEBUG);
    fsm::Logger::getInstance().setPrefix("[FSM-Cloud]");

    LOG_INFO("========================================");
    LOG_INFO("  FSM-Pilot Cloud Server v1.1.0");
    LOG_INFO("  远程驾驶平台云端服务");
    LOG_INFO("========================================");

    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // 加载配置
    LOG_INFO("Loading configuration from: " + config_path);
    fsm::config::CloudConfigManager config;
    if (!config.loadFromFile(config_path)) {
        LOG_ERROR("Failed to load configuration");
        return 1;
    }
    LOG_INFO("Configuration loaded successfully");

    // 创建服务
    LOG_INFO("Initializing services...");

    // 1. 调度服务
    fsm::cloud::SchedulingService scheduler(config);
    scheduler.start();
    LOG_INFO("Scheduling service started");

    // 2. 告警分析器
    fsm::cloud::AlertAnalyzer alert_analyzer(config);

    // 设置告警回调
    alert_analyzer.onAlert([](const fsm::cloud::Alert& alert) {
        std::string severity_str;
        switch (alert.severity) {
            case fsm::cloud::AlertSeverity::INFO: severity_str = "INFO"; break;
            case fsm::cloud::AlertSeverity::WARNING: severity_str = "WARNING"; break;
            case fsm::cloud::AlertSeverity::CRITICAL: severity_str = "CRITICAL"; break;
        }
        LOG_WARNING("[ALERT] " + severity_str + " - " + alert.vehicle_id +
                   ": " + alert.message);
    });

    alert_analyzer.onAlertResolved([](const fsm::cloud::Alert& alert) {
        LOG_INFO("[ALERT RESOLVED] " + alert.vehicle_id + ": " + alert.type);
    });

    LOG_INFO("Alert analyzer initialized");

    // 3. 预测接口
    fsm::cloud::PredictionInterface prediction(config);
    LOG_INFO("Prediction interface initialized (enabled: " +
             std::string(prediction.isEnabled() ? "yes" : "no") + ")");

    // 4. 信令服务器
    fsm::cloud::SignalingServer signaling(config);
    if (!signaling.start()) {
        LOG_ERROR("Failed to start signaling server");
        return 1;
    }
    LOG_INFO("Signaling server started on port " +
             std::to_string(config.getServerConfig().signaling_port));

    // 5. REST API 服务器
    RestApiServer api_server(config.getServerConfig().api_port,
                             scheduler, alert_analyzer, prediction);
    if (!api_server.start()) {
        LOG_ERROR("Failed to start REST API server");
        return 1;
    }
    LOG_INFO("REST API server started on port " +
             std::to_string(config.getServerConfig().api_port));

    LOG_INFO("========================================");
    LOG_INFO("  All services started successfully");
    LOG_INFO("  Press Ctrl+C to shutdown");
    LOG_INFO("========================================");

    // 状态监控线程
    std::thread monitor_thread([&]() {
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::seconds(30));

            if (!g_running) break;

            // 打印状态摘要
            auto queue = scheduler.getSchedulingQueue();
            auto alert_stats = alert_analyzer.getStatistics();

            LOG_INFO("--- Status Summary ---");
            LOG_INFO("Connected vehicles: " + std::to_string(queue.size()));
            LOG_INFO("Active alerts: " + std::to_string(alert_stats.total_alerts) +
                    " (Critical: " + std::to_string(alert_stats.critical_count) +
                    ", Warning: " + std::to_string(alert_stats.warning_count) + ")");

            // 打印调度队列
            if (!queue.empty()) {
                LOG_DEBUG("Scheduling queue:");
                for (size_t i = 0; i < std::min(queue.size(), size_t(5)); i++) {
                    const auto& v = queue[i];
                    LOG_DEBUG("  " + std::to_string(i + 1) + ". " + v.vehicle_id +
                             " (Priority: " + std::to_string(static_cast<int>(v.priority_score)) +
                             ", Latency: " + std::to_string(static_cast<int>(v.latency_ms)) + "ms)");
                }
            }
        }
    });

    // 主循环
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 处理车辆状态更新和告警分析
        auto queue = scheduler.getSchedulingQueue();
        for (const auto& vehicle : queue) {
            fsm::cloud::VehicleState state;
            state.latency_ms = vehicle.latency_ms;
            state.battery_level = static_cast<int>(vehicle.battery_level);
            state.emergency_active = (vehicle.emergency_level >= 4);
            state.last_update_time = vehicle.last_update_time;

            // 分析告警
            alert_analyzer.analyze(vehicle.vehicle_id, state);
        }
    }

    // 关闭
    LOG_INFO("Shutting down services...");

    monitor_thread.join();
    api_server.stop();
    signaling.stop();
    scheduler.stop();

    LOG_INFO("Cloud server shutdown complete");
    return 0;
}
