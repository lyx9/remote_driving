/**
 * FSM-Pilot 操作端客户端
 * 主入口点
 */

#include "fsm/operator/operator_client.hpp"
#include "fsm/logger.hpp"

#include <iostream>
#include <csignal>
#include <atomic>

// 全局运行标志
std::atomic<bool> g_running{true};

// 信号处理
void signalHandler(int signum) {
    LOG_INFO("Received signal " + std::to_string(signum) + ", shutting down...");
    g_running = false;
}

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " [options]\n"
              << "\n"
              << "FSM-Pilot Operator Client\n"
              << "远程驾驶操作端\n"
              << "\n"
              << "Options:\n"
              << "  -s, --signaling URL    Signaling server URL\n"
              << "  -v, --vehicle ID       Vehicle ID to connect\n"
              << "  -h, --help             Show this help\n"
              << "\n"
              << "Example:\n"
              << "  " << program << " -s wss://signal.example.com/ws -v FSM-01\n";
}

int main(int argc, char* argv[]) {
    // 默认配置
    fsm::operator_client::OperatorConfig config;
    config.signaling_url = "wss://localhost:8080/signaling";
    std::string vehicle_id;

    // 解析命令行参数
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if ((arg == "-s" || arg == "--signaling") && i + 1 < argc) {
            config.signaling_url = argv[++i];
        } else if ((arg == "-v" || arg == "--vehicle") && i + 1 < argc) {
            vehicle_id = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
    }

    // 初始化日志
    fsm::Logger::getInstance().setLevel(fsm::LogLevel::DEBUG);
    fsm::Logger::getInstance().setPrefix("[FSM-Operator]");

    LOG_INFO("========================================");
    LOG_INFO("  FSM-Pilot Operator Client v1.1.0");
    LOG_INFO("  远程驾驶操作端");
    LOG_INFO("========================================");

    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // 创建操作端客户端
    fsm::operator_client::OperatorClient client(config);

    // 初始化
    if (!client.initialize()) {
        LOG_ERROR("Failed to initialize operator client");
        return 1;
    }

    // 设置回调
    client.setControlCallback([](const fsm::operator_client::ControlCommand& cmd) {
        // 控制指令已发送
    });

    client.setTelemetryCallback([](const fsm::operator_client::TelemetryData& data) {
        // 显示遥测数据
        LOG_DEBUG("Telemetry - Speed: " + std::to_string(data.speed) +
                 " km/h, Gear: " + std::to_string(data.gear));
    });

    // 启动
    client.start();

    LOG_INFO("========================================");
    LOG_INFO("  Operator client started");
    LOG_INFO("  Press Ctrl+C to exit");
    LOG_INFO("========================================");

    // 如果指定了车辆，自动连接
    if (!vehicle_id.empty()) {
        LOG_INFO("Connecting to vehicle: " + vehicle_id);
        if (client.connect(vehicle_id)) {
            LOG_INFO("Connected to vehicle successfully");
        } else {
            LOG_ERROR("Failed to connect to vehicle");
        }
    } else {
        LOG_INFO("No vehicle specified. Use -v option to connect.");
    }

    // 主循环
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 状态显示
        auto status = client.getStatus();
        if (status.wheel_connected) {
            // 方向盘已连接
        }
    }

    // 关闭
    LOG_INFO("Shutting down...");
    client.stop();

    LOG_INFO("Operator client shutdown complete");
    return 0;
}
