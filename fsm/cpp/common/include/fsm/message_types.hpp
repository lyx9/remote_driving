/**
 * FSM-Pilot 消息类型定义
 * 定义系统内部使用的消息结构
 */

#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <chrono>
#include <nlohmann/json.hpp>

namespace fsm {
namespace common {

/**
 * 车辆状态
 */
enum class VehicleStatus {
    UNKNOWN = 0,
    IDLE = 1,
    ACTIVE = 2,
    EMERGENCY = 3,
    OFFLINE = 4
};

/**
 * 车辆类型
 */
enum class VehicleType {
    ROBO_TAXI = 0,
    LOGISTICS = 1,
    SECURITY = 2
};

/**
 * 档位
 */
enum class Gear {
    PARK = 0,
    REVERSE = 1,
    NEUTRAL = 2,
    DRIVE = 3
};

/**
 * 网络质量
 */
enum class NetworkQuality {
    EXCELLENT = 0,
    GOOD = 1,
    FAIR = 2,
    POOR = 3
};

/**
 * 告警级别
 */
enum class AlertSeverity {
    INFO = 0,
    WARNING = 1,
    CRITICAL = 2,
    EMERGENCY = 3
};

/**
 * 位置信息
 */
struct Location {
    double latitude;
    double longitude;
    double altitude;
    double heading;

    nlohmann::json toJson() const {
        return {
            {"lat", latitude},
            {"lng", longitude},
            {"alt", altitude},
            {"heading", heading}
        };
    }

    static Location fromJson(const nlohmann::json& j) {
        Location loc;
        loc.latitude = j.value("lat", 0.0);
        loc.longitude = j.value("lng", 0.0);
        loc.altitude = j.value("alt", 0.0);
        loc.heading = j.value("heading", 0.0);
        return loc;
    }
};

/**
 * 遥测数据
 */
struct TelemetryData {
    float cpu_usage;
    float gpu_usage;
    float memory_usage;
    NetworkQuality network_quality;
    int signal_strength;

    nlohmann::json toJson() const {
        return {
            {"cpu_usage", cpu_usage},
            {"gpu_usage", gpu_usage},
            {"memory_usage", memory_usage},
            {"network_quality", static_cast<int>(network_quality)},
            {"signal_strength", signal_strength}
        };
    }
};

/**
 * 传感器状态
 */
struct SensorStatus {
    int cameras_online;
    int cameras_total;
    bool lidar_online;
    bool gps_fix;

    nlohmann::json toJson() const {
        return {
            {"cameras_online", cameras_online},
            {"cameras_total", cameras_total},
            {"lidar_online", lidar_online},
            {"gps_fix", gps_fix}
        };
    }
};

/**
 * 车辆状态消息
 */
struct VehicleStatusMessage {
    std::string vehicle_id;
    VehicleStatus status;
    VehicleType type;
    Location location;
    float speed;
    float steering;
    Gear gear;
    int battery_level;
    int emergency_level;
    float latency_ms;
    TelemetryData telemetry;
    SensorStatus sensors;
    uint64_t timestamp;

    nlohmann::json toJson() const {
        return {
            {"vehicle_id", vehicle_id},
            {"status", static_cast<int>(status)},
            {"type", static_cast<int>(type)},
            {"location", location.toJson()},
            {"speed", speed},
            {"steering", steering},
            {"gear", static_cast<int>(gear)},
            {"battery_level", battery_level},
            {"emergency_level", emergency_level},
            {"latency_ms", latency_ms},
            {"telemetry", telemetry.toJson()},
            {"sensors", sensors.toJson()},
            {"timestamp", timestamp}
        };
    }
};

/**
 * 控制指令
 */
struct ControlCommand {
    std::string vehicle_id;
    float steering;      // -1.0 to 1.0
    float throttle;      // 0.0 to 1.0
    float brake;         // 0.0 to 1.0
    Gear gear;
    int turn_signal;     // -1: left, 0: off, 1: right
    bool emergency;
    uint64_t sequence;
    uint64_t timestamp;

    nlohmann::json toJson() const {
        return {
            {"vehicle_id", vehicle_id},
            {"steering", steering},
            {"throttle", throttle},
            {"brake", brake},
            {"gear", static_cast<int>(gear)},
            {"turn_signal", turn_signal},
            {"emergency", emergency},
            {"sequence", sequence},
            {"timestamp", timestamp}
        };
    }

    static ControlCommand fromJson(const nlohmann::json& j) {
        ControlCommand cmd;
        cmd.vehicle_id = j.value("vehicle_id", "");
        cmd.steering = j.value("steering", 0.0f);
        cmd.throttle = j.value("throttle", 0.0f);
        cmd.brake = j.value("brake", 0.0f);
        cmd.gear = static_cast<Gear>(j.value("gear", 0));
        cmd.turn_signal = j.value("turn_signal", 0);
        cmd.emergency = j.value("emergency", false);
        cmd.sequence = j.value("sequence", 0ULL);
        cmd.timestamp = j.value("timestamp", 0ULL);
        return cmd;
    }
};

/**
 * 告警消息
 */
struct AlertMessage {
    std::string id;
    std::string vehicle_id;
    AlertSeverity severity;
    std::string message;
    std::string category;
    uint64_t timestamp;
    bool acknowledged;

    nlohmann::json toJson() const {
        return {
            {"id", id},
            {"vehicle_id", vehicle_id},
            {"severity", static_cast<int>(severity)},
            {"message", message},
            {"category", category},
            {"timestamp", timestamp},
            {"acknowledged", acknowledged}
        };
    }
};

/**
 * 调度队列项
 */
struct SchedulingQueueItem {
    std::string vehicle_id;
    float priority_score;
    int emergency_level;
    float latency_ms;
    float distance_km;

    nlohmann::json toJson() const {
        return {
            {"vehicle_id", vehicle_id},
            {"priority", priority_score},
            {"emergency_level", emergency_level},
            {"latency_ms", latency_ms},
            {"distance_km", distance_km}
        };
    }
};

/**
 * 心跳消息
 */
struct PingMessage {
    uint64_t sequence;
    uint64_t timestamp;

    nlohmann::json toJson() const {
        return {
            {"type", "ping"},
            {"sequence", sequence},
            {"timestamp", timestamp}
        };
    }
};

struct PongMessage {
    uint64_t sequence;
    uint64_t client_timestamp;
    uint64_t server_timestamp;

    nlohmann::json toJson() const {
        return {
            {"type", "pong"},
            {"sequence", sequence},
            {"client_timestamp", client_timestamp},
            {"server_timestamp", server_timestamp}
        };
    }
};

/**
 * WebSocket 事件类型
 */
enum class WSEventType {
    VEHICLE_STATUS,
    CONTROL_COMMAND,
    ALERT,
    SCHEDULING_UPDATE,
    EMERGENCY,
    LATENCY_UPDATE,
    PING,
    PONG
};

/**
 * 工具函数: 获取当前时间戳 (毫秒)
 */
inline uint64_t getCurrentTimestampMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

/**
 * 工具函数: 获取当前时间戳 (微秒)
 */
inline uint64_t getCurrentTimestampUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

} // namespace common
} // namespace fsm
