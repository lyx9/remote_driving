/**
 * FSM-Pilot 消息类型实现
 * 提供消息序列化和转换的辅助函数
 */

#include "fsm/message_types.hpp"
#include "fsm/utils.hpp"
#include <cstring>
#include <sstream>
#include <iomanip>

namespace fsm {
namespace common {

// 状态枚举到字符串的转换
std::string vehicleStatusToString(VehicleStatus status) {
    switch (status) {
        case VehicleStatus::UNKNOWN: return "unknown";
        case VehicleStatus::IDLE: return "idle";
        case VehicleStatus::ACTIVE: return "active";
        case VehicleStatus::EMERGENCY: return "emergency";
        case VehicleStatus::OFFLINE: return "offline";
        default: return "unknown";
    }
}

VehicleStatus stringToVehicleStatus(const std::string& str) {
    if (str == "idle") return VehicleStatus::IDLE;
    if (str == "active") return VehicleStatus::ACTIVE;
    if (str == "emergency") return VehicleStatus::EMERGENCY;
    if (str == "offline") return VehicleStatus::OFFLINE;
    return VehicleStatus::UNKNOWN;
}

std::string gearToString(Gear gear) {
    switch (gear) {
        case Gear::PARK: return "P";
        case Gear::REVERSE: return "R";
        case Gear::NEUTRAL: return "N";
        case Gear::DRIVE: return "D";
        default: return "P";
    }
}

Gear stringToGear(const std::string& str) {
    if (str == "P" || str == "park") return Gear::PARK;
    if (str == "R" || str == "reverse") return Gear::REVERSE;
    if (str == "N" || str == "neutral") return Gear::NEUTRAL;
    if (str == "D" || str == "drive") return Gear::DRIVE;
    return Gear::PARK;
}

std::string networkQualityToString(NetworkQuality quality) {
    switch (quality) {
        case NetworkQuality::EXCELLENT: return "excellent";
        case NetworkQuality::GOOD: return "good";
        case NetworkQuality::FAIR: return "fair";
        case NetworkQuality::POOR: return "poor";
        default: return "unknown";
    }
}

std::string alertSeverityToString(AlertSeverity severity) {
    switch (severity) {
        case AlertSeverity::INFO: return "info";
        case AlertSeverity::WARNING: return "warning";
        case AlertSeverity::CRITICAL: return "critical";
        case AlertSeverity::EMERGENCY: return "emergency";
        default: return "info";
    }
}

// 生成唯一 ID
std::string generateAlertId() {
    static uint64_t counter = 0;
    std::stringstream ss;
    ss << "alert_" << getCurrentTimestampMs() << "_" << (++counter);
    return ss.str();
}

}  // namespace common
}  // namespace fsm
