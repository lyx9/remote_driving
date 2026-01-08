#pragma once

#include <string>
#include <vector>
#include <deque>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace fsm {
namespace utils {

/**
 * @brief 时间工具类
 */
class TimeUtils {
public:
    /**
     * @brief 获取当前时间戳 (纳秒)
     */
    static int64_t nowNanos() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    /**
     * @brief 获取当前时间戳 (毫秒)
     */
    static int64_t nowMillis() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    /**
     * @brief 获取当前时间戳 (秒)
     */
    static double nowSeconds() {
        return static_cast<double>(nowNanos()) / 1e9;
    }

    /**
     * @brief 格式化时间戳为字符串
     */
    static std::string formatTimestamp(int64_t timestamp_ms) {
        auto time_t_val = static_cast<time_t>(timestamp_ms / 1000);
        auto tm = *std::localtime(&time_t_val);
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
        oss << "." << std::setfill('0') << std::setw(3) << (timestamp_ms % 1000);
        return oss.str();
    }
};

/**
 * @brief 数学工具类
 */
class MathUtils {
public:
    static constexpr double PI = 3.14159265358979323846;

    /**
     * @brief 角度转弧度
     */
    static double degToRad(double deg) {
        return deg * PI / 180.0;
    }

    /**
     * @brief 弧度转角度
     */
    static double radToDeg(double rad) {
        return rad * 180.0 / PI;
    }

    /**
     * @brief 将值限制在指定范围内
     */
    template<typename T>
    static T clamp(T value, T min, T max) {
        if (value < min) return min;
        if (value > max) return max;
        return value;
    }

    /**
     * @brief 线性插值
     */
    static double lerp(double a, double b, double t) {
        return a + (b - a) * clamp(t, 0.0, 1.0);
    }

    /**
     * @brief 计算两点之间的欧几里得距离
     */
    static double distance(double x1, double y1, double x2, double y2) {
        return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    /**
     * @brief 计算两个GPS坐标之间的距离 (Haversine公式)
     * @return 距离 (米)
     */
    static double gpsDistance(double lat1, double lon1, double lat2, double lon2) {
        constexpr double R = 6371000.0;  // 地球半径 (米)

        double lat1_rad = degToRad(lat1);
        double lat2_rad = degToRad(lat2);
        double delta_lat = degToRad(lat2 - lat1);
        double delta_lon = degToRad(lon2 - lon1);

        double a = std::sin(delta_lat / 2) * std::sin(delta_lat / 2) +
                   std::cos(lat1_rad) * std::cos(lat2_rad) *
                   std::sin(delta_lon / 2) * std::sin(delta_lon / 2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

        return R * c;
    }

    /**
     * @brief 四元数转欧拉角 (RPY)
     */
    static void quaternionToEuler(
        double qx, double qy, double qz, double qw,
        double& roll, double& pitch, double& yaw) {

        // Roll (x-axis rotation)
        double sinr_cosp = 2 * (qw * qx + qy * qz);
        double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = 2 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(PI / 2, sinp);
        else
            pitch = std::asin(sinp);

        // Yaw (z-axis rotation)
        double siny_cosp = 2 * (qw * qz + qx * qy);
        double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }
};

/**
 * @brief 字符串工具类
 */
class StringUtils {
public:
    /**
     * @brief 分割字符串
     */
    static std::vector<std::string> split(const std::string& str, char delimiter) {
        std::vector<std::string> tokens;
        std::istringstream iss(str);
        std::string token;
        while (std::getline(iss, token, delimiter)) {
            if (!token.empty()) {
                tokens.push_back(token);
            }
        }
        return tokens;
    }

    /**
     * @brief 去除首尾空白字符
     */
    static std::string trim(const std::string& str) {
        size_t start = str.find_first_not_of(" \t\n\r");
        size_t end = str.find_last_not_of(" \t\n\r");
        if (start == std::string::npos) return "";
        return str.substr(start, end - start + 1);
    }

    /**
     * @brief 转换为小写
     */
    static std::string toLower(const std::string& str) {
        std::string result = str;
        for (auto& c : result) {
            c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
        return result;
    }

    /**
     * @brief 生成UUID
     */
    static std::string generateUUID();
};

/**
 * @brief 移动平均计算器
 */
class MovingAverage {
public:
    explicit MovingAverage(size_t window_size = 10)
        : window_size_(window_size) {}

    void add(double value) {
        if (values_.size() >= window_size_) {
            sum_ -= values_.front();
            values_.pop_front();
        }
        values_.push_back(value);
        sum_ += value;
    }

    double get() const {
        if (values_.empty()) return 0.0;
        return sum_ / static_cast<double>(values_.size());
    }

    void reset() {
        values_.clear();
        sum_ = 0.0;
    }

private:
    size_t window_size_;
    std::deque<double> values_;
    double sum_ = 0.0;
};

/**
 * @brief 速率限制器
 */
class RateLimiter {
public:
    explicit RateLimiter(double rate_hz)
        : period_ns_(static_cast<int64_t>(1e9 / rate_hz)),
          last_time_(0) {}

    bool shouldProcess() {
        int64_t now = TimeUtils::nowNanos();
        if (now - last_time_ >= period_ns_) {
            last_time_ = now;
            return true;
        }
        return false;
    }

    void reset() {
        last_time_ = 0;
    }

private:
    int64_t period_ns_;
    int64_t last_time_;
};

}  // namespace utils
}  // namespace fsm
