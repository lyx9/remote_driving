#include "fsm/utils.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <random>
#include <sstream>

namespace fsm {
namespace utils {

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------

int64_t NowNanos() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

int64_t NowMillis() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

double NowSeconds() {
  return static_cast<double>(NowNanos()) / 1e9;
}

std::string FormatTimestampMs(int64_t timestamp_ms) {
  auto time_val = static_cast<std::time_t>(timestamp_ms / 1000);
  auto tm = *std::localtime(&time_val);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
  oss << "." << std::setfill('0') << std::setw(3) << (timestamp_ms % 1000);
  return oss.str();
}

// ---------------------------------------------------------------------------
// Math
// ---------------------------------------------------------------------------

double GpsDistance(double lat1, double lon1, double lat2, double lon2) {
  constexpr double kEarthRadiusM = 6371000.0;
  const double lat1_r = DegToRad(lat1);
  const double lat2_r = DegToRad(lat2);
  const double dlat = DegToRad(lat2 - lat1);
  const double dlon = DegToRad(lon2 - lon1);
  const double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
                   std::cos(lat1_r) * std::cos(lat2_r) *
                   std::sin(dlon / 2) * std::sin(dlon / 2);
  return kEarthRadiusM * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

void QuaternionToEuler(
    double qx, double qy, double qz, double qw,
    double& roll, double& pitch, double& yaw) {
  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (qw * qy - qz * qx);
  pitch = (std::abs(sinp) >= 1.0)
              ? std::copysign(kPi / 2.0, sinp)
              : std::asin(sinp);

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

// ---------------------------------------------------------------------------
// String
// ---------------------------------------------------------------------------

std::vector<std::string> SplitString(const std::string& str, char delimiter) {
  std::vector<std::string> tokens;
  std::istringstream iss(str);
  std::string token;
  while (std::getline(iss, token, delimiter)) {
    if (!token.empty()) tokens.push_back(token);
  }
  return tokens;
}

std::string TrimString(const std::string& str) {
  const size_t start = str.find_first_not_of(" \t\n\r");
  if (start == std::string::npos) return "";
  const size_t end = str.find_last_not_of(" \t\n\r");
  return str.substr(start, end - start + 1);
}

std::string ToLower(const std::string& str) {
  std::string result = str;
  for (auto& c : result) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return result;
}

std::string GenerateUuid() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<> dis(0, 15);
  static const char kHex[] = "0123456789abcdef";

  std::ostringstream oss;
  for (int i = 0; i < 8; ++i) oss << kHex[dis(gen)];
  oss << "-";
  for (int i = 0; i < 4; ++i) oss << kHex[dis(gen)];
  oss << "-4";
  for (int i = 0; i < 3; ++i) oss << kHex[dis(gen)];
  oss << "-" << kHex[(dis(gen) & 0x3) | 0x8];
  for (int i = 0; i < 3; ++i) oss << kHex[dis(gen)];
  oss << "-";
  for (int i = 0; i < 12; ++i) oss << kHex[dis(gen)];
  return oss.str();
}

// ---------------------------------------------------------------------------
// MovingAverage
// ---------------------------------------------------------------------------

void MovingAverage::Add(double value) {
  if (values_.size() >= window_) {
    sum_ -= values_.front();
    values_.pop_front();
  }
  values_.push_back(value);
  sum_ += value;
}

double MovingAverage::Get() const {
  if (values_.empty()) return 0.0;
  return sum_ / static_cast<double>(values_.size());
}

void MovingAverage::Reset() {
  values_.clear();
  sum_ = 0.0;
}

// ---------------------------------------------------------------------------
// RateLimiter
// ---------------------------------------------------------------------------

bool RateLimiter::Allow() {
  const int64_t now = NowNanos();
  if (now - last_ns_ >= period_ns_) {
    last_ns_ = now;
    return true;
  }
  return false;
}

}  // namespace utils
}  // namespace fsm
