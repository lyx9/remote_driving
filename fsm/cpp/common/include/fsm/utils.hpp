#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace fsm {
namespace utils {

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------

int64_t NowNanos();
int64_t NowMillis();
double  NowSeconds();

// Formats a millisecond timestamp as "YYYY-MM-DD HH:MM:SS.mmm".
std::string FormatTimestampMs(int64_t timestamp_ms);

// ---------------------------------------------------------------------------
// Math
// ---------------------------------------------------------------------------

inline constexpr double kPi = 3.14159265358979323846;

inline double DegToRad(double deg) { return deg * kPi / 180.0; }
inline double RadToDeg(double rad) { return rad * 180.0 / kPi; }

template <typename T>
T Clamp(T value, T lo, T hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

inline double Lerp(double a, double b, double t) {
  return a + (b - a) * Clamp(t, 0.0, 1.0);
}

inline double EuclideanDistance(double x1, double y1, double x2, double y2) {
  return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Haversine formula. Returns distance in metres.
double GpsDistance(double lat1, double lon1, double lat2, double lon2);

// Converts quaternion (x,y,z,w) to Euler angles (roll, pitch, yaw) in radians.
void QuaternionToEuler(
    double qx, double qy, double qz, double qw,
    double& roll, double& pitch, double& yaw);

// ---------------------------------------------------------------------------
// String
// ---------------------------------------------------------------------------

std::vector<std::string> SplitString(const std::string& str, char delimiter);
std::string TrimString(const std::string& str);
std::string ToLower(const std::string& str);
std::string GenerateUuid();

// ---------------------------------------------------------------------------
// MovingAverage
// ---------------------------------------------------------------------------

class MovingAverage {
 public:
  explicit MovingAverage(size_t window = 10) : window_(window) {}

  void Add(double value);
  double Get() const;
  void Reset();

 private:
  size_t window_;
  std::deque<double> values_;
  double sum_ = 0.0;
};

// ---------------------------------------------------------------------------
// RateLimiter
// ---------------------------------------------------------------------------

class RateLimiter {
 public:
  explicit RateLimiter(double rate_hz)
      : period_ns_(static_cast<int64_t>(1e9 / rate_hz)), last_ns_(0) {}

  // Returns true if enough time has elapsed since the last allowed call.
  bool Allow();
  void Reset() { last_ns_ = 0; }

 private:
  int64_t period_ns_;
  int64_t last_ns_;
};

}  // namespace utils
}  // namespace fsm
