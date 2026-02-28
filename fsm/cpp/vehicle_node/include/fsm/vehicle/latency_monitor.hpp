#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>

namespace fsm {
namespace vehicle {

struct LatencyStats {
  double   current_rtt_ms   = 0.0;
  double   average_rtt_ms   = 0.0;
  double   min_rtt_ms       = 0.0;
  double   max_rtt_ms       = 0.0;
  double   jitter_ms        = 0.0;
  double   packet_loss_rate = 0.0;
  uint64_t sample_count     = 0;
};

class LatencyMonitor {
 public:
  using LatencyCallback = std::function<void(const LatencyStats&)>;
  using AlertCallback   = std::function<void(const std::string&, double)>;

  explicit LatencyMonitor(size_t window = 100, double alert_threshold_ms = 200.0)
      : window_size_(window), alert_threshold_ms_(alert_threshold_ms) {}

  ~LatencyMonitor() { Stop(); }

  LatencyMonitor(const LatencyMonitor&) = delete;
  LatencyMonitor& operator=(const LatencyMonitor&) = delete;

  void Start() { running_ = true; Reset(); }
  void Stop()  { running_ = false; }

  // Records a sent ping; returns the sequence number to pass to RecordPong.
  uint64_t RecordPing() {
    if (!running_) return 0;
    std::lock_guard<std::mutex> lock(mutex_);
    const uint64_t seq = ++sequence_;
    pending_pings_[seq] = std::chrono::steady_clock::now();
    ++packets_sent_;
    CleanupOldPings();
    return seq;
  }

  // Records a received pong for the given sequence number.
  void RecordPong(uint64_t sequence) {
    if (!running_) return;
    const auto now = std::chrono::steady_clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = pending_pings_.find(sequence);
    if (it == pending_pings_.end()) return;
    const double rtt_ms = std::chrono::duration_cast<std::chrono::microseconds>(
        now - it->second).count() / 1000.0;
    pending_pings_.erase(it);
    ++packets_received_;
    AddSample(rtt_ms);
    CheckAlerts(rtt_ms);
  }

  LatencyStats GetStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    LatencyStats s;
    s.sample_count = samples_.size();
    if (samples_.empty()) return s;

    s.current_rtt_ms = samples_.back();
    s.min_rtt_ms = *std::min_element(samples_.begin(), samples_.end());
    s.max_rtt_ms = *std::max_element(samples_.begin(), samples_.end());

    double sum = 0;
    for (double v : samples_) sum += v;
    s.average_rtt_ms = sum / samples_.size();

    double sq_sum = 0;
    for (double v : samples_) sq_sum += (v - s.average_rtt_ms) * (v - s.average_rtt_ms);
    s.jitter_ms = std::sqrt(sq_sum / samples_.size());

    s.packet_loss_rate = (packets_sent_ > 0)
        ? 1.0 - static_cast<double>(packets_received_) / packets_sent_
        : 0.0;
    return s;
  }

  double current_rtt_ms() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_.empty() ? 0.0 : samples_.back();
  }

  double average_rtt_ms() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.empty()) return 0.0;
    double sum = 0;
    for (double v : samples_) sum += v;
    return sum / samples_.size();
  }

  // Returns "excellent" / "good" / "fair" / "poor".
  std::string NetworkQuality() const {
    const double rtt = average_rtt_ms();
    if (rtt < 50)  return "excellent";
    if (rtt < 100) return "good";
    if (rtt < 200) return "fair";
    return "poor";
  }

  void Reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
    pending_pings_.clear();
    sequence_         = 0;
    packets_sent_     = 0;
    packets_received_ = 0;
  }

  void SetWindowSize(size_t size) {
    std::lock_guard<std::mutex> lock(mutex_);
    window_size_ = size;
    while (samples_.size() > window_size_) samples_.pop_front();
  }

  void set_latency_callback(LatencyCallback cb) { latency_cb_ = std::move(cb); }
  void set_alert_callback(AlertCallback cb)     { alert_cb_   = std::move(cb); }
  void set_alert_threshold_ms(double ms)        { alert_threshold_ms_ = ms; }

 private:
  void AddSample(double rtt_ms) {
    samples_.push_back(rtt_ms);
    while (samples_.size() > window_size_) samples_.pop_front();
    if (latency_cb_) {
      LatencyStats s;
      s.current_rtt_ms = rtt_ms;
      s.sample_count   = samples_.size();
      latency_cb_(s);
    }
  }

  void CheckAlerts(double rtt_ms) {
    if (rtt_ms > alert_threshold_ms_ && alert_cb_) {
      alert_cb_("high_latency", rtt_ms);
    }
  }

  void CleanupOldPings() {
    const auto cutoff = std::chrono::steady_clock::now() - std::chrono::seconds(5);
    for (auto it = pending_pings_.begin(); it != pending_pings_.end(); ) {
      it = (it->second < cutoff) ? pending_pings_.erase(it) : std::next(it);
    }
  }

  mutable std::mutex    mutex_;
  std::atomic<bool>     running_{false};
  std::atomic<uint64_t> sequence_{0};
  std::atomic<uint64_t> packets_sent_{0};
  std::atomic<uint64_t> packets_received_{0};

  std::deque<double> samples_;
  std::map<uint64_t, std::chrono::steady_clock::time_point> pending_pings_;

  size_t window_size_;
  double alert_threshold_ms_;

  LatencyCallback latency_cb_;
  AlertCallback   alert_cb_;
};

}  // namespace vehicle
}  // namespace fsm
