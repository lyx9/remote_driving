/**
 * FSM-Pilot Latency Monitor
 * 延迟监控模块 - 监控车端到云端的网络延迟
 */

#include <chrono>
#include <deque>
#include <mutex>
#include <atomic>
#include <cmath>
#include <algorithm>
#include <functional>

namespace fsm {
namespace vehicle {

/**
 * 延迟统计数据
 */
struct LatencyStats {
    double current_rtt_ms;      // 当前 RTT
    double average_rtt_ms;      // 平均 RTT
    double min_rtt_ms;          // 最小 RTT
    double max_rtt_ms;          // 最大 RTT
    double jitter_ms;           // 抖动
    double packet_loss_rate;    // 丢包率
    uint64_t sample_count;      // 样本数
};

/**
 * 延迟监控器
 */
class LatencyMonitor {
public:
    using LatencyCallback = std::function<void(const LatencyStats&)>;
    using AlertCallback = std::function<void(const std::string&, double)>;

    LatencyMonitor()
        : running_(false)
        , sequence_(0)
        , packets_sent_(0)
        , packets_received_(0)
        , window_size_(100)
        , alert_threshold_ms_(200.0)
    {
    }

    ~LatencyMonitor() {
        stop();
    }

    /**
     * 启动监控
     */
    void start() {
        running_ = true;
        reset();
    }

    /**
     * 停止监控
     */
    void stop() {
        running_ = false;
    }

    /**
     * 记录发送的 ping
     */
    uint64_t recordPingSent() {
        if (!running_) return 0;

        std::lock_guard<std::mutex> lock(mutex_);
        uint64_t seq = ++sequence_;
        auto now = std::chrono::steady_clock::now();

        pending_pings_[seq] = now;
        packets_sent_++;

        // 清理过期的 ping (超过 5 秒)
        cleanupOldPings();

        return seq;
    }

    /**
     * 记录收到的 pong
     */
    void recordPongReceived(uint64_t sequence) {
        if (!running_) return;

        auto now = std::chrono::steady_clock::now();

        std::lock_guard<std::mutex> lock(mutex_);

        auto it = pending_pings_.find(sequence);
        if (it == pending_pings_.end()) {
            return; // 已过期或重复
        }

        auto send_time = it->second;
        pending_pings_.erase(it);

        // 计算 RTT
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
            now - send_time);
        double rtt_ms = duration.count() / 1000.0;

        packets_received_++;
        addSample(rtt_ms);

        // 检查告警
        checkAlerts(rtt_ms);
    }

    /**
     * 获取当前延迟统计
     */
    LatencyStats getStats() const {
        std::lock_guard<std::mutex> lock(mutex_);

        LatencyStats stats;
        stats.sample_count = samples_.size();

        if (samples_.empty()) {
            stats.current_rtt_ms = 0;
            stats.average_rtt_ms = 0;
            stats.min_rtt_ms = 0;
            stats.max_rtt_ms = 0;
            stats.jitter_ms = 0;
            stats.packet_loss_rate = 0;
            return stats;
        }

        stats.current_rtt_ms = samples_.back();
        stats.min_rtt_ms = *std::min_element(samples_.begin(), samples_.end());
        stats.max_rtt_ms = *std::max_element(samples_.begin(), samples_.end());

        // 计算平均值
        double sum = 0;
        for (double s : samples_) {
            sum += s;
        }
        stats.average_rtt_ms = sum / samples_.size();

        // 计算抖动 (标准差)
        double sq_sum = 0;
        for (double s : samples_) {
            sq_sum += (s - stats.average_rtt_ms) * (s - stats.average_rtt_ms);
        }
        stats.jitter_ms = std::sqrt(sq_sum / samples_.size());

        // 计算丢包率
        if (packets_sent_ > 0) {
            stats.packet_loss_rate = 1.0 -
                (static_cast<double>(packets_received_) / packets_sent_);
        } else {
            stats.packet_loss_rate = 0;
        }

        return stats;
    }

    /**
     * 获取当前 RTT
     */
    double getCurrentRttMs() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return samples_.empty() ? 0 : samples_.back();
    }

    /**
     * 获取平均 RTT
     */
    double getAverageRttMs() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (samples_.empty()) return 0;

        double sum = 0;
        for (double s : samples_) {
            sum += s;
        }
        return sum / samples_.size();
    }

    /**
     * 设置延迟回调
     */
    void setLatencyCallback(LatencyCallback callback) {
        latency_callback_ = std::move(callback);
    }

    /**
     * 设置告警回调
     */
    void setAlertCallback(AlertCallback callback) {
        alert_callback_ = std::move(callback);
    }

    /**
     * 设置告警阈值
     */
    void setAlertThreshold(double threshold_ms) {
        alert_threshold_ms_ = threshold_ms;
    }

    /**
     * 设置滑动窗口大小
     */
    void setWindowSize(size_t size) {
        std::lock_guard<std::mutex> lock(mutex_);
        window_size_ = size;
        while (samples_.size() > window_size_) {
            samples_.pop_front();
        }
    }

    /**
     * 重置统计
     */
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        samples_.clear();
        pending_pings_.clear();
        sequence_ = 0;
        packets_sent_ = 0;
        packets_received_ = 0;
    }

    /**
     * 判断网络质量
     */
    std::string getNetworkQuality() const {
        double rtt = getAverageRttMs();

        if (rtt < 50) return "excellent";
        if (rtt < 100) return "good";
        if (rtt < 200) return "fair";
        return "poor";
    }

private:
    void addSample(double rtt_ms) {
        samples_.push_back(rtt_ms);

        while (samples_.size() > window_size_) {
            samples_.pop_front();
        }

        // 触发回调
        if (latency_callback_) {
            LatencyStats stats;
            stats.current_rtt_ms = rtt_ms;
            stats.average_rtt_ms = getAverageRttMs();
            stats.sample_count = samples_.size();
            latency_callback_(stats);
        }
    }

    void checkAlerts(double rtt_ms) {
        if (rtt_ms > alert_threshold_ms_ && alert_callback_) {
            alert_callback_("high_latency", rtt_ms);
        }
    }

    void cleanupOldPings() {
        auto now = std::chrono::steady_clock::now();
        auto timeout = std::chrono::seconds(5);

        for (auto it = pending_pings_.begin(); it != pending_pings_.end(); ) {
            if (now - it->second > timeout) {
                it = pending_pings_.erase(it);
            } else {
                ++it;
            }
        }
    }

private:
    mutable std::mutex mutex_;
    std::atomic<bool> running_;
    std::atomic<uint64_t> sequence_;
    std::atomic<uint64_t> packets_sent_;
    std::atomic<uint64_t> packets_received_;

    std::deque<double> samples_;
    std::map<uint64_t, std::chrono::steady_clock::time_point> pending_pings_;

    size_t window_size_;
    double alert_threshold_ms_;

    LatencyCallback latency_callback_;
    AlertCallback alert_callback_;
};

} // namespace vehicle
} // namespace fsm
