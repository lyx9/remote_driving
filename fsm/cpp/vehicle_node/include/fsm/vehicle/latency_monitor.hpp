/**
 * FSM-Pilot Latency Monitor
 * 延迟监控模块头文件
 */

#pragma once

#include <string>
#include <functional>
#include <memory>
#include <chrono>
#include <deque>
#include <map>
#include <mutex>
#include <atomic>

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
    double jitter_ms;           // 抖动 (标准差)
    double packet_loss_rate;    // 丢包率 (0.0 - 1.0)
    uint64_t sample_count;      // 样本数量
};

/**
 * 延迟监控回调
 */
using LatencyCallback = std::function<void(const LatencyStats&)>;
using AlertCallback = std::function<void(const std::string& alert_type, double value)>;

/**
 * 延迟监控器
 * 用于监控车端到云端的网络延迟
 */
class LatencyMonitor {
public:
    LatencyMonitor();
    ~LatencyMonitor();

    /**
     * 启动监控
     */
    void start();

    /**
     * 停止监控
     */
    void stop();

    /**
     * 记录发送的 ping，返回序列号
     */
    uint64_t recordPingSent();

    /**
     * 记录收到的 pong
     */
    void recordPongReceived(uint64_t sequence);

    /**
     * 获取当前延迟统计
     */
    LatencyStats getStats() const;

    /**
     * 获取当前 RTT (毫秒)
     */
    double getCurrentRttMs() const;

    /**
     * 获取平均 RTT (毫秒)
     */
    double getAverageRttMs() const;

    /**
     * 设置延迟更新回调
     */
    void setLatencyCallback(LatencyCallback callback);

    /**
     * 设置告警回调
     */
    void setAlertCallback(AlertCallback callback);

    /**
     * 设置告警阈值 (毫秒)
     */
    void setAlertThreshold(double threshold_ms);

    /**
     * 设置滑动窗口大小
     */
    void setWindowSize(size_t size);

    /**
     * 重置统计数据
     */
    void reset();

    /**
     * 获取网络质量描述
     */
    std::string getNetworkQuality() const;

private:
    void addSample(double rtt_ms);
    void checkAlerts(double rtt_ms);
    void cleanupOldPings();

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
