#pragma once

#include <string>
#include <memory>
#include <functional>
#include <mutex>
#include <atomic>
#include <thread>
#include <queue>
#include <condition_variable>

#include <opencv2/opencv.hpp>
#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"

namespace fsm {
namespace vehicle {

// Forward declarations
struct ControlCommand;

/**
 * @brief WebRTC连接状态
 */
enum class WebRTCState {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    RECONNECTING,
    FAILED
};

/**
 * @brief 延迟信息
 */
struct LatencyInfo {
    float rtt_ms = 0.0;
    float video_latency_ms = 0.0;
    float control_latency_ms = 0.0;
    float jitter_ms = 0.0;
    float packet_loss_rate = 0.0;
    int64_t timestamp_ns = 0;
};

/**
 * @brief WebRTC客户端配置
 */
struct WebRTCClientConfig {
    std::string vehicle_id;
    std::string signaling_url;
    std::vector<std::string> stun_servers;
    std::vector<std::string> turn_servers;
    std::string turn_username;
    std::string turn_password;

    int reconnect_interval_ms = 5000;
    int ice_timeout_ms = 10000;
    int heartbeat_interval_ms = 1000;

    // 视频编码参数
    std::string video_codec = "h264";
    int video_bitrate = 4000000;
    int video_fps = 30;
};

/**
 * @brief WebRTC事件回调
 */
using ConnectionStateCallback = std::function<void(WebRTCState)>;
using ControlCommandCallback = std::function<void(const ControlCommand&)>;
using LatencyUpdateCallback = std::function<void(const LatencyInfo&)>;

/**
 * @brief WebRTC客户端 (车端)
 *
 * 负责:
 * - 与信令服务器建立WebSocket连接
 * - 处理ICE候选交换
 * - 发送视频流 (H.264编码)
 * - 通过数据通道发送遥测数据
 * - 接收控制指令
 * - 延迟测量
 */
class WebRTCClient {
public:
    /**
     * @brief 构造函数
     * @param config WebRTC配置
     */
    explicit WebRTCClient(const WebRTCClientConfig& config);

    ~WebRTCClient();

    /**
     * @brief 连接到信令服务器
     * @return 是否成功发起连接
     */
    bool connect();

    /**
     * @brief 断开连接
     */
    void disconnect();

    /**
     * @brief 发送视频帧
     * @param camera_id 摄像头ID
     * @param frame OpenCV图像帧
     * @param timestamp_ns 时间戳
     */
    void sendVideoFrame(
        const std::string& camera_id,
        const cv::Mat& frame,
        int64_t timestamp_ns);

    /**
     * @brief 发送遥测数据
     * @param data 序列化的遥测数据 (Protocol Buffers)
     */
    void sendTelemetry(const std::vector<uint8_t>& data);

    /**
     * @brief 发送系统状态
     * @param data 序列化的系统状态数据
     */
    void sendSystemStatus(const std::vector<uint8_t>& data);

    /**
     * @brief 发送心跳
     */
    void sendHeartbeat();

    /**
     * @brief 获取当前连接状态
     */
    WebRTCState getState() const { return state_; }

    /**
     * @brief 获取延迟信息
     */
    LatencyInfo getLatencyInfo() const;

    /**
     * @brief 检查是否已连接
     */
    bool isConnected() const { return state_ == WebRTCState::CONNECTED; }

    /**
     * @brief 设置连接状态回调
     */
    void setConnectionStateCallback(ConnectionStateCallback callback);

    /**
     * @brief 设置控制指令回调
     */
    void setControlCommandCallback(ControlCommandCallback callback);

    /**
     * @brief 设置延迟更新回调
     */
    void setLatencyUpdateCallback(LatencyUpdateCallback callback);

private:
    // WebSocket处理
    void connectSignaling();
    void onSignalingMessage(const std::string& message);
    void onSignalingConnected();
    void onSignalingDisconnected();

    // WebRTC处理
    void createPeerConnection();
    void handleOffer(const std::string& sdp);
    void handleAnswer(const std::string& sdp);
    void handleIceCandidate(const std::string& candidate);
    void onIceConnectionStateChange(int state);
    void onDataChannelMessage(const std::vector<uint8_t>& data);

    // 视频编码
    void initVideoEncoders();
    std::vector<uint8_t> encodeFrame(const cv::Mat& frame);

    // 延迟测量
    void measureLatency();
    void onPongReceived(int64_t original_timestamp);

    // 重连逻辑
    void scheduleReconnect();
    void reconnectThread();

    // 配置
    WebRTCClientConfig config_;

    // 状态
    std::atomic<WebRTCState> state_{WebRTCState::DISCONNECTED};

    // 延迟信息
    mutable std::mutex latency_mutex_;
    LatencyInfo latency_info_;
    std::deque<float> rtt_samples_;

    // 回调
    ConnectionStateCallback connection_callback_;
    ControlCommandCallback control_callback_;
    LatencyUpdateCallback latency_callback_;

    // 线程
    std::unique_ptr<std::thread> reconnect_thread_;
    std::atomic<bool> should_reconnect_{false};
    std::condition_variable reconnect_cv_;
    std::mutex reconnect_mutex_;

    // 心跳
    std::unique_ptr<std::thread> heartbeat_thread_;
    std::atomic<bool> heartbeat_running_{false};
    uint64_t heartbeat_sequence_ = 0;
    int64_t last_pong_time_ = 0;

    // 序列号
    std::atomic<uint64_t> telemetry_sequence_{0};
    std::atomic<uint64_t> video_frame_sequence_{0};

    // TODO: 实际的WebRTC实现需要集成libdatachannel或其他WebRTC库
    // 以下为占位符
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace vehicle
}  // namespace fsm
