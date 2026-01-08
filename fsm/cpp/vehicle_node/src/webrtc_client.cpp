/**
 * FSM-Pilot WebRTC 客户端实现
 * 基于 libdatachannel 实现 P2P 通信
 */

#include "fsm/vehicle/webrtc_client.hpp"
#include "fsm/logger.hpp"
#include <chrono>
#include <thread>

namespace fsm {
namespace vehicle {

WebRTCClient::WebRTCClient(const config::WebRTCConfig& config)
    : config_(config)
    , state_(ConnectionState::DISCONNECTED)
    , running_(false)
    , command_sequence_(0)
    , ping_sequence_(0)
{
}

WebRTCClient::~WebRTCClient() {
    disconnect();
}

bool WebRTCClient::connect() {
    if (state_ == ConnectionState::CONNECTED || state_ == ConnectionState::CONNECTING) {
        return true;
    }

    setState(ConnectionState::CONNECTING);
    LOG_INFO("[WebRTC] Connecting to signaling server: " + config_.signaling_url);

    try {
        // 1. 连接信令服务器
        if (!connectSignaling()) {
            LOG_ERROR("[WebRTC] Failed to connect signaling server");
            setState(ConnectionState::FAILED);
            return false;
        }

        // 2. 创建 PeerConnection
        if (!createPeerConnection()) {
            LOG_ERROR("[WebRTC] Failed to create peer connection");
            setState(ConnectionState::FAILED);
            return false;
        }

        // 3. 发送连接请求
        sendSignalingMessage({
            {"type", "connect"},
            {"vehicle_id", config_.vehicle_id}
        });

        running_ = true;

        // 启动心跳线程
        heartbeat_thread_ = std::thread([this]() {
            while (running_) {
                if (state_ == ConnectionState::CONNECTED) {
                    sendPing();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        });

        LOG_INFO("[WebRTC] Connection initiated");
        return true;

    } catch (const std::exception& e) {
        LOG_ERROR("[WebRTC] Connection error: " + std::string(e.what()));
        setState(ConnectionState::FAILED);
        return false;
    }
}

void WebRTCClient::disconnect() {
    running_ = false;

    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }

    // 关闭数据通道
    {
        std::lock_guard<std::mutex> lock(data_channel_mutex_);
        if (data_channel_) {
            data_channel_->close();
            data_channel_.reset();
        }
    }

    // 关闭 PeerConnection
    if (peer_connection_) {
        peer_connection_->close();
        peer_connection_.reset();
    }

    // 关闭信令 WebSocket
    if (signaling_ws_) {
        signaling_ws_->close();
        signaling_ws_.reset();
    }

    setState(ConnectionState::DISCONNECTED);
    LOG_INFO("[WebRTC] Disconnected");
}

bool WebRTCClient::sendTelemetry(const TelemetryData& data) {
    std::lock_guard<std::mutex> lock(data_channel_mutex_);

    if (!data_channel_ || !data_channel_->isOpen()) {
        return false;
    }

    try {
        nlohmann::json msg;
        msg["type"] = "telemetry";
        msg["sequence"] = ++command_sequence_;
        msg["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

        // 车辆状态
        msg["vehicle_status"] = {
            {"speed", data.speed},
            {"steering_angle", data.steering_angle},
            {"gear", data.gear},
            {"turn_signal", data.turn_signal}
        };

        // 定位信息
        msg["localization"] = {
            {"latitude", data.latitude},
            {"longitude", data.longitude},
            {"heading", data.heading},
            {"altitude", data.altitude}
        };

        // 系统状态
        msg["system"] = {
            {"cpu_usage", data.cpu_usage},
            {"gpu_usage", data.gpu_usage},
            {"memory_usage", data.memory_usage},
            {"battery_level", data.battery_level}
        };

        data_channel_->send(msg.dump());
        return true;

    } catch (const std::exception& e) {
        LOG_ERROR("[WebRTC] Failed to send telemetry: " + std::string(e.what()));
        return false;
    }
}

bool WebRTCClient::addVideoTrack(const std::string& camera_id,
                                  std::shared_ptr<VideoEncoder> encoder) {
    if (!peer_connection_) {
        LOG_ERROR("[WebRTC] Cannot add video track: no peer connection");
        return false;
    }

    try {
        // 创建视频轨道
        // 注意: 实际实现需要根据 libdatachannel 或其他 WebRTC 库的 API
        std::string track_id = "video_" + camera_id;

        // 配置视频编码参数
        rtc::Description::Video video_desc(track_id, rtc::Description::Direction::SendOnly);

        // 添加 H264 编解码器
        video_desc.addH264Codec(96);

        // 添加轨道到 PeerConnection
        auto track = peer_connection_->addTrack(video_desc);

        if (track) {
            video_tracks_[camera_id] = track;
            video_encoders_[camera_id] = encoder;
            LOG_INFO("[WebRTC] Added video track: " + camera_id);
            return true;
        }

    } catch (const std::exception& e) {
        LOG_ERROR("[WebRTC] Failed to add video track: " + std::string(e.what()));
    }

    return false;
}

bool WebRTCClient::sendVideoFrame(const std::string& camera_id,
                                   const uint8_t* data,
                                   size_t size,
                                   int64_t timestamp) {
    auto it = video_tracks_.find(camera_id);
    if (it == video_tracks_.end()) {
        return false;
    }

    try {
        // 发送编码后的视频帧
        auto track = it->second;
        if (track) {
            // 创建 RTP 包并发送
            // 实际实现依赖具体的 WebRTC 库
            rtc::binary frame(data, data + size);
            track->send(frame);
            return true;
        }
    } catch (const std::exception& e) {
        LOG_ERROR("[WebRTC] Failed to send video frame: " + std::string(e.what()));
    }

    return false;
}

void WebRTCClient::onConnectionState(ConnectionStateCallback callback) {
    connection_callback_ = callback;
}

void WebRTCClient::onControlCommand(ControlCommandCallback callback) {
    control_callback_ = callback;
}

void WebRTCClient::onLatencyUpdate(LatencyCallback callback) {
    latency_callback_ = callback;
}

ConnectionState WebRTCClient::getState() const {
    return state_;
}

LatencyInfo WebRTCClient::getLatencyInfo() const {
    std::lock_guard<std::mutex> lock(latency_mutex_);
    return latency_info_;
}

// Private methods

bool WebRTCClient::connectSignaling() {
    try {
        // 创建 WebSocket 连接
        signaling_ws_ = std::make_shared<rtc::WebSocket>();

        signaling_ws_->onOpen([this]() {
            LOG_INFO("[WebRTC] Signaling WebSocket opened");
        });

        signaling_ws_->onError([this](const std::string& error) {
            LOG_ERROR("[WebRTC] Signaling error: " + error);
            if (state_ == ConnectionState::CONNECTED) {
                setState(ConnectionState::RECONNECTING);
                scheduleReconnect();
            }
        });

        signaling_ws_->onClosed([this]() {
            LOG_INFO("[WebRTC] Signaling WebSocket closed");
            if (state_ == ConnectionState::CONNECTED) {
                setState(ConnectionState::RECONNECTING);
                scheduleReconnect();
            }
        });

        signaling_ws_->onMessage([this](rtc::message_variant message) {
            if (std::holds_alternative<std::string>(message)) {
                handleSignalingMessage(std::get<std::string>(message));
            }
        });

        signaling_ws_->open(config_.signaling_url);

        // 等待连接建立
        int timeout = 5000; // 5秒超时
        while (!signaling_ws_->isOpen() && timeout > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            timeout -= 100;
        }

        return signaling_ws_->isOpen();

    } catch (const std::exception& e) {
        LOG_ERROR("[WebRTC] Signaling connection error: " + std::string(e.what()));
        return false;
    }
}

bool WebRTCClient::createPeerConnection() {
    try {
        // 配置 ICE 服务器
        rtc::Configuration rtc_config;

        for (const auto& stun : config_.stun_servers) {
            rtc_config.iceServers.push_back({stun});
        }

        for (const auto& turn : config_.turn_servers) {
            rtc_config.iceServers.push_back({
                turn.url,
                turn.username,
                turn.credential
            });
        }

        // 创建 PeerConnection
        peer_connection_ = std::make_shared<rtc::PeerConnection>(rtc_config);

        // ICE 候选回调
        peer_connection_->onLocalCandidate([this](rtc::Candidate candidate) {
            sendSignalingMessage({
                {"type", "ice-candidate"},
                {"candidate", {
                    {"candidate", candidate.candidate()},
                    {"sdpMid", candidate.mid()},
                    {"sdpMLineIndex", 0}
                }}
            });
        });

        // ICE 状态变化
        peer_connection_->onStateChange([this](rtc::PeerConnection::State state) {
            LOG_INFO("[WebRTC] PeerConnection state: " + std::to_string(static_cast<int>(state)));

            switch (state) {
                case rtc::PeerConnection::State::Connected:
                    setState(ConnectionState::CONNECTED);
                    break;
                case rtc::PeerConnection::State::Disconnected:
                case rtc::PeerConnection::State::Failed:
                    if (state_ == ConnectionState::CONNECTED) {
                        setState(ConnectionState::RECONNECTING);
                        scheduleReconnect();
                    } else {
                        setState(ConnectionState::FAILED);
                    }
                    break;
                default:
                    break;
            }
        });

        // 数据通道回调
        peer_connection_->onDataChannel([this](std::shared_ptr<rtc::DataChannel> dc) {
            LOG_INFO("[WebRTC] Data channel received: " + dc->label());
            setupDataChannel(dc);
        });

        // 创建数据通道 (作为发起方)
        auto dc = peer_connection_->createDataChannel("control");
        setupDataChannel(dc);

        return true;

    } catch (const std::exception& e) {
        LOG_ERROR("[WebRTC] PeerConnection creation error: " + std::string(e.what()));
        return false;
    }
}

void WebRTCClient::setupDataChannel(std::shared_ptr<rtc::DataChannel> channel) {
    std::lock_guard<std::mutex> lock(data_channel_mutex_);
    data_channel_ = channel;

    channel->onOpen([this]() {
        LOG_INFO("[WebRTC] Data channel opened");
    });

    channel->onClosed([this]() {
        LOG_INFO("[WebRTC] Data channel closed");
    });

    channel->onMessage([this](rtc::message_variant message) {
        if (std::holds_alternative<std::string>(message)) {
            handleDataChannelMessage(std::get<std::string>(message));
        }
    });
}

void WebRTCClient::handleSignalingMessage(const std::string& message) {
    try {
        auto json = nlohmann::json::parse(message);
        std::string type = json["type"];

        if (type == "offer") {
            handleOffer(json["sdp"]);
        } else if (type == "answer") {
            handleAnswer(json["sdp"]);
        } else if (type == "ice-candidate") {
            handleIceCandidate(json["candidate"]);
        } else if (type == "error") {
            LOG_ERROR("[WebRTC] Signaling error: " + json["message"].get<std::string>());
            setState(ConnectionState::FAILED);
        }

    } catch (const std::exception& e) {
        LOG_ERROR("[WebRTC] Failed to parse signaling message: " + std::string(e.what()));
    }
}

void WebRTCClient::handleOffer(const std::string& sdp) {
    if (!peer_connection_) return;

    try {
        rtc::Description offer(sdp, rtc::Description::Type::Offer);
        peer_connection_->setRemoteDescription(offer);

        // 创建 Answer
        peer_connection_->setLocalDescription(rtc::Description::Type::Answer);
        auto answer = peer_connection_->localDescription();

        if (answer) {
            sendSignalingMessage({
                {"type", "answer"},
                {"sdp", std::string(*answer)}
            });
        }

    } catch (const std::exception& e) {
        LOG_ERROR("[WebRTC] Failed to handle offer: " + std::string(e.what()));
    }
}

void WebRTCClient::handleAnswer(const std::string& sdp) {
    if (!peer_connection_) return;

    try {
        rtc::Description answer(sdp, rtc::Description::Type::Answer);
        peer_connection_->setRemoteDescription(answer);

    } catch (const std::exception& e) {
        LOG_ERROR("[WebRTC] Failed to handle answer: " + std::string(e.what()));
    }
}

void WebRTCClient::handleIceCandidate(const nlohmann::json& candidate) {
    if (!peer_connection_) return;

    try {
        std::string cand = candidate["candidate"];
        std::string mid = candidate["sdpMid"];

        rtc::Candidate ice_candidate(cand, mid);
        peer_connection_->addRemoteCandidate(ice_candidate);

    } catch (const std::exception& e) {
        LOG_ERROR("[WebRTC] Failed to handle ICE candidate: " + std::string(e.what()));
    }
}

void WebRTCClient::handleDataChannelMessage(const std::string& message) {
    try {
        auto json = nlohmann::json::parse(message);
        std::string type = json["type"];

        if (type == "control") {
            handleControlCommand(json);
        } else if (type == "pong") {
            handlePong(json);
        }

    } catch (const std::exception& e) {
        LOG_ERROR("[WebRTC] Failed to parse data channel message: " + std::string(e.what()));
    }
}

void WebRTCClient::handleControlCommand(const nlohmann::json& json) {
    if (!control_callback_) return;

    ControlCommand cmd;
    cmd.sequence = json.value("sequence", 0);
    cmd.timestamp = json.value("timestamp", 0);
    cmd.steering = json.value("steering", 0.0f);
    cmd.throttle = json.value("throttle", 0.0f);
    cmd.brake = json.value("brake", 0.0f);
    cmd.gear = json.value("gear", 0);
    cmd.turn_signal = json.value("turn_signal", 0);
    cmd.emergency = json.value("emergency", false);

    control_callback_(cmd);
}

void WebRTCClient::handlePong(const nlohmann::json& json) {
    uint32_t sequence = json["sequence"];

    std::lock_guard<std::mutex> lock(latency_mutex_);

    auto it = ping_timestamps_.find(sequence);
    if (it != ping_timestamps_.end()) {
        auto now = std::chrono::steady_clock::now();
        auto rtt = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - it->second
        ).count();

        // 更新延迟信息
        double prev_rtt = latency_info_.rtt_ms;
        latency_info_.rtt_ms = rtt;

        // 计算抖动 (简化的 RFC 3550 算法)
        double diff = std::abs(static_cast<double>(rtt) - prev_rtt);
        latency_info_.jitter_ms = latency_info_.jitter_ms + (diff - latency_info_.jitter_ms) / 16.0;

        ping_timestamps_.erase(it);

        // 通知回调
        if (latency_callback_) {
            latency_callback_(latency_info_);
        }
    }
}

void WebRTCClient::sendSignalingMessage(const nlohmann::json& message) {
    if (signaling_ws_ && signaling_ws_->isOpen()) {
        signaling_ws_->send(message.dump());
    }
}

void WebRTCClient::sendPing() {
    std::lock_guard<std::mutex> lock(data_channel_mutex_);

    if (!data_channel_ || !data_channel_->isOpen()) {
        return;
    }

    uint32_t sequence = ++ping_sequence_;
    auto now = std::chrono::steady_clock::now();

    {
        std::lock_guard<std::mutex> lock(latency_mutex_);
        ping_timestamps_[sequence] = now;
    }

    nlohmann::json ping;
    ping["type"] = "ping";
    ping["sequence"] = sequence;
    ping["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    data_channel_->send(ping.dump());
}

void WebRTCClient::setState(ConnectionState state) {
    if (state_ != state) {
        state_ = state;
        if (connection_callback_) {
            connection_callback_(state);
        }
    }
}

void WebRTCClient::scheduleReconnect() {
    std::thread([this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(config_.reconnect_interval_ms));
        if (state_ == ConnectionState::RECONNECTING) {
            LOG_INFO("[WebRTC] Attempting reconnect...");
            disconnect();
            connect();
        }
    }).detach();
}

}  // namespace vehicle
}  // namespace fsm
