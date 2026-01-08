/**
 * FSM-Pilot 信令服务器头文件
 */

#pragma once

#include "fsm/config_manager.hpp"
#include <memory>
#include <string>

namespace fsm {
namespace cloud {

class WebSocketSession;

/**
 * WebSocket 信令服务器
 * 用于 WebRTC 连接协商
 */
class SignalingServer {
public:
    explicit SignalingServer(const config::CloudConfigManager& config);
    ~SignalingServer();

    // 禁止复制
    SignalingServer(const SignalingServer&) = delete;
    SignalingServer& operator=(const SignalingServer&) = delete;

    /**
     * 启动服务器
     */
    bool start();

    /**
     * 停止服务器
     */
    void stop();

    /**
     * 会话管理
     */
    void addSession(const std::string& id, std::shared_ptr<WebSocketSession> session);
    void removeSession(const std::string& id);
    std::shared_ptr<WebSocketSession> getSession(const std::string& id);

    /**
     * 查找车辆会话
     */
    std::shared_ptr<WebSocketSession> findVehicleSession(const std::string& vehicle_id);

    /**
     * 通知车辆上线
     */
    void notifyVehicleOnline(const std::string& vehicle_id);

    /**
     * 广播消息给所有操作端
     */
    void broadcastToOperators(const std::string& message);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace cloud
}  // namespace fsm
