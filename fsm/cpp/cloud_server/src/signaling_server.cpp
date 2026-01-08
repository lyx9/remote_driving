/**
 * FSM-Pilot 信令服务器实现
 * 基于 WebSocket 的信令服务，用于 WebRTC 连接协商
 */

#include "fsm/cloud/signaling_server.hpp"
#include "fsm/logger.hpp"
#include <boost/asio.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <nlohmann/json.hpp>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <set>

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

namespace fsm {
namespace cloud {

// WebSocket 会话
class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
public:
    explicit WebSocketSession(tcp::socket&& socket, SignalingServer* server)
        : ws_(std::move(socket))
        , server_(server)
    {
    }

    ~WebSocketSession() {
        if (!client_id_.empty()) {
            server_->removeSession(client_id_);
        }
    }

    void run() {
        // 设置 WebSocket 选项
        ws_.set_option(websocket::stream_base::timeout::suggested(
            beast::role_type::server));

        ws_.set_option(websocket::stream_base::decorator(
            [](websocket::response_type& res) {
                res.set(beast::http::field::server, "FSM-Pilot Signaling Server");
            }));

        // 接受 WebSocket 握手
        ws_.async_accept(
            beast::bind_front_handler(
                &WebSocketSession::onAccept,
                shared_from_this()));
    }

    void send(const std::string& message) {
        auto self = shared_from_this();
        net::post(ws_.get_executor(),
            [self, message]() {
                self->doSend(message);
            });
    }

    std::string getClientId() const { return client_id_; }
    std::string getVehicleId() const { return vehicle_id_; }
    std::string getClientType() const { return client_type_; }

private:
    void onAccept(beast::error_code ec) {
        if (ec) {
            FSM_LOG_ERROR("[Signaling] Accept error: " + ec.message());
            return;
        }

        doRead();
    }

    void doRead() {
        ws_.async_read(
            buffer_,
            beast::bind_front_handler(
                &WebSocketSession::onRead,
                shared_from_this()));
    }

    void onRead(beast::error_code ec, std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);

        if (ec == websocket::error::closed) {
            return;
        }

        if (ec) {
            FSM_LOG_ERROR("[Signaling] Read error: " + ec.message());
            return;
        }

        // 处理消息
        std::string message = beast::buffers_to_string(buffer_.data());
        buffer_.consume(buffer_.size());

        handleMessage(message);

        // 继续读取
        doRead();
    }

    void doSend(const std::string& message) {
        send_queue_.push_back(message);

        if (send_queue_.size() > 1) {
            return;  // 已经在发送
        }

        ws_.async_write(
            net::buffer(send_queue_.front()),
            beast::bind_front_handler(
                &WebSocketSession::onWrite,
                shared_from_this()));
    }

    void onWrite(beast::error_code ec, std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);

        if (ec) {
            FSM_LOG_ERROR("[Signaling] Write error: " + ec.message());
            return;
        }

        send_queue_.erase(send_queue_.begin());

        if (!send_queue_.empty()) {
            ws_.async_write(
                net::buffer(send_queue_.front()),
                beast::bind_front_handler(
                    &WebSocketSession::onWrite,
                    shared_from_this()));
        }
    }

    void handleMessage(const std::string& message) {
        try {
            auto json = nlohmann::json::parse(message);
            std::string type = json["type"];

            if (type == "register") {
                handleRegister(json);
            } else if (type == "connect") {
                handleConnect(json);
            } else if (type == "offer" || type == "answer" || type == "ice-candidate") {
                handleSignaling(json);
            } else if (type == "disconnect") {
                handleDisconnect(json);
            }

        } catch (const std::exception& e) {
            FSM_LOG_ERROR("[Signaling] Message parse error: " + std::string(e.what()));
            sendError("Invalid message format");
        }
    }

    void handleRegister(const nlohmann::json& json) {
        client_id_ = json.value("client_id", "");
        client_type_ = json.value("client_type", "operator");  // vehicle or operator
        vehicle_id_ = json.value("vehicle_id", "");

        if (client_id_.empty()) {
            client_id_ = generateClientId();
        }

        server_->addSession(client_id_, shared_from_this());

        FSM_LOG_INFO("[Signaling] Client registered: " + client_id_ +
                 " type: " + client_type_ +
                 (vehicle_id_.empty() ? "" : " vehicle: " + vehicle_id_));

        // 发送确认
        nlohmann::json response;
        response["type"] = "registered";
        response["client_id"] = client_id_;
        send(response.dump());

        // 如果是车端，通知所有操作端
        if (client_type_ == "vehicle" && !vehicle_id_.empty()) {
            server_->notifyVehicleOnline(vehicle_id_);
        }
    }

    void handleConnect(const nlohmann::json& json) {
        std::string target_vehicle = json["vehicle_id"];

        // 查找车辆会话
        auto vehicle_session = server_->findVehicleSession(target_vehicle);
        if (!vehicle_session) {
            sendError("Vehicle not found: " + target_vehicle);
            return;
        }

        // 存储配对信息
        peer_id_ = vehicle_session->getClientId();
        vehicle_session->setPeerId(client_id_);

        FSM_LOG_INFO("[Signaling] Connecting operator " + client_id_ +
                 " to vehicle " + target_vehicle);

        // 通知车端创建 Offer
        nlohmann::json request;
        request["type"] = "create_offer";
        request["operator_id"] = client_id_;
        vehicle_session->send(request.dump());
    }

    void handleSignaling(const nlohmann::json& json) {
        // 转发信令消息给配对方
        if (peer_id_.empty()) {
            sendError("No peer connected");
            return;
        }

        auto peer_session = server_->getSession(peer_id_);
        if (!peer_session) {
            sendError("Peer disconnected");
            peer_id_.clear();
            return;
        }

        // 添加发送者信息并转发
        nlohmann::json forward = json;
        forward["from"] = client_id_;
        peer_session->send(forward.dump());
    }

    void handleDisconnect(const nlohmann::json& json) {
        if (!peer_id_.empty()) {
            auto peer_session = server_->getSession(peer_id_);
            if (peer_session) {
                nlohmann::json notify;
                notify["type"] = "peer_disconnected";
                notify["peer_id"] = client_id_;
                peer_session->send(notify.dump());
                peer_session->clearPeerId();
            }
            peer_id_.clear();
        }
    }

    void sendError(const std::string& message) {
        nlohmann::json error;
        error["type"] = "error";
        error["message"] = message;
        send(error.dump());
    }

    std::string generateClientId() {
        static std::atomic<uint64_t> counter{0};
        return "client_" + std::to_string(++counter);
    }

public:
    void setPeerId(const std::string& id) { peer_id_ = id; }
    void clearPeerId() { peer_id_.clear(); }

private:
    websocket::stream<beast::tcp_stream> ws_;
    SignalingServer* server_;
    beast::flat_buffer buffer_;
    std::vector<std::string> send_queue_;

    std::string client_id_;
    std::string client_type_;
    std::string vehicle_id_;
    std::string peer_id_;
};

// 信令服务器实现
class SignalingServer::Impl {
public:
    Impl(const config::CloudConfigManager& config)
        : config_(config)
        , ioc_(1)
        , acceptor_(ioc_)
        , running_(false)
    {
    }

    bool start() {
        try {
            auto const address = net::ip::make_address("0.0.0.0");
            auto const port = static_cast<unsigned short>(
                config_.getSignalingPort());

            tcp::endpoint endpoint(address, port);

            // 打开 acceptor
            acceptor_.open(endpoint.protocol());
            acceptor_.set_option(net::socket_base::reuse_address(true));
            acceptor_.bind(endpoint);
            acceptor_.listen(net::socket_base::max_listen_connections);

            FSM_LOG_INFO("[Signaling] Server starting on port " + std::to_string(port));

            running_ = true;
            doAccept();

            // 在后台线程运行
            io_thread_ = std::thread([this]() {
                ioc_.run();
            });

            return true;

        } catch (const std::exception& e) {
            FSM_LOG_ERROR("[Signaling] Failed to start server: " + std::string(e.what()));
            return false;
        }
    }

    void stop() {
        running_ = false;
        ioc_.stop();

        if (io_thread_.joinable()) {
            io_thread_.join();
        }

        FSM_LOG_INFO("[Signaling] Server stopped");
    }

    void addSession(const std::string& id, std::shared_ptr<WebSocketSession> session) {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        sessions_[id] = session;
    }

    void removeSession(const std::string& id) {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        sessions_.erase(id);
    }

    std::shared_ptr<WebSocketSession> getSession(const std::string& id) {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        auto it = sessions_.find(id);
        if (it != sessions_.end()) {
            return it->second;
        }
        return nullptr;
    }

    std::shared_ptr<WebSocketSession> findVehicleSession(const std::string& vehicle_id) {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        for (const auto& [id, session] : sessions_) {
            if (session->getClientType() == "vehicle" &&
                session->getVehicleId() == vehicle_id) {
                return session;
            }
        }
        return nullptr;
    }

    void notifyVehicleOnline(const std::string& vehicle_id) {
        nlohmann::json notification;
        notification["type"] = "vehicle_online";
        notification["vehicle_id"] = vehicle_id;

        std::lock_guard<std::mutex> lock(sessions_mutex_);
        for (const auto& [id, session] : sessions_) {
            if (session->getClientType() == "operator") {
                session->send(notification.dump());
            }
        }
    }

    void broadcastToOperators(const std::string& message) {
        std::lock_guard<std::mutex> lock(sessions_mutex_);
        for (const auto& [id, session] : sessions_) {
            if (session->getClientType() == "operator") {
                session->send(message);
            }
        }
    }

private:
    void doAccept() {
        acceptor_.async_accept(
            net::make_strand(ioc_),
            beast::bind_front_handler(
                &Impl::onAccept, this));
    }

    void onAccept(beast::error_code ec, tcp::socket socket) {
        if (ec) {
            if (running_) {
                FSM_LOG_ERROR("[Signaling] Accept error: " + ec.message());
            }
        } else {
            // 创建会话
            std::make_shared<WebSocketSession>(
                std::move(socket),
                reinterpret_cast<SignalingServer*>(this)
            )->run();
        }

        if (running_) {
            doAccept();
        }
    }

private:
    const config::CloudConfigManager& config_;
    net::io_context ioc_;
    tcp::acceptor acceptor_;
    std::thread io_thread_;
    std::atomic<bool> running_;

    std::mutex sessions_mutex_;
    std::unordered_map<std::string, std::shared_ptr<WebSocketSession>> sessions_;
};

// SignalingServer 公共接口实现
SignalingServer::SignalingServer(const config::CloudConfigManager& config)
    : impl_(std::make_unique<Impl>(config))
{
}

SignalingServer::~SignalingServer() = default;

bool SignalingServer::start() {
    return impl_->start();
}

void SignalingServer::stop() {
    impl_->stop();
}

void SignalingServer::addSession(const std::string& id, std::shared_ptr<WebSocketSession> session) {
    impl_->addSession(id, session);
}

void SignalingServer::removeSession(const std::string& id) {
    impl_->removeSession(id);
}

std::shared_ptr<WebSocketSession> SignalingServer::getSession(const std::string& id) {
    return impl_->getSession(id);
}

std::shared_ptr<WebSocketSession> SignalingServer::findVehicleSession(const std::string& vehicle_id) {
    return impl_->findVehicleSession(vehicle_id);
}

void SignalingServer::notifyVehicleOnline(const std::string& vehicle_id) {
    impl_->notifyVehicleOnline(vehicle_id);
}

void SignalingServer::broadcastToOperators(const std::string& message) {
    impl_->broadcastToOperators(message);
}

}  // namespace cloud
}  // namespace fsm
