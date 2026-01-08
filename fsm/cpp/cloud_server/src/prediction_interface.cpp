/**
 * FSM-Pilot 预测模型接口实现
 * 用于延迟补偿可视化
 */

#include "fsm/cloud/prediction_interface.hpp"
#include "fsm/logger.hpp"
#include <cmath>

namespace fsm {
namespace cloud {

PredictionInterface::PredictionInterface(const config::CloudConfigManager& config)
    : config_(config)
    , enabled_(false)
{
    auto pred_config = config.getPredictionConfig();
    enabled_ = pred_config.enabled;
    compensation_enabled_ = pred_config.compensation_enabled;
    model_endpoint_ = pred_config.model_endpoint;

    if (enabled_) {
        LOG_INFO("[Prediction] Interface enabled, endpoint: " + model_endpoint_);
    } else {
        LOG_INFO("[Prediction] Interface disabled");
    }
}

bool PredictionInterface::isEnabled() const {
    return enabled_;
}

void PredictionInterface::setEnabled(bool enabled) {
    enabled_ = enabled;
    LOG_INFO("[Prediction] " + std::string(enabled ? "Enabled" : "Disabled"));
}

bool PredictionInterface::isCompensationEnabled() const {
    return compensation_enabled_;
}

void PredictionInterface::setCompensationEnabled(bool enabled) {
    compensation_enabled_ = enabled;
    LOG_INFO("[Prediction] Compensation " + std::string(enabled ? "Enabled" : "Disabled"));
}

PredictedState PredictionInterface::predict(const VehicleKinematicState& current_state,
                                             double latency_ms) const {
    PredictedState predicted;

    if (!enabled_ || latency_ms <= 0) {
        // 无预测，返回当前状态
        predicted.x = current_state.x;
        predicted.y = current_state.y;
        predicted.heading = current_state.heading;
        predicted.speed = current_state.speed;
        predicted.confidence = 1.0;
        return predicted;
    }

    // 预测时间 (秒)
    double dt = latency_ms / 1000.0;

    // 使用运动学模型预测未来位置
    // 假设匀速直线运动 + 恒定转向率
    double speed = current_state.speed;
    double heading = current_state.heading;
    double steering_angle = current_state.steering_angle;
    double wheelbase = current_state.wheelbase;

    // 计算转弯半径和角速度
    double turn_radius = 0;
    double angular_velocity = 0;

    if (std::abs(steering_angle) > 0.01) {
        turn_radius = wheelbase / std::tan(steering_angle);
        angular_velocity = speed / turn_radius;
    }

    // 预测位置
    if (std::abs(angular_velocity) < 0.001) {
        // 近似直线运动
        predicted.x = current_state.x + speed * dt * std::cos(heading);
        predicted.y = current_state.y + speed * dt * std::sin(heading);
        predicted.heading = heading;
    } else {
        // 曲线运动
        double delta_heading = angular_velocity * dt;
        predicted.heading = heading + delta_heading;

        // 使用弧线积分
        double radius = speed / angular_velocity;
        predicted.x = current_state.x + radius * (std::sin(predicted.heading) - std::sin(heading));
        predicted.y = current_state.y + radius * (std::cos(heading) - std::cos(predicted.heading));
    }

    // 速度假设不变
    predicted.speed = speed;

    // 置信度随延迟降低
    predicted.confidence = calculateConfidence(latency_ms, current_state);

    // 预测时间
    predicted.prediction_time_ms = latency_ms;

    return predicted;
}

std::vector<TrajectoryPoint> PredictionInterface::predictTrajectory(
    const VehicleKinematicState& current_state,
    double horizon_ms,
    double step_ms) const {

    std::vector<TrajectoryPoint> trajectory;

    if (!enabled_ || horizon_ms <= 0) {
        return trajectory;
    }

    VehicleKinematicState state = current_state;
    double elapsed = 0;

    while (elapsed <= horizon_ms) {
        PredictedState pred = predict(state, step_ms);

        TrajectoryPoint point;
        point.x = pred.x;
        point.y = pred.y;
        point.heading = pred.heading;
        point.speed = pred.speed;
        point.time_offset_ms = elapsed;
        point.confidence = pred.confidence;

        trajectory.push_back(point);

        // 更新状态用于下一步预测
        state.x = pred.x;
        state.y = pred.y;
        state.heading = pred.heading;
        state.speed = pred.speed;

        elapsed += step_ms;
    }

    return trajectory;
}

VisualOverlay PredictionInterface::generateOverlay(const PredictedState& predicted,
                                                     const VehicleKinematicState& current) const {
    VisualOverlay overlay;

    if (!compensation_enabled_) {
        overlay.enabled = false;
        return overlay;
    }

    overlay.enabled = true;

    // Ghost 车辆位置 (预测位置)
    overlay.ghost_x = predicted.x;
    overlay.ghost_y = predicted.y;
    overlay.ghost_heading = predicted.heading;

    // 当前到预测位置的连线
    overlay.path_points.push_back({current.x, current.y});
    overlay.path_points.push_back({predicted.x, predicted.y});

    // 透明度基于置信度
    overlay.ghost_opacity = predicted.confidence * 0.6;  // 最大 60% 透明度

    // 颜色基于置信度
    if (predicted.confidence > 0.8) {
        overlay.ghost_color = "#00ff88";  // 绿色
    } else if (predicted.confidence > 0.5) {
        overlay.ghost_color = "#ffaa00";  // 橙色
    } else {
        overlay.ghost_color = "#ff4444";  // 红色
    }

    // 延迟标签
    overlay.latency_label = std::to_string(static_cast<int>(predicted.prediction_time_ms)) + "ms";

    return overlay;
}

double PredictionInterface::calculateConfidence(double latency_ms,
                                                  const VehicleKinematicState& state) const {
    // 基础置信度随延迟线性降低
    double latency_factor = std::max(0.0, 1.0 - latency_ms / 1000.0);

    // 高速时置信度降低更快
    double speed_factor = 1.0;
    if (state.speed > 10) {  // > 36 km/h
        speed_factor = 0.9;
    }
    if (state.speed > 20) {  // > 72 km/h
        speed_factor = 0.7;
    }

    // 转弯时置信度降低
    double steering_factor = 1.0;
    if (std::abs(state.steering_angle) > 0.1) {
        steering_factor = 0.8;
    }

    return latency_factor * speed_factor * steering_factor;
}

// HTTP 客户端调用外部模型 (可选)
PredictedState PredictionInterface::callExternalModel(
    const VehicleKinematicState& state,
    double latency_ms) const {

    // 如果配置了外部模型端点，调用它
    // 这里是占位实现，实际需要 HTTP 客户端

    LOG_DEBUG("[Prediction] External model call (not implemented)");

    // 回退到内置模型
    return predict(state, latency_ms);
}

}  // namespace cloud
}  // namespace fsm
