/**
 * FSM-Pilot 预测模型接口头文件
 * 用于延迟补偿可视化
 */

#pragma once

#include "fsm/config_manager.hpp"
#include <string>
#include <vector>

namespace fsm {
namespace cloud {

/**
 * 车辆运动学状态
 */
struct VehicleKinematicState {
    double x = 0;              // 位置 X (米)
    double y = 0;              // 位置 Y (米)
    double heading = 0;        // 航向角 (弧度)
    double speed = 0;          // 速度 (米/秒)
    double acceleration = 0;   // 加速度 (米/秒²)
    double steering_angle = 0; // 转向角 (弧度)
    double wheelbase = 2.7;    // 轴距 (米)
};

/**
 * 预测状态
 */
struct PredictedState {
    double x = 0;
    double y = 0;
    double heading = 0;
    double speed = 0;
    double confidence = 1.0;     // 预测置信度 [0, 1]
    double prediction_time_ms = 0; // 预测时间窗口
};

/**
 * 轨迹点
 */
struct TrajectoryPoint {
    double x;
    double y;
    double heading;
    double speed;
    double time_offset_ms;  // 相对当前时间的偏移
    double confidence;
};

/**
 * 视觉叠加层
 */
struct VisualOverlay {
    bool enabled = false;

    // Ghost 车辆
    double ghost_x = 0;
    double ghost_y = 0;
    double ghost_heading = 0;
    double ghost_opacity = 0.5;
    std::string ghost_color = "#00ff88";

    // 预测路径
    std::vector<std::pair<double, double>> path_points;

    // 延迟标签
    std::string latency_label;
};

/**
 * 预测模型接口
 * 提供延迟补偿和可视化功能
 */
class PredictionInterface {
public:
    explicit PredictionInterface(const config::CloudConfigManager& config);

    /**
     * 检查预测功能是否启用
     */
    bool isEnabled() const;

    /**
     * 设置预测功能开关
     */
    void setEnabled(bool enabled);

    /**
     * 检查补偿可视化是否启用
     */
    bool isCompensationEnabled() const;

    /**
     * 设置补偿可视化开关
     */
    void setCompensationEnabled(bool enabled);

    /**
     * 预测未来状态
     * @param current_state 当前运动学状态
     * @param latency_ms 需要补偿的延迟 (毫秒)
     * @return 预测的未来状态
     */
    PredictedState predict(const VehicleKinematicState& current_state,
                           double latency_ms) const;

    /**
     * 预测轨迹
     * @param current_state 当前状态
     * @param horizon_ms 预测时间范围 (毫秒)
     * @param step_ms 步长 (毫秒)
     * @return 预测轨迹点序列
     */
    std::vector<TrajectoryPoint> predictTrajectory(
        const VehicleKinematicState& current_state,
        double horizon_ms,
        double step_ms = 50) const;

    /**
     * 生成可视化叠加层
     * @param predicted 预测状态
     * @param current 当前状态
     * @return 可视化叠加层数据
     */
    VisualOverlay generateOverlay(const PredictedState& predicted,
                                   const VehicleKinematicState& current) const;

private:
    /**
     * 计算预测置信度
     */
    double calculateConfidence(double latency_ms,
                                const VehicleKinematicState& state) const;

    /**
     * 调用外部预测模型 (可选)
     */
    PredictedState callExternalModel(const VehicleKinematicState& state,
                                      double latency_ms) const;

private:
    const config::CloudConfigManager& config_;
    bool enabled_;
    bool compensation_enabled_;
    std::string model_endpoint_;
};

}  // namespace cloud
}  // namespace fsm
