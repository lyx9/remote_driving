#include "fsm/cloud/prediction_interface.hpp"
#include "fsm/logger.hpp"
#include <cmath>

namespace fsm {
namespace cloud {

PredictionInterface::PredictionInterface(const config::CloudConfigManager& config)
    : config_(config) {
  const auto& pred = config_.prediction_config();
  enabled_              = pred.enabled;
  compensation_enabled_ = pred.compensation_enabled;
  model_endpoint_       = pred.model_endpoint;

  if (enabled_) {
    FSM_LOG_INFO("PredictionInterface enabled: endpoint={}", model_endpoint_);
  } else {
    FSM_LOG_INFO("PredictionInterface disabled");
  }
}

bool PredictionInterface::is_enabled() const {
  return enabled_;
}

void PredictionInterface::SetEnabled(bool enabled) {
  enabled_ = enabled;
  FSM_LOG_INFO("PredictionInterface {}", enabled ? "enabled" : "disabled");
}

bool PredictionInterface::is_compensation_enabled() const {
  return compensation_enabled_;
}

void PredictionInterface::SetCompensationEnabled(bool enabled) {
  compensation_enabled_ = enabled;
  FSM_LOG_INFO("PredictionInterface compensation {}", enabled ? "enabled" : "disabled");
}

PredictedState PredictionInterface::Predict(
    const VehicleKinematicState& current_state, double latency_ms) const {
  PredictedState predicted;

  if (!enabled_ || latency_ms <= 0.0) {
    predicted.x                  = current_state.x;
    predicted.y                  = current_state.y;
    predicted.heading            = current_state.heading;
    predicted.speed              = current_state.speed;
    predicted.confidence         = 1.0;
    predicted.prediction_time_ms = 0.0;
    return predicted;
  }

  const double dt             = latency_ms / 1000.0;
  const double speed          = current_state.speed;
  const double heading        = current_state.heading;
  const double steering_angle = current_state.steering_angle;
  const double wheelbase      = current_state.wheelbase_m;

  double angular_velocity = 0.0;
  if (std::abs(steering_angle) > 0.01) {
    const double turn_radius = wheelbase / std::tan(steering_angle);
    angular_velocity         = speed / turn_radius;
  }

  if (std::abs(angular_velocity) < 0.001) {
    predicted.x       = current_state.x + speed * dt * std::cos(heading);
    predicted.y       = current_state.y + speed * dt * std::sin(heading);
    predicted.heading = heading;
  } else {
    const double delta_heading = angular_velocity * dt;
    predicted.heading          = heading + delta_heading;
    const double radius        = speed / angular_velocity;
    predicted.x = current_state.x +
                  radius * (std::sin(predicted.heading) - std::sin(heading));
    predicted.y = current_state.y +
                  radius * (std::cos(heading) - std::cos(predicted.heading));
  }

  predicted.speed              = speed;
  predicted.confidence         = CalculateConfidence(latency_ms, current_state);
  predicted.prediction_time_ms = latency_ms;
  return predicted;
}

std::vector<TrajectoryPoint> PredictionInterface::PredictTrajectory(
    const VehicleKinematicState& current_state,
    double horizon_ms,
    double step_ms) const {
  std::vector<TrajectoryPoint> trajectory;
  if (!enabled_ || horizon_ms <= 0.0) return trajectory;

  VehicleKinematicState state = current_state;
  for (double elapsed = 0.0; elapsed <= horizon_ms; elapsed += step_ms) {
    const PredictedState pred = Predict(state, step_ms);

    TrajectoryPoint pt;
    pt.x              = pred.x;
    pt.y              = pred.y;
    pt.heading        = pred.heading;
    pt.speed          = pred.speed;
    pt.time_offset_ms = elapsed;
    pt.confidence     = pred.confidence;
    trajectory.push_back(pt);

    state.x       = pred.x;
    state.y       = pred.y;
    state.heading = pred.heading;
    state.speed   = pred.speed;
  }
  return trajectory;
}

VisualOverlay PredictionInterface::GenerateOverlay(
    const PredictedState& predicted,
    const VehicleKinematicState& current) const {
  VisualOverlay overlay;
  if (!compensation_enabled_) return overlay;

  overlay.enabled       = true;
  overlay.ghost_x       = predicted.x;
  overlay.ghost_y       = predicted.y;
  overlay.ghost_heading = predicted.heading;
  overlay.ghost_opacity = predicted.confidence * 0.6;

  if (predicted.confidence > 0.8) {
    overlay.ghost_color = "#00ff88";
  } else if (predicted.confidence > 0.5) {
    overlay.ghost_color = "#ffaa00";
  } else {
    overlay.ghost_color = "#ff4444";
  }

  overlay.path_points.push_back({current.x, current.y});
  overlay.path_points.push_back({predicted.x, predicted.y});
  overlay.latency_label =
      std::to_string(static_cast<int>(predicted.prediction_time_ms)) + "ms";
  return overlay;
}

double PredictionInterface::CalculateConfidence(
    double latency_ms, const VehicleKinematicState& state) const {
  const double latency_factor = std::max(0.0, 1.0 - latency_ms / 1000.0);

  double speed_factor = 1.0;
  if (state.speed > 20.0) speed_factor = 0.7;
  else if (state.speed > 10.0) speed_factor = 0.9;

  const double steering_factor =
      (std::abs(state.steering_angle) > 0.1) ? 0.8 : 1.0;

  return latency_factor * speed_factor * steering_factor;
}

PredictedState PredictionInterface::CallExternalModel(
    const VehicleKinematicState& state, double latency_ms) const {
  FSM_LOG_DEBUG("External model call not implemented, using built-in");
  return Predict(state, latency_ms);
}

}  // namespace cloud
}  // namespace fsm
