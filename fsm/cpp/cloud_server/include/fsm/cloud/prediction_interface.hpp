#pragma once

#include <string>
#include <vector>

#include "fsm/config_manager.hpp"

namespace fsm {
namespace cloud {

struct VehicleKinematicState {
  double x = 0.0;               // metres
  double y = 0.0;               // metres
  double heading = 0.0;         // radians
  double speed = 0.0;           // m/s
  double acceleration = 0.0;    // m/s²
  double steering_angle = 0.0;  // radians
  double wheelbase_m = 2.7;     // metres
};

struct PredictedState {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed = 0.0;
  double confidence = 1.0;           // [0, 1]
  double prediction_time_ms = 0.0;
};

struct TrajectoryPoint {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  double speed = 0.0;
  double time_offset_ms = 0.0;
  double confidence = 1.0;
};

struct VisualOverlay {
  bool enabled = false;
  double ghost_x = 0.0;
  double ghost_y = 0.0;
  double ghost_heading = 0.0;
  double ghost_opacity = 0.5;
  std::string ghost_color = "#00ff88";
  std::vector<std::pair<double, double>> path_points;
  std::string latency_label;
};

class PredictionInterface {
 public:
  explicit PredictionInterface(const config::CloudConfigManager& config);

  PredictionInterface(const PredictionInterface&) = delete;
  PredictionInterface& operator=(const PredictionInterface&) = delete;

  bool is_enabled() const;
  void SetEnabled(bool enabled);

  bool is_compensation_enabled() const;
  void SetCompensationEnabled(bool enabled);

  PredictedState Predict(const VehicleKinematicState& state,
                         double latency_ms) const;

  std::vector<TrajectoryPoint> PredictTrajectory(
      const VehicleKinematicState& state,
      double horizon_ms,
      double step_ms = 50.0) const;

  VisualOverlay GenerateOverlay(const PredictedState& predicted,
                                const VehicleKinematicState& current) const;

 private:
  double CalculateConfidence(double latency_ms,
                             const VehicleKinematicState& state) const;

  PredictedState CallExternalModel(const VehicleKinematicState& state,
                                   double latency_ms) const;

  const config::CloudConfigManager& config_;
  bool enabled_ = false;
  bool compensation_enabled_ = false;
  std::string model_endpoint_;
};

}  // namespace cloud
}  // namespace fsm
