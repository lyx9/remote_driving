#include "fsm/vehicle/platform_adapter.hpp"
#include "fsm/logger.hpp"
#include "fsm/utils.hpp"

#include <rclcpp/rclcpp.hpp>

// Standard ROS2 messages (always available)
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Autoware messages (optional — guarded by HAVE_AUTOWARE_MSGS)
#ifdef HAVE_AUTOWARE_MSGS
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#endif

#include <cmath>
#include <mutex>
#include <atomic>

namespace fsm {
namespace vehicle {

// ============================================================================
// AutowarePlatformAdapter
// ============================================================================

class AutowarePlatformAdapter : public PlatformAdapter {
 public:
  AutowarePlatformAdapter(rclcpp::Node::SharedPtr node,
                           const config::VehicleConfigManager& config)
      : node_(std::move(node)), config_(config) {}

  bool Initialize() override {
#ifdef HAVE_AUTOWARE_MSGS
    using namespace autoware_auto_control_msgs::msg;
    using namespace autoware_auto_vehicle_msgs::msg;

    const auto& topics = config_.control_topics();
    const auto& sensor = config_.sensor_topics();

    // Publishers — control commands to Autoware
    pub_ctrl_ = node_->create_publisher<AckermannControlCommand>(
        topics.steering_cmd, rclcpp::QoS(1));
    pub_gear_ = node_->create_publisher<GearCommand>(
        topics.gear_cmd, rclcpp::QoS(1));
    pub_turn_ = node_->create_publisher<TurnIndicatorsCommand>(
        topics.turn_signal_cmd, rclcpp::QoS(1));
    pub_hazard_ = node_->create_publisher<HazardLightsCommand>(
        topics.hazard_cmd, rclcpp::QoS(1));

    // Subscribers — vehicle status from Autoware
    sub_vel_ = node_->create_subscription<VelocityReport>(
        sensor.velocity_status, rclcpp::QoS(1),
        [this](VelocityReport::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(status_mu_);
          status_.speed_mps    = msg->longitudinal_velocity;
          status_.timestamp_ns = NowNanos();
          NotifyStatus();
        });

    sub_steer_ = node_->create_subscription<SteeringReport>(
        sensor.steering_status, rclcpp::QoS(1),
        [this](SteeringReport::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(status_mu_);
          status_.steering_rad = msg->steering_tire_angle;
        });

    sub_gear_ = node_->create_subscription<GearReport>(
        sensor.gear_status, rclcpp::QoS(1),
        [this](GearReport::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(status_mu_);
          // GEAR_PARK=22, GEAR_REVERSE=20, GEAR_NEUTRAL=1, GEAR_DRIVE=2
          switch (msg->report) {
            case GearReport::PARK:    status_.gear = 0; break;
            case GearReport::REVERSE: status_.gear = 1; break;
            case GearReport::NEUTRAL: status_.gear = 2; break;
            case GearReport::DRIVE:   status_.gear = 3; break;
            default:                  status_.gear = 3; break;
          }
        });

    // Localization
    sub_loc_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        sensor.localization, rclcpp::QoS(1),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(status_mu_);
          status_.localization_valid = true;
          status_.latitude  = msg->pose.pose.position.x;  // map frame x
          status_.longitude = msg->pose.pose.position.y;  // map frame y
          // heading from quaternion (yaw)
          const auto& q = msg->pose.pose.orientation;
          status_.heading_rad = static_cast<float>(
              std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
        });

    FSM_LOG_INFO("AutowarePlatformAdapter initialized");
    return true;
#else
    FSM_LOG_ERROR("AutowarePlatformAdapter: HAVE_AUTOWARE_MSGS not defined");
    return false;
#endif
  }

  void Shutdown() override {
#ifdef HAVE_AUTOWARE_MSGS
    pub_ctrl_.reset();
    pub_gear_.reset();
    pub_turn_.reset();
    pub_hazard_.reset();
    sub_vel_.reset();
    sub_steer_.reset();
    sub_gear_.reset();
    sub_loc_.reset();
#endif
    FSM_LOG_INFO("AutowarePlatformAdapter shutdown");
  }

  void ApplyCommand(const NormalizedControlCmd& cmd) override {
#ifdef HAVE_AUTOWARE_MSGS
    using namespace autoware_auto_control_msgs::msg;
    using namespace autoware_auto_vehicle_msgs::msg;

    const auto& params = config_.vehicle_params();
    const float max_steer_rad =
        params.max_steering_angle_deg * (static_cast<float>(M_PI) / 180.0f);

    // Emergency stop
    if (cmd.emergency_stop) {
      AckermannControlCommand ctrl;
      ctrl.stamp = node_->now();
      ctrl.longitudinal.speed        = 0.0f;
      ctrl.longitudinal.acceleration = -params.emergency_decel_mps2;
      if (pub_ctrl_) pub_ctrl_->publish(ctrl);
      FSM_LOG_WARN("AutowarePlatformAdapter: EMERGENCY STOP");
      return;
    }

    // Steering + longitudinal control
    AckermannControlCommand ctrl;
    ctrl.stamp = node_->now();
    ctrl.lateral.steering_tire_angle = cmd.steering_normalized * max_steer_rad;
    ctrl.lateral.steering_tire_rotation_rate =
        params.max_steering_rate_dps * (static_cast<float>(M_PI) / 180.0f);

    if (cmd.throttle_normalized > 0.0f) {
      ctrl.longitudinal.speed        = params.max_speed_kph / 3.6f * cmd.throttle_normalized;
      ctrl.longitudinal.acceleration = params.max_acceleration_mps2 * cmd.throttle_normalized;
    } else {
      ctrl.longitudinal.speed        = 0.0f;
      ctrl.longitudinal.acceleration = -params.max_deceleration_mps2 * cmd.brake_normalized;
    }
    if (pub_ctrl_) pub_ctrl_->publish(ctrl);

    // Gear
    GearCommand gear_cmd;
    gear_cmd.stamp = node_->now();
    switch (cmd.gear) {
      case 0: gear_cmd.command = GearCommand::PARK;    break;
      case 1: gear_cmd.command = GearCommand::REVERSE; break;
      case 2: gear_cmd.command = GearCommand::NEUTRAL; break;
      default: gear_cmd.command = GearCommand::DRIVE;  break;
    }
    if (pub_gear_) pub_gear_->publish(gear_cmd);

    // Turn signal
    TurnIndicatorsCommand turn_cmd;
    turn_cmd.stamp = node_->now();
    switch (cmd.turn_signal) {
      case 1: turn_cmd.command = TurnIndicatorsCommand::ENABLE_LEFT;  break;
      case 2: turn_cmd.command = TurnIndicatorsCommand::ENABLE_RIGHT; break;
      default: turn_cmd.command = TurnIndicatorsCommand::DISABLE;     break;
    }
    if (pub_turn_) pub_turn_->publish(turn_cmd);

    // Hazard
    HazardLightsCommand hazard_cmd;
    hazard_cmd.stamp = node_->now();
    hazard_cmd.command = (cmd.turn_signal == 3)
        ? HazardLightsCommand::ENABLE
        : HazardLightsCommand::DISABLE;
    if (pub_hazard_) pub_hazard_->publish(hazard_cmd);
#else
    (void)cmd;
    FSM_LOG_WARN("ApplyCommand: Autoware msgs not compiled");
#endif
  }

  VehicleCapabilities GetCapabilities() const override {
    const auto& params = config_.vehicle_params();
    VehicleCapabilities cap;
    cap.vehicle_id              = config_.vehicle_id();
    cap.platform_type           = PlatformType::kAutoware;
    cap.has_steer_by_wire       = true;
    cap.has_brake_by_wire       = true;
    cap.has_throttle_by_wire    = true;
    cap.has_gear_control        = true;
    cap.has_turn_signal_control = true;
    cap.has_emergency_stop      = true;
    cap.has_lidar               = true;
    cap.has_gps                 = true;
    cap.max_speed_kph           = params.max_speed_kph;
    cap.max_steering_angle_deg  = params.max_steering_angle_deg;
    cap.wheelbase_m             = params.wheelbase_m;
    return cap;
  }

 private:
  void NotifyStatus() {
    if (status_cb_) status_cb_(status_);
  }

  rclcpp::Node::SharedPtr              node_;
  const config::VehicleConfigManager&  config_;
  std::mutex                           status_mu_;
  NormalizedVehicleStatus              status_;

#ifdef HAVE_AUTOWARE_MSGS
  using AckermannPub = rclcpp::Publisher<
      autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr;
  using GearPub = rclcpp::Publisher<
      autoware_auto_vehicle_msgs::msg::GearCommand>::SharedPtr;
  using TurnPub = rclcpp::Publisher<
      autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr;
  using HazardPub = rclcpp::Publisher<
      autoware_auto_vehicle_msgs::msg::HazardLightsCommand>::SharedPtr;

  AckermannPub pub_ctrl_;
  GearPub      pub_gear_;
  TurnPub      pub_turn_;
  HazardPub    pub_hazard_;

  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_vel_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr sub_steer_;
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::GearReport>::SharedPtr     sub_gear_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                         sub_loc_;
#endif
};

// ============================================================================
// GenericRos2PlatformAdapter  — uses configurable topic names
// ============================================================================

class GenericRos2PlatformAdapter : public PlatformAdapter {
 public:
  GenericRos2PlatformAdapter(rclcpp::Node::SharedPtr node,
                              const config::VehicleConfigManager& config)
      : node_(std::move(node)), config_(config) {}

  bool Initialize() override {
    const auto& sensor = config_.sensor_topics();

    // Odometry for status
    sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        sensor.localization, rclcpp::QoS(1),
        [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          NormalizedVehicleStatus s;
          const auto& t = msg->twist.twist;
          s.speed_mps    = static_cast<float>(
              std::sqrt(t.linear.x * t.linear.x + t.linear.y * t.linear.y));
          s.latitude    = msg->pose.pose.position.x;
          s.longitude   = msg->pose.pose.position.y;
          const auto& q = msg->pose.pose.orientation;
          s.heading_rad = static_cast<float>(
              std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z)));
          s.localization_valid = true;
          s.timestamp_ns       = NowNanos();
          if (status_cb_) status_cb_(s);
        });

    // Twist publisher for velocity commands
    pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", rclcpp::QoS(1));

    FSM_LOG_INFO("GenericRos2PlatformAdapter initialized");
    return true;
  }

  void Shutdown() override {
    sub_odom_.reset();
    pub_cmd_vel_.reset();
  }

  void ApplyCommand(const NormalizedControlCmd& cmd) override {
    const auto& params = config_.vehicle_params();
    geometry_msgs::msg::TwistStamped twist;
    twist.header.stamp = node_->now();
    if (cmd.emergency_stop) {
      // zero velocity
    } else {
      twist.twist.linear.x  = params.max_speed_kph / 3.6f * cmd.throttle_normalized;
      const float max_steer = params.max_steering_angle_deg * (static_cast<float>(M_PI) / 180.0f);
      twist.twist.angular.z = cmd.steering_normalized * max_steer;
      if (cmd.brake_normalized > 0.1f) twist.twist.linear.x = 0.0;
    }
    if (pub_cmd_vel_) pub_cmd_vel_->publish(twist);
  }

  VehicleCapabilities GetCapabilities() const override {
    const auto& params = config_.vehicle_params();
    VehicleCapabilities cap;
    cap.vehicle_id           = config_.vehicle_id();
    cap.platform_type        = PlatformType::kGenericRos2;
    cap.has_emergency_stop   = true;
    cap.has_gps              = true;
    cap.max_speed_kph        = params.max_speed_kph;
    cap.max_steering_angle_deg = params.max_steering_angle_deg;
    cap.wheelbase_m          = params.wheelbase_m;
    return cap;
  }

 private:
  rclcpp::Node::SharedPtr             node_;
  const config::VehicleConfigManager& config_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr    pub_cmd_vel_;
};

// ============================================================================
// SimulatorPlatformAdapter  — logs commands, emits fake status
// ============================================================================

class SimulatorPlatformAdapter : public PlatformAdapter {
 public:
  SimulatorPlatformAdapter(rclcpp::Node::SharedPtr node,
                            const config::VehicleConfigManager& config)
      : node_(std::move(node)), config_(config) {}

  bool Initialize() override {
    // Publish fake status at 20 Hz
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() {
          NormalizedVehicleStatus s;
          s.speed_mps          = last_speed_;
          s.steering_rad       = last_steer_;
          s.gear               = last_gear_;
          s.battery_pct        = 80.0f;
          s.localization_valid = true;
          s.timestamp_ns       = NowNanos();
          if (status_cb_) status_cb_(s);
        });
    FSM_LOG_INFO("SimulatorPlatformAdapter initialized (fake status @ 20Hz)");
    return true;
  }

  void Shutdown() override {
    timer_.reset();
  }

  void ApplyCommand(const NormalizedControlCmd& cmd) override {
    if (cmd.emergency_stop) {
      last_speed_ = 0.0f;
      FSM_LOG_WARN("Simulator: EMERGENCY STOP");
      return;
    }
    const float max_kph  = config_.vehicle_params().max_speed_kph;
    const float max_steer = config_.vehicle_params().max_steering_angle_deg
                            * (static_cast<float>(M_PI) / 180.0f);
    last_speed_ = max_kph / 3.6f * (cmd.throttle_normalized - cmd.brake_normalized);
    last_steer_ = cmd.steering_normalized * max_steer;
    last_gear_  = cmd.gear;
    FSM_LOG_DEBUG("Simulator cmd: steer={:.2f} throttle={:.2f} brake={:.2f}",
                  cmd.steering_normalized, cmd.throttle_normalized, cmd.brake_normalized);
  }

  VehicleCapabilities GetCapabilities() const override {
    const auto& params = config_.vehicle_params();
    VehicleCapabilities cap;
    cap.vehicle_id              = config_.vehicle_id();
    cap.platform_type           = PlatformType::kCarlaSim;
    cap.has_steer_by_wire       = true;
    cap.has_brake_by_wire       = true;
    cap.has_throttle_by_wire    = true;
    cap.has_gear_control        = true;
    cap.has_emergency_stop      = true;
    cap.has_lidar               = true;
    cap.has_gps                 = true;
    cap.num_cameras             = 4;
    cap.max_speed_kph           = params.max_speed_kph;
    cap.max_steering_angle_deg  = params.max_steering_angle_deg;
    cap.wheelbase_m             = params.wheelbase_m;
    cap.firmware_version        = "sim-1.0";
    return cap;
  }

 private:
  rclcpp::Node::SharedPtr             node_;
  const config::VehicleConfigManager& config_;
  rclcpp::TimerBase::SharedPtr        timer_;
  float    last_speed_ = 0.0f;
  float    last_steer_ = 0.0f;
  int      last_gear_  = 3;
};

// ============================================================================
// Factory
// ============================================================================

std::unique_ptr<PlatformAdapter> PlatformAdapter::Create(
    PlatformType type,
    rclcpp::Node::SharedPtr node,
    const config::VehicleConfigManager& config) {
  switch (type) {
    case PlatformType::kAutoware:
      return std::make_unique<AutowarePlatformAdapter>(node, config);
    case PlatformType::kGenericRos2:
      return std::make_unique<GenericRos2PlatformAdapter>(node, config);
    case PlatformType::kCarlaSim:
    case PlatformType::kLgsvlSim:
    case PlatformType::kUnknown:
    default:
      FSM_LOG_WARN("PlatformAdapter: using SimulatorPlatformAdapter for type {}",
                   static_cast<int>(type));
      return std::make_unique<SimulatorPlatformAdapter>(node, config);
  }
}

}  // namespace vehicle
}  // namespace fsm
