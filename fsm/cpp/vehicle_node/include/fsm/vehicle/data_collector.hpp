#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"

#ifdef HAVE_AUTOWARE_MSGS
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_report.hpp>
#endif

namespace fsm {
namespace vehicle {

struct VehicleState {
  double longitudinal_velocity = 0.0;  // m/s
  double lateral_velocity = 0.0;       // m/s
  double heading_rate = 0.0;           // rad/s
  double steering_tire_angle = 0.0;    // rad
  int gear = 0;                        // 0:P 1:R 2:N 3:D
  int control_mode = 0;                // 0:MANUAL 1:AUTONOMOUS 2:REMOTE
  int turn_indicator = 0;              // 0:NONE 1:LEFT 2:RIGHT
  bool hazard_lights = false;
  double pose_x = 0.0;
  double pose_y = 0.0;
  double pose_z = 0.0;
  double orientation_x = 0.0;
  double orientation_y = 0.0;
  double orientation_z = 0.0;
  double orientation_w = 1.0;
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  int64_t timestamp_ns = 0;
  bool velocity_valid = false;
  bool steering_valid = false;
  bool localization_valid = false;
};

struct SystemState {
  float cpu_usage = 0.0f;
  float memory_usage = 0.0f;
  float gpu_usage = 0.0f;
  float disk_usage = 0.0f;
  float network_tx_bps = 0.0f;
  float network_rx_bps = 0.0f;
  std::vector<std::pair<std::string, int>> diagnostics;
  int64_t timestamp_ns = 0;
};

struct CameraFrame {
  std::string camera_id;
  cv::Mat image;
  int64_t timestamp_ns = 0;
  uint64_t frame_number = 0;
};

using VehicleStateCallback = std::function<void(const VehicleState&)>;
using SystemStateCallback  = std::function<void(const SystemState&)>;
using CameraFrameCallback  = std::function<void(const CameraFrame&)>;

class DataCollector {
 public:
  DataCollector(rclcpp::Node::SharedPtr node,
                const config::VehicleConfigManager& config);
  ~DataCollector();

  DataCollector(const DataCollector&) = delete;
  DataCollector& operator=(const DataCollector&) = delete;

  void Start();
  void Stop();

  VehicleState GetVehicleState() const;
  SystemState  GetSystemState() const;

  void set_vehicle_state_callback(VehicleStateCallback cb);
  void set_system_state_callback(SystemStateCallback cb);
  void set_camera_frame_callback(CameraFrameCallback cb);

  bool IsHealthy() const;

 private:
  void SetupCameraSubscribers();
  void OnCameraImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
                     const std::string& camera_id);
  void OnCompressedImage(
      const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg,
      const std::string& camera_id);

  void SetupVehicleSubscribers();

#ifdef HAVE_AUTOWARE_MSGS
  void OnVelocityReport(
      const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr& msg);
  void OnSteeringReport(
      const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr& msg);
  void OnGearReport(
      const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr& msg);
  void OnControlModeReport(
      const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr& msg);
  void OnTurnIndicatorsReport(
      const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr& msg);
#else
  void OnTwistStamped(
      const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);
  void OnFloat64Steering(const std_msgs::msg::Float64::ConstSharedPtr& msg);
#endif

  void SetupLocalizationSubscribers();
  void OnOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void OnPoseStamped(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

  void SetupSystemSubscribers();
  void SystemMonitorCallback();

  rclcpp::Node::SharedPtr node_;
  const config::VehicleConfigManager& config_;

  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> camera_subs_;
  std::map<std::string, image_transport::Subscriber> camera_it_subs_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> vehicle_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr system_monitor_timer_;

  mutable std::mutex state_mutex_;
  VehicleState vehicle_state_;
  SystemState  system_state_;
  std::map<std::string, uint64_t> camera_frame_counts_;

  VehicleStateCallback vehicle_state_cb_;
  SystemStateCallback  system_state_cb_;
  CameraFrameCallback  camera_frame_cb_;

  std::atomic<bool> running_{false};
};

}  // namespace vehicle
}  // namespace fsm
