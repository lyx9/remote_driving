#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
// diagnostic_msgs 是可选的，如果需要诊断信息支持请安装
// #include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include "fsm/config_manager.hpp"
#include "fsm/logger.hpp"

#include <memory>
#include <functional>
#include <mutex>
#include <atomic>
#include <map>

// Autoware messages (conditional compilation)
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

/**
 * @brief 车辆状态数据结构
 */
struct VehicleState {
    // 速度信息
    double longitudinal_velocity = 0.0;  // m/s
    double lateral_velocity = 0.0;       // m/s
    double heading_rate = 0.0;           // rad/s

    // 转向信息
    double steering_tire_angle = 0.0;    // rad

    // 档位信息
    int gear = 0;  // 0:P, 1:R, 2:N, 3:D

    // 控制模式
    int control_mode = 0;  // 0:MANUAL, 1:AUTONOMOUS, 2:REMOTE

    // 信号灯
    int turn_indicator = 0;  // 0:NONE, 1:LEFT, 2:RIGHT
    bool hazard_lights = false;

    // 定位
    double pose_x = 0.0;
    double pose_y = 0.0;
    double pose_z = 0.0;
    double orientation_x = 0.0;
    double orientation_y = 0.0;
    double orientation_z = 0.0;
    double orientation_w = 1.0;

    // GPS
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;

    // 时间戳
    int64_t timestamp_ns = 0;

    // 有效性标志
    bool velocity_valid = false;
    bool steering_valid = false;
    bool localization_valid = false;
};

/**
 * @brief 系统状态数据结构
 */
struct SystemState {
    float cpu_usage = 0.0;
    float memory_usage = 0.0;
    float gpu_usage = 0.0;
    float disk_usage = 0.0;
    float network_tx_bps = 0.0;
    float network_rx_bps = 0.0;

    std::vector<std::pair<std::string, int>> diagnostics;  // name, level

    int64_t timestamp_ns = 0;
};

/**
 * @brief 摄像头帧数据
 */
struct CameraFrame {
    std::string camera_id;
    cv::Mat image;
    int64_t timestamp_ns;
    uint64_t frame_number;
};

/**
 * @brief 数据采集器回调类型定义
 */
using VehicleStateCallback = std::function<void(const VehicleState&)>;
using SystemStateCallback = std::function<void(const SystemState&)>;
using CameraFrameCallback = std::function<void(const CameraFrame&)>;

/**
 * @brief ROS2数据采集器
 *
 * 从Autoware.universe采集传感器和车辆状态数据
 */
class DataCollector {
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     * @param config 车辆配置
     */
    DataCollector(
        rclcpp::Node::SharedPtr node,
        const config::VehicleConfigManager& config);

    ~DataCollector();

    /**
     * @brief 启动数据采集
     */
    void start();

    /**
     * @brief 停止数据采集
     */
    void stop();

    /**
     * @brief 获取当前车辆状态
     */
    VehicleState getVehicleState() const;

    /**
     * @brief 获取当前系统状态
     */
    SystemState getSystemState() const;

    /**
     * @brief 注册车辆状态回调
     */
    void setVehicleStateCallback(VehicleStateCallback callback);

    /**
     * @brief 注册系统状态回调
     */
    void setSystemStateCallback(SystemStateCallback callback);

    /**
     * @brief 注册摄像头帧回调
     */
    void setCameraFrameCallback(CameraFrameCallback callback);

    /**
     * @brief 检查数据采集是否正常
     */
    bool isHealthy() const;

private:
    // 摄像头订阅者管理
    void setupCameraSubscribers();
    void onCameraImage(
        const sensor_msgs::msg::Image::ConstSharedPtr& msg,
        const std::string& camera_id);
    void onCompressedImage(
        const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg,
        const std::string& camera_id);

    // 车辆状态订阅
    void setupVehicleSubscribers();

#ifdef HAVE_AUTOWARE_MSGS
    void onVelocityReport(const autoware_auto_vehicle_msgs::msg::VelocityReport::ConstSharedPtr& msg);
    void onSteeringReport(const autoware_auto_vehicle_msgs::msg::SteeringReport::ConstSharedPtr& msg);
    void onGearReport(const autoware_auto_vehicle_msgs::msg::GearReport::ConstSharedPtr& msg);
    void onControlModeReport(const autoware_auto_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr& msg);
    void onTurnIndicatorsReport(const autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport::ConstSharedPtr& msg);
#else
    // 通用消息处理 (geometry_msgs::TwistStamped 等)
    void onTwistStamped(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);
    void onFloat64Steering(const std_msgs::msg::Float64::ConstSharedPtr& msg);
#endif

    // 定位订阅
    void setupLocalizationSubscribers();
    void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void onPoseStamped(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

    // 系统状态订阅
    void setupSystemSubscribers();
    // void onDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr& msg);

    // 系统监控定时器
    void systemMonitorCallback();

    rclcpp::Node::SharedPtr node_;
    const config::VehicleConfigManager& config_;

    // 订阅者
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> camera_subs_;
    std::map<std::string, image_transport::Subscriber> camera_it_subs_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> vehicle_subs_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    // rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;

    // 定时器
    rclcpp::TimerBase::SharedPtr system_monitor_timer_;

    // 状态数据 (线程安全)
    mutable std::mutex state_mutex_;
    VehicleState vehicle_state_;
    SystemState system_state_;
    std::map<std::string, uint64_t> camera_frame_counts_;

    // 回调函数
    VehicleStateCallback vehicle_state_callback_;
    SystemStateCallback system_state_callback_;
    CameraFrameCallback camera_frame_callback_;

    // 运行状态
    std::atomic<bool> running_{false};
};

}  // namespace vehicle
}  // namespace fsm
