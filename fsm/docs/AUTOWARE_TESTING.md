# FSM-Pilot V2.0 Autoware 实车测试指南

## 1. 系统架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      FSM-Pilot + Autoware 系统架构                       │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                         云端 (阿里云)                            │   │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐            │   │
│  │  │  Web 前端   │  │  信令服务器  │  │  TURN 服务器 │            │   │
│  │  │ (FSM-Pilot) │  │ (WebSocket) │  │  (WebRTC)   │            │   │
│  │  └─────────────┘  └─────────────┘  └─────────────┘            │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                              │                                         │
│                              │ 4G/5G/专线                              │
│                              ▼                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                       车端 (实车平台)                            │   │
│  │                                                                  │   │
│  │  ┌────────────────────────────────────────────────────────┐    │   │
│  │  │                    Autoware Universe                    │    │   │
│  │  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐  │    │   │
│  │  │  │ Sensing  │ │ Locali-  │ │ Percep-  │ │ Planning │  │    │   │
│  │  │  │  Stack   │ │ zation   │ │  tion    │ │  Stack   │  │    │   │
│  │  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘  │    │   │
│  │  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐  │    │   │
│  │  │  │ Control  │ │ Vehicle  │ │ System   │ │  API     │  │    │   │
│  │  │  │  Stack   │ │ Interface│ │ Monitor  │ │ Gateway  │  │    │   │
│  │  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘  │    │   │
│  │  └────────────────────────────────────────────────────────┘    │   │
│  │                              │                                  │   │
│  │  ┌────────────────────────────────────────────────────────┐    │   │
│  │  │                 FSM-Pilot 车端适配器                    │    │   │
│  │  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐  │    │   │
│  │  │  │ ROS2     │ │ Video    │ │ Control  │ │ WebRTC   │  │    │   │
│  │  │  │ Bridge   │ │ Encoder  │ │ Handler  │ │ Client   │  │    │   │
│  │  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘  │    │   │
│  │  └────────────────────────────────────────────────────────┘    │   │
│  │                              │                                  │   │
│  │  ┌────────────────────────────────────────────────────────┐    │   │
│  │  │                     硬件层                              │    │   │
│  │  │  ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐       │    │   │
│  │  │  │LiDAR │ │Camera│ │ GNSS │ │ IMU  │ │ CAN  │       │    │   │
│  │  │  └──────┘ └──────┘ └──────┘ └──────┘ └──────┘       │    │   │
│  │  └────────────────────────────────────────────────────────┘    │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## 2. 硬件要求

### 2.1 计算平台

| 组件 | 推荐规格 | 最低规格 |
|-----|---------|---------|
| CPU | Intel i9-12900K 或 AMD Ryzen 9 5950X | Intel i7-10700K |
| GPU | NVIDIA RTX 4090 (24GB) | NVIDIA RTX 3080 (10GB) |
| RAM | 64GB DDR5 | 32GB DDR4 |
| SSD | 2TB NVMe PCIe 4.0 | 1TB NVMe |
| 网络 | 5G/4G LTE 模块 + 以太网 | 4G LTE |

### 2.2 传感器配置

| 传感器类型 | 型号 | 数量 | 用途 |
|-----------|------|------|------|
| LiDAR | Velodyne VLP-16 或 Ouster OS1-64 | 1-2 | 3D 点云感知 |
| Camera | FLIR BFS-PGE-31S4C | 4-6 | 环视感知 |
| GNSS/RTK | Novatel PwrPak7D | 1 | 高精度定位 |
| IMU | Xsens MTi-680G | 1 | 惯性测量 |
| 毫米波雷达 | Continental ARS408 | 4 | 目标检测 |

### 2.3 线控底盘

- **支持车型**:
  - 乘用车: 比亚迪秦 Plus / 广汽 Aion S
  - 商用车: 金旅星辰 / 开沃小巴
- **线控接口**: CAN 2.0B / Ethernet
- **控制模式**: 方向盘、油门、刹车线控

## 3. 软件环境

### 3.1 系统要求

```bash
# 操作系统
Ubuntu 22.04 LTS (Jammy Jellyfish)

# ROS2 版本
ROS 2 Humble Hawksbill

# CUDA
CUDA 12.x
cuDNN 8.x
TensorRT 8.x
```

### 3.2 安装 Autoware

```bash
# 1. 安装依赖
sudo apt update && sudo apt install -y \
  git cmake build-essential \
  python3-pip python3-venv \
  libboost-all-dev libeigen3-dev \
  libpcl-dev libopencv-dev

# 2. 安装 ROS 2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-humble-desktop -y

# 3. 克隆 Autoware
mkdir -p ~/autoware_ws/src
cd ~/autoware_ws/src
git clone https://github.com/autowarefoundation/autoware.git -b humble

# 4. 安装依赖
cd ~/autoware_ws
rosdep install -y --from-paths src --ignore-src --rosdistro humble

# 5. 构建
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 3.3 安装 FSM-Pilot 车端适配器

```bash
# 1. 克隆 FSM-Pilot 车端包
cd ~/autoware_ws/src
git clone https://github.com/your-org/fsm_pilot_vehicle.git

# 2. 安装额外依赖
pip3 install websockets aiortc opencv-python

# 3. 重新构建
cd ~/autoware_ws
colcon build --packages-select fsm_pilot_vehicle --symlink-install
```

## 4. FSM-Pilot 车端适配器

### 4.1 ROS 2 节点架构

```
fsm_pilot_vehicle/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── fsm_pilot_params.yaml
├── launch/
│   └── fsm_pilot_vehicle.launch.py
└── src/
    ├── main.cpp
    ├── fsm_bridge_node.cpp       # ROS2 桥接节点
    ├── video_encoder_node.cpp    # 视频编码节点
    ├── control_handler_node.cpp  # 控制处理节点
    └── webrtc_client.cpp         # WebRTC 客户端
```

### 4.2 配置文件

```yaml
# config/fsm_pilot_params.yaml
fsm_pilot_vehicle:
  ros__parameters:
    # 云端连接配置
    signaling_url: "wss://fsm-pilot.com/signaling"
    vehicle_id: "FSM-01"

    # TURN 服务器
    turn_host: "turn.fsm-pilot.com"
    turn_port: 3478
    turn_username: "turnuser"
    turn_credential: "turnpassword"

    # 视频编码
    video_codec: "h264"
    video_bitrate: 4000000  # 4 Mbps
    video_fps: 30
    video_resolution: [1920, 1080]

    # 摄像头话题
    camera_topics:
      - "/sensing/camera/front/image_raw"
      - "/sensing/camera/rear/image_raw"
      - "/sensing/camera/left/image_raw"
      - "/sensing/camera/right/image_raw"

    # 控制话题
    control_cmd_topic: "/control/command/control_cmd"
    vehicle_status_topic: "/vehicle/status/control_mode"

    # 遥测话题
    gnss_topic: "/sensing/gnss/pose"
    velocity_topic: "/vehicle/status/velocity_status"
    steering_topic: "/vehicle/status/steering_status"

    # 安全参数
    max_steering_angle: 30.0  # 度
    max_speed: 20.0           # m/s
    emergency_decel: -5.0     # m/s^2
    control_timeout: 500      # ms
```

### 4.3 启动文件

```python
# launch/fsm_pilot_vehicle.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('fsm_pilot_vehicle')

    config_file = os.path.join(pkg_dir, 'config', 'fsm_pilot_params.yaml')

    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'vehicle_id',
            default_value='FSM-01',
            description='Vehicle identifier'
        ),
        DeclareLaunchArgument(
            'signaling_url',
            default_value='wss://fsm-pilot.com/signaling',
            description='Signaling server URL'
        ),

        # FSM Bridge 节点
        Node(
            package='fsm_pilot_vehicle',
            executable='fsm_bridge_node',
            name='fsm_bridge',
            parameters=[config_file],
            output='screen',
            remappings=[
                ('/control_cmd', '/control/command/control_cmd'),
                ('/vehicle_status', '/vehicle/status/control_mode')
            ]
        ),

        # 视频编码节点
        Node(
            package='fsm_pilot_vehicle',
            executable='video_encoder_node',
            name='video_encoder',
            parameters=[config_file],
            output='screen'
        ),

        # 控制处理节点
        Node(
            package='fsm_pilot_vehicle',
            executable='control_handler_node',
            name='control_handler',
            parameters=[config_file],
            output='screen'
        ),
    ])
```

### 4.4 控制处理节点

```cpp
// src/control_handler_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <tier4_control_msgs/msg/gate_mode.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

class ControlHandlerNode : public rclcpp::Node
{
public:
    ControlHandlerNode() : Node("control_handler")
    {
        // 参数
        this->declare_parameter("max_steering_angle", 30.0);
        this->declare_parameter("max_speed", 20.0);
        this->declare_parameter("control_timeout", 500);

        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        control_timeout_ms_ = this->get_parameter("control_timeout").as_int();

        // 发布者
        control_cmd_pub_ = this->create_publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>(
            "/control/command/control_cmd", 10);

        gate_mode_pub_ = this->create_publisher<tier4_control_msgs::msg::GateMode>(
            "/control/gate_mode_cmd", 10);

        // 订阅者 (从 FSM Bridge 接收远程控制命令)
        remote_cmd_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
            "/fsm_pilot/remote_cmd", 10,
            std::bind(&ControlHandlerNode::remoteCommandCallback, this, std::placeholders::_1));

        // 安全定时器
        safety_timer_ = this->create_wall_timer(
            100ms, std::bind(&ControlHandlerNode::safetyCheck, this));

        RCLCPP_INFO(this->get_logger(), "Control Handler Node initialized");
    }

private:
    void remoteCommandCallback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
    {
        last_command_time_ = this->now();

        // 验证和限制控制命令
        auto cmd = *msg;

        // 限制转向角
        cmd.lateral.steering_tire_angle = std::clamp(
            cmd.lateral.steering_tire_angle,
            -max_steering_angle_ * M_PI / 180.0,
            max_steering_angle_ * M_PI / 180.0
        );

        // 限制速度
        cmd.longitudinal.speed = std::clamp(cmd.longitudinal.speed, 0.0f, static_cast<float>(max_speed_));

        // 发布控制命令
        control_cmd_pub_->publish(cmd);

        RCLCPP_DEBUG(this->get_logger(), "Remote command: steering=%.2f, speed=%.2f",
            cmd.lateral.steering_tire_angle, cmd.longitudinal.speed);
    }

    void safetyCheck()
    {
        auto now = this->now();
        auto elapsed = (now - last_command_time_).seconds() * 1000;

        if (elapsed > control_timeout_ms_)
        {
            // 超时，发送紧急停车命令
            if (!emergency_stop_sent_)
            {
                RCLCPP_WARN(this->get_logger(), "Control timeout! Sending emergency stop.");
                sendEmergencyStop();
                emergency_stop_sent_ = true;
            }
        }
        else
        {
            emergency_stop_sent_ = false;
        }
    }

    void sendEmergencyStop()
    {
        autoware_auto_control_msgs::msg::AckermannControlCommand cmd;
        cmd.stamp = this->now();
        cmd.lateral.steering_tire_angle = 0.0;
        cmd.longitudinal.speed = 0.0;
        cmd.longitudinal.acceleration = -5.0;  // 紧急制动

        control_cmd_pub_->publish(cmd);
    }

    // 成员变量
    rclcpp::Publisher<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr control_cmd_pub_;
    rclcpp::Publisher<tier4_control_msgs::msg::GateMode>::SharedPtr gate_mode_pub_;
    rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr remote_cmd_sub_;
    rclcpp::TimerBase::SharedPtr safety_timer_;

    rclcpp::Time last_command_time_;
    double max_steering_angle_;
    double max_speed_;
    int control_timeout_ms_;
    bool emergency_stop_sent_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlHandlerNode>());
    rclcpp::shutdown();
    return 0;
}
```

## 5. 测试流程

### 5.1 测试前检查清单

```
□ 硬件检查
  □ 计算平台正常启动
  □ 所有传感器连接正常
  □ 网络连接稳定 (4G/5G 信号 > -80 dBm)
  □ 电池电量充足 (> 50%)
  □ 线控系统自检通过

□ 软件检查
  □ Autoware 正常启动
  □ FSM-Pilot 车端适配器启动
  □ 云端连接成功
  □ 视频流传输正常
  □ 遥测数据上报正常

□ 安全检查
  □ 紧急停车按钮正常
  □ 安全员在车
  □ 测试场地清空
  □ 应急预案准备
```

### 5.2 功能测试

#### 测试 1: 连接测试

```bash
# 在车端启动
source ~/autoware_ws/install/setup.bash
ros2 launch fsm_pilot_vehicle fsm_pilot_vehicle.launch.py

# 检查连接状态
ros2 topic echo /fsm_pilot/connection_status

# 预期输出
# status: "connected"
# signaling: true
# webrtc: true
# latency_ms: 45
```

#### 测试 2: 视频传输测试

```bash
# 检查视频话题
ros2 topic hz /sensing/camera/front/image_raw

# 检查编码器状态
ros2 topic echo /fsm_pilot/encoder_status

# 在云端查看延迟
# 预期: 端到端延迟 < 200ms
```

#### 测试 3: 控制命令测试

```bash
# 模拟远程控制命令 (仅在安全测试场地)
ros2 topic pub /fsm_pilot/remote_cmd \
  autoware_auto_control_msgs/msg/AckermannControlCommand \
  "{stamp: {sec: 0}, lateral: {steering_tire_angle: 0.1}, longitudinal: {speed: 1.0}}"

# 检查实际控制输出
ros2 topic echo /control/command/control_cmd
```

#### 测试 4: 紧急停车测试

```bash
# 1. 启动低速行驶 (< 5 km/h)
# 2. 断开网络连接
# 3. 验证车辆自动停车

# 检查紧急停车日志
ros2 topic echo /fsm_pilot/emergency_stop
```

### 5.3 性能测试

| 测试项 | 目标值 | 测试方法 |
|-------|-------|---------|
| 视频延迟 | < 200ms | ping + 视频时间戳对比 |
| 控制延迟 | < 100ms | 命令发送-执行时间差 |
| 丢包率 | < 1% | WebRTC 统计 |
| 网络切换 | < 2s | 4G/5G 切换测试 |

### 5.4 场地测试路线

```
┌────────────────────────────────────────────────────────────┐
│                      封闭测试场地                           │
│                                                            │
│    START                                                   │
│      ●──────────────────────────────────┐                 │
│                                          │                 │
│      ┌───────────────────────────────────┤                 │
│      │                                   │                 │
│      │   ┌───────────────────────────┐   │                 │
│      │   │      障碍物区域           │   │                 │
│      │   │   (锥桶/假人/停放车辆)    │   │                 │
│      │   └───────────────────────────┘   │                 │
│      │                                   │                 │
│      └───────────────────────────────────┤                 │
│                                          │                 │
│      ┌───────────────────────────────────┘                 │
│      │                                                     │
│      ●                                                     │
│     END                                                    │
│                                                            │
│   测试内容:                                                 │
│   1. 直线行驶 (0-20 km/h)                                  │
│   2. 转弯测试 (左转/右转)                                   │
│   3. 障碍物避让                                             │
│   4. 紧急停车                                               │
└────────────────────────────────────────────────────────────┘
```

## 6. 故障处理

### 6.1 常见问题

| 问题 | 可能原因 | 解决方案 |
|-----|---------|---------|
| 连接失败 | 网络问题 | 检查 4G/5G 信号，切换网络 |
| 视频卡顿 | 带宽不足 | 降低视频分辨率/码率 |
| 控制延迟高 | 网络拥塞 | 使用 TURN TCP 模式 |
| 车辆无响应 | 线控故障 | 检查 CAN 通信 |

### 6.2 紧急处理流程

```
1. 发现异常
     │
     ▼
2. 按下紧急停车按钮 ◄─────┐
     │                   │
     ▼                   │
3. 车辆是否停止?         │
     │                   │
   是│  否              │
     │   └──────────────┘
     ▼
4. 安全员接管
     │
     ▼
5. 记录故障信息
     │
     ▼
6. 分析原因并修复
     │
     ▼
7. 确认安全后恢复测试
```

## 7. 数据记录

### 7.1 RosBag 录制

```bash
# 启动录制
ros2 bag record -a -o ~/rosbag_$(date +%Y%m%d_%H%M%S)

# 指定话题录制
ros2 bag record \
  /sensing/lidar/top/pointcloud_raw \
  /sensing/camera/front/image_raw \
  /sensing/gnss/pose \
  /vehicle/status/velocity_status \
  /control/command/control_cmd \
  -o ~/rosbag_test
```

### 7.2 测试报告模板

```markdown
# FSM-Pilot 实车测试报告

## 基本信息
- 测试日期: YYYY-MM-DD
- 测试地点: XXX
- 测试人员: XXX
- 车辆编号: FSM-XX

## 测试环境
- 天气: 晴/阴/雨
- 温度: XX°C
- 网络: 4G/5G (信号强度: -XX dBm)

## 测试结果

| 测试项 | 结果 | 备注 |
|-------|------|------|
| 连接测试 | ✓/✗ | |
| 视频传输 | ✓/✗ | 延迟: XXms |
| 控制响应 | ✓/✗ | 延迟: XXms |
| 紧急停车 | ✓/✗ | |

## 问题记录
1. 问题描述...
   - 原因分析
   - 解决方案

## 结论与建议
...
```

## 8. 安全须知

1. **必须在封闭场地测试**，禁止在公共道路测试
2. **必须有安全员在车**，随时准备接管
3. **测试前检查紧急停车系统**
4. **保持网络连接监控**，出现断连立即停车
5. **遵守测试速度限制** (通常 < 20 km/h)
6. **记录所有测试数据**，便于分析问题
