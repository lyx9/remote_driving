# FSM-Pilot 远程驾驶系统架构设计文档

## 1. 系统概述

### 1.1 项目目标
构建一个基于ROS2和WebRTC的产品级远程驾驶平台，支持：
- 多车辆远程监控和控制
- 实时视频流传输
- 低延迟指令控制
- 智能调度算法
- 预测模型可视化补偿

### 1.2 技术选型

| 层级 | 技术栈 |
|------|--------|
| 车端OS | Ubuntu 22.04 LTS |
| 车端框架 | ROS2 Humble + Autoware.universe |
| 通信协议 | WebRTC (libwebrtc) |
| 信令服务 | WebSocket (阿里云部署) |
| 云端服务 | C++17 / Node.js |
| 前端 | Vue 3 + TypeScript |
| 数据库 | Redis (实时状态) + PostgreSQL (历史) |

## 2. 系统架构图

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                              FSM-Pilot 远程驾驶系统                               │
└─────────────────────────────────────────────────────────────────────────────────┘

                                    公共网络 (Internet)
                                          │
                    ┌─────────────────────┼─────────────────────┐
                    │                     │                     │
                    ▼                     ▼                     ▼
         ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
         │   Vehicle #1     │  │   Vehicle #2     │  │   Vehicle #3     │
         │   (ROS2 Node)    │  │   (ROS2 Node)    │  │   (ROS2 Node)    │
         └────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘
                  │                     │                     │
                  │              WebRTC P2P / TURN            │
                  │                     │                     │
                  └─────────────────────┼─────────────────────┘
                                        │
                         ┌──────────────▼──────────────┐
                         │     阿里云 TURN/STUN Server  │
                         │     + Signaling Server      │
                         └──────────────┬──────────────┘
                                        │
                         ┌──────────────▼──────────────┐
                         │       云端调度中心           │
                         │   (Scheduling Service)     │
                         │   - 多车辆状态聚合          │
                         │   - 调度算法                │
                         │   - 预测模型接口            │
                         └──────────────┬──────────────┘
                                        │
                         ┌──────────────▼──────────────┐
                         │       远程驾驶工作站         │
                         │   - Vue 3 前端界面          │
                         │   - 罗技方向盘输入          │
                         │   - 多屏显示               │
                         └─────────────────────────────┘


═══════════════════════════════════════════════════════════════════════════════════
                              详细模块架构
═══════════════════════════════════════════════════════════════════════════════════

┌─────────────────────────────────────────────────────────────────────────────────┐
│                              车端系统 (Vehicle Side)                             │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                        Autoware.universe                                 │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐  │   │
│  │  │   Sensing    │  │  Localization │  │   Planning   │  │   Control   │  │   │
│  │  │  /camera/*   │  │  /localization│  │  /planning/* │  │ /control/*  │  │   │
│  │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘  └──────┬──────┘  │   │
│  │         │                 │                 │                 │         │   │
│  └─────────┼─────────────────┼─────────────────┼─────────────────┼─────────┘   │
│            │                 │                 │                 │             │
│            └─────────────────┼─────────────────┼─────────────────┘             │
│                              │                 │                               │
│  ┌───────────────────────────▼─────────────────▼───────────────────────────┐   │
│  │                   FSM Vehicle Node (C++ ROS2)                           │   │
│  │  ┌──────────────────────────────────────────────────────────────────┐  │   │
│  │  │  Data Collector                                                   │  │   │
│  │  │  - Camera Subscriber (多路摄像头)                                  │  │   │
│  │  │  - Vehicle Status Subscriber (底盘信息)                            │  │   │
│  │  │  - Localization Subscriber (定位信息)                              │  │   │
│  │  │  - Planning Status Subscriber (规划状态)                           │  │   │
│  │  │  - System Monitor (CPU/GPU/网络)                                  │  │   │
│  │  └──────────────────────────────────────────────────────────────────┘  │   │
│  │  ┌──────────────────────────────────────────────────────────────────┐  │   │
│  │  │  WebRTC Client                                                    │  │   │
│  │  │  - Video Encoder (H.264/VP8)                                      │  │   │
│  │  │  - Data Channel (底盘数据/状态)                                    │  │   │
│  │  │  - Control Receiver (远程指令)                                     │  │   │
│  │  └──────────────────────────────────────────────────────────────────┘  │   │
│  │  ┌──────────────────────────────────────────────────────────────────┐  │   │
│  │  │  Command Executor                                                 │  │   │
│  │  │  - Steering Command → /control/command/steering                   │  │   │
│  │  │  - Throttle/Brake → /control/command/longitudinal                 │  │   │
│  │  │  - Gear Command → /control/command/gear                           │  │   │
│  │  │  - Emergency Stop → /system/emergency/stop                        │  │   │
│  │  └──────────────────────────────────────────────────────────────────┘  │   │
│  │  ┌──────────────────────────────────────────────────────────────────┐  │   │
│  │  │  Config Manager (YAML配置)                                        │  │   │
│  │  │  - 摄像头配置 (topic, 分辨率, 帧率)                                 │  │   │
│  │  │  - 车辆参数 (轴距, 最大转角等)                                      │  │   │
│  │  │  - 网络配置 (STUN/TURN服务器)                                      │  │   │
│  │  └──────────────────────────────────────────────────────────────────┘  │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘


┌─────────────────────────────────────────────────────────────────────────────────┐
│                              云端系统 (Cloud Side)                               │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                     Signaling Server (Node.js/C++)                      │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                  │   │
│  │  │  WebSocket   │  │  Session Mgr │  │  ICE Handler │                  │   │
│  │  │   Handler    │  │              │  │              │                  │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘                  │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                     Scheduling Service (C++)                            │   │
│  │  ┌──────────────────────────────────────────────────────────────────┐  │   │
│  │  │  Vehicle State Aggregator                                         │  │   │
│  │  │  - 多车辆状态汇总                                                   │  │   │
│  │  │  - 通信延迟监控                                                     │  │   │
│  │  │  - 健康状态检查                                                     │  │   │
│  │  └──────────────────────────────────────────────────────────────────┘  │   │
│  │  ┌──────────────────────────────────────────────────────────────────┐  │   │
│  │  │  Scheduling Algorithm (可配置)                                     │  │   │
│  │  │  - 基于延迟的优先级排序                                             │  │   │
│  │  │  - 基于紧急程度的排序                                               │  │   │
│  │  │  - 基于位置的排序                                                   │  │   │
│  │  │  - 综合权重调度算法                                                 │  │   │
│  │  └──────────────────────────────────────────────────────────────────┘  │   │
│  │  ┌──────────────────────────────────────────────────────────────────┐  │   │
│  │  │  Alert Analyzer                                                   │  │   │
│  │  │  - Topic状态分析                                                    │  │   │
│  │  │  - 异常检测                                                         │  │   │
│  │  │  - 告警生成                                                         │  │   │
│  │  └──────────────────────────────────────────────────────────────────┘  │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                     Prediction Interface (C++)                          │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                  │   │
│  │  │  Model API   │  │  Latency     │  │  Visual      │                  │   │
│  │  │  Connector   │  │  Compensator │  │  Overlay Gen │                  │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘                  │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘


┌─────────────────────────────────────────────────────────────────────────────────┐
│                           远程驾驶工作站 (Operator Side)                          │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                     WebRTC Operator Client (C++)                        │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                  │   │
│  │  │  Video       │  │  Data Channel│  │  Command     │                  │   │
│  │  │  Decoder     │  │  Receiver    │  │  Sender      │                  │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘                  │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                     Logitech Wheel Interface (C++)                      │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                  │   │
│  │  │  G29/G920    │  │  Force       │  │  Input       │                  │   │
│  │  │  Driver      │  │  Feedback    │  │  Mapper      │                  │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘                  │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │                     Vue 3 Frontend (TypeScript)                         │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐ │   │
│  │  │  Video Wall  │  │  Telemetry   │  │  Map View    │  │   Alerts    │ │   │
│  │  │  Component   │  │  Dashboard   │  │  Component   │  │   Panel     │ │   │
│  │  └──────────────┘  └──────────────┘  └──────────────┘  └─────────────┘ │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

## 3. 核心模块详细设计

### 3.1 车端数据采集模块

#### 3.1.1 支持的ROS2 Topics (Autoware.universe兼容)

**摄像头数据:**
```yaml
cameras:
  - name: front_left
    topic: /sensing/camera/front_left/image_raw
    encoding: bgr8
    fps: 30
    resolution: [1280, 720]

  - name: front_center
    topic: /sensing/camera/front_center/image_raw
    encoding: bgr8
    fps: 30
    resolution: [1920, 1080]

  - name: front_right
    topic: /sensing/camera/front_right/image_raw
    encoding: bgr8
    fps: 30
    resolution: [1280, 720]

  - name: rear_left
    topic: /sensing/camera/rear_left/image_raw
    encoding: bgr8
    fps: 30
    resolution: [1280, 720]

  - name: rear_right
    topic: /sensing/camera/rear_right/image_raw
    encoding: bgr8
    fps: 30
    resolution: [1280, 720]
```

**底盘信息:**
```yaml
vehicle_status:
  velocity: /vehicle/status/velocity_status
  steering: /vehicle/status/steering_status
  gear: /vehicle/status/gear_status
  turn_indicators: /vehicle/status/turn_indicators_status
  hazard_lights: /vehicle/status/hazard_lights_status
  control_mode: /vehicle/status/control_mode
```

**定位信息:**
```yaml
localization:
  pose: /localization/kinematic_state
  acceleration: /localization/acceleration
  twist: /sensing/gnss/twist
```

**系统状态:**
```yaml
system:
  diagnostics: /diagnostics_agg
  cpu_monitor: /system/cpu_monitor
  emergency: /system/emergency/emergency_state
```

### 3.2 WebRTC通信模块

#### 3.2.1 数据通道设计

```cpp
// 数据通道类型定义
enum class DataChannelType {
    TELEMETRY,      // 遥测数据 (底盘、定位)
    SYSTEM_STATUS,  // 系统状态 (CPU、网络)
    CONTROL_CMD,    // 控制指令 (方向盘、油门)
    ALERTS,         // 告警信息
    HEARTBEAT       // 心跳检测
};

// 消息格式 (Protocol Buffers)
message TelemetryData {
    double timestamp = 1;
    VelocityStatus velocity = 2;
    SteeringStatus steering = 3;
    GearStatus gear = 4;
    LocalizationPose pose = 5;
}

message ControlCommand {
    double timestamp = 1;
    float steering_angle = 2;  // -1.0 to 1.0
    float throttle = 3;        // 0.0 to 1.0
    float brake = 4;           // 0.0 to 1.0
    int32 gear = 5;            // P=0, R=1, N=2, D=3
    bool emergency_stop = 6;
}
```

#### 3.2.2 延迟测量机制

```
     Vehicle                          Operator
        │                                │
        │  ◄─────── t1: Send Ping ──────│
        │                                │
        │  ─────── t2: Receive Ping ───►│
        │                                │
        │  ◄─────── t3: Send Pong ──────│
        │                                │
        │  ─────── t4: Receive Pong ───►│
        │                                │
    RTT = t4 - t1
    One-way Latency ≈ RTT / 2
```

### 3.3 云端调度算法

#### 3.3.1 多车辆调度优先级计算

```
Priority Score = w1 * Emergency_Score
               + w2 * Latency_Score
               + w3 * Distance_Score
               + w4 * Battery_Score
               + w5 * Task_Priority_Score

其中:
- Emergency_Score: 紧急程度 (0-100)
- Latency_Score: 通信延迟评分 (延迟越高分数越高)
- Distance_Score: 距离目标点的距离评分
- Battery_Score: 电量/燃料评分
- Task_Priority_Score: 任务优先级

权重可配置:
w1 = 0.35 (紧急程度权重最高)
w2 = 0.25
w3 = 0.20
w4 = 0.10
w5 = 0.10
```

### 3.4 远程控制指令映射

#### 3.4.1 罗技方向盘映射表

```yaml
logitech_wheel_mapping:
  # G29/G920 轴映射
  axes:
    steering: 0        # 方向盘 (-32768 to 32767)
    throttle: 2        # 油门踏板 (-32768 to 32767)
    brake: 3           # 刹车踏板 (-32768 to 32767)
    clutch: 1          # 离合器 (可选)

  # 按钮映射
  buttons:
    emergency_stop: 0  # X按钮
    horn: 1            # □按钮
    left_signal: 4     # L1
    right_signal: 5    # R1
    gear_up: 6         # 右拨片
    gear_down: 7       # 左拨片

  # 转换参数
  conversion:
    steering_ratio: 450.0    # 方向盘转角范围 (度)
    steering_deadzone: 0.02  # 死区
    throttle_curve: "linear" # 油门曲线
    brake_curve: "linear"    # 刹车曲线
```

### 3.5 预测模型接口

#### 3.5.1 延迟补偿可视化

```
当前帧 (t0)                    预测帧 (t0 + latency)
    │                                │
    ▼                                ▼
┌─────────┐                    ┌─────────┐
│ 实际位置 │  ───延迟补偿───►  │ 预测位置 │
└─────────┘                    └─────────┘
    │                                │
    │         视觉叠加显示           │
    │                                │
    ▼                                ▼
┌─────────────────────────────────────────┐
│           Video Frame                    │
│   ┌─────┐                  ┌ ─ ─ ─ ┐   │
│   │ Car │  ──────────────► │(Ghost)│   │
│   └─────┘   Predicted Path └ ─ ─ ─ ┘   │
│                                         │
└─────────────────────────────────────────┘
```

## 4. 配置文件设计

### 4.1 车端配置 (vehicle_config.yaml)

```yaml
# FSM-Pilot Vehicle Configuration
# 版本: 1.0

vehicle:
  id: "FSM-01"
  type: "ROBO-TAXI"
  name: "Test Vehicle 1"

  # 车辆物理参数
  parameters:
    wheelbase: 2.7          # 轴距 (m)
    max_steering_angle: 35  # 最大转向角 (度)
    max_speed: 60           # 最大速度 (km/h)

# 传感器配置
sensors:
  cameras:
    enabled: true
    list:
      - id: "cam_front_center"
        topic: "/sensing/camera/front_center/image_raw"
        fps: 30
        width: 1920
        height: 1080
        encoding: "h264"
        bitrate: 4000000
      # ... 更多摄像头

# WebRTC配置
webrtc:
  stun_servers:
    - "stun:stun.l.google.com:19302"
    - "stun:stun.aliyun.com:3478"

  turn_servers:
    - url: "turn:turn.example.com:3478"
      username: "user"
      credential: "password"

  signaling:
    url: "wss://signal.example.com/ws"
    reconnect_interval: 5000

# 控制配置
control:
  # Autoware控制话题
  topics:
    steering_cmd: "/control/command/steering_cmd"
    velocity_cmd: "/control/command/velocity_cmd"
    gear_cmd: "/vehicle/command/gear_cmd"
    emergency: "/system/emergency/emergency_cmd"

  # 安全限制
  safety:
    max_steering_rate: 30    # 度/秒
    max_acceleration: 2.0    # m/s²
    max_deceleration: 5.0    # m/s²
    emergency_decel: 9.0     # m/s²
```

### 4.2 云端配置 (cloud_config.yaml)

```yaml
# FSM-Pilot Cloud Configuration

server:
  signaling_port: 8080
  api_port: 8081

# 调度算法配置
scheduling:
  enabled: true
  algorithm: "weighted_priority"

  weights:
    emergency: 0.35
    latency: 0.25
    distance: 0.20
    battery: 0.10
    task_priority: 0.10

  # 阈值配置
  thresholds:
    max_latency_ms: 200       # 最大允许延迟
    critical_latency_ms: 500  # 临界延迟
    min_battery_percent: 20   # 最低电量

# 告警配置
alerts:
  enabled: true
  rules:
    - name: "high_latency"
      condition: "latency > 300"
      severity: "warning"
    - name: "critical_latency"
      condition: "latency > 500"
      severity: "critical"
    - name: "emergency_state"
      condition: "emergency == true"
      severity: "critical"

# 预测模型配置
prediction:
  enabled: false  # 可配置开关
  model_endpoint: "http://localhost:8090/predict"
  compensation_enabled: false
```

## 5. API接口设计

### 5.1 REST API

```
GET    /api/v1/vehicles              # 获取所有车辆列表
GET    /api/v1/vehicles/{id}         # 获取单个车辆详情
GET    /api/v1/vehicles/{id}/status  # 获取车辆实时状态
POST   /api/v1/vehicles/{id}/connect # 建立WebRTC连接
DELETE /api/v1/vehicles/{id}/connect # 断开连接

GET    /api/v1/scheduling/queue      # 获取调度队列
PUT    /api/v1/scheduling/config     # 更新调度配置

GET    /api/v1/alerts                # 获取告警列表
POST   /api/v1/alerts/{id}/ack       # 确认告警

GET    /api/v1/prediction/config     # 获取预测配置
PUT    /api/v1/prediction/config     # 更新预测配置
```

### 5.2 WebSocket Events

```javascript
// 车辆状态更新
{
  "event": "vehicle_status",
  "data": {
    "vehicle_id": "FSM-01",
    "timestamp": 1703145600000,
    "speed": 35.5,
    "steering": 5.2,
    "gear": "D",
    "location": { "lat": 31.2304, "lng": 121.4737 },
    "latency_ms": 45
  }
}

// 告警事件
{
  "event": "alert",
  "data": {
    "id": "alert_001",
    "vehicle_id": "FSM-01",
    "type": "high_latency",
    "severity": "warning",
    "message": "通信延迟超过阈值: 320ms",
    "timestamp": 1703145600000
  }
}

// 调度更新
{
  "event": "scheduling_update",
  "data": {
    "queue": [
      { "vehicle_id": "FSM-02", "priority": 85, "reason": "紧急任务" },
      { "vehicle_id": "FSM-01", "priority": 60, "reason": "常规任务" },
      { "vehicle_id": "FSM-03", "priority": 45, "reason": "待机" }
    ]
  }
}
```

## 6. 部署架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         阿里云部署                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ┌─────────────┐    ┌─────────────┐    ┌─────────────┐       │
│   │   ECS-1     │    │   ECS-2     │    │   ECS-3     │       │
│   │ Signaling   │    │ Scheduling  │    │   TURN      │       │
│   │  Server     │    │  Service    │    │  Server     │       │
│   └──────┬──────┘    └──────┬──────┘    └──────┬──────┘       │
│          │                  │                  │               │
│   ┌──────▼──────────────────▼──────────────────▼──────┐       │
│   │                  SLB (负载均衡)                    │       │
│   └──────────────────────────┬────────────────────────┘       │
│                              │                                 │
│   ┌──────────────────────────▼────────────────────────┐       │
│   │              Redis (状态缓存)                      │       │
│   └───────────────────────────────────────────────────┘       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## 7. 测试用例设计

### 7.1 单元测试

- WebRTC连接建立/断开
- 视频编解码
- 控制指令序列化/反序列化
- 调度算法计算
- 延迟测量准确性

### 7.2 集成测试

- 端到端延迟测试
- 多车辆同时连接
- 网络中断恢复
- 紧急停车响应

### 7.3 性能测试

- 最大并发连接数
- 视频流带宽占用
- 控制指令响应延迟
- 调度算法执行时间

## 8. 安全设计

- TLS/DTLS加密传输
- 身份认证 (JWT)
- 指令签名验证
- 操作审计日志
- 紧急停车保护
