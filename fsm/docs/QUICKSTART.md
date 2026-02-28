# FSM-Pilot 快速上手指南

> 面向没有远程驾驶/WebRTC 背景的开发者。按照本文档可在 30 分钟内完成本地自测，1 小时内完成云端部署。

---

## 目录

1. [项目架构一览](#1-项目架构一览)
2. [本地自测（无需云服务器）](#2-本地自测)
3. [编译 C++ 组件](#3-编译-c-组件)
4. [云端一键部署（阿里云 ECS）](#4-云端部署)
5. [车端安装](#5-车端安装)
6. [操作员客户端使用](#6-操作员客户端)
7. [常见问题排查](#7-常见问题)

---

## 1. 项目架构一览

```
操作员 PC ──WebSocket──► 阿里云 ECS (Nginx/TLS)
                                │
                         SignalingServer :8080
                         REST API        :8081
                                │
                         ◄── WebSocket ──── 车辆 (4G/5G)
                              (relay mode)
```

**三个核心组件：**

| 组件 | 位置 | 职责 |
|------|------|------|
| `fsm_cloud_server` | 阿里云 ECS | WebSocket 信令中继、调度、REST API |
| `fsm_vehicle_node` | 车载工控机 (ROS2) | 采集传感器数据、执行控制指令 |
| `fsm_operator_client` | 操作员 PC | 方向盘输入、视频显示 |

**通信模式（Relay Mode）：**
所有数据经云端信令服务器中继，无需 P2P WebRTC。

```
遥测数据流: 车辆 → (JSON/WebSocket) → 云端 → (JSON/WebSocket) → 操作员
控制指令流: 操作员 → (JSON/WebSocket) → 云端 → (JSON/WebSocket) → 车辆
```

---

## 2. 本地自测

无需 ECS，在本地一键验证车-云-端通信链路。

### 2.1 安装 Python 依赖

```bash
pip3 install websockets
```

### 2.2 编译云端服务器（先看第 3 节）

```bash
cd /path/to/fsm/cpp
source /opt/ros/humble/setup.bash   # 如果有 ROS2
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build build --target fsm_cloud_server -j$(nproc)
```

### 2.3 运行自测脚本

```bash
bash test/run_self_test.sh
```

**自测流程：**
1. 启动本地 `fsm_cloud_server`（端口 8080/8081）
2. 启动 Python 模拟车辆 (`mock_vehicle.py`) → 每 100ms 发送遥测
3. 启动 Python 模拟操作员 (`mock_operator.py`) → 每 50ms 发送控制指令
4. 自动验证双向通信 + REST API
5. 打印 PASS/FAIL 汇总

**期望输出：**
```
[PASS] Cloud server binary found
[PASS] Cloud server is running
[PASS] Mock vehicle running
[PASS] Mock operator running
[PASS] Vehicle registered with signaling server
[PASS] Vehicle detected operator connection
[PASS] Vehicle is sending telemetry
[PASS] Vehicle received control commands from operator
[PASS] Operator registered with signaling server
[PASS] Operator received vehicle_relay_ready
[PASS] Operator is receiving telemetry from vehicle
[PASS] REST API health endpoint returned 200
[PASS] REST API /api/v1/vehicles shows connected vehicle
[PASS] Cloud server still running
══════════════════════════════
  ALL TESTS PASSED
══════════════════════════════
```

### 2.4 手动运行单个组件（调试用）

```bash
# 终端 1：启动云端服务器
./build/cloud_server/fsm_cloud_server cloud_server/config/cloud_config.yaml

# 终端 2：模拟车辆
python3 test/mock_vehicle.py ws://localhost:8080 FSM-01

# 终端 3：模拟操作员
python3 test/mock_operator.py ws://localhost:8080 FSM-01

# 终端 4：查询 REST API
curl http://localhost:8081/api/v1/vehicles
curl http://localhost:8081/api/v1/health
```

---

## 3. 编译 C++ 组件

### 3.1 依赖项

**系统依赖（Ubuntu 22.04）：**

```bash
sudo apt-get install -y \
    libboost-all-dev \
    libssl-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libprotobuf-dev \
    protobuf-compiler \
    nlohmann-json3-dev \
    libopencv-dev \
    libx264-dev
```

**ROS2 Humble（车端和云端编译时需要，操作员端不需要）：**

```bash
# 参考: https://docs.ros.org/en/humble/Installation.html
sudo apt-get install -y \
    ros-humble-rclcpp \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs
```

### 3.2 编译云端服务器（不需要 ROS2）

```bash
cd /path/to/fsm/cpp
source /opt/ros/humble/setup.bash   # 需要 ROS2 环境（ament_cmake）
cmake -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DUSE_GRPC=OFF
cmake --build build --target fsm_cloud_server -j$(nproc)
```

二进制位于：`build/cloud_server/fsm_cloud_server`

### 3.3 编译操作员客户端（不需要 ROS2）

```bash
source /opt/ros/humble/setup.bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build build --target fsm_operator_client -j$(nproc)
```

二进制位于：`build/operator_client/fsm_operator_client`

### 3.4 编译车端节点（需要 ROS2 Humble）

```bash
source /opt/ros/humble/setup.bash
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build build --target fsm_vehicle_node -j$(nproc)
```

或直接使用车端安装脚本（推荐）：

```bash
sudo bash deploy/scripts/setup_vehicle.sh your.domain.com FSM-01
```

---

## 4. 云端部署

### 4.1 前置条件

- 阿里云 ECS（Ubuntu 22.04 LTS，推荐 4 核 8GB）
- 已注册域名，A 记录解析到 ECS 公网 IP
- 安全组已开放端口（见下表）

| 端口 | 协议 | 说明 |
|------|------|------|
| 22 | TCP | SSH |
| 80 | TCP | HTTP（Let's Encrypt 验证） |
| 443 | TCP | HTTPS/WSS |
| 3478 | TCP+UDP | TURN/STUN |
| 49152-65535 | UDP | TURN relay |

### 4.2 一键部署

```bash
# 在 ECS 上克隆代码
git clone <仓库地址> /opt/fsm
cd /opt/fsm

# 设置参数（可选，脚本会交互提示）
export FSM_DOMAIN="your.domain.com"
export FSM_EMAIL="admin@your.domain.com"
export FSM_PUBLIC_IP="1.2.3.4"       # ECS 公网 IP

# 执行部署（约 10-15 分钟）
sudo bash deploy/scripts/deploy.sh
```

部署完成后输出：
```
═══════════════════════════════════════════════════════
  FSM-Pilot 部署完成！
  信令服务 (WSS):  wss://your.domain.com/ws
  REST API:        https://your.domain.com/api/v1
  TURN 服务:       turn:your.domain.com:3478
═══════════════════════════════════════════════════════
```

### 4.3 验证部署

```bash
# 检查容器状态
docker compose -f /opt/fsm/deploy/docker-compose.yml ps

# 测试 WebSocket 连接
wscat -c wss://your.domain.com/ws
# 发送: {"type":"register","client_type":"operator"}

# 测试 REST API
curl https://your.domain.com/api/v1/health
```

---

## 5. 车端安装

### 5.1 前置条件

- Ubuntu 22.04 + ROS2 Humble 已安装
- 可访问公网（4G/5G 或 WiFi）

### 5.2 安装

```bash
cd /path/to/fsm
sudo bash deploy/scripts/setup_vehicle.sh your.domain.com FSM-01
```

脚本执行：
1. 检查 ROS2 Humble 环境
2. 安装 C++ 依赖
3. 编译 `fsm_vehicle_node`
4. 生成 `/opt/fsm_vehicle/config/vehicle_config.yaml`
5. 安装并启用 `systemd` 服务

### 5.3 启动车端节点

```bash
# 启动服务
sudo systemctl start fsm-vehicle

# 查看日志
journalctl -u fsm-vehicle -f

# 调试模式（前台运行）
source /opt/ros/humble/setup.bash
/opt/fsm_vehicle/bin/fsm_vehicle_node \
    /opt/fsm_vehicle/config/vehicle_config.yaml
```

**成功连接的日志：**
```
[INFO] Connecting to signaling: wss://your.domain.com/ws
[INFO] WS connected (TLS): your.domain.com:443/ws
[INFO] Registering vehicle_id=FSM-01
[INFO] Registered: client_id=client_1
```

### 5.4 多平台支持

在 `vehicle_config.yaml` 中设置 `platform_type`：

```yaml
vehicle:
  platform_type: "autoware"    # autoware | generic_ros2 | carla_sim | lgsvl_sim
```

| 平台 | 说明 |
|------|------|
| `autoware` | Autoware.universe（自动使用 Autoware 话题） |
| `generic_ros2` | 通用 ROS2（发布 `/cmd_vel` TwistStamped） |
| `carla_sim` | CARLA 仿真器 |
| `lgsvl_sim` | LGSVL 仿真器 |

---

## 6. 操作员客户端

### 6.1 运行

```bash
./build/operator_client/fsm_operator_client \
    --signaling wss://your.domain.com/ws \
    --vehicle   FSM-01
```

或本地测试（无 TLS）：

```bash
./build/operator_client/fsm_operator_client \
    --signaling ws://localhost:8080 \
    --vehicle   FSM-TEST-01
```

### 6.2 方向盘支持

支持标准 USB HID 方向盘（Logitech G920/G29/G923 等）：

```
方向盘左/右    → steering (-1.0 ~ +1.0)
右踏板（油门） → throttle (0.0 ~ 1.0)
中踏板（制动） → brake    (0.0 ~ 1.0)
左踏板（离合） → 忽略
换挡拨片 +/-   → 换挡 (P/R/N/D)
左/右转向灯键  → 转向信号
红色按钮       → 紧急停车
```

无方向盘时自动进入键盘模式（功能受限）。

### 6.3 REST API

云端提供 HTTP REST API 用于监控：

```bash
# 车辆列表
curl https://your.domain.com/api/v1/vehicles

# 调度队列
curl https://your.domain.com/api/v1/queue

# 活跃告警
curl https://your.domain.com/api/v1/alerts

# 确认告警
curl -X POST https://your.domain.com/api/v1/alerts/{id}/ack

# 健康检查
curl https://your.domain.com/api/v1/health
```

---

## 7. 常见问题

### Q: 自测时 `mock_vehicle.py` 连接失败

```
[VEHICLE] Connection failed: [Errno 111] Connect call failed. Retrying...
```

**解决：** 确认云端服务器已启动：
```bash
./build/cloud_server/fsm_cloud_server cloud_server/config/cloud_config.yaml
# 应看到: Signaling server on port 8080
```

---

### Q: 车端 `systemctl start fsm-vehicle` 立即失败

```bash
journalctl -u fsm-vehicle -n 50
```

常见原因：
- **配置文件不存在：** `ls /opt/fsm_vehicle/config/vehicle_config.yaml`
- **域名无法解析：** `nslookup your.domain.com`
- **ROS2 话题不存在：** `ros2 topic list | grep vehicle`

---

### Q: 操作员看不到遥测数据

1. 确认车端日志显示 `Operator connected`
2. 确认云端日志没有 relay 错误
3. 检查 `vehicle_config.yaml` 中 `webrtc.signaling.url` 是否正确

---

### Q: 如何修改调度权重？

编辑 `deploy/configs/cloud_config.prod.yaml`：

```yaml
scheduling:
  weights:
    emergency:      0.35   # 紧急情况权重
    latency:        0.25   # 低延迟优先
    distance:       0.20   # 距离权重
    battery:        0.10   # 电量权重
    task_priority:  0.10   # 任务优先级
```

---

### Q: 告警阈值如何调整？

```yaml
alerts:
  rules:
    - name: "high_latency"
      condition: "latency > 300"   # 毫秒
      severity: "warning"
    - name: "critical_latency"
      condition: "latency > 500"
      severity: "critical"
    - name: "low_battery"
      condition: "battery < 20"    # 百分比
      severity: "warning"
```

---

### Q: 看门狗超时导致断连

在 `vehicle_config.yaml` 中调大看门狗超时：

```yaml
control:
  safety:
    watchdog_timeout_ms: 500   # 默认 200ms，网络较差时调大
```

---

## 附录：消息格式参考

### 车辆注册
```json
{"type": "register", "client_type": "vehicle", "vehicle_id": "FSM-01"}
```

### 操作员注册并连接
```json
{"type": "register", "client_type": "operator"}
{"type": "connect",  "vehicle_id": "FSM-01"}
```

### 遥测数据（车辆→云端→操作员）
```json
{
  "type": "telemetry",
  "vehicle_id": "FSM-01",
  "sequence": 1234,
  "data": {
    "speed_mps": 8.3,
    "steering_rad": 0.12,
    "gear": 3,
    "battery_pct": 85,
    "latitude": 30.123,
    "longitude": 120.456,
    "heading_rad": 1.57,
    "timestamp_ns": 1234567890000000
  }
}
```

### 控制指令（操作员→云端→车辆）
```json
{
  "type": "control",
  "vehicle_id": "FSM-01",
  "data": {
    "steering": 0.3,
    "throttle": 0.5,
    "brake": 0.0,
    "gear": 3,
    "turn_signal": 0,
    "emergency": false,
    "timestamp_ns": 1234567890000000,
    "sequence": 42
  }
}
```

### REST API 响应示例
```json
// GET /api/v1/vehicles
{
  "vehicles": [
    {
      "vehicle_id": "FSM-01",
      "priority_score": 0.85,
      "latency_ms": 45.2,
      "battery_pct": 78.0,
      "emergency_level": 0,
      "task_status": 1
    }
  ],
  "count": 1
}
```
