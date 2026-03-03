# FSM-Pilot 远程驾驶系统 — 完整项目说明

> 版本 v1.2.0 | 更新 2026-03-03

---

## 一、系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                     阿里云 ECS（公网）                           │
│                                                                 │
│  ┌──────────────┐   WebSocket   ┌──────────────────────────┐   │
│  │  nginx:443   │◄─────────────►│  fsm_cloud_server:8080   │   │
│  │  (TLS终止)   │               │  - WebSocket 信令中继     │   │
│  └──────┬───────┘               │  - 遥测数据转发           │   │
│         │                      │  - 控制指令转发           │   │
│  ┌──────▼───────┐               │  - REST API :8081        │   │
│  │  coturn      │               │  - 会话管理 / JWT鉴权    │   │
│  │  STUN/TURN   │               └──────────────────────────┘   │
│  │  :3478       │                                               │
│  └──────────────┘               ┌──────────────────────────┐   │
│                                 │  emqx:1883               │   │
│                                 │  (可选 MQTT 通道)         │   │
│                                 └──────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
         ▲ wss://                          ▲ wss://
         │                                │
┌────────┴──────────┐          ┌──────────┴────────────┐
│   操作员端 (本地)  │          │   车端 (ROS2 机器人)   │
│                   │          │                       │
│  fsm_operator     │          │  fsm_vehicle_node     │
│  - 键盘/方向盘输入 │          │  - 摄像头视频上传      │
│  - 视频画面显示   │          │  - 遥测数据上报        │
│  - 遥测数据显示   │          │  - 接收控制指令        │
│  - 紧急停车按钮   │          │  - 安全监控 ASIL-B     │
└───────────────────┘          └───────────────────────┘
```

**通信流程：**
```
操作员 ──[控制指令 JSON]──► 云端 ──[转发]──► 车辆
车辆   ──[视频帧 JPEG]────► 云端 ──[转发]──► 操作员
车辆   ──[遥测 Protobuf]──► 云端 ──[存储]──► REST API
```

---

## 二、组件说明

| 组件 | 位置 | 语言 | 说明 |
|------|------|------|------|
| `fsm_cloud_server` | 阿里云 ECS | C++17 | 信令中继、会话、API |
| `fsm_vehicle_node` | 车辆机载电脑 | C++17 + ROS2 | 传感器采集、控制执行 |
| `fsm_operator` | 操作员电脑 | C++17 | 控制输入、视频显示 |
| nginx | 阿里云 ECS | Docker | TLS 终止、反向代理 |
| coturn | 阿里云 ECS | Docker | WebRTC STUN/TURN |
| emqx | 阿里云 ECS | Docker | MQTT 消息中间件 |

---

## 三、快速开始（5 步完成部署）

### 步骤 1：准备阿里云 ECS

- **规格**：4核8G，Ubuntu 22.04 LTS
- **带宽**：≥ 5 Mbps（视频流需要）
- **安全组开放端口**：22、80、443、3478(UDP)、1883

### 步骤 2：部署云端（1 条命令）

```bash
ssh root@<你的ECS公网IP>
git clone <项目仓库> /opt/fsm-src && cd /opt/fsm-src
export FSM_DOMAIN=fsm.yourdomain.com   # 有域名填域名，没有留空用IP
bash deploy/scripts/aliyun_deploy.sh
```

### 步骤 3：部署车端（1 条命令）

```bash
# 在车辆机载电脑上执行
git clone <项目仓库> ~/fsm-src && cd ~/fsm-src
export FSM_CLOUD_URL=wss://fsm.yourdomain.com/ws
export FSM_VEHICLE_ID=FSM-CAR-01
bash deploy/scripts/vehicle_deploy.sh
```

### 步骤 4：启动操作员客户端

```bash
# 在操作员电脑上（需要 ROS2 Humble 或直接二进制）
./fsm_operator wss://fsm.yourdomain.com/ws FSM-CAR-01
```

### 步骤 5：验证系统

```bash
# 检查云端健康
curl https://fsm.yourdomain.com/api/v1/health

# 查看连接的车辆
curl -H "Authorization: Bearer <JWT>" \
     https://fsm.yourdomain.com/api/v1/vehicles
```

---

## 四、Demo 模式（无真实车辆）

不需要实际机器人，软件模拟车辆运动，用于演示和测试。

```bash
# 编译 Demo 版本
cd ~/fsm-src/cpp
cmake -S . -B build_demo -DFSM_DEMO_MODE=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build_demo --target fsm_vehicle_node -j$(nproc)

# 启动（不需要 ROS2 硬件）
./build_demo/vehicle_node/fsm_vehicle_node config/vehicle_config_demo.yaml
```

**Demo 可选场景：**

| 场景 | 效果 |
|------|------|
| `straight_line` | 直线加速→匀速→制动 |
| `round_about` | 20m 半径匀速转圈 |
| `emergency_brake` | 巡航后突然急刹 |
| `slalom` | 蛇形绕桩 |
| `idle` | 原地待命（响应操作员指令）|

---

## 五、配置说明

### 5.1 云端配置 `/opt/fsm/configs/cloud_config.yaml`

```yaml
server:
  signaling_port: 8080    # WebSocket 信令端口
  api_port: 8081          # REST API 端口

api:
  auth:
    jwt_secret: "your_secret"  # JWT 签名密钥（deploy 脚本自动生成）
  session_timeout_s: 300        # 操作员会话超时（秒）

scheduling:
  algorithm: weighted_priority
  weights:
    emergency: 0.35    # 紧急程度权重
    latency:   0.25    # 网络延迟权重
    battery:   0.10    # 电量权重
```

### 5.2 车端配置 `~/fsm_vehicle/config/vehicle_config.yaml`

```yaml
vehicle:
  id: "FSM-CAR-01"          # 车辆唯一 ID
  type: "robotaxi"

vehicle_params:
  max_speed_kph: 30.0        # 最大速度 km/h
  max_steering_angle_deg: 35.0
  emergency_decel_mps2: 6.0  # 紧急制动加速度

webrtc:
  signaling_url: "wss://fsm.yourdomain.com/ws"  # 云端地址
  stun_servers: ["stun:fsm.yourdomain.com:3478"]
  turn_servers: ["turn:fsm.yourdomain.com:3478"]

cameras:
  - id: "front_center"
    topic: "/sensing/camera/front/image_raw"
    fps: 15
    bitrate_kbps: 2000
```

---

## 六、REST API 接口

所有接口需 `Authorization: Bearer <JWT>` 头（除 `/health` 和 `/metrics`）

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/v1/health` | 健康检查（无需鉴权）|
| GET | `/metrics` | Prometheus 指标（无需鉴权）|
| GET | `/api/v1/vehicles` | 在线车辆列表 |
| GET | `/api/v1/vehicles/{id}` | 单车状态 |
| GET | `/api/v1/vehicles/{id}/diagnostics` | 车辆调试信息 |
| POST | `/api/v1/sessions` | 创建驾驶会话 |
| DELETE | `/api/v1/sessions/{id}` | 结束会话 |
| GET | `/api/v1/alerts` | 当前告警列表 |

**生成 JWT Token（Python）：**
```python
import jwt, time
token = jwt.encode(
    {"sub": "operator-001", "exp": int(time.time()) + 3600},
    "your_jwt_secret",
    algorithm="HS256"
)
print(token)
```

---

## 七、安全架构（ISO 26262 ASIL-B）

```
                    SafetyMonitor
                   ┌────────────────────────────┐
     每条控制指令   │ Feed() ── 看门狗超时 500ms  │
     ────────────► │                            │
                   │ kNominal → kDegraded       │
                   │         → kFault           │──► TriggerEmergencyStop()
                   │         → kFailSafe        │
                   └────────────────────────────┘
                   FaultCode 枚举对应 FMEA 条目：
                   0x0101 控制看门狗超时
                   0x0201 传感器数据过期
                   0x0301 通信链路丢失
                   0x0302 延迟超限 (>500ms)
```

**MISRA C++ 关键合规点：**
- `Execute()` 返回 `[[nodiscard]] bool`，调用方必须检查
- 故障缓冲区使用固定 32 槽数组，不在故障路径分配堆内存
- 所有窄化类型转换显式 `static_cast<>`
- 安全临界函数全部 `[[nodiscard]]` 标注

---

## 八、常见问题

### Q: npm 报错怎么办？
本项目**不使用 npm**。如遇 npm 错误，执行 `apt remove nodejs npm -y` 清理环境即可。

### Q: 证书申请失败？
```bash
# 确认 DNS 解析正确
dig +short fsm.yourdomain.com  # 应返回 ECS 公网 IP
# 确认 80 端口开放
curl -v http://fsm.yourdomain.com
```

### Q: 车辆连接不上云端？
```bash
# 车端检查 WebSocket 连通性
wscat -c wss://fsm.yourdomain.com/ws
# 查看车端日志
journalctl -u fsm_vehicle -f
```

### Q: 视频延迟高？
- 检查带宽：上行 > 3 Mbps（单摄像头 H.264 1080p@15fps）
- 降低码率：`vehicle_config.yaml` 中 `bitrate_kbps: 1000`
- 开启 TURN 中继：确认 coturn 服务运行正常

### Q: Demo 模式下控制无响应？
```bash
# 确认 build 时开启了 DEMO_MODE
strings ./fsm_vehicle_node | grep "build_variant"
# 应输出: build_variant=DEMO
```

### Q: ROS2 相关缺包？
```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths ~/fsm-src/cpp --ignore-src -r -y
```

---

## 九、目录结构

```
fsm/
├── cpp/                          # 所有 C++ 源码
│   ├── common/                   # 共享库（日志、配置、安全监控）
│   │   ├── include/fsm/
│   │   │   ├── safety_monitor.hpp  # ISO 26262 ASIL-B 安全监控
│   │   │   ├── diagnostics.hpp     # 结构化诊断
│   │   │   └── build_mode.hpp      # 编译时 Demo/Production 标志
│   │   └── src/
│   ├── vehicle_node/             # 车端 ROS2 节点
│   │   ├── demo/
│   │   │   └── simulated_vehicle.hpp/.cpp  # Demo 模式虚拟车辆
│   │   ├── include/fsm/vehicle/
│   │   └── src/
│   ├── cloud_server/             # 云端服务器
│   └── operator_client/          # 操作员客户端
├── deploy/
│   ├── scripts/
│   │   ├── aliyun_deploy.sh      # 云端一键部署
│   │   └── vehicle_deploy.sh     # 车端一键部署
│   ├── configs/
│   │   └── cloud_config.yaml     # 云端配置模板
│   └── docs/
│       └── DEPLOYMENT.md         # 详细部署文档
└── docs/
    └── project_final.md          # 本文件
```

---

## 十、版本历史

| 版本 | 日期 | 变更 |
|------|------|------|
| v1.2.0 | 2026-03-03 | ISO 26262 安全层、Demo 模式、MISRA C++ 合规 |
| v1.1.0 | 2026-02-28 | JWT 鉴权、SessionManager、Prometheus 指标、AuditLogger |
| v1.0.0 | 2026-02-27 | 基础远程驾驶功能，Google C++ 风格重构 |
