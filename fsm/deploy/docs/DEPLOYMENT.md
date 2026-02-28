# FSM-Pilot 阿里云部署指南

快速部署远程驾驶系统到阿里云 ECS。

---

## 一、准备工作

| 项目 | 要求 |
|------|------|
| ECS 规格 | 4 vCPU / 8 GB RAM / Ubuntu 22.04 LTS |
| 带宽 | ≥ 10 Mbps（建议 50 Mbps+） |
| 存储 | ≥ 40 GB 系统盘 |
| 域名 | 已解析到 ECS 公网 IP 的 A 记录 |
| 安全组 | 需开放下表端口 |

### 阿里云安全组入规则

| 端口 | 协议 | 用途 |
|------|------|------|
| 22 | TCP | SSH |
| 80 | TCP | HTTP (Let's Encrypt 验证) |
| 443 | TCP | HTTPS / WSS |
| 3478 | TCP+UDP | TURN/STUN |
| 5349 | TCP | TURN TLS |
| 49152-65535 | UDP | WebRTC 媒体中继 |
| 1883 | TCP | MQTT |
| 8883 | TCP | MQTT TLS |
| 8083 | TCP | MQTT WebSocket |
| 8084 | TCP | MQTT WebSocket TLS |

> 注：18083 端口（EMQX Dashboard）**不要**对公网开放，通过 SSH 隧道访问即可。

---

## 二、一键部署（约 10 分钟）

### 2.1 上传项目代码

```bash
# 在本地执行
rsync -avz --exclude '.git' /path/to/fsm/ root@YOUR_ECS_IP:/opt/fsm-src/
```

或直接在 ECS 上克隆:
```bash
git clone <your-repo-url> /opt/fsm-src
```

### 2.2 设置环境变量并执行部署

```bash
cd /opt/fsm-src/deploy/scripts

# 必填：域名和邮箱
export FSM_DOMAIN="your.domain.com"
export FSM_EMAIL="admin@your.domain.com"

# 可选：自定义密钥（不设置则自动生成）
# export FSM_TURN_SECRET="your-turn-secret"
# export FSM_JWT_SECRET="your-jwt-secret"
# export FSM_MQTT_USER="fsm_app"
# export FSM_MQTT_PASS="your-mqtt-password"

chmod +x deploy.sh
sudo -E ./deploy.sh
```

> `-E` 参数确保 sudo 继承环境变量。

### 2.3 部署完成输出示例

```
═══════════════════════════════════════════════════════
  FSM-Pilot 部署完成！
═══════════════════════════════════════════════════════
  信令服务 (WSS):    wss://your.domain.com/ws
  API 接口:          https://your.domain.com/api
  TURN 服务:         turn:your.domain.com:3478
  STUN 服务:         stun:your.domain.com:3478
  MQTT TCP:          mqtt://your.domain.com:1883
  MQTT TLS:          mqtts://your.domain.com:8883
  MQTT WebSocket:    ws://your.domain.com:8083/mqtt
  EMQX Dashboard:    http://1.2.3.4:18083

  车端 vehicle_config.yaml 填写:
    signaling_url: "wss://your.domain.com/ws"
    turn_url:      "turn:your.domain.com:3478"
    turn_secret:   "<自动生成>"
    mqtt.broker_url: "tcp://your.domain.com:1883"
═══════════════════════════════════════════════════════
```

**请立即记录并妥善保存输出中的密钥信息。**

---

## 三、验证部署

```bash
cd /opt/fsm-src/deploy/scripts

# 完整自检（需要安装 jq 和 mosquitto-clients）
sudo apt install -y jq mosquitto-clients
./run_self_test.sh --host localhost

# 指定远程主机
./run_self_test.sh --host your.domain.com --skip-mqtt
```

自检覆盖：REST API、WebSocket、MQTT 消息往返、404 处理。

---

## 四、车端配置

### 4.1 填写 vehicle_config.yaml

将部署完成输出的信息填入车端配置文件：

```yaml
# /opt/fsm_vehicle/config/vehicle_config.yaml

vehicle:
  id: "FSM-01"           # ← 每辆车唯一，如 FSM-01, FSM-02
  type: "ROBO-TAXI"

webrtc:
  stun_servers:
    - "stun:your.domain.com:3478"
  turn_servers:
    - url: "turn:your.domain.com:3478"
      username: "fsm_turn"
      credential: "<TURN_SECRET>"   # ← deploy.sh 输出的 turn_secret
  signaling:
    url: "wss://your.domain.com/ws"

mqtt:
  enabled: true
  broker_url: "tcp://your.domain.com:1883"
  username: "fsm_app"              # ← EMQX MQTT 用户名
  password: "<EMQX_MQTT_PASS>"    # ← deploy.sh 输出的 MQTT 密码
  telemetry_topic: "fsm/telemetry/{vehicle_id}"
  control_topic:   "fsm/control/{vehicle_id}"
  status_topic:    "fsm/status/{vehicle_id}"
```

### 4.2 启动车端节点

```bash
# 编译（首次）
cd /opt/fsm_vehicle && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 运行
ros2 run fsm_vehicle_node fsm_vehicle_node \
  --config /opt/fsm_vehicle/config/vehicle_config.yaml
```

---

## 五、使用阿里云 IoT 平台（可选）

如需将 MQTT 对接至阿里云 IoT 平台（更高可靠性、云端规则引擎），在 `vehicle_config.yaml` 中配置：

```yaml
mqtt:
  enabled: true
  aliyun:
    mode: true
    product_key:   "YOUR_PRODUCT_KEY"      # 阿里云 IoT 产品 Key
    device_name:   "FSM-01"                # 设备名（与 vehicle.id 一致）
    device_secret: "YOUR_DEVICE_SECRET"    # 设备密钥
    region:        "cn-shanghai"
  # broker_url / username / password 由 SDK 自动计算，无需填写
```

云端 `cloud_config.yaml` 配置 MQTT relay：

```yaml
mqtt:
  enabled: true
  broker_url: "tcp://YOUR_PRODUCT_KEY.iot-as-mqtt.cn-shanghai.aliyuncs.com:1883"
  username:   "YOUR_DEVICE_NAME&YOUR_PRODUCT_KEY"
  password:   "<HMAC-SHA256 signature>"   # 参考阿里云文档
```

---

## 六、REST API 鉴权（JWT）

生产环境默认启用 JWT 认证。获取 token 的方式（需对接业务系统）：

```bash
# JWT secret 在 /opt/fsm/configs/cloud_config.yaml 中的 api.auth.jwt_secret
# 用任意 JWT 生成工具生成 token（HS256 算法）：

# 示例：用 Python 生成临时 token（需安装 PyJWT）
pip install PyJWT
python3 -c "
import jwt, time
secret = 'YOUR_JWT_SECRET'
token = jwt.encode({'sub':'operator-01','exp': int(time.time())+86400}, secret, algorithm='HS256')
print(token)
"

# 使用 token 访问 API
curl -H "Authorization: Bearer <token>" https://your.domain.com/api/v1/vehicles
```

不需要认证（开发环境）：在 `cloud_config.yaml` 中将 `api.auth.jwt_secret` 设为空字符串。

---

## 七、监控与运维

### 查看服务状态

```bash
cd /opt/fsm
docker compose ps
docker compose logs -f fsm_cloud   # 云端服务日志
docker compose logs -f emqx        # MQTT broker 日志
docker compose logs -f nginx       # Nginx 日志
```

### Prometheus 指标

```bash
# 无需鉴权
curl http://localhost:8081/metrics
```

关键指标：
- `fsm_connected_vehicles` — 在线车辆数
- `fsm_active_sessions` — 活跃操作员会话数
- `fsm_telemetry_received_total` — 累计遥测帧数
- `fsm_active_alerts_critical` — 当前严重告警数

### EMQX 管理控制台

```bash
# 本地 SSH 转发（推荐，避免暴露 18083 端口）
ssh -L 18083:localhost:18083 root@YOUR_ECS_IP
# 然后浏览器访问 http://localhost:18083
```

### 重启单个服务

```bash
cd /opt/fsm
docker compose restart fsm_cloud   # 重启云端服务
docker compose restart nginx       # 重载 Nginx
```

### TLS 证书续期

证书自动续期已通过 crontab 配置（每天凌晨 3 点检查）。手动续期：
```bash
certbot renew
docker compose exec nginx nginx -s reload
```

---

## 八、更新部署

```bash
# 拉取新代码
cd /opt/fsm-src && git pull

# 重建并重启云端服务（保留配置和数据卷）
cd /opt/fsm
docker compose build fsm_cloud
docker compose up -d fsm_cloud
```

---

## 九、常见问题

### Q: 证书申请失败
确认域名 A 记录已指向 ECS 公网 IP，且 80 端口已在安全组中开放。

### Q: WebSocket 连接失败
```bash
# 检查 Nginx 配置
docker compose exec nginx nginx -t
# 检查 fsm_cloud 健康状态
docker compose ps fsm_cloud
```

### Q: MQTT 无法连接
```bash
# 测试 MQTT 连接
mosquitto_pub -h your.domain.com -p 1883 \
  -u fsm_app -P YOUR_MQTT_PASS \
  -t test -m hello
# 查看 EMQX 日志
docker compose logs --tail=50 emqx
```

### Q: 部署脚本重复运行
```bash
# 清理并重新部署
cd /opt/fsm && docker compose down
sudo rm -rf /opt/fsm/configs
cd /opt/fsm-src/deploy/scripts && sudo -E ./deploy.sh
```

### Q: 自检脚本 MQTT 测试失败
```bash
# 跳过 MQTT 测试（仅验证 HTTP/WS）
./run_self_test.sh --host localhost --skip-mqtt
# 单独测试 MQTT
mosquitto_sub -h localhost -p 1883 -t "fsm/#" -v
```

---

## 十、Demo 模式（室内演示 / CI 测试）

不需要真实车辆即可运行完整系统。

### 编译 Demo 版本

```bash
cd /opt/fsm-src/cpp
cmake -S . -B build_demo \
  -DCMAKE_BUILD_TYPE=Release \
  -DFSM_DEMO_MODE=ON         # 启用模拟车辆
cmake --build build_demo --target fsm_vehicle_node -j$(nproc)
```

Demo 版本会将 `SimulatedVehicle` 代替真实 ROS2 硬件驱动编译进去。日志输出中可见 `build_variant=DEMO`。

### 可用演示场景

在车端配置文件中设置 `demo.scenario` 字段：

| 场景 | 值 | 描述 |
|------|----|------|
| 直线行驶 | `straight_line` | 加速→匀速→制动 |
| 圆形绕行 | `round_about` | 固定半径转弯 |
| 紧急制动 | `emergency_brake` | 巡航→突然全制动 |
| 蛇形避障 | `slalom` | 车道变换波形 |
| 静止待命 | `idle` | 响应操作员指令 |

---

## 十一、车辆远程调试（DiagnosticsManager）

无需 ROS2 工具链，通过 REST API 即可查看车辆内部健康状态。

### 获取车辆诊断信息

```bash
TOKEN="<operator-jwt-token>"

# 获取完整诊断数据（包括安全状态、告警、调度信息）
curl -H "Authorization: Bearer $TOKEN" \
  https://your.domain.com/api/v1/vehicles/FSM-01/diagnostics | jq .
```

**返回示例：**
```json
{
  "vehicle_id": "FSM-01",
  "overall_status": "ok",
  "scheduling": {
    "is_connected": true,
    "latency_ms": 45.2,
    "battery_pct": 82.5,
    "speed_mps": 3.1,
    "emergency_level": 0
  },
  "session": {
    "session_id": "abc123",
    "operator_id": "op-001",
    "duration_s": 127.4
  },
  "active_alerts": [],
  "alert_count": 0
}
```

### ISO 26262 安全状态说明

| 状态 | 含义 | 是否允许远程控制 |
|------|------|----------------|
| `NOMINAL` | 所有子系统正常 | ✅ 允许 |
| `DEGRADED` | 非关键故障，持续监控 | ✅ 允许（降速） |
| `FAULT` | 严重故障，需操作员介入 | ⚠️ 限制 |
| `FAIL-SAFE` | 安全模式已启动，紧急停车 | ❌ 禁止 |

### 查看安全监视器状态

```bash
# 健康接口包含连接车辆数和告警数
curl https://your.domain.com/api/v1/health | jq .

# Prometheus 指标（无需鉴权）
curl http://localhost:8081/metrics | grep fsm_
```

