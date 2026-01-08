# FSM-Pilot 远程驾驶系统 - 使用指南

## 目录

1. [系统架构概述](#1-系统架构概述)
2. [启动顺序](#2-启动顺序)
3. [各组件使用说明](#3-各组件使用说明)
4. [远程控制操作](#4-远程控制操作)
5. [调度系统使用](#5-调度系统使用)
6. [监控与告警](#6-监控与告警)
7. [故障处理](#7-故障处理)

---

## 1. 系统架构概述

```
┌──────────────────────────────────────────────────────────────────┐
│                        FSM-Pilot 系统架构                         │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│   ┌─────────────┐         ┌─────────────┐         ┌───────────┐ │
│   │   车端节点   │◄───────►│   云端服务   │◄───────►│  操作端   │ │
│   │ (Vehicle)   │  WebRTC │   (Cloud)   │ WebSocket│ (Operator)│ │
│   └─────────────┘         └─────────────┘         └───────────┘ │
│         │                       │                       │        │
│         │ ROS2                  │ REST API              │ 方向盘  │
│         ▼                       ▼                       ▼        │
│   ┌─────────────┐         ┌─────────────┐         ┌───────────┐ │
│   │  Autoware   │         │   数据库    │         │  Vue 前端  │ │
│   └─────────────┘         └─────────────┘         └───────────┘ │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

### 组件说明

| 组件 | 可执行文件 | 功能 |
|------|-----------|------|
| 车端节点 | `fsm_vehicle_node` | 采集传感器数据，执行控制指令 |
| 云端服务 | `fsm_cloud_server` | 信令转发，车辆调度，告警分析 |
| 操作端 | `fsm_operator_client` | 方向盘输入，远程控制 |
| 前端界面 | `npm run dev` | 可视化界面，监控管理 |

---

## 2. 启动顺序

### 2.1 标准启动顺序

```bash
# 1. 启动云端服务 (首先启动)
cd ~/fsm/cpp/build
./cloud_server/fsm_cloud_server ../cloud_server/config/cloud_config.yaml

# 2. 启动前端界面
cd ~/fsm
npm run dev

# 3. 启动车端节点 (每辆车)
cd ~/fsm/cpp/build
./vehicle_node/fsm_vehicle_node ../vehicle_node/config/vehicle_config.yaml

# 4. 启动操作端 (可选，如使用方向盘)
cd ~/fsm/cpp/build
./operator_client/fsm_operator_client -s wss://localhost:8080/signaling -v FSM-01
```

### 2.2 一键启动脚本

```bash
#!/bin/bash
# scripts/start_all.sh

# 启动云端服务
echo "Starting cloud server..."
cd ~/fsm/cpp/build
./cloud_server/fsm_cloud_server ../cloud_server/config/cloud_config.yaml &
CLOUD_PID=$!
sleep 2

# 启动前端
echo "Starting frontend..."
cd ~/fsm
npm run dev &
FRONTEND_PID=$!
sleep 3

# 启动 Mock 车辆 (演示用)
echo "Starting mock vehicles..."
cd ~/fsm
node mock/mock_server.js &
MOCK_PID=$!

echo "================================"
echo "FSM-Pilot 系统已启动"
echo "Cloud Server PID: $CLOUD_PID"
echo "Frontend PID: $FRONTEND_PID"
echo "Mock Server PID: $MOCK_PID"
echo ""
echo "访问前端: http://localhost:5173"
echo "================================"

# 等待退出信号
trap "kill $CLOUD_PID $FRONTEND_PID $MOCK_PID 2>/dev/null" EXIT
wait
```

### 2.3 停止系统

```bash
#!/bin/bash
# scripts/stop_all.sh

pkill -f fsm_cloud_server
pkill -f fsm_vehicle_node
pkill -f fsm_operator_client
pkill -f "npm run dev"
pkill -f mock_server.js

echo "FSM-Pilot 系统已停止"
```

---

## 3. 各组件使用说明

### 3.1 云端服务

```bash
# 启动命令
./fsm_cloud_server <config_file>

# 参数说明
#   config_file: 云端配置文件路径

# 示例
./fsm_cloud_server config/cloud_config.yaml
```

**日志输出说明:**

```
[FSM-Cloud] ========================================
[FSM-Cloud]   FSM-Pilot Cloud Server v1.1.0
[FSM-Cloud] ========================================
[FSM-Cloud] Loading configuration from: config/cloud_config.yaml
[FSM-Cloud] Scheduling service started
[FSM-Cloud] Signaling server started on port 8080
[FSM-Cloud] REST API server started on port 8081
```

**REST API 端点:**

| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/v1/vehicles` | GET | 获取所有车辆 |
| `/api/v1/vehicles/{id}` | GET | 获取单个车辆 |
| `/api/v1/scheduling/queue` | GET | 获取调度队列 |
| `/api/v1/alerts` | GET | 获取告警列表 |

### 3.2 车端节点

```bash
# 启动命令
./fsm_vehicle_node <config_file>

# 参数说明
#   config_file: 车端配置文件路径

# 示例 - 不同车辆使用不同配置
./fsm_vehicle_node config/vehicle_fsm01.yaml
./fsm_vehicle_node config/vehicle_fsm02.yaml
```

**ROS2 话题订阅:**

```bash
# 查看车端发布的话题
ros2 topic list | grep fsm

# 监听遥测数据
ros2 topic echo /fsm/telemetry
```

### 3.3 操作端客户端

```bash
# 启动命令
./fsm_operator_client [options]

# 选项说明
#   -s, --signaling URL    信令服务器地址
#   -v, --vehicle ID       要连接的车辆ID
#   -h, --help             显示帮助

# 示例
./fsm_operator_client -s wss://localhost:8080/signaling -v FSM-01
```

**方向盘按钮映射:**

| 按钮 | 功能 |
|------|------|
| X (红色) | 紧急停车 |
| □ (方块) | 喇叭 |
| L1 | 左转向灯 |
| R1 | 右转向灯 |
| 右拨片 | 升档 |
| 左拨片 | 降档 |

### 3.4 前端界面

```bash
# 开发模式
npm run dev

# 生产模式
npm run build
npm run preview
```

**界面功能区域:**

```
┌────────────────────────────────────────────────────────────┐
│ [Header] 系统状态 | 当前车辆 | 连接状态 | 录制控制          │
├────────┬────────────────────────────────┬─────────────────┤
│        │                                │                 │
│ 左侧栏 │         主视频区域              │     右侧栏      │
│ - 车队 │      (5路摄像头画面)            │  - 遥测数据     │
│ - 地图 │                                │  - 系统状态     │
│ - 日志 │                                │  - 告警信息     │
│        │                                │                 │
├────────┴────────────────────────────────┴─────────────────┤
│ [Footer] 调度队列 | 车辆列表 | 快速切换                      │
└────────────────────────────────────────────────────────────┘
```

---

## 4. 远程控制操作

### 4.1 连接车辆

1. 在前端界面底部车队列表中选择目标车辆
2. 点击"连接"按钮
3. 等待 WebRTC 连接建立
4. 视频流开始显示后即可控制

### 4.2 使用方向盘控制

```javascript
// 方向盘输入映射
{
  steering: -1.0 ~ 1.0,    // 左满舵 ~ 右满舵
  throttle: 0.0 ~ 1.0,     // 油门踏板
  brake: 0.0 ~ 1.0,        // 刹车踏板
  gear: 0=P, 1=R, 2=N, 3=D
}
```

### 4.3 使用键盘控制 (备用)

| 按键 | 功能 |
|------|------|
| W | 前进 |
| S | 后退 |
| A | 左转 |
| D | 右转 |
| Space | 刹车 |
| E | 紧急停车 |
| Q/E | 降档/升档 |

### 4.4 紧急停车

**触发方式:**
- 方向盘: 按下红色 X 按钮
- 键盘: 按下 E 键
- 界面: 点击红色"紧急停车"按钮

**紧急停车后恢复:**
1. 确认车辆已完全停止
2. 检查周围环境安全
3. 点击"解除紧急状态"按钮
4. 重新启动远程控制

---

## 5. 调度系统使用

### 5.1 调度算法选择

在前端设置面板中可选择:

| 算法 | 说明 | 适用场景 |
|------|------|----------|
| 加权优先级 | 综合多因素评分 | 默认推荐 |
| 紧急优先 | 紧急级别最高优先 | 应急场景 |
| 延迟优先 | 低延迟车辆优先 | 网络不稳定时 |

### 5.2 权重配置

```yaml
# 调度权重配置
weights:
  emergency: 0.35      # 紧急程度
  latency: 0.25        # 通信延迟
  distance: 0.20       # 距离目标
  battery: 0.10        # 电量状态
  task_priority: 0.10  # 任务优先级
```

### 5.3 查看调度队列

调度队列显示在前端底部:

```
┌────────────────────────────────────────────────────────────┐
│ 调度队列                                                   │
├────────┬──────────┬────────┬────────┬──────────┬─────────┤
│ 排序   │ 车辆ID   │ 状态   │ 延迟   │ 优先级   │ 操作    │
├────────┼──────────┼────────┼────────┼──────────┼─────────┤
│ 1      │ FSM-03   │ 紧急   │ 65ms   │ 85       │ [切换]  │
│ 2      │ FSM-01   │ 活跃   │ 45ms   │ 60       │ [切换]  │
│ 3      │ FSM-02   │ 空闲   │ 120ms  │ 40       │ [切换]  │
└────────┴──────────┴────────┴────────┴──────────┴─────────┘
```

---

## 6. 监控与告警

### 6.1 实时监控指标

| 指标 | 正常范围 | 警告阈值 | 严重阈值 |
|------|----------|----------|----------|
| RTT延迟 | <100ms | 100-300ms | >500ms |
| 视频帧率 | 30fps | 20-30fps | <20fps |
| 电池电量 | >50% | 20-50% | <10% |
| CPU使用率 | <50% | 50-80% | >90% |

### 6.2 告警类型

| 告警类型 | 级别 | 说明 |
|----------|------|------|
| high_latency | WARNING | 延迟超过300ms |
| critical_latency | CRITICAL | 延迟超过500ms |
| low_battery | WARNING | 电量低于20% |
| connection_lost | CRITICAL | 连接断开超过10秒 |
| camera_offline | WARNING | 摄像头离线 |
| emergency_active | CRITICAL | 紧急状态激活 |

### 6.3 告警处理流程

```
1. 告警触发 → 界面弹出提示
2. 查看告警详情
3. 评估影响范围
4. 采取相应措施
5. 确认告警 (Acknowledge)
6. 问题解决后告警自动消除
```

---

## 7. 故障处理

### 7.1 连接问题

**症状**: 无法连接车辆

**排查步骤**:
```bash
# 1. 检查云端服务是否运行
curl http://localhost:8081/api/v1/vehicles

# 2. 检查 WebSocket 连接
wscat -c ws://localhost:8080

# 3. 检查网络连通性
ping <vehicle_ip>

# 4. 检查防火墙
sudo ufw status
```

### 7.2 视频卡顿

**症状**: 视频延迟高或卡顿

**解决方案**:
```bash
# 1. 检查网络带宽
iperf3 -c <server_ip>

# 2. 降低视频分辨率
# 修改 vehicle_config.yaml
cameras:
  - width: 1280    # 从 1920 降低
    height: 720    # 从 1080 降低
    bitrate: 2000000  # 降低比特率

# 3. 检查编码器负载
nvidia-smi  # 如使用 GPU 编码
top -p $(pgrep fsm_vehicle)
```

### 7.3 控制延迟

**症状**: 方向盘操作响应慢

**排查步骤**:
```bash
# 1. 检查 RTT 延迟
# 在前端界面查看延迟显示

# 2. 检查本地处理延迟
./fsm_operator_client --debug

# 3. 优化网络路由
traceroute <cloud_server_ip>
```

### 7.4 车端节点崩溃

**症状**: 车端节点意外退出

**恢复步骤**:
```bash
# 1. 检查日志
tail -f /var/log/fsm/vehicle_node.log

# 2. 检查 ROS2 节点状态
ros2 node list
ros2 doctor

# 3. 重启节点
./fsm_vehicle_node config/vehicle_config.yaml
```

---

## 附录: 常用命令速查

```bash
# 启动系统
./scripts/start_all.sh

# 停止系统
./scripts/stop_all.sh

# 查看日志
tail -f /var/log/fsm/*.log

# 检查服务状态
curl http://localhost:8081/api/v1/health

# 重启单个组件
pkill fsm_cloud_server && ./fsm_cloud_server config/cloud_config.yaml

# 查看 ROS2 话题
ros2 topic list
ros2 topic hz /fsm/telemetry

# 测试 WebSocket
wscat -c ws://localhost:8080

# 检查网络
netstat -tlnp | grep -E "(8080|8081)"
```
