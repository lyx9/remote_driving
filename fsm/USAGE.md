# FSM-Pilot V2.0 - Remote Driving Platform

远程驾驶平台 - 支持游戏手柄/键盘遥控、RosBag回放、多视图可视化

## 🚀 快速开始

### 1. 启动后端服务器

```bash
cd server
npm install
npm run dev
```

后端服务器将运行在:
- **HTTP API**: `http://localhost:8080`
- **WebSocket**: `ws://localhost:8080`

### 2. 启动前端应用

```bash
cd ..  # 返回项目根目录
npm run dev
```

前端将运行在 `http://localhost:3000`

## 🎮 遥控器驾驶功能

### 键盘控制

| 按键 | 功能 |
|------|------|
| `W` 或 `↑` | 油门 |
| `S` 或 `↓` | 刹车 |
| `A` 或 `←` | 左转 |
| `D` 或 `→` | 右转 |
| `P` | 停车档 (Park) |
| `R` | 倒车档 (Reverse) |
| `N` | 空档 (Neutral) |
| `D` | 前进档 (Drive) |
| `Space` | 紧急制动 |

### 游戏手柄控制 (Xbox/PlayStation)

| 控制 | 功能 |
|------|------|
| 左摇杆 (横向) | 转向 |
| RT/R2 (右扳机) | 油门 |
| LT/L2 (左扳机) | 刹车 |
| 方向键 ↑ | 前进档 (D) |
| 方向键 ↓ | 倒车档 (R) |
| 方向键 ← | 空档 (N) |
| 方向键 → | 停车档 (P) |
| B/Circle | 紧急制动 |

## 📦 功能模块

### 1. Remote Control (远程控制)
- ✅ 键盘控制
- ✅ 游戏手柄支持 (自动检测)
- ✅ 实时车辆状态反馈
- ✅ 延迟监控
- ✅ Mock 车辆模拟器 (3辆测试车)

### 2. RosBag Replay (数据回放)
- ✅ 加载 .db3 格式 ROS2 bag 文件
- ✅ 点云可视化 (Three.js)
- ✅ GPS 轨迹显示 (Leaflet)
- ✅ 回放控制 (播放/暂停/跳转)
- ✅ 多视图切换

### 3. 车辆管理
- ✅ 多车辆列表
- ✅ 车辆状态监控
- ✅ 调度队列
- ✅ 告警系统

## 🏗️ 项目结构

```
fsm/
├── server/                 # 后端服务器 (Node.js + TypeScript)
│   ├── src/
│   │   ├── index.ts       # 主入口
│   │   ├── websocket/     # WebSocket 处理
│   │   ├── simulator/     # 车辆模拟器
│   │   ├── services/      # 业务服务
│   │   └── utils/         # 工具函数
│   └── package.json
│
├── src/                    # 前端应用 (Vue 3 + TypeScript)
│   ├── components/        # 组件
│   │   ├── GamepadVisualizer.vue
│   │   ├── RosBagReplayPro.vue
│   │   ├── PointCloudViewer.vue
│   │   └── ...
│   ├── composables/       # 组合式函数
│   │   ├── useGamepadController.ts
│   │   ├── useRemoteControl.ts
│   │   └── ...
│   ├── services/          # 服务层
│   │   ├── websocket.ts
│   │   ├── api.ts
│   │   └── webrtc.ts
│   └── stores/            # 状态管理
│       ├── fleet.ts
│       └── system.ts
│
└── cpp/                    # C++ 后端 (可选)
    ├── cloud_server/
    ├── vehicle_node/
    └── operator_client/
```

## 🔧 API 端点

### REST API

```
GET    /api/v1/health                   # 健康检查
GET    /api/v1/vehicles                 # 获取所有车辆
GET    /api/v1/vehicles/:id             # 获取单个车辆
GET    /api/v1/vehicles/:id/status      # 获取车辆状态
POST   /api/v1/vehicles/:id/connect     # 连接车辆
DELETE /api/v1/vehicles/:id/connect     # 断开车辆
GET    /api/v1/scheduling/queue         # 调度队列
GET    /api/v1/scheduling/config        # 调度配置
PUT    /api/v1/scheduling/config        # 更新配置
GET    /api/v1/alerts                   # 告警列表
GET    /api/v1/system/stats             # 系统统计
```

### WebSocket 消息

#### 客户端发送

```json
{
  "type": "control_command",
  "data": {
    "vehicle_id": "FSM-01",
    "steering": 0.5,        // -1 to 1
    "throttle": 0.8,        // 0 to 1
    "brake": 0,             // 0 to 1
    "gear": "D",            // P/R/N/D
    "emergency": false
  }
}
```

#### 服务器推送

```json
{
  "type": "vehicle_status",
  "data": {
    "vehicle_id": "FSM-01",
    "speed": 45.2,
    "steering": 15.5,
    "gear": "D",
    "location": { "lat": 31.2304, "lng": 121.4737 },
    "latency_ms": 45,
    "timestamp": 1704153600000
  }
}
```

## 🧪 测试

### 后端测试
```bash
cd server
npm test
```

### 前端测试
```bash
npm test
```

## 📊 Mock 车辆

系统自动创建 3 辆模拟车辆用于测试：

| ID | 类型 | 初始位置 | 状态 |
|----|------|----------|------|
| FSM-01 | ROBO-TAXI | (31.2304, 121.4737) | ACTIVE |
| FSM-02 | LOGISTICS | (31.235, 121.480) | IDLE |
| FSM-03 | SECURITY | (31.220, 121.460) | PATROL |

车辆模拟器特性：
- ✅ 真实物理模型 (加速/制动/转向)
- ✅ 自动GPS位置更新
- ✅ 电池消耗模拟
- ✅ 延迟模拟 (50-100ms)
- ✅ 告警触发 (低电量)

## 🎯 使用流程

1. **启动服务**
   ```bash
   # Terminal 1: 启动后端
   cd server && npm run dev

   # Terminal 2: 启动前端
   cd .. && npm run dev
   ```

2. **打开浏览器**
   访问 `http://localhost:3000`

3. **选择车辆**
   在左侧车辆列表选择一辆车 (FSM-01/02/03)

4. **开始控制**
   - 使用键盘 WASD 或方向键
   - 或连接 Xbox/PS 手柄
   - 右侧面板实时显示控制输入

5. **查看反馈**
   - 速度显示
   - 转向角度
   - 档位状态
   - 延迟监控

## 📝 注意事项

- 后端和前端必须同时运行
- 游戏手柄需要浏览器支持 Gamepad API (Chrome/Firefox)
- RosBag 功能需要 .db3 格式文件
- WebRTC 功能需要 HTTPS (生产环境)

## 🔐 安全说明

本项目为演示和开发用途，不适合直接用于生产环境。生产部署需要：
- SSL/TLS 加密
- 用户认证
- 访问控制
- 日志审计

## 📄 License

Proprietary - © 2025 City University of Hong Kong

---

**作者**: Li Yixiang
**机构**: City University of Hong Kong
**版本**: 2.0.0
