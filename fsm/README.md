# FSM-Pilot V2.0 - 远程驾驶平台

<div align="center">

![Version](https://img.shields.io/badge/version-2.0.0-blue.svg)
![Platform](https://img.shields.io/badge/platform-Ubuntu%2022.04-orange.svg)
![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)
![License](https://img.shields.io/badge/license-Proprietary-red.svg)

**企业级远程驾驶平台 - 支持 100+ 车辆调度与管理**

[快速开始](#快速开始) •
[一键演示](#一键演示脚本) •
[功能演示](docs/DEMO.md) •
[API配置](docs/API_CONFIGURATION.md) •
[视频指南](docs/DEMO_VIDEO_GUIDE.md) •
[完成总结](docs/COMPLETION_SUMMARY.md) •
[架构文档](ARCHITECTURE.md)

</div>

---

## 🎯 项目亮点

### 核心创新
- **100+ 车队规模** - 支持超大规模车队的智能调度管理
- **AI智能分析** - 集成豆包LLM进行实时场景分析和驾驶建议
- **四维调度算法** - 优先级+延迟+位置+事件的智能调度
- **Remote Assistant** - 实时AI驾驶建议，四级优先级系统
- **3D Gaussian Splatting** - 从 RosBag 直接生成 3DGS 场景重建
- **自适应视频压缩** - 最高 91.7% 带宽节省，自动网络适配
- **企业级架构** - TypeScript 全栈，模块化设计，完整文档

### 性能指标
- ⚡ **端到端延迟**: 70-120ms
- 📊 **并发接管**: 10 辆车/运营商
- 💾 **带宽优化**: 1-8 Mbps（4 摄像头）
- 🎮 **渲染性能**: 60 FPS @ 50,000 点云

---

## 功能特性

### ✅ 已完成功能 (V2.0)

#### 1. RosBag 回放系统
- [x] ROS2 DB3 文件解析（带验证）
- [x] 多类型数据支持（点云、GPS、图像）
- [x] 5 种可视化视图（点云、GPS、摄像头、3DGS、BEV）
- [x] 精确时间轴控制和播放
- [x] 自动摄像头 topic 映射

#### 2. 视频压缩系统
- [x] 4 种压缩模式（无/轻/中/重）
- [x] 自动网络适配
- [x] 实时带宽统计
- [x] 完整技术文档

#### 3. 3D Gaussian Splatting
- [x] 点云转 3DGS 工具链
- [x] 实时 Canvas 渲染器
- [x] 多种颜色映射
- [x] PLY 格式导出

#### 4. 车队调度管理（100+ 车辆）
- [x] 智能优先级调度
- [x] 延迟感知接入
- [x] 地理位置匹配
- [x] 事件驱动调度
- [x] 可视化管理界面
- [x] **AI场景分析** - 豆包LLM实时风险评估
- [x] **Remote Assistant** - 四级优先级驾驶建议
- [x] **高德地图集成** - 实时车辆位置显示

#### 5. 远程驾驶控制
- [x] WebRTC 低延迟视频传输
- [x] 多摄像头实时显示
- [x] 虚拟方向盘控制
- [x] 车辆状态监控
- [x] 紧急制动功能
- [x] **接管确认对话框** - 企业级双重确认
- [x] **操作员智能匹配** - 92%+ 匹配度
- 📊 **遥测数据** - 速度、方向、档位实时显示
- 📹 **黑盒录制** - 多通道数据同步记录

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                      FSM-Pilot 系统架构                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   ┌─────────────┐         ┌─────────────┐         ┌──────────┐ │
│   │   车端节点   │◄───────►│   云端服务   │◄───────►│  操作端  │ │
│   │  (ROS2 C++) │  WebRTC │   (C++)     │   WS    │  (Vue 3) │ │
│   └─────────────┘         └─────────────┘         └──────────┘ │
│         │                       │                       │       │
│    Autoware              调度/告警/信令              方向盘     │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## 快速开始

### 环境要求

- Ubuntu 22.04 LTS
- Node.js 18+
- (可选) ROS2 Humble, CMake 3.16+

### 安装

```bash
# 克隆项目
git clone <repository_url> fsm
cd fsm

# 安装前端依赖
npm install

# 安装 Mock 服务依赖
cd mock && npm install && cd ..
```

### 运行 Demo

```bash
# 一键启动 Demo 环境 (推荐)
./scripts/start_demo.sh

# 或手动启动
# 终端 1: Mock 服务
cd mock && npm start

# 终端 2: 前端
npm run dev
```

访问 http://localhost:5173 查看界面。

## 一键演示脚本

### 🎬 快速演示

**企业级自动化演示脚本，3分钟完整展示所有功能**

```bash
# 1. 给脚本添加执行权限
chmod +x start_demo.sh

# 2. 运行演示脚本
./start_demo.sh

# 3. 选择演示模式
# 1) Full Automated Demo (3 minutes) - 完整演示 ✨推荐
# 2) Quick Demo (1 minute) - 快速演示
# 3) Custom Scenario - 自定义场景
# 4) Performance Metrics Only - 仅显示性能指标
```

### 演示内容

**完整演示包含8个任务 (3分钟)**:

1. **平台登录** (0-30s) - 自动登录 `cityu/2026`
2. **工作界面** (30-60s) - 展示导航和模块
3. **AD视频集成** (60-90s) - 车辆遥测数据和高德地图
4. **100车辆车队** (90-120s) - 优先级排序展示
5. **碰撞风险与AI分析** (120-150s) - 豆包LLM风险分析
6. **Remote Assistant** (150-165s) - 实时驾驶建议
7. **远程接管执行** (165-175s) - 接管流程完成
8. **数据库存储** (175-180s) - IndexedDB记录

**详细使用说明**: 查看 [DEMO_SCRIPT_USAGE.md](docs/DEMO_SCRIPT_USAGE.md)

### API配置 (可选)

系统支持集成外部API增强功能，但**完全可选** - 未配置时自动使用本地fallback功能。

```bash
# 复制环境配置模板
cp .env.example .env.local

# 编辑 .env.local 添加API密钥
VITE_DOUBAO_API_KEY=your_doubao_api_key_here  # 豆包LLM
VITE_AMAP_API_KEY=your_amap_key_here          # 高德地图
VITE_AMAP_JS_CODE=your_amap_js_code_here      # 高德安全码
```

**配置指南**: 查看 [API_CONFIGURATION.md](docs/API_CONFIGURATION.md)

### 编译 C++ 后端 (生产环境)

```bash
# 安装依赖 (参见 docs/BUILD_GUIDE.md)
cd cpp && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## Demo 演示

### 预置演示场景

| 场景 | 描述 | 时长 |
|------|------|------|
| 单车远程控制 | 展示基本的连接和控制功能 | 30秒 |
| 紧急停车演示 | 展示紧急停车和恢复 | 20秒 |
| 网络异常演示 | 展示网络中断处理 | 25秒 |
| 多车调度演示 | 展示多车切换和调度 | 30秒 |

### 模拟控制命令

```bash
# 模拟网络延迟
./scripts/simulate_network.sh delay

# 模拟网络断开
./scripts/simulate_network.sh disconnect

# 触发紧急停车
./scripts/simulate_emergency.sh FSM-01 trigger

# 恢复正常
./scripts/simulate_network.sh reset
```

## 文档

| 文档 | 说明 |
|------|------|
| [编译指南](docs/BUILD_GUIDE.md) | 详细的编译和安装说明 |
| [使用指南](docs/USAGE_GUIDE.md) | 系统使用和操作说明 |
| [Demo 指南](docs/DEMO_GUIDE.md) | 演示场景和脚本 |
| [系统架构](docs/SYSTEM_ARCHITECTURE.md) | 详细的系统设计文档 |
| [测试用例](docs/TEST_CASES.md) | 测试用例和验收标准 |

## 项目结构

```
fsm/
├── cpp/                      # C++ 后端
│   ├── common/              # 公共库 (配置/日志/工具)
│   ├── vehicle_node/        # 车端 ROS2 节点
│   ├── cloud_server/        # 云端服务 (信令/调度/告警)
│   └── operator_client/     # 操作端客户端 (方向盘)
├── src/                     # Vue 3 前端
│   ├── components/          # Vue 组件
│   ├── stores/              # Pinia 状态管理
│   ├── services/            # 后端服务层
│   └── composables/         # Vue Composables
├── mock/                    # Mock 数据服务
├── scripts/                 # 启动和模拟脚本
└── docs/                    # 文档
```

## 配置

### 车端配置 (vehicle_config.yaml)

```yaml
vehicle:
  id: "FSM-01"
  type: "ROBO-TAXI"

sensors:
  cameras:
    - id: "cam_front_center"
      topic: "/sensing/camera/front_center/image_raw"
      fps: 30
      width: 1920
      height: 1080

webrtc:
  signaling:
    url: "wss://your-server.com:8080/signaling"
```

### 云端配置 (cloud_config.yaml)

```yaml
server:
  signaling_port: 8080
  api_port: 8081

scheduling:
  enabled: true
  algorithm: "weighted_priority"
  weights:
    emergency: 0.35
    latency: 0.25
    distance: 0.20
    battery: 0.10
```

## 技术栈

### 后端 (C++)
- **框架**: ROS2 Humble + Autoware.universe
- **通信**: WebRTC (libdatachannel), WebSocket (Boost.Beast)
- **序列化**: Protocol Buffers
- **配置**: YAML-cpp
- **日志**: spdlog

### 前端 (TypeScript)
- **框架**: Vue 3 Composition API
- **状态**: Pinia
- **构建**: Vite
- **地图**: Leaflet
- **3D**: Three.js

## API 接口

### REST API

```
GET    /api/v1/vehicles              # 获取车辆列表
GET    /api/v1/vehicles/{id}/status  # 获取车辆状态
GET    /api/v1/scheduling/queue      # 获取调度队列
GET    /api/v1/alerts                # 获取告警列表
PUT    /api/v1/scheduling/config     # 更新调度配置
```

### WebSocket 事件

```javascript
// 车辆状态更新
{ "event": "vehicle_status", "data": { vehicle_id, speed, steering, ... } }

// 告警
{ "event": "alert", "data": { id, vehicle_id, severity, message } }

// 调度更新
{ "event": "scheduling_update", "data": { queue: [...] } }

// 紧急状态
{ "event": "emergency", "data": { vehicle_id, active } }
```

## 方向盘支持

| 型号 | 支持状态 |
|------|----------|
| Logitech G29 | ✅ 完全支持 |
| Logitech G920 | ✅ 完全支持 |
| Logitech G27 | ✅ 支持 |
| Logitech G923 | ✅ 支持 |

### 按钮映射

| 按钮 | 功能 |
|------|------|
| X (红色) | 紧急停车 |
| □ (方块) | 喇叭 |
| L1/R1 | 左/右转向灯 |
| 左/右拨片 | 降档/升档 |

## 开发

### 启动开发环境

```bash
# 前端开发
npm run dev

# Mock 服务
cd mock && npm start

# C++ 编译
cd cpp/build && make -j4
```

### 代码风格

- C++: Google C++ Style Guide
- TypeScript: ESLint + Prettier
- Vue: Composition API

## 许可证

MIT License

---

<div align="center">
  <sub>Built with ❤️ for autonomous driving</sub>
</div>
