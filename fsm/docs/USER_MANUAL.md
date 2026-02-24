# FSM-Pilot V2.0 使用手册

## 目录

1. [快速开始](#1-快速开始)
2. [系统概述](#2-系统概述)
3. [远程控制功能](#3-远程控制功能)
4. [RosBag 回放功能](#4-rosbag-回放功能)
5. [配置指南](#5-配置指南)
6. [API 参考](#6-api-参考)
7. [故障排查](#7-故障排查)
8. [安全指南](#8-安全指南)

---

## 1. 快速开始

### 1.1 系统要求

| 项目 | 要求 |
|-----|------|
| 浏览器 | Chrome 90+ / Firefox 88+ / Edge 90+ |
| 网络 | 稳定的宽带连接 (推荐 10Mbps+) |
| 分辨率 | 1920x1080 或更高 |
| 设备 | 支持游戏手柄/方向盘控制器 |

### 1.2 启动应用

```bash
# 安装依赖
cd /home/lyx/fsm
npm install

# 启动开发服务器
npm run dev

# 或构建生产版本
npm run build
npm run preview
```

### 1.3 访问应用

打开浏览器访问: `http://localhost:3000`

应用界面分为两个主要视图:
- **Remote Control**: 远程驾驶控制界面
- **RosBag Replay**: ROS 数据回放界面

---

## 2. 系统概述

### 2.1 功能模块

```
FSM-Pilot V2.0
├── 远程控制模块
│   ├── 多路视频显示
│   ├── 点云可视化
│   ├── 车辆状态监控
│   ├── 方向盘/手柄控制
│   └── AI 辅助驾驶
├── RosBag 回放模块
│   ├── DB3 文件解析
│   ├── 点云回放
│   ├── GPS 轨迹显示
│   └── 时间轴控制
└── 系统模块
    ├── 车队管理
    ├── 告警系统
    └── 日志记录
```

### 2.2 界面布局

```
┌─────────────────────────────────────────────────────────────┐
│                         Header                               │
│  [状态] [告警数] [选中车辆] [收益]       [视图切换按钮]      │
├──────────┬───────────────────────────────────────┬──────────┤
│          │                                       │          │
│   Left   │                                       │  Right   │
│ Sidebar  │           Video Wall                  │ Sidebar  │
│          │                                       │          │
│ - 地图   │  ┌──────────┐ ┌──────────┐          │ - 车辆   │
│ - 调度   │  │  前视图  │ │  后视图  │          │   状态   │
│ - 告警   │  └──────────┘ └──────────┘          │ - 系统   │
│          │  ┌──────────┐ ┌──────────┐          │   监控   │
│          │  │  左视图  │ │  右视图  │          │ - 日志   │
│          │  └──────────┘ └──────────┘          │          │
│          │                                       │          │
│          │           LiDAR Panel                │          │
│          │  [3D 点云可视化]                     │          │
│          │                                       │          │
├──────────┴───────────────────────────────────────┴──────────┤
│                      Fleet Footer                            │
│  [FSM-01 ●] [FSM-02 ○] [FSM-03 ●] [FSM-04 ○]               │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. 远程控制功能

### 3.1 连接车辆

1. 在 Fleet Footer 中选择目标车辆
2. 点击车辆卡片进行连接
3. 等待连接建立 (显示绿色状态指示)

### 3.2 视频监控

#### 视频布局切换

| 布局 | 说明 |
|------|------|
| 单视图 | 全屏显示单个摄像头 |
| 四宫格 | 同时显示4个摄像头 |
| 画中画 | 主视图+小画面 |

#### 视频控制

- **全屏**: 双击视频区域
- **切换摄像头**: 点击摄像头名称
- **截图**: 右键菜单 -> 保存截图

### 3.3 驾驶控制

#### 键盘控制

| 按键 | 功能 |
|------|------|
| W | 加速 |
| S | 刹车 |
| A | 左转 |
| D | 右转 |
| Space | 紧急停车 |
| P | 停车 (P档) |
| R | 倒车 (R档) |
| N | 空档 |
| Tab | 切换AI辅助 |

#### 手柄控制

支持 Xbox / PlayStation 标准游戏手柄:

| 操作 | 功能 |
|------|------|
| 左摇杆 X轴 | 转向 |
| RT (右扳机) | 油门 |
| LT (左扳机) | 刹车 |
| A 按钮 | 确认 |
| B 按钮 | 取消 |
| X 按钮 | 左转灯 |
| Y 按钮 | 右转灯 |
| RB | 鸣笛 |
| LB | 紧急停车 |

#### 方向盘支持

支持罗技 G29/G920 等方向盘:
- 方向盘: 转向控制
- 油门踏板: 加速
- 刹车踏板: 制动
- 换挡拨片: 档位切换

### 3.4 状态监控

#### 车辆状态面板

```
┌────────────────────────────────────┐
│ Vehicle Status                     │
├────────────────────────────────────┤
│ Speed:     35 km/h     ████████░░ │
│ Steering:  -5°         ◄████████► │
│ Gear:      D           [P R N D]  │
│ Battery:   85%         ████████░░ │
│ Signal:    -65 dBm     ████░░░░░░ │
├────────────────────────────────────┤
│ Latency                            │
│ RTT:       45 ms       ● Excellent │
│ Video:     120 ms      ● Good     │
│ Control:   35 ms       ● Excellent│
└────────────────────────────────────┘
```

#### 系统监控面板

```
┌────────────────────────────────────┐
│ System Monitor                     │
├────────────────────────────────────┤
│ CPU:       45%         ████░░░░░░ │
│ Memory:    62%         ██████░░░░ │
│ GPU:       38%         ███░░░░░░░ │
│ Disk:      55%         █████░░░░░ │
├────────────────────────────────────┤
│ AI Confidence: 92%                 │
│ Mode: Assisted Driving             │
└────────────────────────────────────┘
```

### 3.5 AI 辅助功能

#### 功能列表

- **车道保持辅助 (LKA)**: 自动保持车道中央
- **自适应巡航 (ACC)**: 自动调节跟车距离
- **自动紧急制动 (AEB)**: 检测障碍物自动刹车
- **盲点监测 (BSM)**: 提示盲区车辆

#### 启用/禁用

```
AI Bar:
[LKA ✓] [ACC ○] [AEB ✓] [BSM ✓]  AI Confidence: 92%
```

点击功能按钮切换启用状态。

---

## 4. RosBag 回放功能

### 4.1 加载文件

1. 点击右上角 **RosBag Replay** 切换视图
2. 点击 **📁 Open Bag** 按钮
3. 选择 `.db3` 或 `.mcap` 文件
4. 等待解析完成

支持的文件格式:
- ROS 2 SQLite3 (.db3)
- MCAP (.mcap)

### 4.2 数据可视化

#### 点云视图

```
┌─────────────────────────────────────┐
│ 🔵 Point Cloud                      │
├─────────────────────────────────────┤
│                                     │
│        3D 点云可视化区域            │
│                                     │
│  鼠标控制:                          │
│  - 左键拖动: 旋转视角               │
│  - 右键拖动: 平移视角               │
│  - 滚轮: 缩放                       │
│                                     │
└─────────────────────────────────────┘
```

#### GPS 轨迹视图

```
┌─────────────────────────────────────┐
│ 🗺️ GPS Track                        │
├─────────────────────────────────────┤
│                                     │
│     [地图背景]                      │
│                                     │
│      ●──────●──────●                │
│             ↑                       │
│          当前位置                   │
│                                     │
└─────────────────────────────────────┘
```

#### 鸟瞰图视图

显示车辆位置、检测到的物体、规划路径等。

### 4.3 播放控制

#### 时间轴

```
00:15.32 ────────────●────────────────── 05:30.00
         [⏮] [▶] [⏭] [⏹]  [🔁] [1x ▼]
```

#### 控制按钮

| 按钮 | 功能 |
|------|------|
| ⏮ | 后退 10 帧 |
| ▶/⏸ | 播放/暂停 |
| ⏭ | 前进 10 帧 |
| ⏹ | 停止并重置 |
| 🔁 | 循环播放 |
| 1x | 播放速度 (0.25x - 8x) |

#### 键盘快捷键

| 按键 | 功能 |
|------|------|
| Space | 播放/暂停 |
| ← | 后退 1 秒 |
| → | 前进 1 秒 |
| [ | 减慢速度 |
| ] | 加快速度 |
| L | 切换循环 |

### 4.4 Topics 管理

右侧面板显示所有 Topic:

```
Topics (12)
┌────────────────────────────────────┐
│ ● /rslidar_points                  │
│   PointCloud2    10.0 Hz    5,230  │
├────────────────────────────────────┤
│ ○ /gps/fix                         │
│   NavSatFix      10.0 Hz    5,230  │
├────────────────────────────────────┤
│ ○ /camera/front/image              │
│   Image          30.0 Hz   15,690  │
└────────────────────────────────────┘
```

- **●** 表示已选中/启用
- 点击 Topic 切换选中状态

### 4.5 设置面板

点击 ⚙️ 打开设置:

```
Settings
┌────────────────────────────────────┐
│ Point Cloud                        │
│ Max Points: [50,000 ────●────]     │
│ Auto Update: [✓]                   │
├────────────────────────────────────┤
│ Playback                           │
│ Frame Rate: [20 Hz ▼]              │
└────────────────────────────────────┘
```

---

## 5. 配置指南

### 5.1 环境变量

创建 `.env.local` 文件:

```env
# API 服务器
VITE_API_URL=https://api.fsm-pilot.com

# WebSocket 信令服务器
VITE_SIGNALING_URL=wss://signaling.fsm-pilot.com

# TURN 服务器
VITE_TURN_HOST=turn.fsm-pilot.com
VITE_TURN_PORT=3478

# 功能开关
VITE_ENABLE_AI_ASSIST=true
VITE_ENABLE_RECORDING=true
VITE_DEBUG_MODE=false
```

### 5.2 视频设置

```typescript
// src/config/video.ts
export const videoConfig = {
  // 默认分辨率
  resolution: { width: 1920, height: 1080 },

  // 码率 (bps)
  bitrate: 4000000,

  // 帧率
  frameRate: 30,

  // 编码器
  codec: 'h264',

  // 缓冲区大小 (ms)
  bufferSize: 100
}
```

### 5.3 控制设置

```typescript
// src/config/control.ts
export const controlConfig = {
  // 控制频率 (Hz)
  updateRate: 20,

  // 转向灵敏度 (0-1)
  steeringSensitivity: 0.8,

  // 油门灵敏度 (0-1)
  throttleSensitivity: 0.6,

  // 刹车灵敏度 (0-1)
  brakeSensitivity: 1.0,

  // 死区
  deadzone: 0.05,

  // 最大转向角 (度)
  maxSteeringAngle: 30,

  // 紧急停车减速度 (m/s²)
  emergencyDeceleration: -5.0
}
```

---

## 6. API 参考

### 6.1 远程控制 API

#### useRemoteControlOptimized

```typescript
import { useRemoteControlOptimized } from '@/services/remoteControlOptimized'

const {
  // 状态
  connectionState,  // 连接状态
  latencyStats,     // 延迟统计
  telemetry,        // 遥测数据
  isControlEnabled, // 控制是否启用

  // 计算属性
  isConnected,      // 是否已连接
  isConnecting,     // 是否正在连接

  // 方法
  connect,          // 连接车辆
  disconnect,       // 断开连接
  sendControl,      // 发送控制命令
  emergencyStop,    // 紧急停车
  enableControl,    // 启用控制
  disableControl,   // 禁用控制
  onVideoStream,    // 注册视频流回调
  onTelemetry       // 注册遥测回调
} = useRemoteControlOptimized(config)
```

#### 连接车辆

```typescript
// 连接到车辆
await connect('FSM-01')

// 断开连接
disconnect()
```

#### 发送控制命令

```typescript
// 发送控制命令
sendControl({
  steering: 0.5,   // -1 (左) 到 1 (右)
  throttle: 0.3,   // 0 到 1
  brake: 0,        // 0 到 1
  gear: 3          // 0=P, 1=R, 2=N, 3=D
})

// 紧急停车
emergencyStop()
```

### 6.2 RosBag 解析 API

#### useRosBagDb3ParserOptimized

```typescript
import { useRosBagDb3ParserOptimized } from '@/services/rosbagDb3ParserOptimized'

const {
  // 状态
  isLoading,        // 加载中
  error,            // 错误信息
  bagInfo,          // Bag 信息
  topics,           // Topic 列表
  progress,         // 加载进度
  stats,            // 统计信息

  // 方法
  loadDb3File,              // 加载 DB3 文件
  getMessagesInRange,       // 获取时间范围内的消息
  getMessageAtTime,         // 获取指定时间的消息
  preloadAllMessages,       // 预加载消息
  getCachedMessageAtTime,   // 从缓存获取消息
  getGpsTrajectory,         // 获取 GPS 轨迹
  hasTopicByName,           // 检查 Topic 是否存在
  hasTopicByType,           // 检查 Topic 类型是否存在
  getTopicNames,            // 获取 Topic 名称列表
  getPointCloudTopicName,   // 获取点云 Topic 名称
  dispose                   // 释放资源
} = useRosBagDb3ParserOptimized()
```

#### 加载文件

```typescript
// 加载 DB3 文件
const success = await loadDb3File(file)

if (success) {
  console.log('Bag info:', bagInfo.value)
  console.log('Topics:', topics.value)
}
```

#### 获取消息

```typescript
// 获取指定时间的消息
const message = getCachedMessageAtTime('/rslidar_points', timestamp)

// 获取时间范围内的消息
const messages = getMessagesInRange('/gps/fix', startTime, endTime)

// 获取 GPS 轨迹
const trajectory = getGpsTrajectory()
```

---

## 7. 故障排查

### 7.1 连接问题

| 问题 | 可能原因 | 解决方案 |
|-----|---------|---------|
| 无法连接 | 网络问题 | 检查网络连接，尝试刷新页面 |
| 连接超时 | 服务器问题 | 检查服务器状态，联系管理员 |
| 频繁断连 | 网络不稳定 | 切换到更稳定的网络 |

### 7.2 视频问题

| 问题 | 可能原因 | 解决方案 |
|-----|---------|---------|
| 视频黑屏 | 解码失败 | 更新浏览器，检查 GPU 加速 |
| 视频卡顿 | 带宽不足 | 降低视频质量设置 |
| 延迟过高 | 网络拥塞 | 检查网络延迟，使用 TURN |

### 7.3 控制问题

| 问题 | 可能原因 | 解决方案 |
|-----|---------|---------|
| 无法控制 | 控制未启用 | 点击"启用控制"按钮 |
| 响应慢 | 延迟高 | 检查网络延迟 |
| 手柄无响应 | 未检测到 | 重新连接手柄，检查权限 |

### 7.4 RosBag 问题

| 问题 | 可能原因 | 解决方案 |
|-----|---------|---------|
| 加载失败 | 文件损坏 | 检查文件完整性 |
| 内存不足 | 文件过大 | 增加缓存限制或分段加载 |
| 解析错误 | 格式不支持 | 确认文件为 ROS 2 格式 |

### 7.5 浏览器控制台调试

按 F12 打开开发者工具，查看 Console 标签页中的日志:

```
[WebRTC] Connected to signaling
[WebRTC] ICE connection state: connected
[RemoteControl] Control enabled
[DB3 Parser] Loaded: test.db3 (52,300 messages, 523.5s)
```

---

## 8. 安全指南

### 8.1 操作安全

- **始终保持注意力集中**，远程驾驶需要全神贯注
- **了解紧急停车方法**，熟练使用 Space 键或 LB 按钮
- **监控延迟指标**，延迟过高时降低速度或停车
- **确保网络稳定**，避免在信号差的区域操作

### 8.2 数据安全

- **不要分享连接凭证**
- **定期更新密码**
- **使用安全的网络环境**
- **及时登出系统**

### 8.3 紧急情况处理

1. **立即按下紧急停车**
2. **保持冷静，观察周围环境**
3. **联系现场安全员**
4. **记录事件详情**
5. **上报事故**

---

## 附录

### A. 键盘快捷键汇总

| 快捷键 | 功能 | 适用视图 |
|-------|------|---------|
| Space | 紧急停车/播放暂停 | 全部 |
| W/S/A/D | 驾驶控制 | 远程控制 |
| Tab | 切换 AI | 远程控制 |
| ←/→ | 时间跳转 | RosBag |
| [/] | 速度调节 | RosBag |
| L | 循环播放 | RosBag |
| F | 全屏 | 全部 |
| Esc | 退出全屏 | 全部 |

### B. 术语表

| 术语 | 解释 |
|-----|------|
| RTT | Round-Trip Time，往返时延 |
| TURN | 中继服务器，用于 NAT 穿透 |
| STUN | 用于获取公网 IP 的服务器 |
| ICE | 交互式连接建立 |
| SDP | 会话描述协议 |
| LiDAR | 激光雷达 |
| GNSS | 全球导航卫星系统 |
| IMU | 惯性测量单元 |

### C. 联系支持

- **技术支持**: support@fsm-pilot.com
- **问题反馈**: https://github.com/your-org/fsm-pilot/issues
- **文档网站**: https://docs.fsm-pilot.com
