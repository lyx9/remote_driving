# FSM-Pilot V2.0 企业级远程驾驶平台架构设计

## 一、系统概述

### 1.1 项目愿景
构建企业级远程驾驶平台，支持:
- 多车辆实时监控与远程操控
- 低延迟多摄像头视频传输
- 专业方向盘设备支持
- 数据存储、回放与3D重建
- 与 Autoware.universe 无缝集成

### 1.2 升级目标 (V2.0)

| 功能模块 | 描述 | 优先级 |
|---------|------|--------|
| 键盘/方向盘控制 | 本地设备控制远程车辆 | P0 |
| 多摄像头WebRTC | 阿里云TURN/基于WebRTC协议 | P0 |
| 模拟演示系统 | Mock车辆/道路/障碍物 | P0 |
| 数据存储系统 | RosBag/视频存储 + Tag | P1 |
| 本地数据库 | SQLite存储/检索/导出 | P1 |
| 3D-GS渲染 | 3D高斯溅射场景重建 | P2 |

---

## 二、系统架构图

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                         FSM-Pilot V2.0 Enterprise Platform                    │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │                         操作端 (Operator Client)                         │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐ │ │
│  │  │  Vue 3 前端   │  │ 键盘控制模块 │  │ 方向盘模块  │  │ 3DGS渲染器 │ │ │
│  │  │  多摄像头视频 │  │ WASD控制    │  │ G29/G920   │  │ WebGL渲染  │ │ │
│  │  │  地图/雷达    │  │ 箭头+空格   │  │ 力反馈     │  │ 点云融合   │ │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘  └─────────────┘ │ │
│  │                           ↓                                              │ │
│  │  ┌─────────────────────────────────────────────────────────────────────┐│ │
│  │  │                    本地数据存储层 (SQLite)                           ││ │
│  │  │  RosBag存储 │ 视频存储 │ 标签管理 │ 检索引擎 │ 导入导出            ││ │
│  │  └─────────────────────────────────────────────────────────────────────┘│ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                    ↕ WebRTC / WebSocket                       │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │                         云端服务 (Cloud Server)                          │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐ │ │
│  │  │  信令服务器   │  │ 调度引擎    │  │ 告警系统    │  │ TURN服务器 │ │ │
│  │  │  WebSocket   │  │ 优先级调度   │  │ 规则引擎    │  │ 阿里云RTC  │ │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘  └─────────────┘ │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────────────┐ │ │
│  │  │  API 网关    │  │ 预测模型    │  │ 数据同步服务                  │ │ │
│  │  │  REST API   │  │ 延迟补偿    │  │ RosBag/视频 → 云存储         │ │ │
│  │  └──────────────┘  └──────────────┘  └──────────────────────────────┘ │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                    ↕ WebRTC / ROS2 DDS                        │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │                         车端节点 (Vehicle Node)                          │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐ │ │
│  │  │  数据采集    │  │ 指令执行    │  │ WebRTC客户端 │  │ 视频编码器 │ │ │
│  │  │  传感器订阅  │  │ Autoware   │  │ 多通道传输   │  │ H.264/VP8 │ │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘  └─────────────┘ │ │
│  │                                                                          │ │
│  │  ┌─────────────────────────────────────────────────────────────────────┐│ │
│  │  │                    Autoware.universe Integration                     ││ │
│  │  │  /vehicle/status │ /control/command │ /localization │ /perception   ││ │
│  │  └─────────────────────────────────────────────────────────────────────┘│ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                                                               │
│  ┌─────────────────────────────────────────────────────────────────────────┐ │
│  │                         模拟演示系统 (Mock System)                       │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐ │ │
│  │  │  虚拟车辆    │  │ 道路场景    │  │ 障碍物系统  │  │ 场景编辑器 │ │ │
│  │  │  物理仿真    │  │ 多场景支持  │  │ 动态障碍    │  │ 实时编辑   │ │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘  └─────────────┘ │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
│                                                                               │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## 三、核心模块设计

### 3.1 键盘控制模块

```typescript
// 键盘映射方案
interface KeyboardMapping {
  // 方向控制
  'KeyW' | 'ArrowUp': '加速/前进',
  'KeyS' | 'ArrowDown': '减速/后退',
  'KeyA' | 'ArrowLeft': '左转',
  'KeyD' | 'ArrowRight': '右转',

  // 功能键
  'Space': '刹车',
  'KeyE': '紧急停车',
  'KeyQ': '切换档位',
  'KeyR': '重置/归位',

  // 转向灯
  'KeyZ': '左转向灯',
  'KeyC': '右转向灯',
  'KeyX': '双闪',

  // 视图控制
  '1-5': '切换摄像头',
  'Tab': '切换车辆',
  'M': '地图全屏',
}
```

### 3.2 WebRTC多摄像头架构 (阿里云)

```yaml
WebRTC配置:
  TURN服务器:
    - provider: 阿里云RTC
    - urls:
      - turn:rtc.aliyuncs.com:443?transport=tcp
      - turns:rtc.aliyuncs.com:443?transport=tcp
    - 认证: 临时凭证 (STS)

  视频通道 (5路):
    - cam_front_center: 主摄像头 (1080p, 30fps)
    - cam_front_left: 前左 (720p, 25fps)
    - cam_front_right: 前右 (720p, 25fps)
    - cam_rear_left: 后左 (720p, 20fps)
    - cam_rear_right: 后右 (720p, 20fps)

  编码器:
    - 首选: H.264 (硬件加速)
    - 备选: VP8/VP9
    - 自适应码率: 1-8 Mbps
```

### 3.3 数据存储架构

```sql
-- SQLite 数据库结构
CREATE TABLE recordings (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id TEXT NOT NULL,
    vehicle_id TEXT NOT NULL,
    start_time DATETIME,
    end_time DATETIME,
    duration_seconds INTEGER,
    file_path TEXT,
    file_size_bytes INTEGER,
    type TEXT CHECK(type IN ('rosbag', 'video', 'telemetry')),
    status TEXT CHECK(status IN ('recording', 'completed', 'error')),
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE tags (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    recording_id INTEGER REFERENCES recordings(id),
    name TEXT NOT NULL,
    color TEXT,
    timestamp_ms INTEGER,
    description TEXT,
    created_at DATETIME DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE categories (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL UNIQUE,
    parent_id INTEGER REFERENCES categories(id),
    color TEXT
);

CREATE TABLE recording_categories (
    recording_id INTEGER REFERENCES recordings(id),
    category_id INTEGER REFERENCES categories(id),
    PRIMARY KEY (recording_id, category_id)
);

-- 索引优化
CREATE INDEX idx_recordings_vehicle ON recordings(vehicle_id);
CREATE INDEX idx_recordings_time ON recordings(start_time);
CREATE INDEX idx_tags_recording ON tags(recording_id);
```

### 3.4 3D-GS渲染架构

```typescript
interface GaussianSplattingConfig {
  // 渲染参数
  resolution: {
    width: 1920,
    height: 1080
  },

  // 高斯参数
  gaussians: {
    maxCount: 1000000,      // 最大高斯数
    shDegree: 3,            // 球谐函数阶数
    opacity: [0.0, 1.0],    // 不透明度范围
    scale: [0.001, 0.1],    // 尺度范围
  },

  // 相机参数
  camera: {
    fov: 60,
    near: 0.1,
    far: 1000,
  },

  // LOD配置
  lod: {
    enabled: true,
    levels: [100, 500, 2000], // 距离阈值
    quality: ['high', 'medium', 'low']
  }
}
```

---

## 四、数据流图

### 4.1 控制指令流

```
┌────────────┐    ┌────────────┐    ┌────────────┐    ┌────────────┐
│  键盘/方向盘  │───►│  前端处理  │───►│ WebSocket │───►│ 车端执行  │
│   输入事件   │    │  归一化   │    │   传输    │    │ Autoware │
└────────────┘    └────────────┘    └────────────┘    └────────────┘
      │                 │                 │                 │
      │                 ▼                 │                 ▼
      │          ┌────────────┐          │          ┌────────────┐
      │          │ 控制约束   │          │          │ 安全检查  │
      │          │ 速率限制   │          │          │ 指令验证  │
      │          └────────────┘          │          └────────────┘
      │                                  │
      └────── 延迟测量 <─────────────────┘
```

### 4.2 视频流

```
┌────────────┐    ┌────────────┐    ┌────────────┐    ┌────────────┐
│  车载摄像头  │───►│ H.264编码 │───►│  WebRTC   │───►│  前端解码  │
│  5路视频   │    │ 自适应码率 │    │ TURN穿透  │    │  视频渲染  │
└────────────┘    └────────────┘    └────────────┘    └────────────┘
                       │                                    │
                       ▼                                    ▼
                ┌────────────┐                      ┌────────────┐
                │ 本地录制   │                      │ 本地存储   │
                │ RosBag    │                      │ MP4转码   │
                └────────────┘                      └────────────┘
```

### 4.3 数据存储流

```
┌────────────────────────────────────────────────────────────────┐
│                      数据存储管线                               │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  原始数据                                                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                    │
│  │ ROS话题  │  │ 视频流   │  │ 遥测数据 │                    │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘                    │
│       │             │             │                           │
│       ▼             ▼             ▼                           │
│  ┌──────────────────────────────────────┐                    │
│  │           数据采集层                  │                    │
│  │   RosBag Writer │ FFmpeg │ JSON     │                    │
│  └──────────────────────────────────────┘                    │
│                     │                                         │
│                     ▼                                         │
│  ┌──────────────────────────────────────┐                    │
│  │           元数据管理 (SQLite)         │                    │
│  │  • 文件索引 • 标签 • 分类 • 时间戳   │                    │
│  └──────────────────────────────────────┘                    │
│                     │                                         │
│       ┌─────────────┼─────────────┐                          │
│       ▼             ▼             ▼                          │
│  ┌─────────┐  ┌─────────┐  ┌─────────────┐                  │
│  │ 本地存储 │  │ 云同步  │  │ 3DGS处理   │                  │
│  │ /data/  │  │ 阿里OSS │  │ 场景重建   │                  │
│  └─────────┘  └─────────┘  └─────────────┘                  │
│                                                               │
└────────────────────────────────────────────────────────────────┘
```

---

## 五、接口定义

### 5.1 控制指令接口

```typescript
// 统一控制指令格式
interface ControlCommand {
  timestamp: number;           // 时间戳 (ms)
  sequence: number;            // 序列号
  vehicle_id: string;          // 目标车辆

  // 纵向控制
  throttle: number;            // 油门 [0, 1]
  brake: number;               // 刹车 [0, 1]

  // 横向控制
  steering: number;            // 转向 [-1, 1] (左负右正)

  // 档位
  gear: 'P' | 'R' | 'N' | 'D'; // 档位

  // 灯光
  turn_signal: -1 | 0 | 1;     // 转向灯
  hazard: boolean;             // 双闪

  // 紧急
  emergency_stop: boolean;     // 紧急停车

  // 来源
  source: 'keyboard' | 'wheel' | 'api';
}
```

### 5.2 存储接口

```typescript
interface RecordingService {
  // 录制控制
  startRecording(options: RecordingOptions): Promise<string>; // 返回session_id
  stopRecording(sessionId: string): Promise<RecordingResult>;
  pauseRecording(sessionId: string): Promise<void>;
  resumeRecording(sessionId: string): Promise<void>;

  // 标签管理
  addTag(recordingId: number, tag: TagInput): Promise<Tag>;
  removeTag(tagId: number): Promise<void>;
  getTags(recordingId: number): Promise<Tag[]>;

  // 分类管理
  setCategories(recordingId: number, categoryIds: number[]): Promise<void>;

  // 查询
  search(query: SearchQuery): Promise<Recording[]>;
  getRecording(id: number): Promise<Recording>;

  // 导出
  exportRecording(id: number, format: ExportFormat): Promise<Blob>;
  exportBatch(ids: number[], format: ExportFormat): Promise<Blob>;
}

interface RecordingOptions {
  vehicleId: string;
  channels: ('video' | 'rosbag' | 'telemetry')[];
  videoQuality?: 'high' | 'medium' | 'low';
  rosbagTopics?: string[];
}
```

### 5.3 3D-GS接口

```typescript
interface GaussianSplattingRenderer {
  // 初始化
  initialize(canvas: HTMLCanvasElement, config: GSConfig): Promise<void>;

  // 场景管理
  loadScene(plyPath: string): Promise<void>;
  loadFromRosBag(bagPath: string, options: RosBagOptions): Promise<void>;

  // 渲染控制
  render(): void;
  setCamera(pose: CameraPose): void;
  setCameraFromVehicle(vehicleState: VehicleState): void;

  // LOD控制
  setQuality(level: 'high' | 'medium' | 'low'): void;

  // 交互
  onMouseMove(callback: (ray: Ray) => void): void;

  // 资源管理
  dispose(): void;
}
```

---

## 六、技术选型

### 6.1 前端技术栈

| 模块 | 技术 | 版本 | 用途 |
|------|------|------|------|
| 框架 | Vue 3 | 3.4.x | UI框架 |
| 状态 | Pinia | 2.1.x | 状态管理 |
| 类型 | TypeScript | 5.x | 类型安全 |
| 构建 | Vite | 5.x | 构建工具 |
| WebRTC | simple-peer | 9.x | WebRTC封装 |
| 3D渲染 | Three.js | 0.162 | 3D图形 |
| 3D-GS | gsplat.js | 最新 | 高斯溅射 |
| 数据库 | sql.js | 1.8 | SQLite WASM |
| 地图 | Leaflet | 1.9 | 地图导航 |

### 6.2 后端技术栈

| 模块 | 技术 | 版本 | 用途 |
|------|------|------|------|
| 运行时 | ROS2 Humble | - | 机器人框架 |
| 语言 | C++ | 17 | 后端开发 |
| WebRTC | libdatachannel | 0.20 | WebRTC实现 |
| HTTP | Boost.Beast | 1.74 | HTTP服务器 |
| 序列化 | Protobuf | 3.12 | 消息序列化 |
| 配置 | yaml-cpp | 0.7 | YAML解析 |
| 日志 | spdlog | 1.11 | 日志系统 |
| 视频 | FFmpeg | 5.x | 视频编码 |

### 6.3 云服务 (阿里云)

| 服务 | 用途 | 配置 |
|------|------|------|
| 阿里云RTC | TURN服务器 | 按需计费 |
| OSS | 云存储 | 标准存储 |
| ECS | 信令服务器 | 2核4G |
| CDN | 静态资源 | 全国加速 |

---

## 七、部署架构

```yaml
# docker-compose.yml 概要
services:
  # 前端服务
  frontend:
    image: fsm-frontend:v2.0
    ports: ["80:80", "443:443"]

  # 信令服务器
  signaling:
    image: fsm-signaling:v2.0
    ports: ["8080:8080"]

  # API服务器
  api:
    image: fsm-api:v2.0
    ports: ["8081:8081"]
    volumes:
      - ./data:/app/data

  # Mock服务器 (开发环境)
  mock:
    image: fsm-mock:v2.0
    ports: ["3001:3001", "3002:3002"]
```

---

## 八、开发里程碑

### Phase 1: 核心功能 (Week 1-2)
- [x] 项目架构设计
- [ ] 键盘控制模块实现
- [ ] Mock车辆/道路/障碍物系统
- [ ] 控制指令WebSocket传输

### Phase 2: 媒体传输 (Week 3-4)
- [ ] WebRTC多摄像头实现
- [ ] 阿里云TURN集成
- [ ] 视频编码器完善
- [ ] 方向盘设备集成

### Phase 3: 数据存储 (Week 5-6)
- [ ] SQLite数据库集成
- [ ] RosBag存储功能
- [ ] 视频录制功能
- [ ] 标签和分类系统
- [ ] 数据导入导出

### Phase 4: 3D-GS渲染 (Week 7-8)
- [ ] 3D高斯溅射渲染器
- [ ] RosBag → 3D-GS转换
- [ ] Mock 3D-GS数据
- [ ] 交互式场景浏览

### Phase 5: 集成测试 (Week 9-10)
- [ ] 端到端集成测试
- [ ] 性能优化
- [ ] 文档完善
- [ ] Demo演示准备

---

## 九、性能指标

| 指标 | 目标值 | 测量方法 |
|------|--------|----------|
| 控制延迟 | < 100ms | RTT测量 |
| 视频延迟 | < 200ms | 端到端测量 |
| 视频帧率 | 30fps | 前端测量 |
| 丢包率 | < 1% | WebRTC统计 |
| 3D-GS渲染 | > 30fps | requestAnimationFrame |
| 数据库查询 | < 50ms | 查询耗时 |

---

## 十、安全考虑

1. **通信安全**: WebSocket/WebRTC全部使用TLS加密
2. **认证授权**: JWT令牌 + 角色权限控制
3. **数据安全**: 本地数据AES加密存储
4. **操作审计**: 所有控制指令记录审计日志
5. **紧急机制**: 多级紧急停车保障

---

*文档版本: V2.0-Draft*
*最后更新: 2026-01-07*
