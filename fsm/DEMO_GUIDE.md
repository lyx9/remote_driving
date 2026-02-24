# Guardian Mobility v0.0 - 完整演示系统使用指南

## 🚀 一键启动所有功能

### 快速开始（3步）

```bash
# 1. 进入项目目录
cd /home/lyx/fsm

# 2. 启动所有服务
./start_all.sh start

# 3. 查看演示指南
./start_all.sh demo
```

就这么简单！系统会自动启动所有服务并提供完整的演示指南。

---

## 📋 系统功能概览

Guardian Mobility 是一个完整的 AI 驱动的远程驾驶和车队管理平台，包含以下功能模块：

### 1. 🎮 远程控制 (Remote Control)
**地址**: http://localhost:3000/remote-control

**功能**:
- 📹 多路视频墙显示
- 🔵 LiDAR 3D 实时可视化
- 📊 车辆遥测数据（速度、转向、制动等）
- 🤖 AI 驾驶建议
- 🗺️ 高德地图实时定位

**演示要点**:
- 展示多相机实时画面
- 演示 LiDAR 点云可视化
- 查看 AI 实时驾驶建议

---

### 2. 🚗 智能调度 (Intelligent Dispatch)
**地址**: http://localhost:3000/intelligent-dispatch-demo

**功能**:
- 📋 车辆队列管理
- ⚠️ 风险评分系统
- 👤 安全员智能分配
- 🗺️ 车辆位置地图显示
- 📈 实时统计数据

**演示要点**:
- 展示车辆队列和优先级
- 演示风险评分算法
- 查看安全员分配逻辑

---

### 3. 📊 数据库可视化 (Database Visualization)
**地址**: http://localhost:3000/database-visualization

**功能**:
- 🚙 车辆记录管理
- 🎮 接管事件记录
- 🤖 AI 分析记录
- 👥 安全员管理
- 🎨 **3D 场景重建** (3DGS)

**演示要点**:
- 查看历史数据记录
- 展示 3D 重建功能（如果已启动 3DGS 服务器）

---

### 4. 🎬 RosBag 回放 (RosBag Replay)
**地址**: http://localhost:3000/rosbag-replay-pro

**功能**:
- 📁 选择和加载 RosBag 文件
- 📹 多相机同步回放
- 🔵 点云数据可视化
- 🚗 车辆数据回放
- ⏯️ 播放控制（播放/暂停/速度）

**当前状态**: ✅ 已发现 10 个 ROS2 bag 文件 (~18 GB)

**演示要点**:
- 验证 "Connected" 状态
- 选择一个 bag 文件
- 演示多相机实时回放

---

## 🎯 完整演示流程

### 方案 A: 自动演示（推荐）

```bash
cd /home/lyx/fsm
./start_all.sh start    # 启动所有服务
./start_all.sh demo     # 查看演示指南并自动打开浏览器
```

### 方案 B: 手动演示

1. **启动系统**
   ```bash
   ./start_all.sh start
   ```

2. **按顺序访问各功能模块**
   - 远程控制: http://localhost:3000/remote-control
   - 智能调度: http://localhost:3000/intelligent-dispatch-demo
   - 数据库: http://localhost:3000/database-visualization
   - RosBag 回放: http://localhost:3000/rosbag-replay-pro

3. **演示完成后停止**
   ```bash
   ./start_all.sh stop
   ```

---

## 🔧 管理命令

```bash
# 启动所有服务
./start_all.sh start

# 停止所有服务
./start_all.sh stop

# 重启所有服务
./start_all.sh restart

# 查看系统状态
./start_all.sh status

# 查看演示指南
./start_all.sh demo

# 查看服务日志
./start_all.sh logs

# 测试服务连接
./start_all.sh test
```

---

## 📊 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    前端 (Vue 3 + TypeScript)                │
│                    Port: 3000                               │
│  ┌──────────┬──────────┬──────────┬──────────────────┐    │
│  │远程控制  │智能调度  │数据库    │RosBag 回放       │    │
│  └──────────┴──────────┴──────────┴──────────────────┘    │
└─────────────────────────────────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
        ▼                 ▼                 ▼
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│ RosBag       │  │ 3DGS         │  │ 高德地图     │
│ WebSocket    │  │ 重建服务器   │  │ API          │
│ 服务器       │  │              │  │              │
│ Port: 8765   │  │ Port: 5000   │  │              │
└──────────────┘  └──────────────┘  └──────────────┘
```

---

## 🎬 演示脚本建议

### 开场（1分钟）
```
"欢迎来到 Guardian Mobility v0.0 演示。
这是一个完整的 AI 驱动的远程驾驶和车队管理平台，
由香港城市大学开发。"
```

### 远程控制演示（3分钟）
```
"首先看远程控制界面。
这里可以看到多路摄像头的实时画面，
LiDAR 的 3D 点云可视化，
以及 AI 实时提供的驾驶建议。"
```

### 智能调度演示（2分钟）
```
"接下来是智能调度系统。
系统会根据车辆状态和风险评分，
自动分配最合适的安全员进行接管。"
```

### 数据库演示（2分钟）
```
"数据库模块记录了所有的历史数据，
包括车辆记录、接管事件、AI 分析等。
还集成了 3D 场景重建功能。"
```

### RosBag 回放演示（2分钟）
```
"最后是 RosBag 回放功能。
我们可以选择历史记录的 bag 文件，
回放当时的多相机画面和传感器数据。
系统已经发现了 10 个可用的 bag 文件。"
```

---

## ✅ 启动前检查清单

- [ ] 确保在项目目录: `cd /home/lyx/fsm`
- [ ] 脚本有执行权限: `chmod +x start_all.sh`
- [ ] 端口未被占用: 3000, 8765, 5000
- [ ] Python 环境正常
- [ ] Node.js 环境正常

---

## 🐛 故障排查

### 问题 1: 端口被占用

**解决方案**:
```bash
# 停止所有服务
./start_all.sh stop

# 等待几秒
sleep 3

# 重新启动
./start_all.sh start
```

### 问题 2: 服务启动失败

**解决方案**:
```bash
# 查看日志
./start_all.sh logs

# 检查具体错误信息
tail -50 /tmp/guardian_mobility/rosbag_server.log
tail -50 /tmp/guardian_mobility/frontend.log
```

### 问题 3: RosBag 显示 Disconnected

**解决方案**:
```bash
# 检查 RosBag 服务器状态
./start_all.sh status

# 如果未运行，重启服务
./start_all.sh restart
```

---

## 📚 相关文档

| 文档 | 说明 |
|------|------|
| [START_GUIDE.md](START_GUIDE.md) | RosBag 回放启动指南 |
| [ROSBAG_REPLAY_COMPLETE.md](ROSBAG_REPLAY_COMPLETE.md) | RosBag 完整技术报告 |
| [ROSBAG_QUICK_REFERENCE.txt](ROSBAG_QUICK_REFERENCE.txt) | 快速命令参考 |
| [PROJECT_INTEGRATION_COMPLETE.md](PROJECT_INTEGRATION_COMPLETE.md) | 项目整合报告 |

---

## 🎉 就这么简单！

只需一个命令即可启动完整的演示系统：

```bash
./start_all.sh start
```

然后访问任意功能模块开始演示。

**系统已完全就绪，可以立即开始演示！** 🚀

---

**创建日期**: 2026-02-04
**版本**: 1.0.0
**项目**: Guardian Mobility v0.0
**作者**: Li Yixiang
**机构**: City University of Hong Kong
