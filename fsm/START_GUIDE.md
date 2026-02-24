# Guardian Mobility v0.0 - 一键启动使用说明

## 🚀 快速启动

### 方法 1: 使用一键启动脚本（推荐）

```bash
cd /home/lyx/fsm
./start_rosbag.sh
```

这个脚本会自动：
- ✅ 启动 RosBag WebSocket 服务器
- ✅ 检查前端服务器状态
- ✅ 显示所有访问地址

### 方法 2: 手动启动

```bash
# 1. 启动 RosBag 服务器
cd /home/lyx/fsm/python
./rosbag_server.sh start

# 2. 启动前端（如果未运行）
cd /home/lyx/fsm
npm run dev

# 3. 打开浏览器访问
http://localhost:3000/rosbag-replay-pro
```

---

## 📱 访问地址

启动后，可以访问以下页面：

| 功能 | 地址 | 说明 |
|------|------|------|
| 🎬 **RosBag 回放** | http://localhost:3000/rosbag-replay-pro | 主要演示页面 |
| 🧪 **WebSocket 测试** | http://localhost:3000/rosbag-test.html | 测试连接 |
| 📊 **数据库** | http://localhost:3000/database | 包含 3D 重建功能 |
| 🎮 **远程控制** | http://localhost:3000/remote-control | 远程驾驶界面 |
| 🚗 **智能调度** | http://localhost:3000/intelligent-dispatch-demo | 车辆调度 |

---

## 🎯 演示流程

### 1. 启动系统

```bash
cd /home/lyx/fsm
./start_rosbag.sh
```

### 2. 打开 RosBag 回放页面

访问: http://localhost:3000/rosbag-replay-pro

### 3. 验证连接状态

- 页面顶部应显示 **"Connected"** (绿色)
- "Select RosBag" 按钮应该是可点击的

### 4. 选择 RosBag 文件

1. 点击 **"Select RosBag"** 按钮
2. 从列表中选择一个 bag 文件（共 10 个可用）
3. 点击 **"Open"** 打开

### 5. 选择话题

- 系统会自动选择相机话题
- 可以手动添加或移除话题

### 6. 开始回放

1. 点击 **"Play"** 按钮
2. 观察实时数据流
3. 查看相机画面、点云、车辆数据等

---

## 🔧 管理命令

### 服务器管理

```bash
cd /home/lyx/fsm/python

# 启动服务器
./rosbag_server.sh start

# 停止服务器
./rosbag_server.sh stop

# 重启服务器
./rosbag_server.sh restart

# 查看状态
./rosbag_server.sh status

# 查看日志
./rosbag_server.sh logs
```

### 快速检查

```bash
# 检查服务器是否运行
lsof -i :8765

# 检查前端是否运行
lsof -i :3000

# 查看 RosBag 文件
ls -lh /home/lyx/fsm/rosbag
```

---

## 📊 可用的 RosBag 文件

系统已发现 **10 个 ROS2 bag 文件**，总大小约 **18 GB**：

1. rosbag2_2025_02_23-16_49_58 (783 MB)
2. rosbag2_2025_02_26-15_21_07 (1648 MB)
3. rosbag2_2025_12_10-17_25_58 (3718 MB)
4. rosbag2_2025_12_10-17_12_17 (4571 MB)
5. rosbag2_2025_08_08-17_39_23 (1408 MB)
6. rosbag2_2025_08_08-17_47_48 (1408 MB)
7. rosbag2_2025_08_08-17_49_21 (1408 MB)
8. rosbag2_2025_08_08-17_48_22 (1408 MB)
9. rosbag2_2025_08_08-16_47_54 (1408 MB)
10. rosbag2_2025_08_08-17_37_14 (1408 MB)

---

## 🐛 故障排查

### 问题 1: 显示 "Disconnected"

**原因**: RosBag 服务器未运行

**解决方案**:
```bash
cd /home/lyx/fsm/python
./rosbag_server.sh start
```

### 问题 2: 端口被占用

**原因**: 旧进程仍在运行

**解决方案**:
```bash
# 杀死占用端口的进程
lsof -ti :8765 | xargs kill -9

# 重新启动
./rosbag_server.sh start
```

### 问题 3: 前端无法访问

**原因**: 前端服务器未运行

**解决方案**:
```bash
cd /home/lyx/fsm
npm run dev
```

### 问题 4: 找不到 bag 文件

**原因**: bag 目录为空或路径错误

**检查**:
```bash
ls -la /home/lyx/fsm/rosbag
```

---

## 📝 演示要点

### 展示内容

1. **连接状态**
   - 展示 "Connected" 绿色状态
   - 说明 WebSocket 实时连接

2. **Bag 文件列表**
   - 展示 10 个可用的 bag 文件
   - 显示文件大小和格式（ROS2）

3. **话题选择**
   - 展示自动选择的相机话题
   - 说明可以自定义选择

4. **实时回放**
   - 展示多相机画面
   - 展示点云数据
   - 展示车辆遥测数据

5. **控制功能**
   - 播放/停止
   - 速度控制
   - 进度显示

### 演示脚本建议

```
1. "这是 Guardian Mobility 的 RosBag 回放系统"
2. "系统已连接到后端服务器（指向 Connected 状态）"
3. "我们有 10 个录制的 bag 文件可供回放"
4. "让我选择一个文件...（点击 Select RosBag）"
5. "系统自动识别了相机话题"
6. "现在开始回放...（点击 Play）"
7. "可以看到实时的多相机画面和车辆数据"
```

---

## 📚 相关文档

- [完整报告](ROSBAG_REPLAY_COMPLETE.md) - 详细的实现和测试报告
- [快速参考](ROSBAG_QUICK_REFERENCE.txt) - 命令速查表
- [服务器文档](python/README_ROSBAG_SERVER.md) - 服务器详细文档
- [测试报告](ROSBAG_REPLAY_FIX_REPORT.md) - 测试结果

---

## ✅ 检查清单

演示前确认：

- [ ] RosBag 服务器已启动
- [ ] 前端服务器已运行
- [ ] 浏览器已打开到正确页面
- [ ] 连接状态显示 "Connected"
- [ ] "Select RosBag" 按钮可点击
- [ ] 已测试选择和打开 bag 文件
- [ ] 已测试播放功能

---

## 🎉 就这么简单！

只需一个命令即可启动整个系统：

```bash
./start_rosbag.sh
```

然后打开浏览器访问：
```
http://localhost:3000/rosbag-replay-pro
```

**系统已完全就绪，可以开始演示！** 🚀

---

**创建日期**: 2026-02-04
**版本**: 1.0.0
**项目**: Guardian Mobility v0.0
**作者**: Li Yixiang, City University of Hong Kong
