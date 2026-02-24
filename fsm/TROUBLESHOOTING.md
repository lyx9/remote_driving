# RosBag Replay 故障排查指南

## 问题：无法选择 RosBag 文件

### 症状
- 页面显示 "Connected" (绿色)
- 显示 "Topics (0)"
- 看不到或无法点击 "Select RosBag" 按钮

### 排查步骤

#### 1. 检查服务器状态

```bash
cd /home/lyx/fsm
./start_all.sh status
```

确认：
- ✓ RosBag WebSocket 服务器运行中 (Port 8765)
- ✓ 前端开发服务器运行中 (Port 3000)

#### 2. 测试 WebSocket 连接

打开调试页面：
```
http://localhost:3000/rosbag-debug.html
```

步骤：
1. 点击 "Test Connection"
2. 等待连接成功
3. 点击 "List Bags"
4. 应该看到 10 个 bag 文件

如果调试页面正常，说明服务器没问题。

#### 3. 检查浏览器控制台

在 RosBag Replay Pro 页面：
```
http://localhost:3000/rosbag-replay-pro
```

1. 按 F12 打开开发者工具
2. 切换到 Console 标签
3. 刷新页面 (Ctrl+Shift+R 强制刷新)

查找以下日志：
```
[RosBagReplayPro] Component mounted
[RosBagStreamService] Connecting to ws://localhost:8765
[RosBagStreamService] Connected
[RosBagStreamService] Sending list_bags command
[RosBagStreamService] Received message: connection_state
[RosBagStreamService] Received message: bag_list
[RosBagStreamService] Setting availableBags, count: 10
[RosBagStreamService] list_bags handler called, bags: 10
[RosBagReplayPro] Bag list refreshed, count: 10
```

#### 4. 检查 Network 标签

在开发者工具中：
1. 切换到 Network 标签
2. 过滤 WS (WebSocket)
3. 应该看到一个到 `ws://localhost:8765` 的连接
4. 点击该连接，查看 Messages 标签
5. 应该看到发送和接收的消息

#### 5. 常见问题和解决方案

##### 问题 A: 控制台显示 "WebSocket connection error"

**解决方案**：
```bash
# 重启 RosBag 服务器
cd /home/lyx/fsm/python
./rosbag_server.sh restart
```

##### 问题 B: 控制台显示 "No bags found"

**解决方案**：
```bash
# 检查 bag 文件是否存在
ls -la /home/lyx/fsm/rosbag

# 查看服务器日志
tail -50 /tmp/guardian_mobility/rosbag_server.log
```

##### 问题 C: 页面显示 "Disconnected"

**解决方案**：
```bash
# 确保服务器运行
./start_all.sh start

# 检查端口
lsof -i :8765
```

##### 问题 D: "Select RosBag" 按钮不可见

**可能原因**：
1. CSS 样式问题
2. 按钮被其他元素遮挡
3. 浏览器缓存问题

**解决方案**：
```bash
# 强制刷新浏览器
Ctrl + Shift + R

# 清除浏览器缓存
Ctrl + Shift + Delete
```

##### 问题 E: availableBags 为空

**检查**：
在浏览器控制台输入：
```javascript
// 检查 Vue 组件状态
window.__VUE_DEVTOOLS_GLOBAL_HOOK__
```

或者在控制台查看日志中的 bag count。

#### 6. 手动测试 WebSocket

在浏览器控制台运行：
```javascript
const ws = new WebSocket('ws://localhost:8765');
ws.onopen = () => console.log('Connected');
ws.onmessage = (e) => console.log('Received:', JSON.parse(e.data));
ws.onopen = () => {
    console.log('Connected');
    setTimeout(() => {
        ws.send(JSON.stringify({type: 'list_bags'}));
        console.log('Sent list_bags');
    }, 1000);
};
```

应该看到：
```
Connected
Received: {type: "connection_state", state: "connected"}
Sent list_bags
Received: {type: "bag_list", bags: Array(10)}
```

#### 7. 完全重启系统

如果以上都不行：
```bash
cd /home/lyx/fsm

# 停止所有服务
./start_all.sh stop

# 等待几秒
sleep 5

# 重新启动
./start_all.sh start

# 等待服务启动
sleep 10

# 测试连接
./start_all.sh test
```

#### 8. 查看详细日志

```bash
# RosBag 服务器日志
tail -100 /tmp/guardian_mobility/rosbag_server.log

# 前端服务器日志
tail -100 /tmp/guardian_mobility/frontend.log

# 查看所有日志
./start_all.sh logs
```

### 预期的正常行为

1. 页面加载后自动连接到 WebSocket
2. 连接成功后显示 "Connected" (绿色)
3. 自动加载 bag 文件列表
4. "Select RosBag" 按钮可见且可点击
5. 点击按钮后显示 10 个可用的 bag 文件

### 需要提供的调试信息

如果问题仍然存在，请提供：

1. 浏览器控制台的完整日志
2. Network 标签中 WebSocket 的消息记录
3. RosBag 服务器日志 (`/tmp/guardian_mobility/rosbag_server.log`)
4. 截图显示问题

---

**创建日期**: 2026-02-04
**版本**: 1.0.0
