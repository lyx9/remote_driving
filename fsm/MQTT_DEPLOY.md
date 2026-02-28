# FSM-Pilot MQTT 远程驾驶部署指南

## 架构总览

```
[Autoware 小车]                    [阿里云 IoT]                [本地/云端浏览器]
  ROS2 Topics                       MQTT Broker                  Vue3 前端
                                  (MQTT over WSS)
  /sensing/camera/camera{0-5} ──JPEG──▶ fsm/FSM-01/cam/{0-5} ──▶ MQTTRemoteControl
  /localization/kinematic_state ──▶ fsm/FSM-01/telemetry      ──▶ 遥测面板
  /vehicle/status/*             ──▶ fsm/FSM-01/status         ──▶ 状态栏
                                ◀── fsm/FSM-01/control        ◀── 键盘/手柄
```

---

## 第一步: 阿里云 IoT 平台配置

### 1.1 创建产品

1. 登录 https://iot.console.aliyun.com
2. 设备管理 → 产品 → 创建产品
   - 产品名称: `FSM-Pilot`
   - 所属品类: 自定义品类
   - 节点类型: 直连设备
   - 联网方式: Wi-Fi / 蜂窝 (4G/5G)
   - 数据格式: 透传/自定义
3. 记录 **ProductKey**

### 1.2 创建设备

创建两个设备:

| DeviceName       | 用途           |
|------------------|----------------|
| `FSM-01`         | 车端设备       |
| `web-operator-001` | 前端操作端   |

每个设备创建后记录 **DeviceSecret**

### 1.3 配置规则引擎 (Topic 转发)

阿里云 IoT 默认只允许同产品设备间通信。需要配置规则引擎让 `FSM-01` 和 `web-operator-001` 互通:

1. 规则引擎 → 云产品流转 → 创建规则
2. SQL: `SELECT * FROM "/YOUR_PRODUCT_KEY/FSM-01/user/#"`
3. 转发到: 同产品下的 `web-operator-001`

**或者** 使用自定义 Topic，在产品 Topic 列表中添加:
- `fsm/${deviceName}/cam/${camIdx}` (发布)
- `fsm/${deviceName}/telemetry` (发布)
- `fsm/${deviceName}/control` (订阅)

---

## 第二步: 生成 MQTT 认证信息

### 车端认证 (在小车上运行)

```bash
cd /home/lyx/fsm/vehicle_ros2
python3 gen_auth.py
# 输入:
#   ProductKey: <你的ProductKey>
#   DeviceName: FSM-01
#   DeviceSecret: <FSM-01的DeviceSecret>
```

将输出填入 `vehicle_ros2/config.yaml`:

```yaml
aliyun:
  host: "YOUR_PRODUCT_KEY.iot-as-mqtt.cn-shanghai.aliyuncs.com"
  port: 8883
  client_id: "FSM-01|securemode=3,signmethod=hmacsha256,timestamp=...|"
  username: "FSM-01&YOUR_PRODUCT_KEY"
  password: "生成的密码"
```

### 前端认证 (操作端)

```bash
python3 gen_auth.py
# 输入:
#   ProductKey: <你的ProductKey>
#   DeviceName: web-operator-001
#   DeviceSecret: <web-operator-001的DeviceSecret>
```

将输出填入 `/home/lyx/fsm/.env.local`:

```env
VITE_MQTT_BROKER_URL=wss://YOUR_PRODUCT_KEY.iot-as-mqtt.cn-shanghai.aliyuncs.com/mqtt
VITE_MQTT_CLIENT_ID=web-operator-001|securemode=3,...|
VITE_MQTT_USERNAME=web-operator-001&YOUR_PRODUCT_KEY
VITE_MQTT_PASSWORD=生成的密码
VITE_VEHICLE_ID=FSM-01
```

---

## 第三步: 车端部署

### 3.1 安装依赖 (在 Autoware 小车上)

```bash
# Python 依赖
pip3 install paho-mqtt pyyaml opencv-python numpy

# ROS2 依赖 (已随 Autoware 安装)
# ros-humble-cv-bridge
# ros-humble-sensor-msgs
```

### 3.2 上传文件到小车

```bash
scp -r /home/lyx/fsm/vehicle_ros2/ user@VEHICLE_IP:~/fsm_bridge/
```

### 3.3 修改摄像头 Topic (按实际 Autoware 配置)

编辑 `mqtt_bridge.py` 中的 `CAMERA_TOPICS`:

```python
CAMERA_TOPICS = [
    "/sensing/camera/camera0/image_raw",   # 前视主摄
    "/sensing/camera/camera1/image_raw",   # 前视广角
    "/sensing/camera/camera2/image_raw",   # 后视
    "/sensing/camera/camera3/image_raw",   # 左后视
    "/sensing/camera/camera4/image_raw",   # 右后视
    "/sensing/camera/camera5/image_raw",   # 360环视
]
```

查看实际 Topic:
```bash
ros2 topic list | grep camera
```

### 3.4 启动 Bridge 节点

```bash
cd ~/fsm_bridge
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash  # Autoware 工作空间

python3 mqtt_bridge.py --config config.yaml
```

### 3.5 设置开机自启 (systemd)

```bash
sudo tee /etc/systemd/system/fsm-mqtt-bridge.service << 'EOF'
[Unit]
Description=FSM-Pilot MQTT Bridge
After=network.target

[Service]
Type=simple
User=autoware
WorkingDirectory=/home/autoware/fsm_bridge
Environment="PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages"
ExecStart=/usr/bin/python3 mqtt_bridge.py --config config.yaml
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable fsm-mqtt-bridge
sudo systemctl start fsm-mqtt-bridge
```

---

## 第四步: 前端启动

```bash
cd /home/lyx/fsm
npm run dev
```

访问: http://localhost:3000/mqtt-remote-control

---

## 第五步: 带宽估算

| 摄像头数 | 分辨率  | FPS | JPEG质量 | 单路带宽 | 总带宽  |
|---------|---------|-----|---------|---------|--------|
| 6路     | 640×360 | 10  | 60%     | ~200KB/s | ~1.2MB/s ≈ 10Mbps |
| 6路     | 640×360 | 5   | 50%     | ~100KB/s | ~600KB/s ≈ 5Mbps  |

**推荐网络**: 4G/5G，上行带宽 ≥ 10Mbps

调整 `config.yaml` 中的参数降低带宽:
```yaml
camera:
  jpeg_quality: 50   # 降低质量
  max_width: 480     # 降低分辨率
  fps: 5             # 降低帧率
```

---

## 控制延迟优化

| 优化项 | 方法 |
|--------|------|
| 控制指令 | QoS 0 (无确认，最低延迟) |
| 视频帧 | QoS 0 + 丢帧策略 |
| 遥测 | QoS 0, 10Hz |
| 状态心跳 | QoS 1, 1Hz |

目标延迟: 控制指令 < 50ms (局域网), < 150ms (4G)

---

## 常见问题

**Q: MQTT 连接失败**
- 检查 ProductKey/DeviceName/DeviceSecret 是否正确
- 检查 timestamp 是否过期 (重新运行 gen_auth.py)
- 阿里云控制台查看设备在线状态

**Q: 摄像头无画面**
- `ros2 topic echo /sensing/camera/camera0/image_raw` 确认有数据
- 检查 cv_bridge 是否安装: `python3 -c "from cv_bridge import CvBridge"`

**Q: 控制指令无响应**
- 确认 Autoware 处于 Remote 控制模式
- 检查 `/control/command/control_cmd` topic 是否有数据
- 查看 bridge 日志: `journalctl -u fsm-mqtt-bridge -f`
