# FSM-Pilot 远程驾驶系统 - 编译与安装指南

## 目录

1. [系统要求](#1-系统要求)
2. [依赖安装](#2-依赖安装)
3. [编译C++后端](#3-编译c后端)
4. [编译Vue前端](#4-编译vue前端)
5. [配置说明](#5-配置说明)
6. [常见问题](#6-常见问题)

---

## 1. 系统要求

### 硬件要求

| 组件 | 最低配置 | 推荐配置 |
|------|----------|----------|
| CPU | 4核 | 8核+ |
| 内存 | 8GB | 16GB+ |
| GPU | - | NVIDIA (支持NVENC) |
| 网络 | 100Mbps | 1Gbps |

### 软件要求

| 软件 | 版本 |
|------|------|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble |
| Node.js | 18.x+ |
| CMake | 3.16+ |
| GCC/G++ | 11+ |

---

## 2. 依赖安装

### 2.1 基础依赖

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装基础开发工具
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    pkg-config
```

### 2.2 ROS2 Humble 安装

```bash
# 设置 locale
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加 ROS2 源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2 Humble
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-ros-base ros-dev-tools

# 设置环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.3 C++ 依赖库

```bash
# 安装 C++ 依赖
sudo apt install -y \
    libboost-all-dev \
    libssl-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libyaml-cpp-dev \
    libspdlog-dev \
    nlohmann-json3-dev \
    libopencv-dev

# ROS2 相关包
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-diagnostic-msgs
```

### 2.4 WebRTC 库 (libdatachannel)

```bash
# 安装 libdatachannel
git clone https://github.com/paullouisageneau/libdatachannel.git
cd libdatachannel
git submodule update --init --recursive

mkdir build && cd build
cmake .. -DUSE_GNUTLS=0 -DUSE_NICE=0 -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig

cd ../..
```

### 2.5 FFmpeg (可选，用于硬件编码)

```bash
# 安装 FFmpeg 开发库
sudo apt install -y \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libswscale-dev \
    libx264-dev

# NVIDIA 硬件编码支持 (如有 NVIDIA GPU)
# sudo apt install -y libnvidia-encode-520
```

### 2.6 Node.js 安装

```bash
# 使用 NodeSource 安装 Node.js 18
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs

# 验证安装
node --version
npm --version
```

### 2.7 Autoware 消息包 (可选)

```bash
# 如果需要与 Autoware 集成
sudo apt install -y \
    ros-humble-autoware-auto-vehicle-msgs \
    ros-humble-autoware-auto-control-msgs \
    ros-humble-tier4-vehicle-msgs
```

---

## 3. 编译C++后端

### 3.1 克隆项目

```bash
cd ~
git clone <repository_url> fsm
cd fsm
```

### 3.2 编译

```bash
# 进入 C++ 目录
cd cpp

# 创建构建目录
mkdir -p build && cd build

# 配置 CMake
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=ON

# 编译
make -j$(nproc)

# 或使用 colcon (ROS2 方式)
cd ~/fsm/cpp
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 3.3 安装

```bash
# 系统安装 (可选)
sudo make install

# 或使用 ROS2 方式
source install/setup.bash
```

### 3.4 验证编译

```bash
# 检查可执行文件
ls build/vehicle_node/fsm_vehicle_node
ls build/cloud_server/fsm_cloud_server
ls build/operator_client/fsm_operator_client

# 检查库文件
ls build/libfsm_common.so
```

---

## 4. 编译Vue前端

### 4.1 安装依赖

```bash
cd ~/fsm

# 安装 npm 依赖
npm install
```

### 4.2 开发模式运行

```bash
# 启动开发服务器
npm run dev

# 浏览器访问 http://localhost:5173
```

### 4.3 生产构建

```bash
# 构建生产版本
npm run build

# 预览生产版本
npm run preview

# 构建产物在 dist/ 目录
```

### 4.4 部署

```bash
# 使用 Nginx 部署
sudo cp -r dist/* /var/www/html/fsm-pilot/

# Nginx 配置示例
# server {
#     listen 80;
#     server_name your-domain.com;
#     root /var/www/html/fsm-pilot;
#     index index.html;
#     location / {
#         try_files $uri $uri/ /index.html;
#     }
# }
```

---

## 5. 配置说明

### 5.1 车端配置 (vehicle_config.yaml)

```yaml
# cpp/vehicle_node/config/vehicle_config.yaml
vehicle:
  id: "FSM-01"              # 车辆唯一标识
  type: "ROBO-TAXI"         # 车辆类型

sensors:
  cameras:
    enabled: true           # 是否启用摄像头
    list:
      - id: "cam_front_center"
        topic: "/sensing/camera/front_center/image_raw"
        fps: 30
        width: 1920
        height: 1080

webrtc:
  stun_servers:
    - "stun:stun.l.google.com:19302"
  signaling:
    url: "wss://your-server.com:8080/signaling"
```

### 5.2 云端配置 (cloud_config.yaml)

```yaml
# cpp/cloud_server/config/cloud_config.yaml
server:
  signaling_port: 8080      # 信令端口
  api_port: 8081            # REST API 端口

scheduling:
  enabled: true             # 启用调度
  algorithm: "weighted_priority"
  weights:
    emergency: 0.35
    latency: 0.25
    distance: 0.20
    battery: 0.10
    task_priority: 0.10
```

### 5.3 前端环境变量

创建 `.env.local` 文件:

```env
VITE_API_URL=http://localhost:8081
VITE_WS_URL=ws://localhost:8080
VITE_SIGNALING_URL=wss://localhost:8080/signaling
```

---

## 6. 常见问题

### Q1: CMake 找不到 ROS2 包

```bash
# 确保已 source ROS2 环境
source /opt/ros/humble/setup.bash

# 或添加到 ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Q2: Anaconda/Conda 与系统库冲突 (Protobuf/gRPC 错误)

**错误现象:**
```
CMake Error at /opt/anaconda3/lib/cmake/protobuf/protobuf-targets.cmake:42 (message):
  Some (but not all) targets in this export set were already defined.
```

**原因:** Anaconda 安装的 Protobuf/gRPC 与系统版本冲突。

**解决方案 1: 使用编译脚本 (推荐)**

```bash
# 使用项目提供的编译脚本，会自动处理 Anaconda 冲突
./scripts/build_cpp.sh
```

**解决方案 2: 手动处理**

```bash
# 方法 A: 临时停用 Conda 环境
conda deactivate

# 清理旧的构建缓存
cd cpp/build
rm -rf CMakeCache.txt CMakeFiles/

# 排除 Anaconda 路径后重新编译
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -E "anaconda|conda" | tr '\n' ':')
source /opt/ros/humble/setup.bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DUSE_GRPC=OFF
make -j$(nproc)
```

```bash
# 方法 B: 明确指定系统 Protobuf 路径
cd cpp/build
rm -rf CMakeCache.txt CMakeFiles/

cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DProtobuf_DIR=/usr/lib/x86_64-linux-gnu/cmake/protobuf \
    -DCMAKE_PREFIX_PATH="/opt/ros/humble;/usr;/usr/local" \
    -DUSE_GRPC=OFF

make -j$(nproc)
```

**解决方案 3: 永久配置**

在 `~/.bashrc` 中添加:
```bash
# 编译 FSM-Pilot 时排除 Anaconda
alias fsm-build='(conda deactivate 2>/dev/null; cd ~/fsm && ./scripts/build_cpp.sh)'
```

### Q3: libdatachannel 链接错误

```bash
# 更新动态链接库缓存
sudo ldconfig

# 检查库是否安装
ldconfig -p | grep datachannel
```

### Q4: npm install 失败

```bash
# 清除缓存重试
rm -rf node_modules package-lock.json
npm cache clean --force
npm install
```

### Q5: 视频编码不工作

```bash
# 检查 FFmpeg 支持的编码器
ffmpeg -encoders | grep h264

# 如果使用 NVIDIA GPU，检查 NVENC
nvidia-smi
```

### Q6: WebSocket 连接失败

- 检查防火墙设置
- 确保信令服务器已启动
- 检查 SSL 证书 (如使用 wss://)

```bash
# 开放端口
sudo ufw allow 8080
sudo ufw allow 8081
```

---

## 快速验证脚本

```bash
#!/bin/bash
# verify_installation.sh

echo "=== FSM-Pilot 安装验证 ==="

# 检查 ROS2
echo -n "ROS2 Humble: "
if ros2 --version 2>/dev/null | grep -q "humble"; then
    echo "✓"
else
    echo "✗"
fi

# 检查 Node.js
echo -n "Node.js: "
if node --version 2>/dev/null | grep -q "v18"; then
    echo "✓"
else
    echo "✗"
fi

# 检查编译产物
echo -n "C++ Backend: "
if [ -f "cpp/build/cloud_server/fsm_cloud_server" ]; then
    echo "✓"
else
    echo "✗"
fi

echo -n "Vue Frontend: "
if [ -d "dist" ]; then
    echo "✓"
else
    echo "✗"
fi

echo "=== 验证完成 ==="
```

---

## 下一步

- 查看 [使用指南](./USAGE_GUIDE.md) 了解如何运行系统
- 查看 [Demo 演示](./DEMO_GUIDE.md) 了解演示场景
- 查看 [API 文档](./API_REFERENCE.md) 了解接口详情
