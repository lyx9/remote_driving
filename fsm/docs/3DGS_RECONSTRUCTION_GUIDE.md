# Guardian Mobility - 3D Gaussian Splatting重建系统

## 项目概述

企业级ROS Bag到3D Gaussian Splatting重建系统，用于从自动驾驶车辆采集的ROS bag数据重建高质量3D场景。

### 主要特性

- ✅ **ROS Bag数据提取** - 自动从bag文件提取图像和相机参数
- ✅ **COLMAP重建** - 使用COLMAP进行Structure from Motion
- ✅ **3DGS训练** - 基于3D Gaussian Splatting的场景重建
- ✅ **实时进度监控** - WebSocket实时推送重建进度
- ✅ **Web可视化** - 浏览器内3D场景查看器
- ✅ **企业级架构** - 前后端分离，可扩展设计

### 技术栈

**后端**:
- Python 3.8+
- Flask + Flask-SocketIO
- ROS (rosbag, cv_bridge)
- COLMAP
- 3D Gaussian Splatting

**前端**:
- Vue 3 + TypeScript
- WebSocket
- Three.js (3D可视化)

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                     Guardian Mobility                        │
│                   Database Visualization                     │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│              ReconstructionPanel.vue (前端)                  │
│  ┌──────────┬──────────┬──────────┬──────────┐             │
│  │ 文件上传 │ 参数配置 │ 进度监控 │ 结果查看 │             │
│  └──────────┴──────────┴──────────┴──────────┘             │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼ REST API + WebSocket
┌─────────────────────────────────────────────────────────────┐
│         reconstruction_server.py (Flask后端)                 │
│  ┌──────────────────────────────────────────────┐           │
│  │  任务管理 │ 进度推送 │ 结果查询 │ 文件管理  │           │
│  └──────────────────────────────────────────────┘           │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│          rosbag_to_3dgs.py (重建流水线)                      │
│  ┌──────────────┬──────────────┬──────────────┐            │
│  │ ROS Bag提取  │ COLMAP重建   │ 3DGS训练     │            │
│  └──────────────┴──────────────┴──────────────┘            │
└─────────────────────────────────────────────────────────────┘
```

---

## 安装指南

### 1. 系统要求

- **操作系统**: Ubuntu 20.04+ / Ubuntu 22.04 (推荐)
- **GPU**: NVIDIA GPU (推荐RTX 3060+)
- **内存**: 16GB+ RAM
- **存储**: 100GB+ 可用空间
- **Python**: 3.8+
- **Node.js**: 16+

### 2. 安装ROS

```bash
# Ubuntu 20.04 - ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
sudo apt install python3-rosbag python3-cv-bridge

# 配置环境
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. 安装COLMAP

```bash
# 方法1: 从源码编译 (推荐)
sudo apt install \
    git \
    cmake \
    build-essential \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-system-dev \
    libboost-test-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libfreeimage-dev \
    libmetis-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev \
    libcgal-qt5-dev

git clone https://github.com/colmap/colmap.git
cd colmap
mkdir build
cd build
cmake .. -DCMAKE_CUDA_ARCHITECTURES=native
make -j
sudo make install

# 方法2: 使用预编译包
sudo apt install colmap
```

### 4. 安装3D Gaussian Splatting

```bash
# 克隆官方仓库
cd ~
git clone https://github.com/graphdeco-inria/gaussian-splatting --recursive
cd gaussian-splatting

# 创建conda环境
conda create -n gaussian_splatting python=3.8
conda activate gaussian_splatting

# 安装依赖
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install plyfile tqdm

# 安装子模块
pip install submodules/diff-gaussian-rasterization
pip install submodules/simple-knn

# 测试安装
python train.py -h
```

### 5. 安装Python依赖

```bash
cd /home/lyx/fsm/python

# 创建虚拟环境
python3 -m venv venv
source venv/bin/activate

# 安装依赖
pip install flask flask-cors flask-socketio
pip install opencv-python
pip install numpy pandas
pip install python-socketio
```

### 6. 配置环境变量

```bash
# 编辑 ~/.bashrc
nano ~/.bashrc

# 添加以下内容
export GAUSSIAN_SPLATTING_PATH=~/gaussian-splatting
export PYTHONPATH=$PYTHONPATH:$GAUSSIAN_SPLATTING_PATH

# 重新加载
source ~/.bashrc
```

---

## 使用指南

### 1. 启动后端服务器

```bash
cd /home/lyx/fsm/python
source venv/bin/activate
python reconstruction_server.py
```

服务器将在 `http://localhost:5000` 启动

### 2. 启动前端

```bash
cd /home/lyx/fsm
npm run dev
```

访问 `http://localhost:3000/database`

### 3. 使用流程

#### 步骤1: 上传ROS Bag

1. 点击"3D Gaussian Splatting Reconstruction"面板
2. 拖拽或选择 `.bag` 文件
3. 点击"下一步"

#### 步骤2: 配置参数

**ROS配置**:
- 图像话题: `/camera/image_raw`
- 相机信息话题: `/camera/camera_info`
- 最大图像数量: 留空使用全部

**COLMAP配置**:
- 重建质量: 高 (推荐)
- 匹配器类型: 穷举匹配 (精确)

**3DGS配置**:
- 训练迭代次数: 30000 (推荐)
- 分辨率: 全分辨率
- 球谐函数阶数: 3

**性能配置**:
- 使用GPU加速: ✅
- GPU ID: 0
- 图像下采样因子: 1.0

#### 步骤3: 监控进度

实时查看:
- 当前阶段
- 进度百分比
- 已用时间
- 预计剩余时间
- 详细日志

#### 步骤4: 查看结果

- 🎨 打开3D查看器
- 📥 下载结果
- 📁 打开文件夹
- 查看统计信息

---

## 命令行使用

### 基本用法

```bash
python rosbag_to_3dgs.py \
  --rosbag /path/to/your.bag \
  --output /path/to/output \
  --image-topic /camera/image_raw \
  --iterations 30000
```

### 高级选项

```bash
python rosbag_to_3dgs.py \
  --rosbag /data/rosbags/scene01.bag \
  --output /data/outputs/scene01 \
  --image-topic /camera/color/image_raw \
  --camera-info-topic /camera/color/camera_info \
  --max-images 500 \
  --iterations 30000 \
  --gpu 0
```

### 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--rosbag` | ROS bag文件路径 | 必需 |
| `--output` | 输出目录 | 必需 |
| `--image-topic` | 图像话题名称 | `/camera/image_raw` |
| `--camera-info-topic` | 相机信息话题 | `/camera/camera_info` |
| `--max-images` | 最大图像数量 | 无限制 |
| `--iterations` | 训练迭代次数 | 30000 |
| `--gpu` | GPU ID | 0 |

---

## API文档

### REST API

#### 1. 开始重建

```http
POST /api/reconstruction/start
Content-Type: application/json

{
  "rosbagPath": "/path/to/file.bag",
  "imageTopic": "/camera/image_raw",
  "iterations": 30000,
  "useGpu": true
}

Response:
{
  "success": true,
  "jobId": "uuid-string",
  "message": "重建任务已启动"
}
```

#### 2. 获取状态

```http
GET /api/reconstruction/status/{jobId}

Response:
{
  "jobId": "uuid-string",
  "status": "running",
  "progress": 45,
  "phase": "COLMAP特征提取..."
}
```

#### 3. 获取结果

```http
GET /api/reconstruction/result/{jobId}

Response:
{
  "success": true,
  "message": "重建成功",
  "outputPath": "/path/to/output",
  "viewerUrl": "http://localhost:8080/viewer",
  "stats": {
    "image_count": 150,
    "duration_seconds": 1200
  }
}
```

#### 4. 取消重建

```http
POST /api/reconstruction/cancel/{jobId}

Response:
{
  "success": true,
  "message": "任务已取消"
}
```

### WebSocket API

```javascript
// 连接WebSocket
const socket = io('http://localhost:5000')

// 加入任务房间
socket.emit('join', { jobId: 'uuid-string' })

// 监听进度
socket.on('progress', (data) => {
  console.log('进度:', data.percentage)
})

// 监听日志
socket.on('log', (data) => {
  console.log('日志:', data.message)
})

// 监听完成
socket.on('completed', (data) => {
  console.log('完成:', data.result)
})
```

---

## 输出结构

```
output_dir/
├── images/                    # 提取的图像
│   ├── 000000.jpg
│   ├── 000001.jpg
│   └── ...
├── colmap/                    # COLMAP输出
│   ├── database.db           # 特征数据库
│   ├── sparse/               # 稀疏重建
│   │   └── 0/
│   │       ├── cameras.bin
│   │       ├── images.bin
│   │       └── points3D.bin
│   └── dense/                # 密集重建
│       ├── images/
│       ├── sparse/
│       └── stereo/
└── gaussian_splatting/        # 3DGS输出
    ├── point_cloud/
    │   └── iteration_30000/
    │       └── point_cloud.ply
    ├── cameras.json
    └── cfg_args
```

---

## 性能优化

### 1. GPU加速

确保CUDA正确安装:
```bash
nvidia-smi
nvcc --version
```

### 2. 图像数量

- 推荐: 100-500张图像
- 最少: 50张
- 最多: 1000张 (更多会显著增加时间)

### 3. 分辨率

- 全分辨率: 最高质量，最慢
- 1/2分辨率: 平衡质量和速度
- 1/4分辨率: 快速预览

### 4. 迭代次数

- 快速预览: 7000次
- 标准质量: 30000次 (推荐)
- 高质量: 50000次

---

## 故障排查

### 问题1: ROS bag读取失败

**症状**: `rosbag info` 命令失败

**解决方案**:
```bash
# 检查ROS安装
rosversion -d

# 重新source环境
source /opt/ros/noetic/setup.bash

# 检查bag文件
rosbag info your_file.bag
```

### 问题2: COLMAP失败

**症状**: 特征提取或匹配失败

**解决方案**:
```bash
# 检查COLMAP安装
colmap -h

# 降低质量设置
--colmap-quality medium

# 使用顺序匹配
--colmap-matcher sequential
```

### 问题3: GPU内存不足

**症状**: CUDA out of memory

**解决方案**:
```bash
# 降低分辨率
--resolution 2  # 使用1/2分辨率

# 减少图像数量
--max-images 200

# 降低batch size (修改3DGS配置)
```

### 问题4: 训练时间过长

**症状**: 训练超过2小时

**解决方案**:
```bash
# 减少迭代次数
--iterations 15000

# 使用更少图像
--max-images 100

# 降低分辨率
--resolution 2
```

---

## 参考资源

### 开源项目

1. **3D Gaussian Splatting**
   - GitHub: https://github.com/graphdeco-inria/gaussian-splatting
   - 论文: https://repo-sam.inria.fr/fungraph/3d-gaussian-splatting/

2. **COLMAP**
   - GitHub: https://github.com/colmap/colmap
   - 文档: https://colmap.github.io/

3. **Nerfstudio**
   - GitHub: https://github.com/nerfstudio-project/nerfstudio
   - 文档: https://docs.nerf.studio/

### 相关论文

- 3D Gaussian Splatting for Real-Time Radiance Field Rendering (SIGGRAPH 2023)
- Structure-from-Motion Revisited (CVPR 2016)
- NeRF: Representing Scenes as Neural Radiance Fields (ECCV 2020)

---

## 许可证

本项目为Guardian Mobility的一部分，版权归香港城市大学所有。

---

## 联系方式

- **作者**: Li Yixiang
- **机构**: City University of Hong Kong
- **项目**: Guardian Mobility v0.0

---

**最后更新**: 2026-01-21
