# FSM-Pilot 快速启动指南

## 🚀 第一次运行

### 1. 安装依赖

```bash
cd /home/lyx/fsm
npm install
```

### 2. 启动开发服务器

```bash
npm run dev
```

启动后访问: http://localhost:3000

### 3. 测试功能

1. **查看界面**: 应该能看到完整的驾驶舱界面
2. **切换车辆**: 点击底部的车队卡片（FSM-01, FSM-02, FSM-03）
3. **加载视频**:
   - 鼠标悬停在任意摄像头区域
   - 点击"LOAD"按钮上传视频文件
   - 支持mp4, webm等格式
4. **控制面板**:
   - 点击顶部按钮切换各个面板
   - MAP: 地图和日志
   - AI-BAR: AI信息栏
   - LIDAR: 激光雷达
   - PIP: 悬浮摄像头
   - CTRL: 控制面板

## 📁 项目文件说明

```
核心文件:
├── src/App.vue              主应用入口
├── src/main.ts              启动文件
├── src/style.css            全局样式
│
├── src/components/          所有Vue组件
│   ├── Header.vue           顶栏（品牌+控制按钮）
│   ├── AIBar.vue            AI信息栏
│   ├── FleetFooter.vue      车队选择底栏
│   ├── LeftSidebar.vue      左侧（地图+日志）
│   ├── RightSidebar.vue     右侧（档位+录制+遥测）
│   ├── VideoWall.vue        视频墙容器
│   ├── CameraSlot.vue       单个摄像头
│   └── LidarPanel.vue       激光雷达3D
│
├── src/stores/              状态管理
│   ├── fleet.ts             车队数据
│   ├── camera.ts            摄像头控制
│   └── system.ts            系统状态
│
└── src/types/index.ts       TypeScript类型
```

## 🎯 主要功能

### 视频功能
- ✅ 5路摄像头（1主+4 PIP）
- ✅ 同步播放控制
- ✅ 视频文件上传
- ✅ 自动格式转码（FFmpeg）

### 地图功能
- ✅ 车辆位置实时显示
- ✅ 行驶轨迹可视化
- ✅ 暗色主题地图

### 3D可视化
- ✅ 激光雷达点云
- ✅ 路径规划显示
- ✅ 动态网格

### 数据记录
- ✅ 黑盒录制
- ✅ 系统日志
- ✅ 遥测数据

## 🛠️ 开发技巧

### 修改车辆数据
编辑 `src/stores/fleet.ts` 中的 `vehicles` 数组

### 修改主题颜色
编辑 `src/style.css` 中的CSS变量:
```css
--primary: #00f2ff;   /* 青色主题 */
--warn: #ffcc00;      /* 警告黄 */
--danger: #ff0055;    /* 危险红 */
```

### 添加新摄像头
1. 在 `src/stores/camera.ts` 添加摄像头定义
2. 在 `src/components/VideoWall.vue` 添加CameraSlot组件

### 调试技巧
- 打开浏览器开发者工具 (F12)
- Vue Devtools可查看组件状态
- Console查看日志输出

## ⚡ 性能优化

- 视频建议使用640x480或更低分辨率
- 浏览器建议Chrome/Edge最新版
- 关闭不需要的面板提升性能

## 🐛 常见问题

**Q: 视频无法播放?**
A: 检查视频格式，浏览器支持mp4 (H.264)最佳

**Q: 地图不显示?**
A: 检查网络连接，需要加载在线地图瓦片

**Q: 界面卡顿?**
A: 降低视频分辨率，或关闭激光雷达面板

## 📦 打包部署

```bash
# 构建生产版本
npm run build

# 生成的文件在 dist/ 目录
# 可以部署到任何静态服务器
```

## 🎨 UI设计原则

- 暗色科技风格
- 青色 (#00f2ff) 作为主色调
- 最小化UI，聚焦数据
- 流畅的动画过渡
- 专业的仪表盘布局

Happy Coding! 🚗💨
