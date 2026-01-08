# 项目配置说明

## 环境要求

- Node.js >= 16.x
- npm >= 7.x 或 pnpm >= 8.x

## 依赖说明

### 核心依赖

- **vue@^3.4.21**: Vue 3框架
- **pinia@^2.1.7**: 状态管理库
- **leaflet@^1.9.4**: 开源地图库
- **three@^0.162.0**: WebGL 3D库
- **@ffmpeg/ffmpeg@^0.12.10**: WebAssembly视频处理

### 开发依赖

- **vite@^5.1.6**: 构建工具
- **typescript@~5.4.0**: TypeScript编译器
- **vue-tsc@^2.0.6**: Vue TypeScript检查
- **@vitejs/plugin-vue@^5.0.4**: Vite的Vue插件

## 配置文件说明

### vite.config.ts

```typescript
{
  resolve: {
    alias: {
      '@': path.resolve(__dirname, './src')  // @ 指向 src 目录
    }
  },
  server: {
    port: 3000,        // 开发服务器端口
    host: true         // 允许外部访问
  },
  optimizeDeps: {
    exclude: ['@ffmpeg/ffmpeg', '@ffmpeg/util']  // 排除FFmpeg预构建
  }
}
```

### tsconfig.json

- **target**: ES2020
- **module**: ESNext
- **strict**: true (严格模式)
- **baseUrl**: "." (启用路径别名)
- **paths**: `@/*` 映射到 `./src/*`

## 目录结构约定

```
src/
├── components/       # 可复用的Vue组件
├── composables/      # Vue组合式函数
├── stores/          # Pinia状态管理
├── types/           # TypeScript类型定义
├── assets/          # 静态资源（图片等）
├── App.vue          # 根组件
├── main.ts          # 应用入口
└── style.css        # 全局样式
```

## 开发规范

### 命名约定

- **组件**: PascalCase (例: `CameraSlot.vue`)
- **组合式函数**: use前缀 (例: `useVideoSync.ts`)
- **Store**: 名词 (例: `fleet.ts`, `camera.ts`)
- **类型文件**: index.ts

### 组件结构

```vue
<template>
  <!-- HTML -->
</template>

<script setup lang="ts">
// TypeScript 逻辑
</script>

<style scoped>
/* 组件样式 */
</style>
```

### 状态管理

使用Pinia进行状态管理：
- **fleet**: 车辆相关状态
- **camera**: 摄像头和视频状态
- **system**: UI、日志、录制状态

### 类型安全

所有组件都使用TypeScript的 `<script setup lang="ts">`，确保类型安全。

## 样式约定

### CSS变量

在 `:root` 中定义全局变量：

```css
--primary: #00f2ff;       /* 主题色 */
--primary-dim: rgba(...); /* 主题色半透明 */
--warn: #ffcc00;          /* 警告色 */
--danger: #ff0055;        /* 危险色 */
--success: #00ff88;       /* 成功色 */
--bg-dark: #020408;       /* 背景深色 */
--bg-panel: #0a1018;      /* 面板背景 */
--border: #1f3a52;        /* 边框色 */
```

### 布局变量

```css
--header-h: 40px;    /* 顶栏高度 */
--ai-h: 30px;        /* AI栏高度 */
--footer-h: 85px;    /* 底栏高度 */
--side-w: 280px;     /* 侧边栏宽度 */
```

### Scoped样式

所有组件样式使用 `<style scoped>` 避免污染全局样式。

## 性能优化建议

1. **视频优化**
   - 使用较低分辨率 (640x480推荐)
   - 使用H.264编码的MP4格式
   - 控制同时播放的视频数量

2. **Canvas优化**
   - 限制重绘频率 (100ms间隔)
   - 仅在可见时渲染

3. **Three.js优化**
   - 使用requestAnimationFrame
   - 合理控制几何体复杂度
   - 启用抗锯齿

4. **状态更新**
   - 遥测数据500ms更新一次
   - 避免不必要的响应式依赖

## 构建配置

### 开发模式
```bash
npm run dev
```
- 启动Vite开发服务器
- 支持热更新(HMR)
- Source Map调试

### 生产构建
```bash
npm run build
```
- TypeScript类型检查
- 代码压缩优化
- 输出到 `dist/` 目录

### 预览构建
```bash
npm run preview
```
- 本地预览生产构建结果

## 部署

### 静态托管

生产构建后，将 `dist/` 目录部署到：
- Nginx
- Apache
- Vercel
- Netlify
- GitHub Pages

### Nginx配置示例

```nginx
server {
  listen 80;
  server_name your-domain.com;
  root /path/to/dist;
  index index.html;

  location / {
    try_files $uri $uri/ /index.html;
  }
}
```

## 环境变量

如需配置环境变量，创建 `.env` 文件：

```bash
# .env.development
VITE_API_URL=http://localhost:8080

# .env.production
VITE_API_URL=https://api.yourdomain.com
```

在代码中使用：
```typescript
const apiUrl = import.meta.env.VITE_API_URL
```

## 扩展建议

### WebSocket集成

```typescript
// src/services/websocket.ts
export class WebSocketService {
  private ws: WebSocket | null = null

  connect(url: string) {
    this.ws = new WebSocket(url)
    // 处理消息...
  }
}
```

### API集成

```typescript
// src/services/api.ts
export const vehicleAPI = {
  getVehicles: () => fetch('/api/vehicles'),
  getTelemetry: (id: string) => fetch(`/api/vehicles/${id}/telemetry`)
}
```

## 故障排查

### 常见问题

1. **端口被占用**
   ```bash
   # 修改vite.config.ts中的port配置
   ```

2. **类型错误**
   ```bash
   # 运行类型检查
   npx vue-tsc --noEmit
   ```

3. **依赖问题**
   ```bash
   # 清理并重装
   rm -rf node_modules package-lock.json
   npm install
   ```

## 贡献指南

1. Fork项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启Pull Request

## 技术支持

如有问题，请查看：
- [Vue 3 文档](https://vuejs.org/)
- [Pinia 文档](https://pinia.vuejs.org/)
- [Vite 文档](https://vitejs.dev/)
- [TypeScript 文档](https://www.typescriptlang.org/)
