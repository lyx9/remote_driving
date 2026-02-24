<!--
  FSM-Pilot V2.0 - 3D Gaussian Splatting Viewer

  @description 3DGS 点云渲染器
-->
<template>
  <div class="gaussian-viewer">
    <canvas ref="canvasRef" class="viewer-canvas"></canvas>

    <!-- 控制面板 -->
    <div class="viewer-controls">
      <div class="control-group">
        <label>Color Mapping:</label>
        <select v-model="colorMapping" @change="regenerateScene">
          <option value="intensity">Intensity</option>
          <option value="height">Height</option>
          <option value="distance">Distance</option>
        </select>
      </div>

      <div class="control-group">
        <label>Voxel Size:</label>
        <input
          type="range"
          v-model.number="voxelSize"
          min="0.05"
          max="0.5"
          step="0.05"
          @change="regenerateScene"
        />
        <span>{{ voxelSize }}m</span>
      </div>

      <div class="control-group">
        <label>Point Size:</label>
        <input
          type="range"
          v-model.number="pointSize"
          min="1"
          max="10"
          step="0.5"
        />
        <span>{{ pointSize }}</span>
      </div>

      <div class="control-group">
        <button @click="exportScene" class="export-btn">
          Export PLY
        </button>
        <button @click="resetCamera" class="reset-btn">
          Reset Camera
        </button>
      </div>

      <div class="stats">
        <div>Points: {{ stats.totalPoints.toLocaleString() }}</div>
        <div>FPS: {{ stats.fps }}</div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted, watch } from 'vue'
import type { PointCloudPoint } from '@/services/rosbagDb3ParserOptimized'
import {
  convertPointCloudToGaussian,
  exportToPLY,
  type GaussianScene,
  type ConversionConfig
} from '@/services/rosbagTo3DGS'

const props = defineProps<{
  points: PointCloudPoint[]
}>()

const canvasRef = ref<HTMLCanvasElement | null>(null)
const colorMapping = ref<'intensity' | 'height' | 'distance'>('intensity')
const voxelSize = ref(0.1)
const pointSize = ref(2)

const stats = ref({
  totalPoints: 0,
  fps: 0
})

let scene: GaussianScene | null = null
let ctx: CanvasRenderingContext2D | null = null
let animationId: number | null = null

// 相机参数
const camera = ref({
  x: 0,
  y: 0,
  z: 10,
  rotX: 0,
  rotY: 0,
  zoom: 1
})

let isDragging = false
let lastMouseX = 0
let lastMouseY = 0

onMounted(() => {
  if (!canvasRef.value) return

  ctx = canvasRef.value.getContext('2d')
  if (!ctx) return

  // 设置 canvas 大小
  resizeCanvas()
  window.addEventListener('resize', resizeCanvas)

  // 鼠标控制
  canvasRef.value.addEventListener('mousedown', handleMouseDown)
  canvasRef.value.addEventListener('mousemove', handleMouseMove)
  canvasRef.value.addEventListener('mouseup', handleMouseUp)
  canvasRef.value.addEventListener('wheel', handleWheel)

  // 生成场景
  regenerateScene()

  // 启动渲染循环
  startRenderLoop()
})

onUnmounted(() => {
  window.removeEventListener('resize', resizeCanvas)
  if (animationId !== null) {
    cancelAnimationFrame(animationId)
  }
})

watch(() => props.points, () => {
  regenerateScene()
})

function resizeCanvas() {
  if (!canvasRef.value) return
  const parent = canvasRef.value.parentElement
  if (!parent) return

  canvasRef.value.width = parent.clientWidth
  canvasRef.value.height = parent.clientHeight
}

function regenerateScene() {
  if (props.points.length === 0) return

  const config: Partial<ConversionConfig> = {
    voxelSize: voxelSize.value,
    colorMapping: colorMapping.value,
    maxDistance: 100,
    minIntensity: 0,
    smoothing: true
  }

  scene = convertPointCloudToGaussian(props.points, config)
  stats.value.totalPoints = scene.splats.length

  console.log(`[3DGS] Generated scene with ${scene.splats.length} splats`)
}

function startRenderLoop() {
  let lastTime = performance.now()
  let frameCount = 0

  const render = () => {
    if (!ctx || !canvasRef.value || !scene) {
      animationId = requestAnimationFrame(render)
      return
    }

    // 清空画布
    ctx.fillStyle = '#0a0a12'
    ctx.fillRect(0, 0, canvasRef.value.width, canvasRef.value.height)

    // 渲染 Gaussian Splats（简化为点渲染）
    renderPoints()

    // 计算 FPS
    frameCount++
    const now = performance.now()
    if (now - lastTime > 1000) {
      stats.value.fps = frameCount
      frameCount = 0
      lastTime = now
    }

    animationId = requestAnimationFrame(render)
  }

  render()
}

function renderPoints() {
  if (!ctx || !canvasRef.value || !scene) return

  const width = canvasRef.value.width
  const height = canvasRef.value.height
  const centerX = width / 2
  const centerY = height / 2

  // 投影矩阵（简化的正交投影）
  const scale = 20 * camera.value.zoom

  // 按深度排序（远到近）
  const sortedSplats = [...scene.splats].sort((a, b) => {
    const az = a.position[2] - camera.value.z
    const bz = b.position[2] - camera.value.z
    return az - bz
  })

  for (const splat of sortedSplats) {
    // 应用相机旋转
    const rx = splat.position[0] - camera.value.x
    const ry = splat.position[1] - camera.value.y
    const rz = splat.position[2] - camera.value.z

    // 简单的旋转变换
    const cosY = Math.cos(camera.value.rotY)
    const sinY = Math.sin(camera.value.rotY)
    const cosX = Math.cos(camera.value.rotX)
    const sinX = Math.sin(camera.value.rotX)

    let x = rx * cosY - rz * sinY
    let z = rx * sinY + rz * cosY
    let y = ry * cosX - z * sinX
    z = ry * sinX + z * cosX

    // 投影到屏幕
    const screenX = centerX + x * scale
    const screenY = centerY - y * scale

    // 深度衰减
    const depth = z
    const depthFactor = Math.max(0, 1 - Math.abs(depth) / 50)

    if (depthFactor <= 0) continue

    // 绘制点
    ctx.globalAlpha = splat.opacity * depthFactor
    ctx.fillStyle = `rgb(${splat.color[0] * 255}, ${splat.color[1] * 255}, ${splat.color[2] * 255})`

    const size = pointSize.value * (1 + (1 - Math.abs(depth) / 50))
    ctx.beginPath()
    ctx.arc(screenX, screenY, size, 0, Math.PI * 2)
    ctx.fill()
  }

  ctx.globalAlpha = 1
}

function handleMouseDown(e: MouseEvent) {
  isDragging = true
  lastMouseX = e.clientX
  lastMouseY = e.clientY
}

function handleMouseMove(e: MouseEvent) {
  if (!isDragging) return

  const deltaX = e.clientX - lastMouseX
  const deltaY = e.clientY - lastMouseY

  camera.value.rotY += deltaX * 0.01
  camera.value.rotX += deltaY * 0.01

  // 限制 X 旋转
  camera.value.rotX = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, camera.value.rotX))

  lastMouseX = e.clientX
  lastMouseY = e.clientY
}

function handleMouseUp() {
  isDragging = false
}

function handleWheel(e: WheelEvent) {
  e.preventDefault()
  camera.value.zoom *= e.deltaY > 0 ? 0.9 : 1.1
  camera.value.zoom = Math.max(0.1, Math.min(10, camera.value.zoom))
}

function resetCamera() {
  camera.value = {
    x: 0,
    y: 0,
    z: 10,
    rotX: 0,
    rotY: 0,
    zoom: 1
  }
}

function exportScene() {
  if (!scene) return

  const ply = exportToPLY(scene)
  const blob = new Blob([ply], { type: 'text/plain' })
  const url = URL.createObjectURL(blob)

  const a = document.createElement('a')
  a.href = url
  a.download = `gaussian_scene_${Date.now()}.ply`
  a.click()

  URL.revokeObjectURL(url)
  console.log('[3DGS] Scene exported as PLY')
}
</script>

<style scoped>
.gaussian-viewer {
  position: relative;
  width: 100%;
  height: 100%;
  background: #0a0a12;
}

.viewer-canvas {
  width: 100%;
  height: 100%;
  display: block;
  cursor: grab;
}

.viewer-canvas:active {
  cursor: grabbing;
}

.viewer-controls {
  position: absolute;
  top: 16px;
  right: 16px;
  background: rgba(20, 20, 31, 0.9);
  border: 1px solid #2a2a3a;
  border-radius: 8px;
  padding: 16px;
  min-width: 250px;
  backdrop-filter: blur(10px);
}

.control-group {
  margin-bottom: 12px;
}

.control-group label {
  display: block;
  font-size: 11px;
  color: var(--primary, #ff5722);
  margin-bottom: 6px;
  text-transform: uppercase;
  font-weight: 600;
}

.control-group select,
.control-group input[type="range"] {
  width: 100%;
  padding: 6px;
  background: #1a1a2e;
  border: 1px solid #3a3a4a;
  color: #fff;
  border-radius: 4px;
  font-size: 12px;
}

.control-group input[type="range"] {
  margin-right: 8px;
  width: calc(100% - 60px);
}

.control-group span {
  font-size: 11px;
  color: #888;
  font-family: 'JetBrains Mono', monospace;
}

.export-btn,
.reset-btn {
  width: calc(50% - 4px);
  padding: 8px;
  background: var(--primary, #ff5722);
  border: none;
  color: #000;
  font-size: 11px;
  font-weight: 600;
  border-radius: 4px;
  cursor: pointer;
  transition: all 0.2s;
}

.reset-btn {
  background: #2a2a3a;
  color: #fff;
  margin-left: 8px;
}

.export-btn:hover {
  background: #00d4e0;
}

.reset-btn:hover {
  background: #3a3a4a;
}

.stats {
  margin-top: 16px;
  padding-top: 12px;
  border-top: 1px solid #2a2a3a;
  font-size: 11px;
  color: #888;
}

.stats div {
  display: flex;
  justify-content: space-between;
  margin-bottom: 4px;
  font-family: 'JetBrains Mono', monospace;
}
</style>
