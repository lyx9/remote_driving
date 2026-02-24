<!--
  FSM-Pilot V2.0 - Remote Driving System

  @project     FSM-Pilot Remote Driving Platform
  @author      Li Yixiang
  @institution City University of Hong Kong
  @copyright   2025 City University of Hong Kong. All rights reserved.
  @license     Proprietary

  @component   RosBagVisualization
  @description RosBag数据可视化组件，显示车辆轨迹、感知目标和传感器数据
-->
<template>
  <div class="rosbag-viz">
    <!-- 顶部工具栏 -->
    <div class="viz-toolbar">
      <div class="view-mode">
        <button
          v-for="mode in viewModes"
          :key="mode.id"
          :class="['mode-btn', { active: currentViewMode === mode.id }]"
          @click="currentViewMode = mode.id"
        >
          {{ mode.name }}
        </button>
      </div>
      <div class="viz-options">
        <label class="option">
          <input type="checkbox" v-model="showGrid" />
          <span>Grid</span>
        </label>
        <label class="option">
          <input type="checkbox" v-model="showTrajectory" />
          <span>Trajectory</span>
        </label>
        <label class="option">
          <input type="checkbox" v-model="showObjects" />
          <span>Objects</span>
        </label>
        <label class="option">
          <input type="checkbox" v-model="showVehicle" />
          <span>Ego Vehicle</span>
        </label>
      </div>
    </div>

    <!-- 可视化画布 -->
    <div class="viz-canvas-container" ref="containerRef">
      <canvas
        ref="canvasRef"
        class="viz-canvas"
        @mousedown="handleMouseDown"
        @mouseup="handleMouseUp"
        @mousemove="handleMouseMove"
        @wheel="handleWheel"
      ></canvas>

      <!-- 比例尺 -->
      <div class="scale-indicator">
        <div class="scale-bar"></div>
        <span>{{ scaleText }}</span>
      </div>

      <!-- 坐标信息 -->
      <div class="cursor-info" v-if="cursorWorldPos">
        X: {{ cursorWorldPos.x.toFixed(1) }}m, Y: {{ cursorWorldPos.y.toFixed(1) }}m
      </div>

      <!-- 图例 -->
      <div class="legend">
        <div class="legend-item">
          <span class="legend-color ego"></span>
          <span>Ego Vehicle</span>
        </div>
        <div class="legend-item">
          <span class="legend-color car"></span>
          <span>Vehicle</span>
        </div>
        <div class="legend-item">
          <span class="legend-color pedestrian"></span>
          <span>Pedestrian</span>
        </div>
        <div class="legend-item">
          <span class="legend-color cyclist"></span>
          <span>Cyclist</span>
        </div>
        <div class="legend-item">
          <span class="legend-color trajectory"></span>
          <span>Trajectory</span>
        </div>
      </div>
    </div>

    <!-- 底部状态栏 -->
    <div class="viz-statusbar">
      <span class="status-item">
        View: {{ currentViewMode.toUpperCase() }}
      </span>
      <span class="status-item">
        Zoom: {{ (zoom * 100).toFixed(0) }}%
      </span>
      <span class="status-item">
        Objects: {{ objectCount }}
      </span>
      <span class="status-item">
        FPS: {{ fps }}
      </span>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, reactive, computed, onMounted, onUnmounted, watch } from 'vue'
import {
  quaternionToEuler,
  type AutowareKinematicState,
  type AutowareDetectedObjects
} from '@/services/rosbagPlayer'

// Props
interface Props {
  kinematicState?: AutowareKinematicState | null
  detectedObjects?: AutowareDetectedObjects | null
}

const props = withDefaults(defineProps<Props>(), {
  kinematicState: null,
  detectedObjects: null
})

// 视图模式类型
type ViewMode = 'bev' | 'follow' | 'free'

// 视图模式
const viewModes: Array<{ id: ViewMode; name: string }> = [
  { id: 'bev', name: 'Bird Eye' },
  { id: 'follow', name: 'Follow' },
  { id: 'free', name: 'Free' }
]

// 状态
const containerRef = ref<HTMLElement | null>(null)
const canvasRef = ref<HTMLCanvasElement | null>(null)
const currentViewMode = ref<'bev' | 'follow' | 'free'>('bev')

// 显示选项
const showGrid = ref(true)
const showTrajectory = ref(true)
const showObjects = ref(true)
const showVehicle = ref(true)

// 视图状态
const zoom = ref(1)
const panOffset = reactive({ x: 0, y: 0 })
const isDragging = ref(false)
const lastMousePos = reactive({ x: 0, y: 0 })
const cursorWorldPos = ref<{ x: number; y: number } | null>(null)

// 轨迹历史
const trajectoryHistory = ref<Array<{ x: number; y: number; yaw: number }>>([])
const maxTrajectoryPoints = 500

// 渲染统计
const fps = ref(0)
let frameCount = 0
let lastFpsTime = 0
let animationId: number | null = null

// 计算属性
const objectCount = computed(() => {
  return props.detectedObjects?.objects?.length || 0
})

const scaleText = computed(() => {
  const metersPerPixel = 1 / (zoom.value * 5)  // 5 pixels per meter at zoom 1
  if (metersPerPixel < 1) {
    return `${(metersPerPixel * 100).toFixed(0)}cm`
  }
  return `${metersPerPixel.toFixed(1)}m`
})

// 坐标转换
const worldToScreen = (worldX: number, worldY: number): { x: number; y: number } => {
  const canvas = canvasRef.value
  if (!canvas) return { x: 0, y: 0 }

  const centerX = canvas.width / 2 + panOffset.x
  const centerY = canvas.height / 2 + panOffset.y
  const scale = zoom.value * 5  // 5 pixels per meter

  return {
    x: centerX + worldX * scale,
    y: centerY - worldY * scale  // Y轴向上
  }
}

const screenToWorld = (screenX: number, screenY: number): { x: number; y: number } => {
  const canvas = canvasRef.value
  if (!canvas) return { x: 0, y: 0 }

  const centerX = canvas.width / 2 + panOffset.x
  const centerY = canvas.height / 2 + panOffset.y
  const scale = zoom.value * 5

  return {
    x: (screenX - centerX) / scale,
    y: -(screenY - centerY) / scale
  }
}

// 渲染函数
const render = () => {
  const canvas = canvasRef.value
  const ctx = canvas?.getContext('2d')
  if (!canvas || !ctx) return

  // 清除画布
  ctx.fillStyle = '#0a0a12'
  ctx.fillRect(0, 0, canvas.width, canvas.height)

  // 获取自车位置
  const egoPos = props.kinematicState?.pose?.pose?.position || { x: 0, y: 0, z: 0 }
  const egoOrientation = props.kinematicState?.pose?.pose?.orientation || { x: 0, y: 0, z: 0, w: 1 }
  const egoYaw = quaternionToEuler(egoOrientation).yaw

  // 跟随模式下更新pan偏移
  if (currentViewMode.value === 'follow') {
    const screenPos = worldToScreen(egoPos.x, egoPos.y)
    panOffset.x = canvas.width / 2 - screenPos.x + panOffset.x
    panOffset.y = canvas.height / 2 - screenPos.y + panOffset.y
  }

  // 绘制网格
  if (showGrid.value) {
    drawGrid(ctx, canvas)
  }

  // 绘制轨迹
  if (showTrajectory.value && trajectoryHistory.value.length > 1) {
    drawTrajectory(ctx)
  }

  // 绘制检测目标
  if (showObjects.value && props.detectedObjects) {
    drawDetectedObjects(ctx, egoPos, egoYaw)
  }

  // 绘制自车
  if (showVehicle.value) {
    drawEgoVehicle(ctx, egoPos, egoYaw)
  }

  // 更新FPS
  frameCount++
  const now = performance.now()
  if (now - lastFpsTime >= 1000) {
    fps.value = frameCount
    frameCount = 0
    lastFpsTime = now
  }

  animationId = requestAnimationFrame(render)
}

// 绘制网格
const drawGrid = (ctx: CanvasRenderingContext2D, canvas: HTMLCanvasElement) => {
  const gridSize = 10  // 10米网格

  ctx.strokeStyle = '#1a1a2a'
  ctx.lineWidth = 1

  // 计算可见范围
  const startWorld = screenToWorld(0, canvas.height)
  const endWorld = screenToWorld(canvas.width, 0)

  const startX = Math.floor(startWorld.x / gridSize) * gridSize
  const startY = Math.floor(startWorld.y / gridSize) * gridSize
  const endX = Math.ceil(endWorld.x / gridSize) * gridSize
  const endY = Math.ceil(endWorld.y / gridSize) * gridSize

  // 绘制垂直线
  for (let x = startX; x <= endX; x += gridSize) {
    const screenStart = worldToScreen(x, startY)
    const screenEnd = worldToScreen(x, endY)

    ctx.beginPath()
    ctx.moveTo(screenStart.x, screenStart.y)
    ctx.lineTo(screenEnd.x, screenEnd.y)
    ctx.stroke()
  }

  // 绘制水平线
  for (let y = startY; y <= endY; y += gridSize) {
    const screenStart = worldToScreen(startX, y)
    const screenEnd = worldToScreen(endX, y)

    ctx.beginPath()
    ctx.moveTo(screenStart.x, screenStart.y)
    ctx.lineTo(screenEnd.x, screenEnd.y)
    ctx.stroke()
  }

  // 绘制原点
  const origin = worldToScreen(0, 0)
  ctx.strokeStyle = '#333'
  ctx.lineWidth = 2

  ctx.beginPath()
  ctx.moveTo(origin.x - 20, origin.y)
  ctx.lineTo(origin.x + 20, origin.y)
  ctx.stroke()

  ctx.beginPath()
  ctx.moveTo(origin.x, origin.y - 20)
  ctx.lineTo(origin.x, origin.y + 20)
  ctx.stroke()
}

// 绘制轨迹
const drawTrajectory = (ctx: CanvasRenderingContext2D) => {
  if (trajectoryHistory.value.length < 2) return

  ctx.strokeStyle = 'rgba(255, 87, 34, 0.5)'
  ctx.lineWidth = 2
  ctx.setLineDash([5, 5])

  ctx.beginPath()
  const firstPoint = worldToScreen(trajectoryHistory.value[0].x, trajectoryHistory.value[0].y)
  ctx.moveTo(firstPoint.x, firstPoint.y)

  for (let i = 1; i < trajectoryHistory.value.length; i++) {
    const point = worldToScreen(trajectoryHistory.value[i].x, trajectoryHistory.value[i].y)
    ctx.lineTo(point.x, point.y)
  }

  ctx.stroke()
  ctx.setLineDash([])
}

// 绘制检测目标
const drawDetectedObjects = (
  ctx: CanvasRenderingContext2D,
  egoPos: { x: number; y: number; z: number },
  egoYaw: number
) => {
  if (!props.detectedObjects) return

  for (const obj of props.detectedObjects.objects) {
    const objPos = obj.kinematics.pose_with_covariance.pose.position
    const objDim = obj.shape.dimensions
    const label = obj.classification[0]?.label || 0

    // 转换到世界坐标 (目标位置是相对于自车的)
    const worldX = egoPos.x + objPos.x * Math.cos(egoYaw) - objPos.y * Math.sin(egoYaw)
    const worldY = egoPos.y + objPos.x * Math.sin(egoYaw) + objPos.y * Math.cos(egoYaw)

    const screenPos = worldToScreen(worldX, worldY)
    const scale = zoom.value * 5

    // 根据类型设置颜色
    const colors = ['#0096ff', '#ffc800', '#00ff64', '#ff6400', '#aa00ff', '#888888']
    ctx.strokeStyle = colors[label] || '#888888'
    ctx.fillStyle = colors[label] + '33' || '#88888833'
    ctx.lineWidth = 2

    // 绘制边界框
    const halfWidth = (objDim.y / 2) * scale
    const halfLength = (objDim.x / 2) * scale

    ctx.save()
    ctx.translate(screenPos.x, screenPos.y)

    ctx.beginPath()
    ctx.rect(-halfLength, -halfWidth, halfLength * 2, halfWidth * 2)
    ctx.fill()
    ctx.stroke()

    // 绘制朝向指示
    ctx.beginPath()
    ctx.moveTo(0, 0)
    ctx.lineTo(halfLength + 5, 0)
    ctx.stroke()

    ctx.restore()

    // 绘制距离标签
    const distance = Math.sqrt(objPos.x * objPos.x + objPos.y * objPos.y)
    ctx.fillStyle = '#fff'
    ctx.font = '10px monospace'
    ctx.textAlign = 'center'
    ctx.fillText(`${distance.toFixed(1)}m`, screenPos.x, screenPos.y - halfWidth - 5)
  }
}

// 绘制自车
const drawEgoVehicle = (
  ctx: CanvasRenderingContext2D,
  pos: { x: number; y: number; z: number },
  yaw: number
) => {
  const screenPos = worldToScreen(pos.x, pos.y)
  const scale = zoom.value * 5

  // 车辆尺寸 (米)
  const length = 4.5
  const width = 1.8

  const halfLength = (length / 2) * scale
  const halfWidth = (width / 2) * scale

  ctx.save()
  ctx.translate(screenPos.x, screenPos.y)
  ctx.rotate(-yaw)  // Canvas Y轴向下，需要取负

  // 车身
  ctx.fillStyle = 'rgba(255, 87, 34, 0.3)'
  ctx.strokeStyle = '#ff5722'
  ctx.lineWidth = 2

  ctx.beginPath()
  ctx.rect(-halfLength, -halfWidth, halfLength * 2, halfWidth * 2)
  ctx.fill()
  ctx.stroke()

  // 车头指示
  ctx.beginPath()
  ctx.moveTo(halfLength, -halfWidth)
  ctx.lineTo(halfLength + 10, 0)
  ctx.lineTo(halfLength, halfWidth)
  ctx.stroke()

  // 中心点
  ctx.fillStyle = '#ff5722'
  ctx.beginPath()
  ctx.arc(0, 0, 4, 0, Math.PI * 2)
  ctx.fill()

  ctx.restore()
}

// 鼠标事件处理
const handleMouseDown = (e: MouseEvent) => {
  isDragging.value = true
  lastMousePos.x = e.clientX
  lastMousePos.y = e.clientY
}

const handleMouseUp = () => {
  isDragging.value = false
}

const handleMouseMove = (e: MouseEvent) => {
  // 更新光标位置
  const rect = canvasRef.value?.getBoundingClientRect()
  if (rect) {
    const worldPos = screenToWorld(e.clientX - rect.left, e.clientY - rect.top)
    cursorWorldPos.value = worldPos
  }

  // 拖拽平移
  if (isDragging.value) {
    panOffset.x += e.clientX - lastMousePos.x
    panOffset.y += e.clientY - lastMousePos.y
    lastMousePos.x = e.clientX
    lastMousePos.y = e.clientY
  }
}

const handleWheel = (e: WheelEvent) => {
  e.preventDefault()
  const delta = e.deltaY > 0 ? 0.9 : 1.1
  zoom.value = Math.max(0.1, Math.min(10, zoom.value * delta))
}

// 调整Canvas大小
const resizeCanvas = () => {
  const container = containerRef.value
  const canvas = canvasRef.value
  if (!container || !canvas) return

  canvas.width = container.clientWidth
  canvas.height = container.clientHeight
}

// 监听数据变化，更新轨迹
watch(() => props.kinematicState, (newState) => {
  if (newState?.pose?.pose?.position) {
    const pos = newState.pose.pose.position
    const orientation = newState.pose.pose.orientation
    const yaw = quaternionToEuler(orientation).yaw

    // 添加到轨迹历史
    trajectoryHistory.value.push({ x: pos.x, y: pos.y, yaw })

    // 限制轨迹点数
    if (trajectoryHistory.value.length > maxTrajectoryPoints) {
      trajectoryHistory.value.shift()
    }
  }
}, { deep: true })

// 生命周期
onMounted(() => {
  resizeCanvas()
  window.addEventListener('resize', resizeCanvas)

  // 开始渲染循环
  lastFpsTime = performance.now()
  render()
})

onUnmounted(() => {
  window.removeEventListener('resize', resizeCanvas)
  if (animationId) {
    cancelAnimationFrame(animationId)
  }
})

// 暴露方法
defineExpose({
  clearTrajectory: () => {
    trajectoryHistory.value = []
  },
  resetView: () => {
    zoom.value = 1
    panOffset.x = 0
    panOffset.y = 0
  }
})
</script>

<style scoped>
.rosbag-viz {
  display: flex;
  flex-direction: column;
  height: 100%;
  background: #0a0a12;
}

/* Toolbar */
.viz-toolbar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 8px 12px;
  background: #14141f;
  border-bottom: 1px solid #2a2a3a;
}

.view-mode {
  display: flex;
  gap: 4px;
}

.mode-btn {
  padding: 4px 12px;
  background: #1a1a2e;
  border: 1px solid #333;
  color: #888;
  font-size: 11px;
  cursor: pointer;
  transition: all 0.2s;
}

.mode-btn:hover {
  color: #fff;
}

.mode-btn.active {
  background: rgba(255, 87, 34, 0.15);
  border-color: var(--primary, #ff5722);
  color: var(--primary, #ff5722);
}

.viz-options {
  display: flex;
  gap: 12px;
}

.option {
  display: flex;
  align-items: center;
  gap: 4px;
  font-size: 11px;
  color: #888;
  cursor: pointer;
}

.option input[type="checkbox"] {
  accent-color: var(--primary, #ff5722);
}

/* Canvas Container */
.viz-canvas-container {
  flex: 1;
  position: relative;
  overflow: hidden;
}

.viz-canvas {
  width: 100%;
  height: 100%;
  cursor: crosshair;
}

.viz-canvas:active {
  cursor: grabbing;
}

/* Scale Indicator */
.scale-indicator {
  position: absolute;
  bottom: 40px;
  left: 12px;
  display: flex;
  align-items: center;
  gap: 6px;
  color: #888;
  font-size: 10px;
}

.scale-bar {
  width: 50px;
  height: 3px;
  background: #888;
  border-left: 2px solid #888;
  border-right: 2px solid #888;
}

/* Cursor Info */
.cursor-info {
  position: absolute;
  bottom: 40px;
  right: 12px;
  padding: 4px 8px;
  background: rgba(20, 20, 31, 0.9);
  border: 1px solid #2a2a3a;
  color: #aaa;
  font-size: 10px;
  font-family: 'JetBrains Mono', monospace;
}

/* Legend */
.legend {
  position: absolute;
  top: 12px;
  right: 12px;
  background: rgba(20, 20, 31, 0.9);
  border: 1px solid #2a2a3a;
  padding: 8px 12px;
}

.legend-item {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 10px;
  color: #888;
  margin-bottom: 4px;
}

.legend-item:last-child {
  margin-bottom: 0;
}

.legend-color {
  width: 12px;
  height: 12px;
  border-radius: 2px;
}

.legend-color.ego { background: rgba(255, 87, 34, 0.5); border: 1px solid #ff5722; }
.legend-color.car { background: rgba(0, 150, 255, 0.5); border: 1px solid #0096ff; }
.legend-color.pedestrian { background: rgba(255, 200, 0, 0.5); border: 1px solid #ffc800; }
.legend-color.cyclist { background: rgba(0, 255, 100, 0.5); border: 1px solid #00ff64; }
.legend-color.trajectory { background: transparent; border: 2px dashed rgba(255, 87, 34, 0.5); }

/* Status Bar */
.viz-statusbar {
  display: flex;
  gap: 16px;
  padding: 6px 12px;
  background: #14141f;
  border-top: 1px solid #2a2a3a;
}

.status-item {
  font-size: 10px;
  color: #666;
}
</style>
