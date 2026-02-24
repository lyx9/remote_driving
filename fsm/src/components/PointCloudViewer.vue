<!--
  FSM-Pilot V2.0 - Remote Driving System

  @project     FSM-Pilot Remote Driving Platform
  @author      Li Yixiang
  @institution City University of Hong Kong
  @copyright   2025 City University of Hong Kong. All rights reserved.
  @license     Proprietary

  @component   PointCloudViewer
  @description WebGL 点云可视化组件，使用 Three.js 渲染 PointCloud2 数据
-->
<template>
  <div class="point-cloud-viewer" ref="containerRef">
    <!-- 工具栏 -->
    <div class="viewer-toolbar">
      <div class="toolbar-left">
        <span class="viewer-title">Point Cloud Viewer</span>
        <span class="point-count">{{ pointCount.toLocaleString() }} points</span>
      </div>
      <div class="toolbar-right">
        <button class="tool-btn" @click="resetView" title="Reset View">
          🔄
        </button>
        <button class="tool-btn" @click="toggleTopView" :class="{ active: isTopView }" title="Top View">
          ⬆️
        </button>
        <button class="tool-btn" @click="toggleAutoRotate" :class="{ active: autoRotate }" title="Auto Rotate">
          🔁
        </button>
        <select v-model="colorMode" class="color-select">
          <option value="height">Height</option>
          <option value="intensity">Intensity</option>
          <option value="distance">Distance</option>
          <option value="ring">Ring</option>
        </select>
        <input
          type="range"
          v-model.number="pointSize"
          min="1"
          max="10"
          step="0.5"
          class="size-slider"
          title="Point Size"
        />
      </div>
    </div>

    <!-- 渲染画布 -->
    <div class="canvas-container" ref="canvasContainerRef"></div>

    <!-- 信息面板 -->
    <div class="info-panel">
      <div class="info-item">
        <span class="info-label">FPS</span>
        <span class="info-value">{{ fps }}</span>
      </div>
      <div class="info-item">
        <span class="info-label">Range</span>
        <span class="info-value">{{ maxRange.toFixed(1) }}m</span>
      </div>
      <div class="info-item">
        <span class="info-label">Height</span>
        <span class="info-value">{{ heightRange.min.toFixed(1) }} ~ {{ heightRange.max.toFixed(1) }}m</span>
      </div>
    </div>

    <!-- 颜色图例 -->
    <div class="color-legend">
      <div class="legend-gradient" :style="legendGradientStyle"></div>
      <div class="legend-labels">
        <span>{{ legendMin }}</span>
        <span>{{ legendMax }}</span>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, shallowRef, onMounted, onUnmounted, watch, computed } from 'vue'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import type { PointCloudPoint } from '@/services/rosbagDb3Parser'

// Props
interface Props {
  points?: PointCloudPoint[]
  autoUpdate?: boolean
}

const props = withDefaults(defineProps<Props>(), {
  points: () => [],
  autoUpdate: true
})

// Refs
const containerRef = ref<HTMLElement | null>(null)
const canvasContainerRef = ref<HTMLElement | null>(null)

// Three.js 对象
const scene = shallowRef<THREE.Scene | null>(null)
const camera = shallowRef<THREE.PerspectiveCamera | null>(null)
const renderer = shallowRef<THREE.WebGLRenderer | null>(null)
const controls = shallowRef<OrbitControls | null>(null)
const pointCloud = shallowRef<THREE.Points | null>(null)

// 状态
const pointCount = ref(0)
const fps = ref(0)
const maxRange = ref(0)
const heightRange = ref({ min: 0, max: 0 })
const colorMode = ref<'height' | 'intensity' | 'distance' | 'ring'>('height')
const pointSize = ref(2)
const autoRotate = ref(false)
const isTopView = ref(false)

// FPS 计算
let frameCount = 0
let lastFpsTime = performance.now()
let animationId: number | null = null

// 颜色图例
const legendGradientStyle = computed(() => {
  switch (colorMode.value) {
    case 'height':
      return { background: 'linear-gradient(to right, #0000ff, #00ffff, #00ff00, #ffff00, #ff0000)' }
    case 'intensity':
      return { background: 'linear-gradient(to right, #000000, #ffffff)' }
    case 'distance':
      return { background: 'linear-gradient(to right, #00ff00, #ffff00, #ff0000)' }
    case 'ring':
      return { background: 'linear-gradient(to right, #ff0000, #00ff00, #0000ff, #ff00ff)' }
  }
})

const legendMin = computed(() => {
  switch (colorMode.value) {
    case 'height': return `${heightRange.value.min.toFixed(1)}m`
    case 'intensity': return '0'
    case 'distance': return '0m'
    case 'ring': return '0'
  }
})

const legendMax = computed(() => {
  switch (colorMode.value) {
    case 'height': return `${heightRange.value.max.toFixed(1)}m`
    case 'intensity': return '255'
    case 'distance': return `${maxRange.value.toFixed(1)}m`
    case 'ring': return '32'
  }
})

// 初始化 Three.js
const initThree = () => {
  if (!canvasContainerRef.value) return

  const width = canvasContainerRef.value.clientWidth
  const height = canvasContainerRef.value.clientHeight

  // 场景
  scene.value = new THREE.Scene()
  scene.value.background = new THREE.Color(0x0a0a12)

  // 相机
  camera.value = new THREE.PerspectiveCamera(60, width / height, 0.1, 1000)
  camera.value.position.set(30, 30, 30)
  camera.value.lookAt(0, 0, 0)

  // 渲染器
  renderer.value = new THREE.WebGLRenderer({ antialias: true })
  renderer.value.setSize(width, height)
  renderer.value.setPixelRatio(window.devicePixelRatio)
  canvasContainerRef.value.appendChild(renderer.value.domElement)

  // 控制器
  controls.value = new OrbitControls(camera.value, renderer.value.domElement)
  controls.value.enableDamping = true
  controls.value.dampingFactor = 0.05
  controls.value.screenSpacePanning = false
  controls.value.minDistance = 5
  controls.value.maxDistance = 200
  controls.value.maxPolarAngle = Math.PI

  // 添加辅助元素
  addHelpers()

  // 开始渲染循环
  animate()
}

// 添加辅助元素
const addHelpers = () => {
  if (!scene.value) return

  // 网格
  const gridHelper = new THREE.GridHelper(100, 50, 0x333333, 0x222222)
  gridHelper.position.y = -2
  scene.value.add(gridHelper)

  // 坐标轴
  const axesHelper = new THREE.AxesHelper(10)
  scene.value.add(axesHelper)

  // 添加车辆参考框
  const vehicleGeometry = new THREE.BoxGeometry(4.5, 1.8, 1.5)
  const vehicleEdges = new THREE.EdgesGeometry(vehicleGeometry)
  const vehicleMaterial = new THREE.LineBasicMaterial({ color: 0x00f2ff })
  const vehicleWireframe = new THREE.LineSegments(vehicleEdges, vehicleMaterial)
  vehicleWireframe.position.set(0, 0.75, 0)
  scene.value.add(vehicleWireframe)
}

// 更新点云
const updatePointCloud = (points: PointCloudPoint[]) => {
  if (!scene.value) return

  // 移除旧的点云
  if (pointCloud.value) {
    scene.value.remove(pointCloud.value)
    pointCloud.value.geometry.dispose()
    if (pointCloud.value.material instanceof THREE.Material) {
      pointCloud.value.material.dispose()
    }
  }

  if (points.length === 0) {
    pointCount.value = 0
    return
  }

  // 计算统计数据
  let minZ = Infinity, maxZ = -Infinity
  let maxDist = 0

  for (const p of points) {
    if (p.z < minZ) minZ = p.z
    if (p.z > maxZ) maxZ = p.z
    const dist = Math.sqrt(p.x * p.x + p.y * p.y)
    if (dist > maxDist) maxDist = dist
  }

  heightRange.value = { min: minZ, max: maxZ }
  maxRange.value = maxDist

  // 创建几何体
  const geometry = new THREE.BufferGeometry()
  const positions = new Float32Array(points.length * 3)
  const colors = new Float32Array(points.length * 3)

  const color = new THREE.Color()
  const heightSpan = maxZ - minZ || 1

  for (let i = 0; i < points.length; i++) {
    const p = points[i]
    positions[i * 3] = p.x
    positions[i * 3 + 1] = p.z  // Three.js Y轴向上
    positions[i * 3 + 2] = -p.y // 调整坐标系

    // 根据颜色模式设置颜色
    let t: number
    switch (colorMode.value) {
      case 'height':
        t = (p.z - minZ) / heightSpan
        color.setHSL(0.7 - t * 0.7, 1, 0.5)
        break
      case 'intensity':
        t = (p.intensity || 0) / 255
        color.setRGB(t, t, t)
        break
      case 'distance':
        const dist = Math.sqrt(p.x * p.x + p.y * p.y)
        t = Math.min(dist / maxDist, 1)
        color.setHSL(0.3 - t * 0.3, 1, 0.5)
        break
      case 'ring':
        t = ((p.ring || 0) % 32) / 32
        color.setHSL(t, 1, 0.5)
        break
    }

    colors[i * 3] = color.r
    colors[i * 3 + 1] = color.g
    colors[i * 3 + 2] = color.b
  }

  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3))
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3))

  // 创建材质
  const material = new THREE.PointsMaterial({
    size: pointSize.value * 0.05,
    vertexColors: true,
    sizeAttenuation: true
  })

  // 创建点云对象
  pointCloud.value = new THREE.Points(geometry, material)
  scene.value.add(pointCloud.value)

  pointCount.value = points.length
}

// 动画循环
const animate = () => {
  animationId = requestAnimationFrame(animate)

  if (controls.value) {
    controls.value.autoRotate = autoRotate.value
    controls.value.update()
  }

  if (renderer.value && scene.value && camera.value) {
    renderer.value.render(scene.value, camera.value)
  }

  // 更新 FPS
  frameCount++
  const now = performance.now()
  if (now - lastFpsTime >= 1000) {
    fps.value = frameCount
    frameCount = 0
    lastFpsTime = now
  }
}

// 重置视图
const resetView = () => {
  if (camera.value && controls.value) {
    camera.value.position.set(30, 30, 30)
    camera.value.lookAt(0, 0, 0)
    controls.value.reset()
    isTopView.value = false
  }
}

// 切换俯视图
const toggleTopView = () => {
  if (!camera.value || !controls.value) return

  isTopView.value = !isTopView.value

  if (isTopView.value) {
    camera.value.position.set(0, 50, 0)
    camera.value.lookAt(0, 0, 0)
    controls.value.update()
  } else {
    camera.value.position.set(30, 30, 30)
    camera.value.lookAt(0, 0, 0)
    controls.value.update()
  }
}

// 切换自动旋转
const toggleAutoRotate = () => {
  autoRotate.value = !autoRotate.value
}

// 处理窗口大小变化
const handleResize = () => {
  if (!canvasContainerRef.value || !camera.value || !renderer.value) return

  const width = canvasContainerRef.value.clientWidth
  const height = canvasContainerRef.value.clientHeight

  camera.value.aspect = width / height
  camera.value.updateProjectionMatrix()
  renderer.value.setSize(width, height)
}

// 监听点云数据变化
watch(() => props.points, (newPoints) => {
  if (props.autoUpdate && newPoints) {
    updatePointCloud(newPoints)
  }
}, { deep: true })

// 监听颜色模式变化
watch(colorMode, () => {
  if (props.points) {
    updatePointCloud(props.points)
  }
})

// 监听点大小变化
watch(pointSize, () => {
  if (pointCloud.value) {
    const material = pointCloud.value.material as THREE.PointsMaterial
    material.size = pointSize.value * 0.05
  }
})

// 生命周期
onMounted(() => {
  initThree()
  window.addEventListener('resize', handleResize)

  if (props.points && props.points.length > 0) {
    updatePointCloud(props.points)
  }
})

onUnmounted(() => {
  window.removeEventListener('resize', handleResize)

  if (animationId) {
    cancelAnimationFrame(animationId)
  }

  if (pointCloud.value) {
    pointCloud.value.geometry.dispose()
    if (pointCloud.value.material instanceof THREE.Material) {
      pointCloud.value.material.dispose()
    }
  }

  if (renderer.value) {
    renderer.value.dispose()
  }
})

// 暴露方法
defineExpose({
  updatePointCloud,
  resetView,
  toggleTopView
})
</script>

<style scoped>
.point-cloud-viewer {
  position: relative;
  width: 100%;
  height: 100%;
  display: flex;
  flex-direction: column;
  background: #0a0a12;
}

/* 工具栏 */
.viewer-toolbar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 8px 12px;
  background: #14141f;
  border-bottom: 1px solid #2a2a3a;
  z-index: 10;
}

.toolbar-left {
  display: flex;
  align-items: center;
  gap: 12px;
}

.viewer-title {
  font-size: 13px;
  font-weight: bold;
  color: var(--primary, #ff5722);
}

.point-count {
  font-size: 11px;
  color: #888;
  padding: 2px 8px;
  background: rgba(255, 87, 34, 0.1);
  border-radius: 10px;
}

.toolbar-right {
  display: flex;
  align-items: center;
  gap: 8px;
}

.tool-btn {
  width: 32px;
  height: 32px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: #1a1a2e;
  border: 1px solid #333;
  color: #888;
  font-size: 14px;
  cursor: pointer;
  transition: all 0.2s;
  border-radius: 4px;
}

.tool-btn:hover {
  background: #252540;
  color: #fff;
}

.tool-btn.active {
  background: rgba(255, 87, 34, 0.2);
  border-color: var(--primary, #ff5722);
  color: var(--primary, #ff5722);
}

.color-select {
  padding: 4px 8px;
  background: #1a1a2e;
  border: 1px solid #333;
  color: #fff;
  font-size: 11px;
  cursor: pointer;
  border-radius: 4px;
}

.size-slider {
  width: 60px;
  accent-color: var(--primary, #ff5722);
}

/* 画布容器 */
.canvas-container {
  flex: 1;
  position: relative;
  overflow: hidden;
}

.canvas-container canvas {
  display: block;
}

/* 信息面板 */
.info-panel {
  position: absolute;
  top: 60px;
  left: 12px;
  display: flex;
  flex-direction: column;
  gap: 4px;
  background: rgba(20, 20, 31, 0.9);
  border: 1px solid #2a2a3a;
  padding: 8px 12px;
  border-radius: 4px;
  z-index: 10;
}

.info-item {
  display: flex;
  justify-content: space-between;
  gap: 16px;
  font-size: 11px;
}

.info-label {
  color: #666;
}

.info-value {
  color: #fff;
  font-family: 'JetBrains Mono', monospace;
}

/* 颜色图例 */
.color-legend {
  position: absolute;
  bottom: 20px;
  left: 12px;
  width: 150px;
  background: rgba(20, 20, 31, 0.9);
  border: 1px solid #2a2a3a;
  padding: 8px;
  border-radius: 4px;
  z-index: 10;
}

.legend-gradient {
  height: 12px;
  border-radius: 2px;
  margin-bottom: 4px;
}

.legend-labels {
  display: flex;
  justify-content: space-between;
  font-size: 10px;
  color: #888;
}
</style>
