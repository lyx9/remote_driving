/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      3D Gaussian Splatting Service
 * @description 3D-GS渲染服务，用于场景重建和可视化
 */

import { ref, reactive, onUnmounted } from 'vue'

// ======================== 类型定义 ========================

export interface GaussianPoint {
  position: [number, number, number]  // xyz
  scale: [number, number, number]     // 各轴缩放
  rotation: [number, number, number, number]  // 四元数
  color: [number, number, number]     // RGB (0-1)
  opacity: number                     // 透明度 (0-1)
  sphericalHarmonics?: number[]       // SH 系数 (可选)
}

export interface GaussianSplat {
  id: string
  name: string
  pointCount: number
  points: GaussianPoint[]
  boundingBox: {
    min: [number, number, number]
    max: [number, number, number]
  }
  metadata?: {
    captureDate?: string
    location?: string
    sourceType?: 'rosbag' | 'video' | 'images'
    resolution?: string
  }
}

export interface CameraView {
  position: [number, number, number]
  target: [number, number, number]
  up: [number, number, number]
  fov: number  // 视场角 (度)
  near: number
  far: number
}

export interface RenderSettings {
  pointSize: number
  splatScale: number
  maxPoints: number
  sortingMethod: 'radix' | 'bitonic' | 'none'
  cullingEnabled: boolean
  lodEnabled: boolean
  antialias: boolean
  backgroundColor: [number, number, number]
}

// ======================== 默认配置 ========================

const DEFAULT_RENDER_SETTINGS: RenderSettings = {
  pointSize: 2.0,
  splatScale: 1.0,
  maxPoints: 1000000,
  sortingMethod: 'radix',
  cullingEnabled: true,
  lodEnabled: true,
  antialias: true,
  backgroundColor: [0.05, 0.05, 0.1]
}

const DEFAULT_CAMERA: CameraView = {
  position: [5, 3, 5],
  target: [0, 0, 0],
  up: [0, 1, 0],
  fov: 60,
  near: 0.1,
  far: 1000
}

// ======================== Mock 3D-GS 数据生成 ========================

/**
 * 生成 Mock Gaussian Splat 数据
 * 模拟从 RosBag 重建的道路场景
 */
export function generateMockGaussianSplat(
  scenarioType: 'road' | 'intersection' | 'parking' | 'urban' = 'road',
  pointCount: number = 10000
): GaussianSplat {
  const points: GaussianPoint[] = []

  switch (scenarioType) {
    case 'road':
      generateRoadScene(points, pointCount)
      break
    case 'intersection':
      generateIntersectionScene(points, pointCount)
      break
    case 'parking':
      generateParkingScene(points, pointCount)
      break
    case 'urban':
      generateUrbanScene(points, pointCount)
      break
  }

  // 计算边界框
  let minX = Infinity, minY = Infinity, minZ = Infinity
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity

  for (const p of points) {
    minX = Math.min(minX, p.position[0])
    minY = Math.min(minY, p.position[1])
    minZ = Math.min(minZ, p.position[2])
    maxX = Math.max(maxX, p.position[0])
    maxY = Math.max(maxY, p.position[1])
    maxZ = Math.max(maxZ, p.position[2])
  }

  return {
    id: `mock-${scenarioType}-${Date.now()}`,
    name: `Mock ${scenarioType.charAt(0).toUpperCase() + scenarioType.slice(1)} Scene`,
    pointCount: points.length,
    points,
    boundingBox: {
      min: [minX, minY, minZ],
      max: [maxX, maxY, maxZ]
    },
    metadata: {
      captureDate: new Date().toISOString(),
      location: 'Mock Location',
      sourceType: 'rosbag',
      resolution: 'high'
    }
  }
}

/**
 * 生成道路场景
 */
function generateRoadScene(points: GaussianPoint[], count: number) {
  const roadLength = 100
  const roadWidth = 8
  const laneMarkingWidth = 0.15

  // 路面
  for (let i = 0; i < count * 0.4; i++) {
    const x = (Math.random() - 0.5) * roadLength
    const z = (Math.random() - 0.5) * roadWidth
    const y = Math.random() * 0.05  // 轻微高度变化

    points.push({
      position: [x, y, z],
      scale: [0.1, 0.02, 0.1],
      rotation: [0, 0, 0, 1],
      color: [0.3, 0.3, 0.35],  // 深灰色
      opacity: 0.95
    })
  }

  // 车道标线
  for (let i = 0; i < count * 0.1; i++) {
    const x = (Math.random() - 0.5) * roadLength
    const y = 0.02
    const laneOffset = Math.floor(Math.random() * 3) - 1  // -1, 0, 1
    const z = laneOffset * 2.5

    // 虚线效果
    if (Math.abs(x % 3) < 1.5) {
      points.push({
        position: [x, y, z],
        scale: [0.5, 0.01, laneMarkingWidth],
        rotation: [0, 0, 0, 1],
        color: [0.9, 0.9, 0.9],  // 白色
        opacity: 0.9
      })
    }
  }

  // 路边
  for (let i = 0; i < count * 0.15; i++) {
    const x = (Math.random() - 0.5) * roadLength
    const side = Math.random() > 0.5 ? 1 : -1
    const z = side * (roadWidth / 2 + Math.random() * 3)
    const y = Math.random() * 0.3

    // 草地/土地
    points.push({
      position: [x, y, z],
      scale: [0.15, 0.1, 0.15],
      rotation: randomRotation(),
      color: [0.2 + Math.random() * 0.1, 0.35 + Math.random() * 0.15, 0.15],
      opacity: 0.85
    })
  }

  // 树木
  for (let i = 0; i < count * 0.1; i++) {
    const x = (Math.random() - 0.5) * roadLength
    const side = Math.random() > 0.5 ? 1 : -1
    const z = side * (roadWidth / 2 + 5 + Math.random() * 5)
    const height = 2 + Math.random() * 3

    // 树干
    points.push({
      position: [x, height * 0.3, z],
      scale: [0.15, height * 0.4, 0.15],
      rotation: [0, 0, 0, 1],
      color: [0.35, 0.25, 0.15],
      opacity: 0.9
    })

    // 树冠
    for (let j = 0; j < 20; j++) {
      const leafX = x + (Math.random() - 0.5) * 2
      const leafY = height * 0.5 + Math.random() * height * 0.5
      const leafZ = z + (Math.random() - 0.5) * 2

      points.push({
        position: [leafX, leafY, leafZ],
        scale: [0.3, 0.3, 0.3],
        rotation: randomRotation(),
        color: [0.1 + Math.random() * 0.1, 0.4 + Math.random() * 0.2, 0.1],
        opacity: 0.8
      })
    }
  }

  // 车辆
  generateVehiclePoints(points, count * 0.05, roadLength, roadWidth)

  // 天空/环境
  for (let i = 0; i < count * 0.1; i++) {
    const angle = Math.random() * Math.PI * 2
    const distance = 50 + Math.random() * 30
    const height = 10 + Math.random() * 20

    points.push({
      position: [Math.cos(angle) * distance, height, Math.sin(angle) * distance],
      scale: [2, 2, 0.1],
      rotation: randomRotation(),
      color: [0.4, 0.6, 0.9],  // 天蓝色
      opacity: 0.3
    })
  }
}

/**
 * 生成十字路口场景
 */
function generateIntersectionScene(points: GaussianPoint[], count: number) {
  const roadWidth = 10
  const intersectionSize = 20

  // 路面
  for (let i = 0; i < count * 0.5; i++) {
    const x = (Math.random() - 0.5) * intersectionSize * 2
    const z = (Math.random() - 0.5) * intersectionSize * 2
    const y = Math.random() * 0.03

    // 只在道路区域生成
    const inNSRoad = Math.abs(x) < roadWidth / 2
    const inEWRoad = Math.abs(z) < roadWidth / 2

    if (inNSRoad || inEWRoad) {
      points.push({
        position: [x, y, z],
        scale: [0.1, 0.02, 0.1],
        rotation: [0, 0, 0, 1],
        color: [0.3, 0.3, 0.35],
        opacity: 0.95
      })
    }
  }

  // 斑马线
  for (let dir = 0; dir < 4; dir++) {
    const angle = dir * Math.PI / 2
    for (let i = 0; i < 10; i++) {
      const offset = (i - 4.5) * 0.6
      const distance = roadWidth / 2 + 1

      const x = Math.cos(angle) * distance + Math.sin(angle) * offset
      const z = Math.sin(angle) * distance - Math.cos(angle) * offset

      points.push({
        position: [x, 0.02, z],
        scale: [0.8, 0.01, 0.3],
        rotation: [0, Math.sin(angle / 2), 0, Math.cos(angle / 2)],
        color: [0.95, 0.95, 0.95],
        opacity: 0.95
      })
    }
  }

  // 信号灯
  for (let dir = 0; dir < 4; dir++) {
    const angle = dir * Math.PI / 2
    const x = Math.cos(angle) * (roadWidth / 2 + 2)
    const z = Math.sin(angle) * (roadWidth / 2 + 2)

    // 灯杆
    points.push({
      position: [x, 2, z],
      scale: [0.1, 4, 0.1],
      rotation: [0, 0, 0, 1],
      color: [0.2, 0.2, 0.2],
      opacity: 0.9
    })

    // 信号灯
    const colors = [[1, 0, 0], [1, 1, 0], [0, 1, 0]]
    for (let c = 0; c < 3; c++) {
      points.push({
        position: [x, 4.5 - c * 0.4, z],
        scale: [0.15, 0.15, 0.15],
        rotation: [0, 0, 0, 1],
        color: colors[c] as [number, number, number],
        opacity: c === 2 ? 1 : 0.3  // 绿灯亮
      })
    }
  }

  // 建筑物
  for (let corner = 0; corner < 4; corner++) {
    const xSign = corner % 2 === 0 ? 1 : -1
    const zSign = corner < 2 ? 1 : -1
    const baseX = xSign * (roadWidth / 2 + 8)
    const baseZ = zSign * (roadWidth / 2 + 8)
    const buildingWidth = 5 + Math.random() * 5
    const buildingHeight = 5 + Math.random() * 10

    for (let i = 0; i < count * 0.03; i++) {
      const x = baseX + (Math.random() - 0.5) * buildingWidth
      const z = baseZ + (Math.random() - 0.5) * buildingWidth
      const y = Math.random() * buildingHeight

      points.push({
        position: [x, y, z],
        scale: [0.2, 0.2, 0.2],
        rotation: [0, 0, 0, 1],
        color: [0.5 + Math.random() * 0.2, 0.5 + Math.random() * 0.2, 0.5 + Math.random() * 0.2],
        opacity: 0.9
      })
    }
  }
}

/**
 * 生成停车场场景
 */
function generateParkingScene(points: GaussianPoint[], count: number) {
  const lotWidth = 40
  const lotLength = 30
  const spotWidth = 2.5
  const spotLength = 5

  // 地面
  for (let i = 0; i < count * 0.4; i++) {
    const x = (Math.random() - 0.5) * lotWidth
    const z = (Math.random() - 0.5) * lotLength
    const y = Math.random() * 0.02

    points.push({
      position: [x, y, z],
      scale: [0.1, 0.02, 0.1],
      rotation: [0, 0, 0, 1],
      color: [0.4, 0.4, 0.42],
      opacity: 0.95
    })
  }

  // 停车位标线
  const rows = 3
  const spotsPerRow = Math.floor(lotWidth / (spotWidth + 0.5))

  for (let row = 0; row < rows; row++) {
    for (let spot = 0; spot < spotsPerRow; spot++) {
      const x = (spot - spotsPerRow / 2) * (spotWidth + 0.5)
      const z = (row - 1) * (spotLength + 2)

      // 停车位边线
      for (let line = 0; line < 3; line++) {
        const lineX = x + (line - 1) * spotWidth / 2
        points.push({
          position: [lineX, 0.02, z],
          scale: [0.05, 0.01, spotLength],
          rotation: [0, 0, 0, 1],
          color: [0.9, 0.9, 0.9],
          opacity: 0.85
        })
      }

      // 随机放置车辆
      if (Math.random() > 0.4) {
        generateSingleVehicle(points, x, z, 0)
      }
    }
  }

  // 路灯
  for (let i = 0; i < 6; i++) {
    const x = (i % 3 - 1) * 15
    const z = (i < 3 ? 1 : -1) * 12

    points.push({
      position: [x, 3, z],
      scale: [0.08, 6, 0.08],
      rotation: [0, 0, 0, 1],
      color: [0.3, 0.3, 0.3],
      opacity: 0.9
    })

    // 灯光
    points.push({
      position: [x, 6, z],
      scale: [0.3, 0.3, 0.3],
      rotation: [0, 0, 0, 1],
      color: [1, 0.95, 0.8],
      opacity: 1
    })
  }
}

/**
 * 生成城市场景
 */
function generateUrbanScene(points: GaussianPoint[], count: number) {
  // 结合道路和建筑
  generateIntersectionScene(points, count * 0.6)

  // 额外的建筑群
  for (let i = 0; i < 8; i++) {
    const angle = (i / 8) * Math.PI * 2
    const distance = 25 + Math.random() * 15
    const x = Math.cos(angle) * distance
    const z = Math.sin(angle) * distance
    const height = 8 + Math.random() * 20

    for (let j = 0; j < count * 0.02; j++) {
      const px = x + (Math.random() - 0.5) * 8
      const pz = z + (Math.random() - 0.5) * 8
      const py = Math.random() * height

      points.push({
        position: [px, py, pz],
        scale: [0.25, 0.25, 0.25],
        rotation: [0, 0, 0, 1],
        color: [0.6 + Math.random() * 0.2, 0.6 + Math.random() * 0.2, 0.65 + Math.random() * 0.15],
        opacity: 0.85
      })
    }
  }
}

/**
 * 生成车辆点云
 */
function generateVehiclePoints(
  points: GaussianPoint[],
  count: number,
  roadLength: number,
  _roadWidth: number
) {
  const vehicleCount = Math.floor(count / 200)

  for (let v = 0; v < vehicleCount; v++) {
    const x = (Math.random() - 0.5) * roadLength * 0.8
    const lane = Math.floor(Math.random() * 2)
    const z = (lane - 0.5) * 3
    const heading = Math.random() * 0.2 - 0.1

    generateSingleVehicle(points, x, z, heading)
  }
}

/**
 * 生成单个车辆
 */
function generateSingleVehicle(
  points: GaussianPoint[],
  x: number,
  z: number,
  heading: number
) {
  const vehicleLength = 4.5
  const vehicleWidth = 1.8
  const vehicleHeight = 1.5

  // 车身颜色 (随机选择)
  const colors: [number, number, number][] = [
    [0.8, 0.1, 0.1],  // 红
    [0.1, 0.3, 0.8],  // 蓝
    [0.9, 0.9, 0.9],  // 白
    [0.15, 0.15, 0.15], // 黑
    [0.7, 0.7, 0.7],  // 银
  ]
  const bodyColor = colors[Math.floor(Math.random() * colors.length)]

  // 车身点
  for (let i = 0; i < 100; i++) {
    const lx = (Math.random() - 0.5) * vehicleLength
    const ly = Math.random() * vehicleHeight
    const lz = (Math.random() - 0.5) * vehicleWidth

    // 旋转
    const rx = lx * Math.cos(heading) - lz * Math.sin(heading)
    const rz = lx * Math.sin(heading) + lz * Math.cos(heading)

    points.push({
      position: [x + rx, ly, z + rz],
      scale: [0.08, 0.08, 0.08],
      rotation: [0, Math.sin(heading / 2), 0, Math.cos(heading / 2)],
      color: ly > vehicleHeight * 0.7 ? [0.3, 0.4, 0.5] : bodyColor,  // 车窗用深色
      opacity: 0.95
    })
  }

  // 轮子
  const wheelPositions = [
    [-1.5, 0.3, -0.8],
    [-1.5, 0.3, 0.8],
    [1.5, 0.3, -0.8],
    [1.5, 0.3, 0.8]
  ]

  for (const wp of wheelPositions) {
    const rx = wp[0] * Math.cos(heading) - wp[2] * Math.sin(heading)
    const rz = wp[0] * Math.sin(heading) + wp[2] * Math.cos(heading)

    points.push({
      position: [x + rx, wp[1], z + rz],
      scale: [0.15, 0.3, 0.3],
      rotation: [0, 0, 0, 1],
      color: [0.1, 0.1, 0.1],
      opacity: 0.95
    })
  }
}

/**
 * 生成随机旋转四元数
 */
function randomRotation(): [number, number, number, number] {
  const angle = Math.random() * Math.PI * 2
  return [0, Math.sin(angle / 2), 0, Math.cos(angle / 2)]
}

// ======================== 3D-GS 渲染服务 ========================

export function use3DGaussianSplatting() {
  // 状态
  const isInitialized = ref(false)
  const isLoading = ref(false)
  const error = ref<string | null>(null)

  // 当前加载的 splat 数据
  const currentSplat = ref<GaussianSplat | null>(null)

  // 相机
  const camera = reactive<CameraView>({ ...DEFAULT_CAMERA })

  // 渲染设置
  const settings = reactive<RenderSettings>({ ...DEFAULT_RENDER_SETTINGS })

  // 渲染统计
  const stats = reactive({
    fps: 0,
    visiblePoints: 0,
    renderTime: 0,
    sortTime: 0
  })

  // Canvas 和 WebGL
  let canvas: HTMLCanvasElement | null = null
  let gl: WebGL2RenderingContext | null = null
  let animationId: number | null = null

  // ==================== 初始化 ====================

  /**
   * 初始化渲染器
   */
  const initialize = async (canvasElement: HTMLCanvasElement): Promise<boolean> => {
    try {
      canvas = canvasElement
      gl = canvas.getContext('webgl2', {
        antialias: settings.antialias,
        alpha: false,
        powerPreference: 'high-performance'
      })

      if (!gl) {
        throw new Error('WebGL2 not supported')
      }

      // 设置 WebGL 状态
      gl.enable(gl.DEPTH_TEST)
      gl.enable(gl.BLEND)
      gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA)

      isInitialized.value = true
      console.log('[3D-GS] Initialized successfully')

      return true
    } catch (e) {
      error.value = `Initialization failed: ${e}`
      console.error('[3D-GS]', e)
      return false
    }
  }

  /**
   * 加载 Splat 数据
   */
  const loadSplat = async (splat: GaussianSplat): Promise<boolean> => {
    if (!isInitialized.value) {
      error.value = 'Renderer not initialized'
      return false
    }

    isLoading.value = true
    error.value = null

    try {
      currentSplat.value = splat

      // 调整相机到合适位置
      const bbox = splat.boundingBox
      const center: [number, number, number] = [
        (bbox.min[0] + bbox.max[0]) / 2,
        (bbox.min[1] + bbox.max[1]) / 2,
        (bbox.min[2] + bbox.max[2]) / 2
      ]
      const size = Math.max(
        bbox.max[0] - bbox.min[0],
        bbox.max[1] - bbox.min[1],
        bbox.max[2] - bbox.min[2]
      )

      camera.target = center
      camera.position = [
        center[0] + size * 0.8,
        center[1] + size * 0.5,
        center[2] + size * 0.8
      ]

      console.log(`[3D-GS] Loaded splat: ${splat.name} with ${splat.pointCount} points`)

      isLoading.value = false
      return true
    } catch (e) {
      error.value = `Failed to load splat: ${e}`
      isLoading.value = false
      return false
    }
  }

  /**
   * 加载 Mock 数据
   */
  const loadMockScene = async (
    scenarioType: 'road' | 'intersection' | 'parking' | 'urban' = 'road',
    pointCount: number = 10000
  ): Promise<boolean> => {
    const splat = generateMockGaussianSplat(scenarioType, pointCount)
    return loadSplat(splat)
  }

  // ==================== 渲染 ====================

  /**
   * 渲染一帧
   */
  const render = () => {
    if (!gl || !canvas || !currentSplat.value) return

    const startTime = performance.now()

    // 清除画布
    gl.clearColor(...settings.backgroundColor, 1.0)
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT)

    // 简化的渲染 (实际应使用着色器)
    // 这里只是概念展示，真实实现需要完整的 WebGL 程序

    const renderTime = performance.now() - startTime
    stats.renderTime = renderTime
    stats.visiblePoints = currentSplat.value.pointCount
  }

  /**
   * 启动渲染循环
   */
  const startRenderLoop = () => {
    if (animationId) return

    let lastTime = performance.now()
    let frameCount = 0

    const loop = (time: number) => {
      frameCount++

      if (time - lastTime >= 1000) {
        stats.fps = frameCount
        frameCount = 0
        lastTime = time
      }

      render()
      animationId = requestAnimationFrame(loop)
    }

    animationId = requestAnimationFrame(loop)
  }

  /**
   * 停止渲染循环
   */
  const stopRenderLoop = () => {
    if (animationId) {
      cancelAnimationFrame(animationId)
      animationId = null
    }
  }

  // ==================== 相机控制 ====================

  /**
   * 设置相机位置
   */
  const setCameraPosition = (position: [number, number, number]) => {
    camera.position = position
  }

  /**
   * 设置相机目标
   */
  const setCameraTarget = (target: [number, number, number]) => {
    camera.target = target
  }

  /**
   * 轨道相机控制
   */
  const orbitCamera = (deltaX: number, deltaY: number) => {
    const [px, py, pz] = camera.position
    const [tx, ty, tz] = camera.target

    // 计算到目标的距离
    const dx = px - tx
    const dy = py - ty
    const dz = pz - tz
    const distance = Math.sqrt(dx * dx + dy * dy + dz * dz)

    // 计算当前角度
    let theta = Math.atan2(dz, dx)
    let phi = Math.acos(dy / distance)

    // 更新角度
    theta += deltaX * 0.01
    phi = Math.max(0.1, Math.min(Math.PI - 0.1, phi + deltaY * 0.01))

    // 计算新位置
    camera.position = [
      tx + distance * Math.sin(phi) * Math.cos(theta),
      ty + distance * Math.cos(phi),
      tz + distance * Math.sin(phi) * Math.sin(theta)
    ]
  }

  /**
   * 缩放相机
   */
  const zoomCamera = (delta: number) => {
    const [px, py, pz] = camera.position
    const [tx, ty, tz] = camera.target

    const dx = px - tx
    const dy = py - ty
    const dz = pz - tz
    const distance = Math.sqrt(dx * dx + dy * dy + dz * dz)

    const newDistance = Math.max(1, distance * (1 - delta * 0.1))
    const scale = newDistance / distance

    camera.position = [
      tx + dx * scale,
      ty + dy * scale,
      tz + dz * scale
    ]
  }

  // ==================== 清理 ====================

  /**
   * 销毁渲染器
   */
  const dispose = () => {
    stopRenderLoop()
    currentSplat.value = null
    isInitialized.value = false
    gl = null
    canvas = null
  }

  onUnmounted(() => {
    dispose()
  })

  return {
    // 状态
    isInitialized,
    isLoading,
    error,
    currentSplat,
    camera,
    settings,
    stats,

    // 初始化
    initialize,
    dispose,

    // 加载数据
    loadSplat,
    loadMockScene,

    // 渲染
    render,
    startRenderLoop,
    stopRenderLoop,

    // 相机控制
    setCameraPosition,
    setCameraTarget,
    orbitCamera,
    zoomCamera,

    // Mock 数据生成
    generateMockGaussianSplat
  }
}

// ======================== 预置场景 ========================

export const PRESET_3DGS_SCENES = [
  { id: 'road', name: '高速公路', description: '直线道路场景，包含车辆和路边植被' },
  { id: 'intersection', name: '十字路口', description: '城市十字路口，包含信号灯和建筑' },
  { id: 'parking', name: '停车场', description: '室外停车场，包含多辆停放车辆' },
  { id: 'urban', name: '城市街区', description: '综合城市场景，包含道路和建筑群' }
]
