/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      RosBag to 3D Gaussian Splatting Converter
 * @description 将 RosBag 点云数据转换为 3DGS 格式用于渲染
 */

import type { PointCloudPoint } from './rosbagDb3ParserOptimized'

// ======================== 类型定义 ========================

/**
 * 3DGS 点数据结构
 */
export interface GaussianSplat {
  position: [number, number, number]
  scale: [number, number, number]
  rotation: [number, number, number, number] // quaternion
  color: [number, number, number]
  opacity: number
}

/**
 * 3DGS 场景数据
 */
export interface GaussianScene {
  splats: GaussianSplat[]
  bounds: {
    min: [number, number, number]
    max: [number, number, number]
  }
  metadata: {
    totalPoints: number
    timestamp: number
    source: string
  }
}

/**
 * 转换配置
 */
export interface ConversionConfig {
  voxelSize: number // 体素化大小，用于降采样
  minIntensity: number // 最小强度阈值
  maxDistance: number // 最大距离过滤
  colorMapping: 'intensity' | 'height' | 'distance'
  smoothing: boolean
}

// ======================== 默认配置 ========================

const DEFAULT_CONFIG: ConversionConfig = {
  voxelSize: 0.1, // 10cm 体素
  minIntensity: 0,
  maxDistance: 100, // 100m
  colorMapping: 'intensity',
  smoothing: true
}

// ======================== 转换函数 ========================

/**
 * 将 PointCloudPoints 转换为 GaussianSplats
 */
export function convertPointCloudToGaussian(
  points: PointCloudPoint[],
  config: Partial<ConversionConfig> = {}
): GaussianScene {
  const cfg = { ...DEFAULT_CONFIG, ...config }

  // 过滤点云
  const filteredPoints = filterPoints(points, cfg)

  // 体素化降采样
  const downsampledPoints = voxelDownsample(filteredPoints, cfg.voxelSize)

  // 转换为 Gaussian Splats
  const splats = downsampledPoints.map(point => pointToGaussian(point, cfg))

  // 计算边界
  const bounds = calculateBounds(downsampledPoints)

  return {
    splats,
    bounds,
    metadata: {
      totalPoints: splats.length,
      timestamp: Date.now(),
      source: 'rosbag_pointcloud'
    }
  }
}

/**
 * 过滤无效点
 */
function filterPoints(
  points: PointCloudPoint[],
  config: ConversionConfig
): PointCloudPoint[] {
  return points.filter(p => {
    // 过滤无效坐标
    if (!isFinite(p.x) || !isFinite(p.y) || !isFinite(p.z)) return false

    // 过滤原点
    if (p.x === 0 && p.y === 0 && p.z === 0) return false

    // 过滤距离过远的点
    const distance = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z)
    if (distance > config.maxDistance) return false

    // 过滤强度过低的点
    if (p.intensity !== undefined && p.intensity < config.minIntensity) return false

    return true
  })
}

/**
 * 体素化降采样
 */
function voxelDownsample(
  points: PointCloudPoint[],
  voxelSize: number
): PointCloudPoint[] {
  const voxelMap = new Map<string, PointCloudPoint[]>()

  // 将点分配到体素
  for (const point of points) {
    const vx = Math.floor(point.x / voxelSize)
    const vy = Math.floor(point.y / voxelSize)
    const vz = Math.floor(point.z / voxelSize)
    const key = `${vx},${vy},${vz}`

    if (!voxelMap.has(key)) {
      voxelMap.set(key, [])
    }
    voxelMap.get(key)!.push(point)
  }

  // 对每个体素计算平均点
  const downsampledPoints: PointCloudPoint[] = []

  for (const voxelPoints of voxelMap.values()) {
    if (voxelPoints.length === 0) continue

    const avgPoint: PointCloudPoint = {
      x: 0,
      y: 0,
      z: 0,
      intensity: 0
    }

    for (const p of voxelPoints) {
      avgPoint.x += p.x
      avgPoint.y += p.y
      avgPoint.z += p.z
      avgPoint.intensity = (avgPoint.intensity || 0) + (p.intensity || 0)
    }

    const count = voxelPoints.length
    avgPoint.x /= count
    avgPoint.y /= count
    avgPoint.z /= count
    avgPoint.intensity = (avgPoint.intensity || 0) / count

    downsampledPoints.push(avgPoint)
  }

  return downsampledPoints
}

/**
 * 将单个点转换为 Gaussian Splat
 */
function pointToGaussian(
  point: PointCloudPoint,
  config: ConversionConfig
): GaussianSplat {
  // 位置
  const position: [number, number, number] = [point.x, point.y, point.z]

  // 尺度（基于体素大小）
  const scale: [number, number, number] = [
    config.voxelSize,
    config.voxelSize,
    config.voxelSize
  ]

  // 旋转（默认无旋转）
  const rotation: [number, number, number, number] = [0, 0, 0, 1]

  // 颜色映射
  const color = mapColor(point, config)

  // 不透明度（基于强度）
  const opacity = point.intensity !== undefined
    ? Math.min(1.0, point.intensity / 255)
    : 0.8

  return {
    position,
    scale,
    rotation,
    color,
    opacity
  }
}

/**
 * 颜色映射
 */
function mapColor(
  point: PointCloudPoint,
  config: ConversionConfig
): [number, number, number] {
  switch (config.colorMapping) {
    case 'intensity': {
      // 基于强度的灰度映射
      const intensity = point.intensity !== undefined ? point.intensity / 255 : 0.5
      return [intensity, intensity, intensity]
    }

    case 'height': {
      // 基于高度的彩色映射
      const minHeight = -2
      const maxHeight = 5
      const normalizedHeight = (point.z - minHeight) / (maxHeight - minHeight)
      return heightToColor(normalizedHeight)
    }

    case 'distance': {
      // 基于距离的彩色映射
      const distance = Math.sqrt(point.x * point.x + point.y * point.y + point.z * point.z)
      const normalizedDistance = Math.min(1.0, distance / 50)
      return distanceToColor(normalizedDistance)
    }

    default:
      return [0.5, 0.5, 0.5]
  }
}

/**
 * 高度到颜色的映射 (蓝 -> 绿 -> 红)
 */
function heightToColor(t: number): [number, number, number] {
  t = Math.max(0, Math.min(1, t))

  if (t < 0.5) {
    // 蓝色到绿色
    const s = t * 2
    return [0, s, 1 - s]
  } else {
    // 绿色到红色
    const s = (t - 0.5) * 2
    return [s, 1 - s, 0]
  }
}

/**
 * 距离到颜色的映射 (绿 -> 黄 -> 红)
 */
function distanceToColor(t: number): [number, number, number] {
  t = Math.max(0, Math.min(1, t))

  if (t < 0.5) {
    // 绿色到黄色
    const s = t * 2
    return [s, 1, 0]
  } else {
    // 黄色到红色
    const s = (t - 0.5) * 2
    return [1, 1 - s, 0]
  }
}

/**
 * 计算点云边界
 */
function calculateBounds(
  points: PointCloudPoint[]
): {
  min: [number, number, number]
  max: [number, number, number]
} {
  if (points.length === 0) {
    return {
      min: [0, 0, 0],
      max: [0, 0, 0]
    }
  }

  const min: [number, number, number] = [
    Number.POSITIVE_INFINITY,
    Number.POSITIVE_INFINITY,
    Number.POSITIVE_INFINITY
  ]

  const max: [number, number, number] = [
    Number.NEGATIVE_INFINITY,
    Number.NEGATIVE_INFINITY,
    Number.NEGATIVE_INFINITY
  ]

  for (const point of points) {
    min[0] = Math.min(min[0], point.x)
    min[1] = Math.min(min[1], point.y)
    min[2] = Math.min(min[2], point.z)

    max[0] = Math.max(max[0], point.x)
    max[1] = Math.max(max[1], point.y)
    max[2] = Math.max(max[2], point.z)
  }

  return { min, max }
}

// ======================== 批量转换 ========================

/**
 * 从多帧点云创建 3DGS 场景
 */
export function createGaussianSceneFromFrames(
  frames: PointCloudPoint[][],
  config: Partial<ConversionConfig> = {}
): GaussianScene {
  // 合并所有帧
  const allPoints: PointCloudPoint[] = []
  for (const frame of frames) {
    allPoints.push(...frame)
  }

  // 转换
  return convertPointCloudToGaussian(allPoints, config)
}

/**
 * 导出 3DGS 场景为 PLY 格式
 */
export function exportToPLY(scene: GaussianScene): string {
  const lines: string[] = []

  // PLY 文件头
  lines.push('ply')
  lines.push('format ascii 1.0')
  lines.push(`element vertex ${scene.splats.length}`)
  lines.push('property float x')
  lines.push('property float y')
  lines.push('property float z')
  lines.push('property float nx')
  lines.push('property float ny')
  lines.push('property float nz')
  lines.push('property uchar red')
  lines.push('property uchar green')
  lines.push('property uchar blue')
  lines.push('property float opacity')
  lines.push('property float scale_0')
  lines.push('property float scale_1')
  lines.push('property float scale_2')
  lines.push('property float rot_0')
  lines.push('property float rot_1')
  lines.push('property float rot_2')
  lines.push('property float rot_3')
  lines.push('end_header')

  // 数据
  for (const splat of scene.splats) {
    const [x, y, z] = splat.position
    const [r, g, b] = splat.color
    const [sx, sy, sz] = splat.scale
    const [rx, ry, rz, rw] = splat.rotation

    // 法向量（默认指向上）
    const nx = 0, ny = 0, nz = 1

    // 颜色转换为 0-255
    const red = Math.floor(r * 255)
    const green = Math.floor(g * 255)
    const blue = Math.floor(b * 255)

    lines.push(
      `${x} ${y} ${z} ${nx} ${ny} ${nz} ${red} ${green} ${blue} ${splat.opacity} ${sx} ${sy} ${sz} ${rx} ${ry} ${rz} ${rw}`
    )
  }

  return lines.join('\n')
}

/**
 * 导出场景为 JSON 格式（用于 Web 渲染）
 */
export function exportToJSON(scene: GaussianScene): string {
  return JSON.stringify(scene, null, 2)
}

export default {
  convertPointCloudToGaussian,
  createGaussianSceneFromFrames,
  exportToPLY,
  exportToJSON
}
