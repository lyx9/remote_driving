/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      VideoCompressionService
 * @description 视频压缩服务，用于控制远程驾驶车辆的视频流压缩
 */

import { ref, computed, readonly } from 'vue'

// ======================== 类型定义 ========================

/**
 * 压缩模式
 */
export type CompressionMode = 'none' | 'light' | 'medium' | 'heavy' | 'auto'

/**
 * 压缩配置
 */
export interface CompressionConfig {
  enabled: boolean
  mode: CompressionMode
  targetFps: number
  targetBitrate: number // kbps
  autoAdjust: boolean
}

/**
 * 压缩统计
 */
export interface CompressionStats {
  originalBandwidth: number // Mbps (估算)
  compressedBandwidth: number // Mbps (实际)
  savingsPercent: number // 节省百分比
  currentFps: number
  currentBitrate: number // kbps
  droppedFrames: number
  totalFrames: number
}

/**
 * 网络质量指标
 */
export interface NetworkQuality {
  bandwidth: number // Mbps
  latency: number // ms
  packetLoss: number // %
  jitter: number // ms
}

// ======================== 压缩模式预设 ========================

interface ModePreset {
  fps: number
  bitrate: number // kbps
  description: string
  savingsPercent: number
}

export const COMPRESSION_MODES: Record<Exclude<CompressionMode, 'auto'>, ModePreset> = {
  none: {
    fps: 30,
    bitrate: 2000,
    description: '无压缩 - 最佳画质',
    savingsPercent: 0
  },
  light: {
    fps: 30,
    bitrate: 1000,
    description: '轻度压缩 - 优质画质',
    savingsPercent: 50
  },
  medium: {
    fps: 15,
    bitrate: 1000,
    description: '中度压缩 - 平衡模式',
    savingsPercent: 75
  },
  heavy: {
    fps: 10,
    bitrate: 500,
    description: '重度压缩 - 省流模式',
    savingsPercent: 91.7
  }
}

// ======================== 压缩服务 ========================

export function useVideoCompressionService() {
  // 状态
  const enabled = ref(false)
  const mode = ref<CompressionMode>('none')
  const autoAdjust = ref(false)

  // 统计
  const stats = ref<CompressionStats>({
    originalBandwidth: 0,
    compressedBandwidth: 0,
    savingsPercent: 0,
    currentFps: 30,
    currentBitrate: 2000,
    droppedFrames: 0,
    totalFrames: 0
  })

  // 网络质量
  const networkQuality = ref<NetworkQuality>({
    bandwidth: 10,
    latency: 50,
    packetLoss: 0,
    jitter: 10
  })

  // 计算属性
  const currentConfig = computed((): CompressionConfig => {
    const preset = mode.value === 'auto'
      ? autoSelectMode(networkQuality.value)
      : COMPRESSION_MODES[mode.value as Exclude<CompressionMode, 'auto'>]

    return {
      enabled: enabled.value,
      mode: mode.value,
      targetFps: preset.fps,
      targetBitrate: preset.bitrate,
      autoAdjust: autoAdjust.value
    }
  })

  const currentPreset = computed(() => {
    return mode.value === 'auto'
      ? autoSelectMode(networkQuality.value)
      : COMPRESSION_MODES[mode.value as Exclude<CompressionMode, 'auto'>]
  })

  const bandwidthSavings = computed(() => {
    if (!enabled.value) return 0
    return currentPreset.value.savingsPercent
  })

  // 自动选择压缩模式
  function autoSelectMode(quality: NetworkQuality): ModePreset {
    const { bandwidth, latency, packetLoss } = quality

    // 考虑 4 摄像头场景
    const requiredBandwidth = 8 // 4 cameras × 2 Mbps

    // 网络条件差
    if (bandwidth < requiredBandwidth * 0.3 || packetLoss > 5 || latency > 200) {
      return COMPRESSION_MODES.heavy
    }

    // 网络条件一般
    if (bandwidth < requiredBandwidth * 0.6 || packetLoss > 2 || latency > 150) {
      return COMPRESSION_MODES.medium
    }

    // 网络条件良好
    if (bandwidth < requiredBandwidth * 0.8 || packetLoss > 1) {
      return COMPRESSION_MODES.light
    }

    // 网络条件优秀
    return COMPRESSION_MODES.none
  }

  // 启用压缩
  function enableCompression(compressionMode: CompressionMode = 'light') {
    enabled.value = true
    mode.value = compressionMode
    console.log(`[Compression] Enabled with mode: ${compressionMode}`)
  }

  // 禁用压缩
  function disableCompression() {
    enabled.value = false
    mode.value = 'none'
    console.log('[Compression] Disabled')
  }

  // 切换压缩模式
  function setCompressionMode(compressionMode: CompressionMode) {
    mode.value = compressionMode
    console.log(`[Compression] Mode changed to: ${compressionMode}`)
  }

  // 切换自动调整
  function toggleAutoAdjust() {
    autoAdjust.value = !autoAdjust.value
    if (autoAdjust.value) {
      mode.value = 'auto'
      console.log('[Compression] Auto-adjust enabled')
    } else {
      console.log('[Compression] Auto-adjust disabled')
    }
  }

  // 更新网络质量
  function updateNetworkQuality(quality: Partial<NetworkQuality>) {
    networkQuality.value = {
      ...networkQuality.value,
      ...quality
    }

    // 自动调整模式
    if (autoAdjust.value && mode.value === 'auto') {
      const recommended = autoSelectMode(networkQuality.value)
      console.log(`[Compression] Auto-adjusted to: ${recommended.description}`)
    }
  }

  // 更新统计信息
  function updateStats(newStats: Partial<CompressionStats>) {
    stats.value = {
      ...stats.value,
      ...newStats
    }

    // 计算带宽节省
    if (stats.value.originalBandwidth > 0) {
      stats.value.savingsPercent =
        ((stats.value.originalBandwidth - stats.value.compressedBandwidth) /
         stats.value.originalBandwidth) * 100
    }
  }

  // 估算带宽使用
  function estimateBandwidth(cameraCount: number = 4): {
    uncompressed: number
    compressed: number
    savings: number
  } {
    const preset = currentPreset.value
    const uncompressedPerCamera = 2 // Mbps
    const compressedPerCamera = (preset.bitrate * preset.fps / 30) / 1000

    return {
      uncompressed: uncompressedPerCamera * cameraCount,
      compressed: compressedPerCamera * cameraCount,
      savings: ((uncompressedPerCamera - compressedPerCamera) / uncompressedPerCamera) * 100
    }
  }

  // 获取推荐模式
  function getRecommendedMode(bandwidth: number, cameraCount: number = 4): {
    mode: Exclude<CompressionMode, 'auto'>
    reason: string
  } {
    const requiredBandwidth = cameraCount * 2 // 理想带宽

    if (bandwidth > requiredBandwidth) {
      return { mode: 'none', reason: '网络带宽充足，无需压缩' }
    } else if (bandwidth > requiredBandwidth * 0.6) {
      return { mode: 'light', reason: '网络良好，建议轻度压缩' }
    } else if (bandwidth > requiredBandwidth * 0.3) {
      return { mode: 'medium', reason: '网络一般，建议中度压缩' }
    } else {
      return { mode: 'heavy', reason: '网络较差，建议重度压缩' }
    }
  }

  // 重置统计
  function resetStats() {
    stats.value = {
      originalBandwidth: 0,
      compressedBandwidth: 0,
      savingsPercent: 0,
      currentFps: currentPreset.value.fps,
      currentBitrate: currentPreset.value.bitrate,
      droppedFrames: 0,
      totalFrames: 0
    }
  }

  return {
    // 只读状态
    enabled: readonly(enabled),
    mode: readonly(mode),
    autoAdjust: readonly(autoAdjust),
    currentConfig: readonly(currentConfig),
    currentPreset: readonly(currentPreset),
    stats: readonly(stats),
    networkQuality: readonly(networkQuality),
    bandwidthSavings,

    // 方法
    enableCompression,
    disableCompression,
    setCompressionMode,
    toggleAutoAdjust,
    updateNetworkQuality,
    updateStats,
    estimateBandwidth,
    getRecommendedMode,
    resetStats,

    // 常量
    COMPRESSION_MODES
  }
}

// 创建全局单例
let compressionServiceInstance: ReturnType<typeof useVideoCompressionService> | null = null

export function getCompressionService() {
  if (!compressionServiceInstance) {
    compressionServiceInstance = useVideoCompressionService()
  }
  return compressionServiceInstance
}

export default useVideoCompressionService
