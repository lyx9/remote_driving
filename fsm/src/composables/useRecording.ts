/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      Recording Composable
 * @description 录制功能模块，支持RosBag和视频录制，以及标签管理
 */

import { ref, reactive, computed, onUnmounted } from 'vue'
import { useLocalDatabase, type Recording, type Tag, type Category } from '@/services/database'

// ======================== 类型定义 ========================

export interface RecordingSession {
  id: string
  vehicleId: string
  startTime: Date
  endTime?: Date
  duration: number  // 秒
  status: 'idle' | 'recording' | 'paused' | 'stopping' | 'completed' | 'error'
  types: RecordingType[]
  tags: TagInfo[]
  size: number  // 字节
  error?: string
}

export type RecordingType = 'rosbag' | 'video' | 'telemetry'

export interface TagInfo {
  id?: number
  name: string
  color: string
  timestamp?: number  // 相对于录制开始的毫秒数
  description?: string
}

export interface RecordingConfig {
  autoSave: boolean
  maxDuration: number  // 最大录制时长（秒），0 = 无限制
  maxSize: number  // 最大文件大小（字节），0 = 无限制
  splitSize: number  // 分割大小（字节），0 = 不分割
  compression: 'none' | 'lz4' | 'zstd'
  includeTopics: string[]  // 要录制的 ROS topics
  excludeTopics: string[]  // 排除的 topics
  videoQuality: 'low' | 'medium' | 'high'
  videoCodec: 'h264' | 'h265' | 'vp9'
}

// ======================== 默认配置 ========================

const DEFAULT_CONFIG: RecordingConfig = {
  autoSave: true,
  maxDuration: 3600,  // 1小时
  maxSize: 10 * 1024 * 1024 * 1024,  // 10GB
  splitSize: 1024 * 1024 * 1024,  // 1GB
  compression: 'lz4',
  includeTopics: [
    '/vehicle/status',
    '/vehicle/odom',
    '/perception/objects',
    '/planning/trajectory',
    '/control/command',
    '/camera/front/image_raw',
    '/camera/rear/image_raw',
    '/lidar/points'
  ],
  excludeTopics: [],
  videoQuality: 'high',
  videoCodec: 'h264'
}

// ======================== 预设标签 ========================

export const PRESET_TAGS: TagInfo[] = [
  { name: '紧急情况', color: '#F44336' },
  { name: '异常行为', color: '#FF9800' },
  { name: '有趣场景', color: '#4CAF50' },
  { name: '测试标记', color: '#2196F3' },
  { name: '问题点', color: '#9C27B0' },
  { name: '正常运行', color: '#607D8B' },
  { name: '变道', color: '#00BCD4' },
  { name: '急刹车', color: '#E91E63' },
  { name: '避障', color: '#8BC34A' },
  { name: '人工接管', color: '#FF5722' }
]

// ======================== 录制服务 ========================

export function useRecording(vehicleId: string) {
  // 数据库服务
  const db = useLocalDatabase()

  // 当前会话
  const session = ref<RecordingSession | null>(null)

  // 配置
  const config = reactive<RecordingConfig>({ ...DEFAULT_CONFIG })

  // 状态
  const isRecording = computed(() => session.value?.status === 'recording')
  const isPaused = computed(() => session.value?.status === 'paused')
  const canRecord = computed(() => !session.value || session.value.status === 'idle' || session.value.status === 'completed')

  // 录制历史
  const recordings = ref<Recording[]>([])

  // 可用标签
  const availableTags = ref<TagInfo[]>([...PRESET_TAGS])

  // 计时器
  let durationTimer: number | null = null
  let sizeUpdateTimer: number | null = null

  // ==================== 录制控制 ====================

  /**
   * 开始录制
   */
  const startRecording = async (types: RecordingType[] = ['rosbag', 'video']): Promise<boolean> => {
    if (!canRecord.value) {
      console.warn('[Recording] Already recording or not ready')
      return false
    }

    try {
      const sessionId = generateSessionId()

      session.value = {
        id: sessionId,
        vehicleId,
        startTime: new Date(),
        duration: 0,
        status: 'recording',
        types,
        tags: [],
        size: 0
      }

      // 初始化数据库
      if (!db.isReady.value) {
        await db.initialize()
      }

      // 创建录制记录
      for (const type of types) {
        db.createRecording({
          sessionId,
          vehicleId,
          startTime: new Date(),
          type,
          status: 'recording'
        })
      }

      // 启动计时器
      startTimers()

      // TODO: 发送录制开始命令到车端
      console.log('[Recording] Started recording:', sessionId, types)

      return true
    } catch (e) {
      console.error('[Recording] Failed to start:', e)
      if (session.value) {
        session.value.status = 'error'
        session.value.error = String(e)
      }
      return false
    }
  }

  /**
   * 停止录制
   */
  const stopRecording = async (): Promise<boolean> => {
    if (!session.value || session.value.status === 'idle') {
      return false
    }

    try {
      session.value.status = 'stopping'

      // 停止计时器
      stopTimers()

      // 更新数据库记录
      const endTime = new Date()
      session.value.endTime = endTime

      // 查找并更新所有相关记录
      const sessionRecordings = await db.searchRecordings({
        vehicleId,
        status: 'recording'
      })

      for (const rec of sessionRecordings) {
        if (rec.sessionId === session.value.id) {
          db.updateRecording(rec.id, {
            endTime,
            durationSeconds: session.value.duration,
            status: 'completed'
          })

          // 保存标签
          for (const tag of session.value.tags) {
            db.addTag({
              recordingId: rec.id,
              name: tag.name,
              color: tag.color,
              timestampMs: tag.timestamp,
              description: tag.description
            })
          }
        }
      }

      session.value.status = 'completed'

      // TODO: 发送停止命令到车端
      console.log('[Recording] Stopped recording:', session.value.id)

      return true
    } catch (e) {
      console.error('[Recording] Failed to stop:', e)
      if (session.value) {
        session.value.status = 'error'
        session.value.error = String(e)
      }
      return false
    }
  }

  /**
   * 暂停录制
   */
  const pauseRecording = (): boolean => {
    if (!session.value || session.value.status !== 'recording') {
      return false
    }

    session.value.status = 'paused'
    stopTimers()

    // TODO: 发送暂停命令到车端
    console.log('[Recording] Paused')
    return true
  }

  /**
   * 恢复录制
   */
  const resumeRecording = (): boolean => {
    if (!session.value || session.value.status !== 'paused') {
      return false
    }

    session.value.status = 'recording'
    startTimers()

    // TODO: 发送恢复命令到车端
    console.log('[Recording] Resumed')
    return true
  }

  // ==================== 标签管理 ====================

  /**
   * 添加标签到当前录制
   */
  const addTag = (tag: TagInfo): boolean => {
    if (!session.value || session.value.status === 'idle') {
      console.warn('[Recording] No active recording session')
      return false
    }

    const tagWithTimestamp: TagInfo = {
      ...tag,
      timestamp: session.value.duration * 1000  // 当前时间点
    }

    session.value.tags.push(tagWithTimestamp)
    console.log('[Recording] Added tag:', tagWithTimestamp)

    return true
  }

  /**
   * 快速添加预设标签
   */
  const addPresetTag = (tagName: string): boolean => {
    const preset = PRESET_TAGS.find(t => t.name === tagName)
    if (preset) {
      return addTag(preset)
    }
    return false
  }

  /**
   * 移除标签
   */
  const removeTag = (index: number): boolean => {
    if (!session.value || index < 0 || index >= session.value.tags.length) {
      return false
    }

    session.value.tags.splice(index, 1)
    return true
  }

  /**
   * 创建自定义标签
   */
  const createCustomTag = (name: string, color: string, description?: string): TagInfo => {
    const tag: TagInfo = { name, color, description }
    availableTags.value.push(tag)
    return tag
  }

  // ==================== 录制历史 ====================

  /**
   * 加载录制历史
   */
  const loadRecordings = async (query?: {
    startDate?: Date
    endDate?: Date
    types?: RecordingType[]
    tags?: string[]
    limit?: number
  }): Promise<Recording[]> => {
    if (!db.isReady.value) {
      await db.initialize()
    }

    const result = db.searchRecordings({
      vehicleId,
      startDate: query?.startDate,
      endDate: query?.endDate,
      type: query?.types?.[0],
      tags: query?.tags,
      limit: query?.limit || 50
    })

    recordings.value = result
    return result
  }

  /**
   * 获取录制详情
   */
  const getRecordingDetails = (id: number): {
    recording: Recording | null
    tags: Tag[]
    categories: Category[]
  } => {
    const recording = db.getRecording(id)
    if (!recording) {
      return { recording: null, tags: [], categories: [] }
    }

    return {
      recording,
      tags: db.getTags(id),
      categories: db.getRecordingCategories(id)
    }
  }

  /**
   * 删除录制
   */
  const deleteRecording = (id: number): boolean => {
    try {
      db.deleteRecording(id)
      recordings.value = recordings.value.filter(r => r.id !== id)
      return true
    } catch (e) {
      console.error('[Recording] Failed to delete:', e)
      return false
    }
  }

  // ==================== 导出功能 ====================

  /**
   * 导出为 JSON
   */
  const exportToJson = (recordingIds?: number[]): string => {
    return db.exportToJson(recordingIds)
  }

  /**
   * 导出为 CSV
   */
  const exportToCsv = (recordingIds?: number[]): string => {
    return db.exportToCsv(recordingIds)
  }

  /**
   * 下载导出文件
   */
  const downloadExport = (format: 'json' | 'csv', recordingIds?: number[]) => {
    const content = format === 'json'
      ? exportToJson(recordingIds)
      : exportToCsv(recordingIds)

    const blob = new Blob([content], {
      type: format === 'json' ? 'application/json' : 'text/csv'
    })

    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `fsm-recordings-${new Date().toISOString().slice(0, 10)}.${format}`
    a.click()
    URL.revokeObjectURL(url)
  }

  // ==================== 统计信息 ====================

  /**
   * 获取统计信息
   */
  const getStats = () => {
    return db.getStats()
  }

  // ==================== 内部方法 ====================

  /**
   * 生成会话 ID
   */
  const generateSessionId = (): string => {
    const timestamp = Date.now().toString(36)
    const random = Math.random().toString(36).substring(2, 8)
    return `${vehicleId}-${timestamp}-${random}`
  }

  /**
   * 启动计时器
   */
  const startTimers = () => {
    // 持续时间计时器
    durationTimer = window.setInterval(() => {
      if (session.value && session.value.status === 'recording') {
        session.value.duration++

        // 检查最大时长
        if (config.maxDuration > 0 && session.value.duration >= config.maxDuration) {
          console.log('[Recording] Max duration reached, stopping...')
          stopRecording()
        }
      }
    }, 1000)

    // 文件大小更新计时器 (模拟)
    sizeUpdateTimer = window.setInterval(() => {
      if (session.value && session.value.status === 'recording') {
        // 模拟：每秒约 5MB (高质量视频 + RosBag)
        session.value.size += 5 * 1024 * 1024

        // 检查最大大小
        if (config.maxSize > 0 && session.value.size >= config.maxSize) {
          console.log('[Recording] Max size reached, stopping...')
          stopRecording()
        }
      }
    }, 1000)
  }

  /**
   * 停止计时器
   */
  const stopTimers = () => {
    if (durationTimer) {
      clearInterval(durationTimer)
      durationTimer = null
    }
    if (sizeUpdateTimer) {
      clearInterval(sizeUpdateTimer)
      sizeUpdateTimer = null
    }
  }

  // ==================== 格式化工具 ====================

  /**
   * 格式化时长
   */
  const formatDuration = (seconds: number): string => {
    const h = Math.floor(seconds / 3600)
    const m = Math.floor((seconds % 3600) / 60)
    const s = seconds % 60
    return h > 0
      ? `${h}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`
      : `${m}:${s.toString().padStart(2, '0')}`
  }

  /**
   * 格式化文件大小
   */
  const formatSize = (bytes: number): string => {
    if (bytes < 1024) return `${bytes} B`
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`
    if (bytes < 1024 * 1024 * 1024) return `${(bytes / 1024 / 1024).toFixed(1)} MB`
    return `${(bytes / 1024 / 1024 / 1024).toFixed(2)} GB`
  }

  // 清理
  onUnmounted(() => {
    stopTimers()
    if (session.value?.status === 'recording') {
      stopRecording()
    }
  })

  return {
    // 状态
    session,
    config,
    isRecording,
    isPaused,
    canRecord,
    recordings,
    availableTags,

    // 录制控制
    startRecording,
    stopRecording,
    pauseRecording,
    resumeRecording,

    // 标签管理
    addTag,
    addPresetTag,
    removeTag,
    createCustomTag,

    // 历史记录
    loadRecordings,
    getRecordingDetails,
    deleteRecording,

    // 导出
    exportToJson,
    exportToCsv,
    downloadExport,

    // 统计
    getStats,

    // 工具
    formatDuration,
    formatSize,

    // 预设
    PRESET_TAGS
  }
}

// ======================== Mock 录制服务 ========================

/**
 * Mock 录制服务 (演示用)
 */
export function useMockRecording(vehicleId: string) {
  const session = ref<RecordingSession | null>(null)
  const isRecording = computed(() => session.value?.status === 'recording')
  const isPaused = computed(() => session.value?.status === 'paused')

  let timer: number | null = null

  const startRecording = async (types: RecordingType[] = ['rosbag', 'video']): Promise<boolean> => {
    session.value = {
      id: `mock-${Date.now()}`,
      vehicleId,
      startTime: new Date(),
      duration: 0,
      status: 'recording',
      types,
      tags: [],
      size: 0
    }

    timer = window.setInterval(() => {
      if (session.value && session.value.status === 'recording') {
        session.value.duration++
        session.value.size += 5 * 1024 * 1024
      }
    }, 1000)

    return true
  }

  const stopRecording = async (): Promise<boolean> => {
    if (timer) {
      clearInterval(timer)
      timer = null
    }
    if (session.value) {
      session.value.status = 'completed'
      session.value.endTime = new Date()
    }
    return true
  }

  const pauseRecording = (): boolean => {
    if (session.value) session.value.status = 'paused'
    return true
  }

  const resumeRecording = (): boolean => {
    if (session.value) session.value.status = 'recording'
    return true
  }

  const addTag = (tag: TagInfo): boolean => {
    if (session.value) {
      session.value.tags.push({
        ...tag,
        timestamp: session.value.duration * 1000
      })
      return true
    }
    return false
  }

  const formatDuration = (seconds: number): string => {
    const m = Math.floor(seconds / 60)
    const s = seconds % 60
    return `${m}:${s.toString().padStart(2, '0')}`
  }

  const formatSize = (bytes: number): string => {
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`
    if (bytes < 1024 * 1024 * 1024) return `${(bytes / 1024 / 1024).toFixed(1)} MB`
    return `${(bytes / 1024 / 1024 / 1024).toFixed(2)} GB`
  }

  onUnmounted(() => {
    if (timer) clearInterval(timer)
  })

  return {
    session,
    isRecording,
    isPaused,
    startRecording,
    stopRecording,
    pauseRecording,
    resumeRecording,
    addTag,
    formatDuration,
    formatSize,
    availableTags: ref([...PRESET_TAGS]),
    PRESET_TAGS
  }
}
