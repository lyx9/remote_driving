import logger from '@/utils/logger'
/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      RosBag DB3 Parser
 * @description 解析 ROS2 db3 格式的 RosBag 文件
 *              支持 SQLite3 存储格式，CDR 序列化
 */

// @ts-ignore - sql.js lacks type definitions
import initSqlJs, { Database } from 'sql.js'
import { CdrReader } from '@foxglove/cdr'
import { ref, shallowRef } from 'vue'
import type {
  RosBagInfo,
  TopicInfo,
  RosMessage,
  PointCloud2
} from './rosbagPlayer'

// ======================== 类型定义 ========================

/**
 * ROS2 db3 Topic 表结构
 */
interface Db3Topic {
  id: number
  name: string
  type: string
  serialization_format: string
  offered_qos_profiles: string
}


/**
 * NavSatFix 消息类型
 */
export interface NavSatFix {
  header: {
    stamp: { sec: number; nanosec: number }
    frame_id: string
  }
  status: {
    status: number
    service: number
  }
  latitude: number
  longitude: number
  altitude: number
  position_covariance: number[]
  position_covariance_type: number
}

/**
 * 解析后的点云点
 */
export interface PointCloudPoint {
  x: number
  y: number
  z: number
  intensity?: number
  ring?: number
}

// ======================== CDR 消息解析器 ========================

/**
 * 解析 ROS2 Header
 */
function parseHeader(reader: CdrReader): { stamp: { sec: number; nanosec: number }; frame_id: string } {
  const sec = reader.int32()
  const nanosec = reader.uint32()
  const frame_id = reader.string()
  return {
    stamp: { sec, nanosec },
    frame_id
  }
}

/**
 * 解析 NavSatFix 消息
 */
function parseNavSatFix(data: Uint8Array): NavSatFix {
  const reader = new CdrReader(data)

  const header = parseHeader(reader)
  const status = {
    status: reader.int8(),
    service: reader.uint16()
  }
  const latitude = reader.float64()
  const longitude = reader.float64()
  const altitude = reader.float64()

  const position_covariance: number[] = []
  for (let i = 0; i < 9; i++) {
    position_covariance.push(reader.float64())
  }
  const position_covariance_type = reader.uint8()

  return {
    header,
    status,
    latitude,
    longitude,
    altitude,
    position_covariance,
    position_covariance_type
  }
}

/**
 * 解析 PointCloud2 消息
 */
function parsePointCloud2(data: Uint8Array): PointCloud2 {
  const reader = new CdrReader(data)

  const header = parseHeader(reader)
  const height = reader.uint32()
  const width = reader.uint32()

  // 解析 fields 数组
  const fieldsLength = reader.sequenceLength()
  const fields: PointCloud2['fields'] = []
  for (let i = 0; i < fieldsLength; i++) {
    const name = reader.string()
    const offset = reader.uint32()
    const datatype = reader.uint8()
    const count = reader.uint32()
    fields.push({ name, offset, datatype, count })
  }

  const is_bigendian = reader.uint8() !== 0
  const point_step = reader.uint32()
  const row_step = reader.uint32()

  // 读取点云数据
  const dataLength = reader.sequenceLength()
  const pointData = new Uint8Array(dataLength)
  for (let i = 0; i < dataLength; i++) {
    pointData[i] = reader.uint8()
  }

  const is_dense = reader.uint8() !== 0

  return {
    header,
    height,
    width,
    fields,
    is_bigendian,
    point_step,
    row_step,
    data: pointData,
    is_dense
  }
}

/**
 * 从 PointCloud2 提取点云点
 */
export function extractPointCloudPoints(pc2: PointCloud2, maxPoints: number = 50000): PointCloudPoint[] {
  const points: PointCloudPoint[] = []
  const { fields, point_step, data, width, height } = pc2

  // 找到 x, y, z, intensity 字段的偏移量
  let xOffset = -1, yOffset = -1, zOffset = -1, intensityOffset = -1, ringOffset = -1

  for (const field of fields) {
    switch (field.name) {
      case 'x': xOffset = field.offset; break
      case 'y': yOffset = field.offset; break
      case 'z': zOffset = field.offset; break
      case 'intensity': intensityOffset = field.offset; break
      case 'ring': ringOffset = field.offset; break
    }
  }

  if (xOffset < 0 || yOffset < 0 || zOffset < 0) {
    logger.warn('PointCloud2 missing x/y/z fields', 'ros')
    return points
  }

  const totalPoints = width * height
  const step = Math.max(1, Math.floor(totalPoints / maxPoints))
  const dataView = new DataView(data.buffer, data.byteOffset, data.byteLength)
  const isLittleEndian = !pc2.is_bigendian

  for (let i = 0; i < totalPoints; i += step) {
    const offset = i * point_step

    if (offset + point_step > data.length) break

    const x = dataView.getFloat32(offset + xOffset, isLittleEndian)
    const y = dataView.getFloat32(offset + yOffset, isLittleEndian)
    const z = dataView.getFloat32(offset + zOffset, isLittleEndian)

    // 跳过无效点
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue
    if (x === 0 && y === 0 && z === 0) continue

    const point: PointCloudPoint = { x, y, z }

    if (intensityOffset >= 0) {
      point.intensity = dataView.getFloat32(offset + intensityOffset, isLittleEndian)
    }

    if (ringOffset >= 0) {
      point.ring = dataView.getUint16(offset + ringOffset, isLittleEndian)
    }

    points.push(point)
  }

  return points
}

// ======================== DB3 解析服务 ========================

/**
 * 使用 RosBag DB3 解析器
 */
export function useRosBagDb3Parser() {
  const isLoading = ref(false)
  const error = ref<string | null>(null)
  const bagInfo = shallowRef<RosBagInfo | null>(null)
  const topics = shallowRef<Db3Topic[]>([])
  const db = shallowRef<Database | null>(null)

  // 消息缓存
  const messageCache = new Map<string, RosMessage[]>()

  /**
   * 加载 db3 文件
   */
  const loadDb3File = async (file: File): Promise<boolean> => {
    isLoading.value = true
    error.value = null
    messageCache.clear()

    try {
      // 初始化 sql.js
      const SQL = await initSqlJs({
        locateFile: (file: string) => `https://sql.js.org/dist/${file}`
      })

      // 读取文件内容
      const arrayBuffer = await file.arrayBuffer()
      const uint8Array = new Uint8Array(arrayBuffer)

      // 创建数据库
      db.value = new SQL.Database(uint8Array)

      // 读取 topics
      const topicsResult = db.value.exec('SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics')
      if (topicsResult.length > 0) {
        topics.value = topicsResult[0].values.map((row: unknown[]) => ({
          id: row[0] as number,
          name: row[1] as string,
          type: row[2] as string,
          serialization_format: row[3] as string,
          offered_qos_profiles: row[4] as string
        }))
      }

      // 获取消息统计
      const statsResult = db.value.exec(`
        SELECT
          topic_id,
          COUNT(*) as count,
          MIN(timestamp) as min_ts,
          MAX(timestamp) as max_ts
        FROM messages
        GROUP BY topic_id
      `)

      const topicStats = new Map<number, { count: number; minTs: bigint; maxTs: bigint }>()
      let globalMinTs = BigInt(Number.MAX_SAFE_INTEGER)
      let globalMaxTs = BigInt(0)
      let totalMessages = 0

      if (statsResult.length > 0) {
        for (const row of statsResult[0].values) {
          const topicId = row[0] as number
          const count = row[1] as number
          const minTs = BigInt(row[2] as number)
          const maxTs = BigInt(row[3] as number)

          topicStats.set(topicId, { count, minTs, maxTs })
          totalMessages += count

          if (minTs < globalMinTs) globalMinTs = minTs
          if (maxTs > globalMaxTs) globalMaxTs = maxTs
        }
      }

      // 构建 topic 信息
      const topicInfos: TopicInfo[] = topics.value.map(topic => {
        const stats = topicStats.get(topic.id)
        const count = stats?.count || 0
        const duration = stats
          ? Number(stats.maxTs - stats.minTs) / 1e9
          : 0
        const frequency = duration > 0 ? count / duration : 0

        return {
          name: topic.name,
          type: topic.type,
          messageCount: count,
          frequency: Math.round(frequency * 10) / 10
        }
      })

      // 计算时长
      const durationNs = globalMaxTs - globalMinTs
      const durationSec = Number(durationNs) / 1e9

      // 构建 bag 信息
      bagInfo.value = {
        id: `db3_${Date.now()}`,
        name: file.name,
        path: file.name,
        format: 'db3',
        size: file.size,
        duration: durationSec,
        startTime: Number(globalMinTs),
        endTime: Number(globalMaxTs),
        messageCount: totalMessages,
        topics: topicInfos,
        metadata: {
          date: new Date(Number(globalMinTs) / 1e6).toISOString()
        }
      }

      logger.info(`[DB3 Parser] Loaded: ${file.name}`, 'ros')
      logger.info(`[DB3 Parser] Topics: ${topics.value.length}`, 'ros')
      logger.info(`[DB3 Parser] Messages: ${totalMessages}`, 'ros')
      logger.info(`[DB3 Parser] Duration: ${durationSec.toFixed(2)}s`, 'ros')

      isLoading.value = false
      return true
    } catch (e) {
      error.value = `Failed to load db3: ${e}`
      logger.error('[DB3 Parser] Error:', e, 'ros')
      isLoading.value = false
      return false
    }
  }

  /**
   * 获取指定时间范围的消息
   */
  const getMessagesInRange = (
    topicName: string,
    startTime: number,
    endTime: number
  ): RosMessage[] => {
    if (!db.value) return []

    const topic = topics.value.find(t => t.name === topicName)
    if (!topic) return []

    try {
      const result = db.value.exec(`
        SELECT id, timestamp, data
        FROM messages
        WHERE topic_id = ${topic.id}
          AND timestamp >= ${startTime}
          AND timestamp <= ${endTime}
        ORDER BY timestamp
        LIMIT 100
      `)

      if (result.length === 0) return []

      return result[0].values.map((row: unknown[]) => {
        const timestamp = Number(row[1])
        const data = row[2] as Uint8Array

        return {
          topic: topicName,
          timestamp,
          data: parseMessageData(topic.type, data),
          type: topic.type
        }
      })
    } catch (e) {
      logger.error(`[DB3 Parser] Error getting messages for ${topicName}:`, e, 'ros')
      return []
    }
  }

  /**
   * 获取指定时间点最近的消息
   */
  const getMessageAtTime = (topicName: string, timestamp: number): RosMessage | null => {
    if (!db.value) return null

    const topic = topics.value.find(t => t.name === topicName)
    if (!topic) return null

    try {
      // 获取时间戳之前最近的消息
      const result = db.value.exec(`
        SELECT id, timestamp, data
        FROM messages
        WHERE topic_id = ${topic.id}
          AND timestamp <= ${timestamp}
        ORDER BY timestamp DESC
        LIMIT 1
      `)

      if (result.length === 0 || result[0].values.length === 0) return null

      const row = result[0].values[0]
      const msgTimestamp = Number(row[1])
      const data = row[2] as Uint8Array

      return {
        topic: topicName,
        timestamp: msgTimestamp,
        data: parseMessageData(topic.type, data),
        type: topic.type
      }
    } catch (e) {
      logger.error(`[DB3 Parser] Error getting message for ${topicName}:`, e, 'ros')
      return null
    }
  }

  /**
   * 预加载所有消息到缓存
   */
  const preloadAllMessages = async (topicName: string): Promise<void> => {
    if (!db.value || messageCache.has(topicName)) return

    const topic = topics.value.find(t => t.name === topicName)
    if (!topic) return

    try {
      const result = db.value.exec(`
        SELECT id, timestamp, data
        FROM messages
        WHERE topic_id = ${topic.id}
        ORDER BY timestamp
      `)

      if (result.length === 0) return

      const messages: RosMessage[] = result[0].values.map((row: unknown[]) => {
        const timestamp = Number(row[1])
        const data = row[2] as Uint8Array

        return {
          topic: topicName,
          timestamp,
          data: parseMessageData(topic.type, data),
          type: topic.type
        }
      })

      messageCache.set(topicName, messages)
      logger.info(`[DB3 Parser] Preloaded ${messages.length} messages for ${topicName}`, 'ros')
    } catch (e) {
      logger.error(`[DB3 Parser] Error preloading messages for ${topicName}:`, e, 'ros')
    }
  }

  /**
   * 从缓存获取消息
   */
  const getCachedMessageAtTime = (topicName: string, timestamp: number): RosMessage | null => {
    const messages = messageCache.get(topicName)
    if (!messages || messages.length === 0) return null

    // 二分查找
    let left = 0
    let right = messages.length - 1

    while (left < right) {
      const mid = Math.floor((left + right + 1) / 2)
      if (messages[mid].timestamp <= timestamp) {
        left = mid
      } else {
        right = mid - 1
      }
    }

    return messages[left]
  }

  /**
   * 解析消息数据
   */
  const parseMessageData = (type: string, data: Uint8Array): any => {
    try {
      switch (type) {
        case 'sensor_msgs/msg/NavSatFix':
          return parseNavSatFix(data)
        case 'sensor_msgs/msg/PointCloud2':
          return parsePointCloud2(data)
        default:
          // 返回原始数据
          return { raw: data }
      }
    } catch (e) {
      logger.error(`[DB3 Parser] Error parsing message of type ${type}:`, e, 'ros')
      return { raw: data, error: String(e) }
    }
  }

  /**
   * 获取所有 GPS 轨迹点
   */
  const getGpsTrajectory = (): Array<{ lat: number; lng: number; alt: number; timestamp: number }> => {
    if (!db.value) return []

    const topic = topics.value.find(t => t.type === 'sensor_msgs/msg/NavSatFix')
    if (!topic) return []

    try {
      const result = db.value.exec(`
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = ${topic.id}
        ORDER BY timestamp
      `)

      if (result.length === 0) return []

      return result[0].values.map((row: unknown[]) => {
        const timestamp = Number(row[0])
        const data = row[1] as Uint8Array
        const navSatFix = parseNavSatFix(data)

        return {
          lat: navSatFix.latitude,
          lng: navSatFix.longitude,
          alt: navSatFix.altitude,
          timestamp
        }
      }).filter((p: { lat: number; lng: number; alt: number; timestamp: number }) => p.lat !== 0 && p.lng !== 0)
    } catch (e) {
      logger.error('[DB3 Parser] Error getting GPS trajectory:', e, 'ros')
      return []
    }
  }

  /**
   * 清理资源
   */
  const dispose = () => {
    if (db.value) {
      db.value.close()
      db.value = null
    }
    bagInfo.value = null
    topics.value = []
    messageCache.clear()
  }

  return {
    // 状态
    isLoading,
    error,
    bagInfo,
    topics,

    // 方法
    loadDb3File,
    getMessagesInRange,
    getMessageAtTime,
    preloadAllMessages,
    getCachedMessageAtTime,
    getGpsTrajectory,
    extractPointCloudPoints,
    dispose
  }
}

/**
 * 从 metadata.yaml 解析 RosBag 信息
 */
export async function parseMetadataYaml(yamlContent: string): Promise<Partial<RosBagInfo>> {
  const lines = yamlContent.split('\n')
  const info: Partial<RosBagInfo> = {
    topics: []
  }

  let currentSection = ''
  let currentTopic: Partial<TopicInfo> | null = null

  for (const line of lines) {
    const trimmed = line.trim()

    if (trimmed.startsWith('duration:')) {
      currentSection = 'duration'
    } else if (trimmed.startsWith('nanoseconds:') && currentSection === 'duration') {
      const ns = parseInt(trimmed.split(':')[1].trim())
      info.duration = ns / 1e9
    } else if (trimmed.startsWith('starting_time:')) {
      currentSection = 'starting_time'
    } else if (trimmed.startsWith('nanoseconds_since_epoch:') && currentSection === 'starting_time') {
      info.startTime = parseInt(trimmed.split(':')[1].trim())
    } else if (trimmed.startsWith('message_count:') && !currentTopic) {
      info.messageCount = parseInt(trimmed.split(':')[1].trim())
    } else if (trimmed.startsWith('- topic_metadata:')) {
      if (currentTopic) {
        info.topics!.push(currentTopic as TopicInfo)
      }
      currentTopic = {}
      currentSection = 'topic'
    } else if (currentSection === 'topic' && currentTopic) {
      if (trimmed.startsWith('name:')) {
        currentTopic.name = trimmed.split(':')[1].trim()
      } else if (trimmed.startsWith('type:')) {
        currentTopic.type = trimmed.split(':')[1].trim()
      } else if (trimmed.startsWith('message_count:')) {
        currentTopic.messageCount = parseInt(trimmed.split(':')[1].trim())
      }
    }
  }

  if (currentTopic && currentTopic.name) {
    info.topics!.push(currentTopic as TopicInfo)
  }

  return info
}
