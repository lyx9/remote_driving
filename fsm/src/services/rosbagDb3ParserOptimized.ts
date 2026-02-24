/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      RosBag DB3 Parser (Optimized)
 * @description 优化版 ROS2 db3 格式解析器
 *              修复安全漏洞、内存泄漏、添加验证和错误处理
 */

// @ts-ignore - sql.js lacks type definitions
import initSqlJs, { Database, SqlJsStatic } from 'sql.js'
import { CdrReader } from '@foxglove/cdr'
import { ref, shallowRef, readonly } from 'vue'
import type {
  RosBagInfo,
  TopicInfo,
  RosMessage,
  PointCloud2
} from './rosbagPlayer'

// ======================== 配置常量 ========================

/** SQL.js CDN URL with fallback */
const SQL_JS_CDN_URLS = [
  'https://sql.js.org/dist/',
  'https://cdn.jsdelivr.net/npm/sql.js@1.8.0/dist/',
  'https://unpkg.com/sql.js@1.8.0/dist/'
]

/** 消息缓存最大大小 (MB) */
const MAX_CACHE_SIZE_MB = 500

/** 单次查询最大返回消息数 */
const MAX_QUERY_RESULTS = 1000

/** GPS 坐标有效范围 */
const GPS_BOUNDS = {
  minLat: -90, maxLat: 90,
  minLng: -180, maxLng: 180
}

// ======================== 类型定义 ========================

interface Db3Topic {
  id: number
  name: string
  type: string
  serialization_format: string
  offered_qos_profiles: string
}

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

export interface PointCloudPoint {
  x: number
  y: number
  z: number
  intensity?: number
  ring?: number
}

export interface RosImage {
  header: {
    stamp: { sec: number; nanosec: number }
    frame_id: string
  }
  height: number
  width: number
  encoding: string
  is_bigendian: number
  step: number
  data: Uint8Array
}

export interface CompressedImage {
  header: {
    stamp: { sec: number; nanosec: number }
    frame_id: string
  }
  format: string
  data: Uint8Array
}

export interface ParserStats {
  totalMessagesLoaded: number
  cacheSizeMB: number
  parseErrors: number
  lastError: string | null
}

export interface LoadProgress {
  phase: 'init' | 'reading' | 'parsing' | 'indexing' | 'complete' | 'error'
  percent: number
  message: string
}

// ======================== 验证函数 ========================

/**
 * 验证数值是否为有效的有限数
 */
function isValidNumber(val: unknown): val is number {
  return typeof val === 'number' && Number.isFinite(val)
}

/**
 * 验证 GPS 坐标是否有效
 */
function isValidGpsCoordinate(lat: number, lng: number): boolean {
  return isValidNumber(lat) && isValidNumber(lng) &&
    lat >= GPS_BOUNDS.minLat && lat <= GPS_BOUNDS.maxLat &&
    lng >= GPS_BOUNDS.minLng && lng <= GPS_BOUNDS.maxLng
}

/**
 * 验证 topic ID 是否为正整数
 */
function isValidTopicId(id: unknown): id is number {
  return typeof id === 'number' && Number.isInteger(id) && id > 0
}

/**
 * 验证时间戳是否有效 (纳秒)
 */
function isValidTimestamp(ts: unknown): ts is number {
  return typeof ts === 'number' && Number.isFinite(ts) && ts >= 0
}

// ======================== CDR 消息解析器 ========================

function parseHeader(reader: CdrReader): { stamp: { sec: number; nanosec: number }; frame_id: string } {
  const sec = reader.int32()
  const nanosec = reader.uint32()
  const frame_id = reader.string()
  return { stamp: { sec, nanosec }, frame_id }
}

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

function parsePointCloud2(data: Uint8Array): PointCloud2 {
  const reader = new CdrReader(data)
  const header = parseHeader(reader)
  const height = reader.uint32()
  const width = reader.uint32()

  const fieldsLength = reader.sequenceLength()
  const fields: PointCloud2['fields'] = []
  for (let i = 0; i < fieldsLength; i++) {
    const name = reader.string()
    const offset = reader.uint32()
    const datatype = reader.uint8()
    const count = reader.uint32()

    // 验证 datatype 在有效范围内 (ROS2 定义 1-8)
    if (datatype >= 1 && datatype <= 8) {
      fields.push({ name, offset, datatype, count })
    }
  }

  const is_bigendian = reader.uint8() !== 0
  const point_step = reader.uint32()
  const row_step = reader.uint32()

  const dataLength = reader.sequenceLength()
  const pointData = new Uint8Array(dataLength)
  for (let i = 0; i < dataLength; i++) {
    pointData[i] = reader.uint8()
  }

  const is_dense = reader.uint8() !== 0

  return {
    header, height, width, fields,
    is_bigendian, point_step, row_step,
    data: pointData, is_dense
  }
}

function parseImage(data: Uint8Array): RosImage {
  const reader = new CdrReader(data)
  const header = parseHeader(reader)

  const height = reader.uint32()
  const width = reader.uint32()
  const encoding = reader.string()
  const is_bigendian = reader.uint8()
  const step = reader.uint32()

  const dataLength = reader.sequenceLength()
  const imageData = new Uint8Array(dataLength)
  for (let i = 0; i < dataLength; i++) {
    imageData[i] = reader.uint8()
  }

  return {
    header,
    height,
    width,
    encoding,
    is_bigendian,
    step,
    data: imageData
  }
}

function parseCompressedImage(data: Uint8Array): CompressedImage {
  const reader = new CdrReader(data)
  const header = parseHeader(reader)
  const format = reader.string()

  const dataLength = reader.sequenceLength()
  const imageData = new Uint8Array(dataLength)
  for (let i = 0; i < dataLength; i++) {
    imageData[i] = reader.uint8()
  }

  return {
    header,
    format,
    data: imageData
  }
}

/**
 * 从 PointCloud2 提取点云点 (优化版)
 */
export function extractPointCloudPoints(pc2: PointCloud2, maxPoints: number = 50000): PointCloudPoint[] {
  const points: PointCloudPoint[] = []
  const { fields, point_step, data, width, height } = pc2

  if (!fields || fields.length === 0 || !data || data.length === 0) {
    return points
  }

  // 查找字段偏移量
  let xOffset = -1, yOffset = -1, zOffset = -1
  let intensityOffset = -1, ringOffset = -1

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
    console.warn('[PointCloud] Missing x/y/z fields')
    return points
  }

  const totalPoints = width * height
  if (totalPoints === 0) return points

  const step = Math.max(1, Math.floor(totalPoints / maxPoints))

  // 验证数据边界
  if (data.byteLength < point_step) {
    console.warn('[PointCloud] Data buffer too small')
    return points
  }

  const dataView = new DataView(data.buffer, data.byteOffset, data.byteLength)
  const isLittleEndian = !pc2.is_bigendian

  for (let i = 0; i < totalPoints; i += step) {
    const offset = i * point_step

    // 边界检查
    if (offset + point_step > data.byteLength) break

    try {
      const x = dataView.getFloat32(offset + xOffset, isLittleEndian)
      const y = dataView.getFloat32(offset + yOffset, isLittleEndian)
      const z = dataView.getFloat32(offset + zOffset, isLittleEndian)

      // 跳过无效点
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue
      if (x === 0 && y === 0 && z === 0) continue

      // 跳过距离过远的点 (可能是噪声)
      const distance = Math.sqrt(x * x + y * y + z * z)
      if (distance > 200) continue // 200m 以外认为是噪声

      const point: PointCloudPoint = { x, y, z }

      if (intensityOffset >= 0 && offset + intensityOffset + 4 <= data.byteLength) {
        const intensity = dataView.getFloat32(offset + intensityOffset, isLittleEndian)
        if (Number.isFinite(intensity)) {
          point.intensity = intensity
        }
      }

      if (ringOffset >= 0 && offset + ringOffset + 2 <= data.byteLength) {
        point.ring = dataView.getUint16(offset + ringOffset, isLittleEndian)
      }

      points.push(point)
    } catch {
      // 跳过解析失败的点
      continue
    }
  }

  return points
}

/**
 * 将 ROS Image 转换为 Data URL (用于在 <img> 标签显示)
 */
export function convertImageToDataUrl(image: RosImage | CompressedImage): string | null {
  try {
    // 处理压缩图像 (JPEG/PNG)
    if ('format' in image) {
      const blob = new Blob([image.data], {
        type: image.format === 'jpeg' ? 'image/jpeg' : 'image/png'
      })
      return URL.createObjectURL(blob)
    }

    // 处理原始图像
    const { width, height, encoding, data: imgData } = image

    // 创建 canvas 进行转换
    const canvas = document.createElement('canvas')
    canvas.width = width
    canvas.height = height
    const ctx = canvas.getContext('2d')

    if (!ctx) return null

    const imageData = ctx.createImageData(width, height)

    // 根据编码格式转换
    switch (encoding) {
      case 'rgb8':
        // RGB8 -> RGBA
        for (let i = 0, j = 0; i < imgData.length; i += 3, j += 4) {
          imageData.data[j] = imgData[i]       // R
          imageData.data[j + 1] = imgData[i + 1] // G
          imageData.data[j + 2] = imgData[i + 2] // B
          imageData.data[j + 3] = 255            // A
        }
        break

      case 'bgr8':
        // BGR8 -> RGBA
        for (let i = 0, j = 0; i < imgData.length; i += 3, j += 4) {
          imageData.data[j] = imgData[i + 2]   // R
          imageData.data[j + 1] = imgData[i + 1] // G
          imageData.data[j + 2] = imgData[i]     // B
          imageData.data[j + 3] = 255            // A
        }
        break

      case 'mono8':
      case 'bayer_grbg8':
        // Grayscale -> RGBA
        for (let i = 0, j = 0; i < imgData.length; i++, j += 4) {
          const gray = imgData[i]
          imageData.data[j] = gray
          imageData.data[j + 1] = gray
          imageData.data[j + 2] = gray
          imageData.data[j + 3] = 255
        }
        break

      default:
        console.warn(`[Image] Unsupported encoding: ${encoding}`)
        return null
    }

    ctx.putImageData(imageData, 0, 0)
    return canvas.toDataURL('image/jpeg', 0.85)
  } catch (e) {
    console.error('[Image] Conversion failed:', e)
    return null
  }
}

// ======================== SQL.js 初始化 ========================

let sqlJsInstance: SqlJsStatic | null = null

/**
 * 初始化 SQL.js，带 CDN 故障转移
 */
async function initSqlJsWithFallback(): Promise<SqlJsStatic> {
  if (sqlJsInstance) return sqlJsInstance

  let lastError: Error | null = null

  for (const baseUrl of SQL_JS_CDN_URLS) {
    try {
      sqlJsInstance = await initSqlJs({
        locateFile: (file: string) => `${baseUrl}${file}`
      })
      console.log(`[SQL.js] Initialized from ${baseUrl}`)
      return sqlJsInstance
    } catch (e) {
      lastError = e as Error
      console.warn(`[SQL.js] Failed to load from ${baseUrl}:`, e)
    }
  }

  throw new Error(`Failed to initialize SQL.js from all CDNs: ${lastError?.message}`)
}

// ======================== 优化版 DB3 解析器 ========================

export function useRosBagDb3ParserOptimized() {
  // 状态
  const isLoading = ref(false)
  const error = ref<string | null>(null)
  const bagInfo = shallowRef<RosBagInfo | null>(null)
  const topics = shallowRef<Db3Topic[]>([])
  const progress = ref<LoadProgress>({ phase: 'init', percent: 0, message: '' })

  // 统计
  const stats = ref<ParserStats>({
    totalMessagesLoaded: 0,
    cacheSizeMB: 0,
    parseErrors: 0,
    lastError: null
  })

  // 内部状态
  let db: Database | null = null
  const messageCache = new Map<string, RosMessage[]>()
  let currentCacheSizeBytes = 0

  /**
   * 更新进度
   */
  const updateProgress = (phase: LoadProgress['phase'], percent: number, message: string) => {
    progress.value = { phase, percent, message }
  }

  /**
   * 估算消息大小
   */
  const estimateMessageSize = (msg: RosMessage): number => {
    // 粗略估算: timestamp + topic name + data
    const dataSize = msg.data?.raw?.byteLength ||
                    (msg.data?.data?.byteLength || 100)
    return 8 + msg.topic.length * 2 + dataSize
  }

  /**
   * 检查并清理缓存
   */
  const checkAndCleanCache = () => {
    const maxBytes = MAX_CACHE_SIZE_MB * 1024 * 1024

    if (currentCacheSizeBytes > maxBytes) {
      // 清理最早添加的 topic 缓存
      const firstKey = messageCache.keys().next().value
      if (firstKey) {
        const messages = messageCache.get(firstKey)
        if (messages) {
          const freedBytes = messages.reduce((sum, m) => sum + estimateMessageSize(m), 0)
          currentCacheSizeBytes -= freedBytes
          messageCache.delete(firstKey)
          console.log(`[Cache] Evicted ${firstKey}, freed ${(freedBytes / 1024 / 1024).toFixed(2)}MB`)
        }
      }
    }

    stats.value.cacheSizeMB = currentCacheSizeBytes / 1024 / 1024
  }

  /**
   * 加载 db3 文件 (优化版)
   */
  const loadDb3File = async (file: File): Promise<boolean> => {
    isLoading.value = true
    error.value = null
    messageCache.clear()
    currentCacheSizeBytes = 0
    stats.value = { totalMessagesLoaded: 0, cacheSizeMB: 0, parseErrors: 0, lastError: null }

    try {
      // 阶段1: 初始化 SQL.js
      updateProgress('init', 10, 'Initializing SQL engine...')
      const SQL = await initSqlJsWithFallback()

      // 阶段2: 读取文件
      updateProgress('reading', 30, `Reading ${(file.size / 1024 / 1024).toFixed(1)}MB file...`)

      // 验证文件大小
      if (file.size === 0) {
        throw new Error('File is empty')
      }

      if (file.size > 2 * 1024 * 1024 * 1024) { // 2GB limit
        throw new Error('File too large (max 2GB)')
      }

      const arrayBuffer = await file.arrayBuffer()
      const uint8Array = new Uint8Array(arrayBuffer)

      // 验证 SQLite 文件头 (magic number)
      if (uint8Array.length < 16) {
        throw new Error('File too small to be a valid db3 file')
      }

      const magic = String.fromCharCode(...uint8Array.slice(0, 16))
      if (!magic.startsWith('SQLite format 3')) {
        throw new Error('Not a valid SQLite database file. Please select a .db3 ROS2 bag file.')
      }

      // 阶段3: 解析数据库
      updateProgress('parsing', 50, 'Parsing database structure...')

      try {
        db = new SQL.Database(uint8Array)
      } catch (dbError) {
        throw new Error(`Failed to open database: ${dbError}. The file may be corrupted or incomplete.`)
      }

      // 读取 topics (使用参数化查询模式)
      let topicsResult
      try {
        topicsResult = db.exec('SELECT id, name, type, serialization_format, offered_qos_profiles FROM topics')
      } catch (schemaError) {
        throw new Error('Invalid ROS2 bag database schema. Missing "topics" table. This may not be a valid ROS2 bag file.')
      }

      if (topicsResult.length === 0 || topicsResult[0].values.length === 0) {
        throw new Error('No topics found in the bag file. The bag may be empty or corrupted.')
      }

      if (topicsResult.length > 0) {
        topics.value = topicsResult[0].values.map((row: unknown[]) => ({
          id: row[0] as number,
          name: row[1] as string,
          type: row[2] as string,
          serialization_format: row[3] as string,
          offered_qos_profiles: row[4] as string
        }))
      }

      // 阶段4: 建立索引和统计
      updateProgress('indexing', 70, 'Building message index...')

      const statsResult = db.exec(`
        SELECT
          topic_id,
          COUNT(*) as count,
          MIN(timestamp) as min_ts,
          MAX(timestamp) as max_ts
        FROM messages
        GROUP BY topic_id
      `)

      const topicStats = new Map<number, { count: number; minTs: number; maxTs: number }>()
      let globalMinTs = Number.MAX_SAFE_INTEGER
      let globalMaxTs = 0
      let totalMessages = 0

      if (statsResult.length > 0) {
        for (const row of statsResult[0].values) {
          const topicId = row[0] as number
          const count = row[1] as number
          // 使用字符串方式处理大整数
          const minTs = Number(String(row[2]))
          const maxTs = Number(String(row[3]))

          if (isValidTopicId(topicId)) {
            topicStats.set(topicId, { count, minTs, maxTs })
            totalMessages += count
            if (minTs < globalMinTs) globalMinTs = minTs
            if (maxTs > globalMaxTs) globalMaxTs = maxTs
          }
        }
      }

      // 构建 topic 信息
      const topicInfos: TopicInfo[] = topics.value.map(topic => {
        const topicStat = topicStats.get(topic.id)
        const count = topicStat?.count || 0
        const duration = topicStat ? (topicStat.maxTs - topicStat.minTs) / 1e9 : 0
        const frequency = duration > 0 ? count / duration : 0

        return {
          name: topic.name,
          type: topic.type,
          messageCount: count,
          frequency: Math.round(frequency * 10) / 10
        }
      })

      // 构建 bag 信息
      const durationSec = (globalMaxTs - globalMinTs) / 1e9
      bagInfo.value = {
        id: `db3_${Date.now()}`,
        name: file.name,
        path: file.name,
        format: 'db3',
        size: file.size,
        duration: durationSec,
        startTime: globalMinTs,
        endTime: globalMaxTs,
        messageCount: totalMessages,
        topics: topicInfos,
        metadata: {
          date: new Date(globalMinTs / 1e6).toISOString()
        }
      }

      updateProgress('complete', 100, 'Load complete')
      console.log(`[DB3 Parser] Loaded: ${file.name} (${totalMessages} messages, ${durationSec.toFixed(2)}s)`)

      isLoading.value = false
      return true
    } catch (e) {
      const errorMsg = `Failed to load db3: ${e}`
      error.value = errorMsg
      stats.value.lastError = errorMsg
      updateProgress('error', 0, errorMsg)
      console.error('[DB3 Parser] Error:', e)
      isLoading.value = false
      return false
    }
  }

  /**
   * 获取指定时间范围的消息 (带参数验证)
   */
  const getMessagesInRange = (
    topicName: string,
    startTime: number,
    endTime: number
  ): RosMessage[] => {
    if (!db) return []

    // 参数验证
    if (!isValidTimestamp(startTime) || !isValidTimestamp(endTime)) {
      console.error('[DB3 Parser] Invalid timestamp range')
      return []
    }

    if (startTime > endTime) {
      console.error('[DB3 Parser] startTime must be <= endTime')
      return []
    }

    const topic = topics.value.find(t => t.name === topicName)
    if (!topic || !isValidTopicId(topic.id)) return []

    try {
      // 使用参数化查询避免 SQL 注入
      const stmt = db.prepare(`
        SELECT id, timestamp, data
        FROM messages
        WHERE topic_id = ?
          AND timestamp >= ?
          AND timestamp <= ?
        ORDER BY timestamp
        LIMIT ?
      `)

      stmt.bind([topic.id, startTime, endTime, MAX_QUERY_RESULTS])

      const messages: RosMessage[] = []
      while (stmt.step()) {
        const row = stmt.get()
        const timestamp = Number(row[1])
        const data = row[2] as Uint8Array

        messages.push({
          topic: topicName,
          timestamp,
          data: parseMessageData(topic.type, data),
          type: topic.type
        })
      }
      stmt.free()

      return messages
    } catch (e) {
      stats.value.parseErrors++
      stats.value.lastError = String(e)
      console.error(`[DB3 Parser] Error getting messages for ${topicName}:`, e)
      return []
    }
  }

  /**
   * 获取指定时间点最近的消息
   */
  const getMessageAtTime = (topicName: string, timestamp: number): RosMessage | null => {
    if (!db || !isValidTimestamp(timestamp)) return null

    const topic = topics.value.find(t => t.name === topicName)
    if (!topic || !isValidTopicId(topic.id)) return null

    try {
      const stmt = db.prepare(`
        SELECT id, timestamp, data
        FROM messages
        WHERE topic_id = ?
          AND timestamp <= ?
        ORDER BY timestamp DESC
        LIMIT 1
      `)

      stmt.bind([topic.id, timestamp])

      if (stmt.step()) {
        const row = stmt.get()
        const msgTimestamp = Number(row[1])
        const data = row[2] as Uint8Array
        stmt.free()

        return {
          topic: topicName,
          timestamp: msgTimestamp,
          data: parseMessageData(topic.type, data),
          type: topic.type
        }
      }
      stmt.free()
      return null
    } catch (e) {
      stats.value.parseErrors++
      console.error(`[DB3 Parser] Error getting message for ${topicName}:`, e)
      return null
    }
  }

  /**
   * 预加载消息到缓存 (带内存限制)
   */
  const preloadAllMessages = async (topicName: string): Promise<void> => {
    if (!db || messageCache.has(topicName)) return

    const topic = topics.value.find(t => t.name === topicName)
    if (!topic || !isValidTopicId(topic.id)) {
      console.warn(`[DB3 Parser] Topic not found: ${topicName}`)
      return
    }

    try {
      const stmt = db.prepare(`
        SELECT id, timestamp, data
        FROM messages
        WHERE topic_id = ?
        ORDER BY timestamp
      `)

      stmt.bind([topic.id])

      const messages: RosMessage[] = []
      let batchSize = 0

      while (stmt.step()) {
        const row = stmt.get()
        const timestamp = Number(row[1])
        const data = row[2] as Uint8Array

        const msg: RosMessage = {
          topic: topicName,
          timestamp,
          data: parseMessageData(topic.type, data),
          type: topic.type
        }

        messages.push(msg)
        batchSize += estimateMessageSize(msg)

        // 每处理 100 条检查一次内存
        if (messages.length % 100 === 0) {
          checkAndCleanCache()
        }
      }
      stmt.free()

      messageCache.set(topicName, messages)
      currentCacheSizeBytes += batchSize
      stats.value.totalMessagesLoaded += messages.length
      stats.value.cacheSizeMB = currentCacheSizeBytes / 1024 / 1024

      console.log(`[DB3 Parser] Preloaded ${messages.length} messages for ${topicName} (${(batchSize / 1024 / 1024).toFixed(2)}MB)`)
    } catch (e) {
      stats.value.parseErrors++
      console.error(`[DB3 Parser] Error preloading messages for ${topicName}:`, e)
    }
  }

  /**
   * 从缓存获取消息 (二分查找)
   */
  const getCachedMessageAtTime = (topicName: string, timestamp: number): RosMessage | null => {
    const messages = messageCache.get(topicName)
    if (!messages || messages.length === 0) return null
    if (!isValidTimestamp(timestamp)) return null

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
  const parseMessageData = (type: string, data: Uint8Array): unknown => {
    try {
      switch (type) {
        case 'sensor_msgs/msg/NavSatFix':
          return parseNavSatFix(data)
        case 'sensor_msgs/msg/PointCloud2':
          return parsePointCloud2(data)
        case 'sensor_msgs/msg/Image':
          return parseImage(data)
        case 'sensor_msgs/msg/CompressedImage':
          return parseCompressedImage(data)
        default:
          return { raw: data }
      }
    } catch (e) {
      stats.value.parseErrors++
      console.error(`[DB3 Parser] Error parsing message of type ${type}:`, e)
      return { raw: data, error: String(e) }
    }
  }

  /**
   * 获取 GPS 轨迹 (带验证)
   */
  const getGpsTrajectory = (): Array<{ lat: number; lng: number; alt: number; timestamp: number }> => {
    if (!db) return []

    const topic = topics.value.find(t => t.type === 'sensor_msgs/msg/NavSatFix')
    if (!topic || !isValidTopicId(topic.id)) return []

    try {
      const stmt = db.prepare(`
        SELECT timestamp, data
        FROM messages
        WHERE topic_id = ?
        ORDER BY timestamp
      `)

      stmt.bind([topic.id])

      const trajectory: Array<{ lat: number; lng: number; alt: number; timestamp: number }> = []

      while (stmt.step()) {
        const row = stmt.get()
        const timestamp = Number(row[0])
        const data = row[1] as Uint8Array

        try {
          const navSatFix = parseNavSatFix(data)

          // 验证 GPS 坐标
          if (isValidGpsCoordinate(navSatFix.latitude, navSatFix.longitude)) {
            trajectory.push({
              lat: navSatFix.latitude,
              lng: navSatFix.longitude,
              alt: isValidNumber(navSatFix.altitude) ? navSatFix.altitude : 0,
              timestamp
            })
          }
        } catch {
          stats.value.parseErrors++
        }
      }
      stmt.free()

      return trajectory
    } catch (e) {
      console.error('[DB3 Parser] Error getting GPS trajectory:', e)
      return []
    }
  }

  /**
   * 检查 topic 是否存在
   */
  const hasTopicByName = (topicName: string): boolean => {
    return topics.value.some(t => t.name === topicName)
  }

  /**
   * 检查 topic 类型是否存在
   */
  const hasTopicByType = (topicType: string): boolean => {
    return topics.value.some(t => t.type === topicType)
  }

  /**
   * 获取 topic 列表
   */
  const getTopicNames = (): string[] => {
    return topics.value.map(t => t.name)
  }

  /**
   * 获取点云 topic 名称
   */
  const getPointCloudTopicName = (): string | null => {
    const topic = topics.value.find(t => t.type === 'sensor_msgs/msg/PointCloud2')
    return topic?.name || null
  }

  /**
   * 获取所有相机 topic 名称
   */
  const getCameraTopicNames = (): string[] => {
    return topics.value
      .filter(t =>
        t.type === 'sensor_msgs/msg/Image' ||
        t.type === 'sensor_msgs/msg/CompressedImage'
      )
      .map(t => t.name)
  }

  /**
   * 根据名称模式获取相机 topic
   */
  const getCameraTopicByPattern = (pattern: string): string | null => {
    const cameraTopics = getCameraTopicNames()
    return cameraTopics.find(name => name.includes(pattern)) || null
  }

  /**
   * 清理资源
   */
  const dispose = () => {
    if (db) {
      db.close()
      db = null
    }
    bagInfo.value = null
    topics.value = []
    messageCache.clear()
    currentCacheSizeBytes = 0
    stats.value = { totalMessagesLoaded: 0, cacheSizeMB: 0, parseErrors: 0, lastError: null }
  }

  return {
    // 只读状态
    isLoading: readonly(isLoading),
    error: readonly(error),
    bagInfo: readonly(bagInfo),
    topics: readonly(topics),
    progress: readonly(progress),
    stats: readonly(stats),

    // 方法
    loadDb3File,
    getMessagesInRange,
    getMessageAtTime,
    preloadAllMessages,
    getCachedMessageAtTime,
    getGpsTrajectory,
    hasTopicByName,
    hasTopicByType,
    getTopicNames,
    getPointCloudTopicName,
    getCameraTopicNames,
    getCameraTopicByPattern,
    dispose,

    // 工具函数
    extractPointCloudPoints,
    convertImageToDataUrl
  }
}

export default useRosBagDb3ParserOptimized
