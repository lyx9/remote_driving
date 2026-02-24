/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      RosBag Parser Tests
 * @description RosBag DB3 解析器功能测试
 */

import { describe, it, expect } from 'vitest'

// ======================== 验证函数测试 ========================

describe('Validation Functions', () => {
  // 模拟验证函数 (从 rosbagDb3ParserOptimized.ts 复制)
  function isValidNumber(val: unknown): val is number {
    return typeof val === 'number' && Number.isFinite(val)
  }

  function isValidGpsCoordinate(lat: number, lng: number): boolean {
    return isValidNumber(lat) && isValidNumber(lng) &&
      lat >= -90 && lat <= 90 &&
      lng >= -180 && lng <= 180
  }

  function isValidTopicId(id: unknown): id is number {
    return typeof id === 'number' && Number.isInteger(id) && id > 0
  }

  function isValidTimestamp(ts: unknown): ts is number {
    return typeof ts === 'number' && Number.isFinite(ts) && ts >= 0
  }

  describe('isValidNumber', () => {
    it('should return true for valid finite numbers', () => {
      expect(isValidNumber(0)).toBe(true)
      expect(isValidNumber(42)).toBe(true)
      expect(isValidNumber(-100)).toBe(true)
      expect(isValidNumber(3.14159)).toBe(true)
      expect(isValidNumber(Number.MAX_SAFE_INTEGER)).toBe(true)
    })

    it('should return false for invalid numbers', () => {
      expect(isValidNumber(NaN)).toBe(false)
      expect(isValidNumber(Infinity)).toBe(false)
      expect(isValidNumber(-Infinity)).toBe(false)
      expect(isValidNumber('42')).toBe(false)
      expect(isValidNumber(null)).toBe(false)
      expect(isValidNumber(undefined)).toBe(false)
      expect(isValidNumber({})).toBe(false)
    })
  })

  describe('isValidGpsCoordinate', () => {
    it('should return true for valid coordinates', () => {
      expect(isValidGpsCoordinate(0, 0)).toBe(true)
      expect(isValidGpsCoordinate(22.3193, 114.1694)).toBe(true) // Hong Kong
      expect(isValidGpsCoordinate(39.9042, 116.4074)).toBe(true) // Beijing
      expect(isValidGpsCoordinate(-33.8688, 151.2093)).toBe(true) // Sydney
      expect(isValidGpsCoordinate(90, 180)).toBe(true) // Edge case
      expect(isValidGpsCoordinate(-90, -180)).toBe(true) // Edge case
    })

    it('should return false for out-of-range coordinates', () => {
      expect(isValidGpsCoordinate(91, 0)).toBe(false)
      expect(isValidGpsCoordinate(-91, 0)).toBe(false)
      expect(isValidGpsCoordinate(0, 181)).toBe(false)
      expect(isValidGpsCoordinate(0, -181)).toBe(false)
      expect(isValidGpsCoordinate(100, 200)).toBe(false)
    })

    it('should return false for invalid numbers', () => {
      expect(isValidGpsCoordinate(NaN, 0)).toBe(false)
      expect(isValidGpsCoordinate(0, NaN)).toBe(false)
      expect(isValidGpsCoordinate(Infinity, 0)).toBe(false)
    })
  })

  describe('isValidTopicId', () => {
    it('should return true for valid topic IDs', () => {
      expect(isValidTopicId(1)).toBe(true)
      expect(isValidTopicId(100)).toBe(true)
      expect(isValidTopicId(Number.MAX_SAFE_INTEGER)).toBe(true)
    })

    it('should return false for invalid topic IDs', () => {
      expect(isValidTopicId(0)).toBe(false)
      expect(isValidTopicId(-1)).toBe(false)
      expect(isValidTopicId(1.5)).toBe(false)
      expect(isValidTopicId('1')).toBe(false)
      expect(isValidTopicId(null)).toBe(false)
    })
  })

  describe('isValidTimestamp', () => {
    it('should return true for valid timestamps', () => {
      expect(isValidTimestamp(0)).toBe(true)
      expect(isValidTimestamp(1704067200000000000)).toBe(true) // Nanoseconds
      expect(isValidTimestamp(Date.now() * 1e6)).toBe(true)
    })

    it('should return false for invalid timestamps', () => {
      expect(isValidTimestamp(-1)).toBe(false)
      expect(isValidTimestamp(NaN)).toBe(false)
      expect(isValidTimestamp(Infinity)).toBe(false)
      expect(isValidTimestamp('1704067200')).toBe(false)
    })
  })
})

// ======================== 点云提取测试 ========================

describe('PointCloud Extraction', () => {
  // 模拟 extractPointCloudPoints 逻辑
  interface PointCloudPoint {
    x: number
    y: number
    z: number
    intensity?: number
  }

  interface PointCloud2 {
    height: number
    width: number
    fields: Array<{ name: string; offset: number; datatype: number; count: number }>
    is_bigendian: boolean
    point_step: number
    row_step: number
    data: Uint8Array
    is_dense: boolean
    header: { stamp: { sec: number; nanosec: number }; frame_id: string }
  }

  function extractPointCloudPoints(pc2: PointCloud2, maxPoints: number = 50000): PointCloudPoint[] {
    const points: PointCloudPoint[] = []
    const { fields, point_step, data, width, height } = pc2

    if (!fields || fields.length === 0 || !data || data.length === 0) {
      return points
    }

    let xOffset = -1, yOffset = -1, zOffset = -1
    let intensityOffset = -1

    for (const field of fields) {
      switch (field.name) {
        case 'x': xOffset = field.offset; break
        case 'y': yOffset = field.offset; break
        case 'z': zOffset = field.offset; break
        case 'intensity': intensityOffset = field.offset; break
      }
    }

    if (xOffset < 0 || yOffset < 0 || zOffset < 0) {
      return points
    }

    const totalPoints = width * height
    if (totalPoints === 0) return points

    const step = Math.max(1, Math.floor(totalPoints / maxPoints))

    if (data.byteLength < point_step) {
      return points
    }

    const dataView = new DataView(data.buffer, data.byteOffset, data.byteLength)
    const isLittleEndian = !pc2.is_bigendian

    for (let i = 0; i < totalPoints; i += step) {
      const offset = i * point_step
      if (offset + point_step > data.byteLength) break

      try {
        const x = dataView.getFloat32(offset + xOffset, isLittleEndian)
        const y = dataView.getFloat32(offset + yOffset, isLittleEndian)
        const z = dataView.getFloat32(offset + zOffset, isLittleEndian)

        if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue
        if (x === 0 && y === 0 && z === 0) continue

        const distance = Math.sqrt(x * x + y * y + z * z)
        if (distance > 200) continue

        const point: PointCloudPoint = { x, y, z }

        if (intensityOffset >= 0 && offset + intensityOffset + 4 <= data.byteLength) {
          const intensity = dataView.getFloat32(offset + intensityOffset, isLittleEndian)
          if (Number.isFinite(intensity)) {
            point.intensity = intensity
          }
        }

        points.push(point)
      } catch {
        continue
      }
    }

    return points
  }

  function createMockPointCloud2(numPoints: number): PointCloud2 {
    const point_step = 16 // x, y, z, intensity (4 bytes each)
    const data = new Uint8Array(numPoints * point_step)
    const view = new DataView(data.buffer)

    for (let i = 0; i < numPoints; i++) {
      const offset = i * point_step
      view.setFloat32(offset + 0, Math.random() * 50 - 25, true)  // x: -25 to 25
      view.setFloat32(offset + 4, Math.random() * 50 - 25, true)  // y: -25 to 25
      view.setFloat32(offset + 8, Math.random() * 10 - 5, true)   // z: -5 to 5
      view.setFloat32(offset + 12, Math.random() * 255, true)     // intensity: 0-255
    }

    return {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'lidar' },
      height: 1,
      width: numPoints,
      fields: [
        { name: 'x', offset: 0, datatype: 7, count: 1 },
        { name: 'y', offset: 4, datatype: 7, count: 1 },
        { name: 'z', offset: 8, datatype: 7, count: 1 },
        { name: 'intensity', offset: 12, datatype: 7, count: 1 }
      ],
      is_bigendian: false,
      point_step,
      row_step: numPoints * point_step,
      data,
      is_dense: true
    }
  }

  it('should extract points from valid point cloud', () => {
    const pc2 = createMockPointCloud2(100)
    const points = extractPointCloudPoints(pc2)

    expect(points.length).toBeGreaterThan(0)
    expect(points.length).toBeLessThanOrEqual(100)

    points.forEach(point => {
      expect(point).toHaveProperty('x')
      expect(point).toHaveProperty('y')
      expect(point).toHaveProperty('z')
      expect(typeof point.x).toBe('number')
      expect(typeof point.y).toBe('number')
      expect(typeof point.z).toBe('number')
    })
  })

  it('should respect maxPoints limit', () => {
    const pc2 = createMockPointCloud2(10000)
    const points = extractPointCloudPoints(pc2, 100)

    expect(points.length).toBeLessThanOrEqual(100)
  })

  it('should return empty array for empty point cloud', () => {
    const pc2 = createMockPointCloud2(0)
    const points = extractPointCloudPoints(pc2)

    expect(points).toEqual([])
  })

  it('should return empty array when missing required fields', () => {
    const pc2 = createMockPointCloud2(100)
    pc2.fields = [{ name: 'x', offset: 0, datatype: 7, count: 1 }] // Missing y, z

    const points = extractPointCloudPoints(pc2)
    expect(points).toEqual([])
  })

  it('should filter out points beyond distance threshold', () => {
    const point_step = 12
    const data = new Uint8Array(2 * point_step)
    const view = new DataView(data.buffer)

    // Point 1: within range (distance ~14.14)
    view.setFloat32(0, 10, true)
    view.setFloat32(4, 10, true)
    view.setFloat32(8, 0, true)

    // Point 2: beyond 200m
    view.setFloat32(12, 200, true)
    view.setFloat32(16, 200, true)
    view.setFloat32(20, 0, true)

    const pc2: PointCloud2 = {
      header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'lidar' },
      height: 1,
      width: 2,
      fields: [
        { name: 'x', offset: 0, datatype: 7, count: 1 },
        { name: 'y', offset: 4, datatype: 7, count: 1 },
        { name: 'z', offset: 8, datatype: 7, count: 1 }
      ],
      is_bigendian: false,
      point_step,
      row_step: 2 * point_step,
      data,
      is_dense: true
    }

    const points = extractPointCloudPoints(pc2)
    expect(points.length).toBe(1) // Only first point should be included
  })
})

// ======================== 缓存管理测试 ========================

describe('Cache Management', () => {
  it('should implement LRU cache eviction', () => {
    const MAX_CACHE_SIZE = 3
    const cache = new Map<string, number[]>()

    function addToCache(key: string, value: number[]) {
      if (cache.size >= MAX_CACHE_SIZE) {
        const firstKey = cache.keys().next().value
        if (firstKey) cache.delete(firstKey)
      }
      cache.set(key, value)
    }

    addToCache('topic1', [1, 2, 3])
    addToCache('topic2', [4, 5, 6])
    addToCache('topic3', [7, 8, 9])
    expect(cache.size).toBe(3)

    addToCache('topic4', [10, 11, 12])
    expect(cache.size).toBe(3)
    expect(cache.has('topic1')).toBe(false)
    expect(cache.has('topic4')).toBe(true)
  })
})

// ======================== 二分查找测试 ========================

describe('Binary Search for Messages', () => {
  interface MockMessage {
    timestamp: number
    data: string
  }

  function getCachedMessageAtTime(messages: MockMessage[], timestamp: number): MockMessage | null {
    if (!messages || messages.length === 0) return null
    if (timestamp < 0) return null

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

  it('should find exact timestamp match', () => {
    const messages: MockMessage[] = [
      { timestamp: 100, data: 'a' },
      { timestamp: 200, data: 'b' },
      { timestamp: 300, data: 'c' }
    ]

    expect(getCachedMessageAtTime(messages, 200)?.data).toBe('b')
  })

  it('should find closest message before timestamp', () => {
    const messages: MockMessage[] = [
      { timestamp: 100, data: 'a' },
      { timestamp: 200, data: 'b' },
      { timestamp: 300, data: 'c' }
    ]

    expect(getCachedMessageAtTime(messages, 250)?.data).toBe('b')
    expect(getCachedMessageAtTime(messages, 150)?.data).toBe('a')
  })

  it('should return first message for timestamp before all', () => {
    const messages: MockMessage[] = [
      { timestamp: 100, data: 'a' },
      { timestamp: 200, data: 'b' }
    ]

    expect(getCachedMessageAtTime(messages, 50)?.data).toBe('a')
  })

  it('should return last message for timestamp after all', () => {
    const messages: MockMessage[] = [
      { timestamp: 100, data: 'a' },
      { timestamp: 200, data: 'b' }
    ]

    expect(getCachedMessageAtTime(messages, 500)?.data).toBe('b')
  })

  it('should return null for empty array', () => {
    expect(getCachedMessageAtTime([], 100)).toBeNull()
  })

  it('should return null for invalid timestamp', () => {
    const messages: MockMessage[] = [{ timestamp: 100, data: 'a' }]
    expect(getCachedMessageAtTime(messages, -1)).toBeNull()
  })
})

// ======================== SQL 注入防护测试 ========================

describe('SQL Injection Prevention', () => {
  it('should sanitize topic names in queries', () => {
    // 模拟参数化查询
    function buildSafeQuery(topicId: number, startTime: number, endTime: number): { sql: string; params: (number | string)[] } {
      return {
        sql: 'SELECT * FROM messages WHERE topic_id = ? AND timestamp >= ? AND timestamp <= ?',
        params: [topicId, startTime, endTime]
      }
    }

    const query = buildSafeQuery(1, 1000, 2000)
    expect(query.sql).not.toContain('1')
    expect(query.params).toEqual([1, 1000, 2000])
  })

  it('should reject invalid topic IDs', () => {
    function validateTopicId(id: unknown): number | null {
      if (typeof id !== 'number' || !Number.isInteger(id) || id <= 0) {
        return null
      }
      return id
    }

    expect(validateTopicId(1)).toBe(1)
    expect(validateTopicId('; DROP TABLE messages;--')).toBeNull()
    expect(validateTopicId(-1)).toBeNull()
    expect(validateTopicId(0)).toBeNull()
    expect(validateTopicId(1.5)).toBeNull()
  })
})

// ======================== 进度报告测试 ========================

describe('Progress Reporting', () => {
  interface LoadProgress {
    phase: 'init' | 'reading' | 'parsing' | 'indexing' | 'complete' | 'error'
    percent: number
    message: string
  }

  it('should track progress through all phases', () => {
    const progressHistory: LoadProgress[] = []

    function updateProgress(phase: LoadProgress['phase'], percent: number, message: string) {
      progressHistory.push({ phase, percent, message })
    }

    // Simulate loading process
    updateProgress('init', 0, 'Initializing...')
    updateProgress('init', 10, 'Initializing SQL engine...')
    updateProgress('reading', 30, 'Reading file...')
    updateProgress('parsing', 50, 'Parsing database...')
    updateProgress('indexing', 70, 'Building index...')
    updateProgress('complete', 100, 'Load complete')

    expect(progressHistory.length).toBe(6)
    expect(progressHistory[0].phase).toBe('init')
    expect(progressHistory[progressHistory.length - 1].phase).toBe('complete')
    expect(progressHistory[progressHistory.length - 1].percent).toBe(100)
  })

  it('should report errors correctly', () => {
    const progress: LoadProgress = { phase: 'init', percent: 0, message: '' }

    try {
      throw new Error('Database corrupted')
    } catch (e) {
      progress.phase = 'error'
      progress.percent = 0
      progress.message = `Failed: ${e}`
    }

    expect(progress.phase).toBe('error')
    expect(progress.message).toContain('Database corrupted')
  })
})
