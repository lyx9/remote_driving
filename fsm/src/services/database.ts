/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      Database Service
 * @description 本地数据库服务，使用sql.js (SQLite WASM)实现本地数据存储
 */

import { ref } from 'vue'

// sql.js 类型声明
declare class Database {
  run(sql: string, params?: any[]): Database
  exec(sql: string): QueryExecResult[]
  prepare(sql: string): Statement
  close(): void
  export(): Uint8Array
}

declare class Statement {
  bind(params?: any[]): boolean
  step(): boolean
  getAsObject(params?: any): Record<string, any>
  free(): boolean
}

interface QueryExecResult {
  columns: string[]
  values: any[][]
}

interface SqlJsStatic {
  Database: new (data?: ArrayLike<number>) => Database
}

// 加载 sql.js
let SQL: SqlJsStatic | null = null

async function initSqlJs(): Promise<SqlJsStatic> {
  if (SQL) return SQL

  // 动态加载 sql.js
  const sqlPromise = (window as any).initSqlJs?.({
    locateFile: (file: string) => `https://sql.js.org/dist/${file}`
  })

  if (!sqlPromise) {
    // 如果没有预加载，动态引入脚本
    await new Promise<void>((resolve, reject) => {
      const script = document.createElement('script')
      script.src = 'https://sql.js.org/dist/sql-wasm.js'
      script.onload = () => resolve()
      script.onerror = reject
      document.head.appendChild(script)
    })

    SQL = await (window as any).initSqlJs({
      locateFile: (file: string) => `https://sql.js.org/dist/${file}`
    })
  } else {
    SQL = await sqlPromise
  }

  return SQL!
}

// ======================== 类型定义 ========================

export interface Recording {
  id: number
  sessionId: string
  vehicleId: string
  startTime: Date
  endTime?: Date
  durationSeconds?: number
  filePath?: string
  fileSizeBytes?: number
  type: 'rosbag' | 'video' | 'telemetry'
  status: 'recording' | 'completed' | 'error'
  createdAt: Date
}

export interface Tag {
  id: number
  recordingId: number
  name: string
  color?: string
  timestampMs?: number
  description?: string
  createdAt: Date
}

export interface Category {
  id: number
  name: string
  parentId?: number
  color?: string
}

export interface SearchQuery {
  vehicleId?: string
  type?: Recording['type']
  status?: Recording['status']
  startDate?: Date
  endDate?: Date
  tags?: string[]
  categories?: number[]
  keyword?: string
  limit?: number
  offset?: number
}

export interface ExportFormat {
  format: 'json' | 'csv' | 'sqlite'
  includeFiles?: boolean
}

// ======================== 数据库服务 ========================

export function useLocalDatabase() {
  const db = ref<Database | null>(null)
  const isReady = ref(false)
  const error = ref<string | null>(null)

  // 初始化数据库
  const initialize = async (): Promise<boolean> => {
    try {
      const SQL = await initSqlJs()

      // 尝试从 localStorage 加载现有数据库
      const savedDb = localStorage.getItem('fsm_database')
      if (savedDb) {
        const data = Uint8Array.from(atob(savedDb), c => c.charCodeAt(0))
        db.value = new SQL.Database(data)
      } else {
        db.value = new SQL.Database()
        await createTables()
      }

      isReady.value = true
      return true
    } catch (e) {
      error.value = `数据库初始化失败: ${e}`
      console.error(e)
      return false
    }
  }

  // 创建表结构
  const createTables = async () => {
    if (!db.value) return

    const schema = `
      -- 录制记录表
      CREATE TABLE IF NOT EXISTS recordings (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        session_id TEXT NOT NULL,
        vehicle_id TEXT NOT NULL,
        start_time TEXT,
        end_time TEXT,
        duration_seconds INTEGER,
        file_path TEXT,
        file_size_bytes INTEGER,
        type TEXT CHECK(type IN ('rosbag', 'video', 'telemetry')),
        status TEXT CHECK(status IN ('recording', 'completed', 'error')),
        created_at TEXT DEFAULT (datetime('now'))
      );

      -- 标签表
      CREATE TABLE IF NOT EXISTS tags (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        recording_id INTEGER REFERENCES recordings(id),
        name TEXT NOT NULL,
        color TEXT,
        timestamp_ms INTEGER,
        description TEXT,
        created_at TEXT DEFAULT (datetime('now'))
      );

      -- 分类表
      CREATE TABLE IF NOT EXISTS categories (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL UNIQUE,
        parent_id INTEGER REFERENCES categories(id),
        color TEXT
      );

      -- 录制-分类关联表
      CREATE TABLE IF NOT EXISTS recording_categories (
        recording_id INTEGER REFERENCES recordings(id),
        category_id INTEGER REFERENCES categories(id),
        PRIMARY KEY (recording_id, category_id)
      );

      -- 索引
      CREATE INDEX IF NOT EXISTS idx_recordings_vehicle ON recordings(vehicle_id);
      CREATE INDEX IF NOT EXISTS idx_recordings_time ON recordings(start_time);
      CREATE INDEX IF NOT EXISTS idx_recordings_type ON recordings(type);
      CREATE INDEX IF NOT EXISTS idx_tags_recording ON tags(recording_id);
      CREATE INDEX IF NOT EXISTS idx_tags_name ON tags(name);

      -- 默认分类
      INSERT OR IGNORE INTO categories (name, color) VALUES
        ('测试数据', '#4CAF50'),
        ('演示数据', '#2196F3'),
        ('问题数据', '#F44336'),
        ('正常运行', '#9C27B0'),
        ('紧急情况', '#FF5722');
    `

    db.value.run(schema)
    saveToStorage()
  }

  // 保存到 localStorage
  const saveToStorage = () => {
    if (!db.value) return

    const data = db.value.export()
    const base64 = btoa(String.fromCharCode(...data))
    localStorage.setItem('fsm_database', base64)
  }

  // ==================== 录制记录操作 ====================

  // 创建录制记录
  const createRecording = (recording: Omit<Recording, 'id' | 'createdAt'>): number => {
    if (!db.value) throw new Error('数据库未初始化')

    const sql = `
      INSERT INTO recordings (session_id, vehicle_id, start_time, end_time, duration_seconds, file_path, file_size_bytes, type, status)
      VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
    `

    db.value.run(sql, [
      recording.sessionId,
      recording.vehicleId,
      recording.startTime?.toISOString(),
      recording.endTime?.toISOString() || null,
      recording.durationSeconds || null,
      recording.filePath || null,
      recording.fileSizeBytes || null,
      recording.type,
      recording.status
    ])

    // 获取插入的 ID
    const result = db.value.exec('SELECT last_insert_rowid() as id')
    const id = result[0]?.values[0]?.[0] as number

    saveToStorage()
    return id
  }

  // 更新录制记录
  const updateRecording = (id: number, updates: Partial<Recording>): void => {
    if (!db.value) throw new Error('数据库未初始化')

    const fields: string[] = []
    const values: any[] = []

    if (updates.endTime !== undefined) {
      fields.push('end_time = ?')
      values.push(updates.endTime?.toISOString() || null)
    }
    if (updates.durationSeconds !== undefined) {
      fields.push('duration_seconds = ?')
      values.push(updates.durationSeconds)
    }
    if (updates.filePath !== undefined) {
      fields.push('file_path = ?')
      values.push(updates.filePath)
    }
    if (updates.fileSizeBytes !== undefined) {
      fields.push('file_size_bytes = ?')
      values.push(updates.fileSizeBytes)
    }
    if (updates.status !== undefined) {
      fields.push('status = ?')
      values.push(updates.status)
    }

    if (fields.length === 0) return

    values.push(id)
    db.value.run(`UPDATE recordings SET ${fields.join(', ')} WHERE id = ?`, values)
    saveToStorage()
  }

  // 获取录制记录
  const getRecording = (id: number): Recording | null => {
    if (!db.value) throw new Error('数据库未初始化')

    const stmt = db.value.prepare('SELECT * FROM recordings WHERE id = ?')
    stmt.bind([id])

    if (stmt.step()) {
      const row = stmt.getAsObject()
      stmt.free()
      return rowToRecording(row)
    }

    stmt.free()
    return null
  }

  // 搜索录制记录
  const searchRecordings = (query: SearchQuery): Recording[] => {
    if (!db.value) throw new Error('数据库未初始化')

    let sql = 'SELECT DISTINCT r.* FROM recordings r'
    const conditions: string[] = []
    const params: any[] = []

    // 标签过滤需要 JOIN
    if (query.tags && query.tags.length > 0) {
      sql += ' LEFT JOIN tags t ON r.id = t.recording_id'
    }

    // 分类过滤需要 JOIN
    if (query.categories && query.categories.length > 0) {
      sql += ' LEFT JOIN recording_categories rc ON r.id = rc.recording_id'
    }

    // 构建条件
    if (query.vehicleId) {
      conditions.push('r.vehicle_id = ?')
      params.push(query.vehicleId)
    }

    if (query.type) {
      conditions.push('r.type = ?')
      params.push(query.type)
    }

    if (query.status) {
      conditions.push('r.status = ?')
      params.push(query.status)
    }

    if (query.startDate) {
      conditions.push('r.start_time >= ?')
      params.push(query.startDate.toISOString())
    }

    if (query.endDate) {
      conditions.push('r.start_time <= ?')
      params.push(query.endDate.toISOString())
    }

    if (query.tags && query.tags.length > 0) {
      conditions.push(`t.name IN (${query.tags.map(() => '?').join(',')})`)
      params.push(...query.tags)
    }

    if (query.categories && query.categories.length > 0) {
      conditions.push(`rc.category_id IN (${query.categories.map(() => '?').join(',')})`)
      params.push(...query.categories)
    }

    if (conditions.length > 0) {
      sql += ' WHERE ' + conditions.join(' AND ')
    }

    sql += ' ORDER BY r.start_time DESC'

    if (query.limit) {
      sql += ` LIMIT ${query.limit}`
      if (query.offset) {
        sql += ` OFFSET ${query.offset}`
      }
    }

    const stmt = db.value.prepare(sql)
    stmt.bind(params)

    const results: Recording[] = []
    while (stmt.step()) {
      results.push(rowToRecording(stmt.getAsObject()))
    }
    stmt.free()

    return results
  }

  // 删除录制记录
  const deleteRecording = (id: number): void => {
    if (!db.value) throw new Error('数据库未初始化')

    db.value.run('DELETE FROM tags WHERE recording_id = ?', [id])
    db.value.run('DELETE FROM recording_categories WHERE recording_id = ?', [id])
    db.value.run('DELETE FROM recordings WHERE id = ?', [id])
    saveToStorage()
  }

  // ==================== 标签操作 ====================

  // 添加标签
  const addTag = (tag: Omit<Tag, 'id' | 'createdAt'>): number => {
    if (!db.value) throw new Error('数据库未初始化')

    db.value.run(
      'INSERT INTO tags (recording_id, name, color, timestamp_ms, description) VALUES (?, ?, ?, ?, ?)',
      [tag.recordingId, tag.name, tag.color || null, tag.timestampMs || null, tag.description || null]
    )

    const result = db.value.exec('SELECT last_insert_rowid() as id')
    saveToStorage()
    return result[0]?.values[0]?.[0] as number
  }

  // 获取录制的所有标签
  const getTags = (recordingId: number): Tag[] => {
    if (!db.value) throw new Error('数据库未初始化')

    const stmt = db.value.prepare('SELECT * FROM tags WHERE recording_id = ? ORDER BY timestamp_ms')
    stmt.bind([recordingId])

    const results: Tag[] = []
    while (stmt.step()) {
      const row = stmt.getAsObject()
      results.push({
        id: row.id,
        recordingId: row.recording_id,
        name: row.name,
        color: row.color,
        timestampMs: row.timestamp_ms,
        description: row.description,
        createdAt: new Date(row.created_at)
      })
    }
    stmt.free()

    return results
  }

  // 删除标签
  const removeTag = (tagId: number): void => {
    if (!db.value) throw new Error('数据库未初始化')
    db.value.run('DELETE FROM tags WHERE id = ?', [tagId])
    saveToStorage()
  }

  // ==================== 分类操作 ====================

  // 获取所有分类
  const getCategories = (): Category[] => {
    if (!db.value) throw new Error('数据库未初始化')

    const result = db.value.exec('SELECT * FROM categories ORDER BY name')
    if (!result[0]) return []

    return result[0].values.map(row => ({
      id: row[0] as number,
      name: row[1] as string,
      parentId: row[2] as number | undefined,
      color: row[3] as string | undefined
    }))
  }

  // 创建分类
  const createCategory = (name: string, color?: string, parentId?: number): number => {
    if (!db.value) throw new Error('数据库未初始化')

    db.value.run(
      'INSERT INTO categories (name, color, parent_id) VALUES (?, ?, ?)',
      [name, color || null, parentId || null]
    )

    const result = db.value.exec('SELECT last_insert_rowid() as id')
    saveToStorage()
    return result[0]?.values[0]?.[0] as number
  }

  // 设置录制的分类
  const setRecordingCategories = (recordingId: number, categoryIds: number[]): void => {
    if (!db.value) throw new Error('数据库未初始化')

    // 先删除旧的关联
    db.value.run('DELETE FROM recording_categories WHERE recording_id = ?', [recordingId])

    // 添加新的关联
    for (const categoryId of categoryIds) {
      db.value.run(
        'INSERT INTO recording_categories (recording_id, category_id) VALUES (?, ?)',
        [recordingId, categoryId]
      )
    }

    saveToStorage()
  }

  // 获取录制的分类
  const getRecordingCategories = (recordingId: number): Category[] => {
    if (!db.value) throw new Error('数据库未初始化')

    const sql = `
      SELECT c.* FROM categories c
      JOIN recording_categories rc ON c.id = rc.category_id
      WHERE rc.recording_id = ?
    `
    const stmt = db.value.prepare(sql)
    stmt.bind([recordingId])

    const results: Category[] = []
    while (stmt.step()) {
      const row = stmt.getAsObject()
      results.push({
        id: row.id,
        name: row.name,
        parentId: row.parent_id,
        color: row.color
      })
    }
    stmt.free()

    return results
  }

  // ==================== 导出功能 ====================

  // 导出为 JSON
  const exportToJson = (recordingIds?: number[]): string => {
    if (!db.value) throw new Error('数据库未初始化')

    let recordings: Recording[]
    if (recordingIds && recordingIds.length > 0) {
      recordings = recordingIds.map(id => getRecording(id)).filter(r => r !== null) as Recording[]
    } else {
      recordings = searchRecordings({})
    }

    const data = recordings.map(r => ({
      ...r,
      tags: getTags(r.id),
      categories: getRecordingCategories(r.id)
    }))

    return JSON.stringify(data, null, 2)
  }

  // 导出为 CSV
  const exportToCsv = (recordingIds?: number[]): string => {
    if (!db.value) throw new Error('数据库未初始化')

    let recordings: Recording[]
    if (recordingIds && recordingIds.length > 0) {
      recordings = recordingIds.map(id => getRecording(id)).filter(r => r !== null) as Recording[]
    } else {
      recordings = searchRecordings({})
    }

    const headers = ['id', 'sessionId', 'vehicleId', 'startTime', 'endTime', 'durationSeconds', 'type', 'status', 'tags', 'categories']
    const rows = recordings.map(r => {
      const tags = getTags(r.id).map(t => t.name).join(';')
      const categories = getRecordingCategories(r.id).map(c => c.name).join(';')
      return [
        r.id,
        r.sessionId,
        r.vehicleId,
        r.startTime?.toISOString() || '',
        r.endTime?.toISOString() || '',
        r.durationSeconds || '',
        r.type,
        r.status,
        tags,
        categories
      ].join(',')
    })

    return [headers.join(','), ...rows].join('\n')
  }

  // 导出数据库文件
  const exportDatabase = (): Uint8Array => {
    if (!db.value) throw new Error('数据库未初始化')
    return db.value.export()
  }

  // 导入数据库文件
  const importDatabase = async (data: Uint8Array): Promise<boolean> => {
    try {
      const SQL = await initSqlJs()
      db.value = new SQL.Database(data)
      saveToStorage()
      return true
    } catch (e) {
      error.value = `导入失败: ${e}`
      return false
    }
  }

  // ==================== 统计功能 ====================

  const getStats = () => {
    if (!db.value) throw new Error('数据库未初始化')

    const totalRecordings = db.value.exec('SELECT COUNT(*) FROM recordings')[0]?.values[0]?.[0] || 0
    const totalDuration = db.value.exec('SELECT SUM(duration_seconds) FROM recordings WHERE status = "completed"')[0]?.values[0]?.[0] || 0
    const totalSize = db.value.exec('SELECT SUM(file_size_bytes) FROM recordings')[0]?.values[0]?.[0] || 0
    const byType = db.value.exec('SELECT type, COUNT(*) FROM recordings GROUP BY type')
    const byVehicle = db.value.exec('SELECT vehicle_id, COUNT(*) FROM recordings GROUP BY vehicle_id')

    return {
      totalRecordings,
      totalDuration,
      totalSize,
      byType: byType[0]?.values.reduce((acc, [type, count]) => {
        acc[type as string] = count as number
        return acc
      }, {} as Record<string, number>) || {},
      byVehicle: byVehicle[0]?.values.reduce((acc, [vehicle, count]) => {
        acc[vehicle as string] = count as number
        return acc
      }, {} as Record<string, number>) || {}
    }
  }

  // 辅助函数: 行转对象
  const rowToRecording = (row: Record<string, any>): Recording => ({
    id: row.id,
    sessionId: row.session_id,
    vehicleId: row.vehicle_id,
    startTime: row.start_time ? new Date(row.start_time) : new Date(),
    endTime: row.end_time ? new Date(row.end_time) : undefined,
    durationSeconds: row.duration_seconds,
    filePath: row.file_path,
    fileSizeBytes: row.file_size_bytes,
    type: row.type,
    status: row.status,
    createdAt: new Date(row.created_at)
  })

  return {
    // 状态
    db,
    isReady,
    error,

    // 初始化
    initialize,

    // 录制操作
    createRecording,
    updateRecording,
    getRecording,
    searchRecordings,
    deleteRecording,

    // 标签操作
    addTag,
    getTags,
    removeTag,

    // 分类操作
    getCategories,
    createCategory,
    setRecordingCategories,
    getRecordingCategories,

    // 导入导出
    exportToJson,
    exportToCsv,
    exportDatabase,
    importDatabase,

    // 统计
    getStats,

    // 持久化
    saveToStorage,
  }
}
