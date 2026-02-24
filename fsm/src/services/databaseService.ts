/**
 * FSM-Pilot V2.0 - Database Visualization Service
 *
 * 数据库可视化服务 - 用于存储、查询和导出自动驾驶场景数据
 *
 * @author Li Yixiang
 * @institution City University of Hong Kong
 */

// ======================== 类型定义 ========================

export interface VehicleRecord {
  id: string
  timestamp: number
  location: {
    longitude: number
    latitude: number
    street?: string
    district?: string
  }
  riskScore: number
  urgencyLevel: 'critical' | 'high' | 'medium' | 'low'
  scenario: string
  weather: string
  speed: number
  controlMode: string
}

export interface TakeoverEvent {
  id: string
  vehicleId: string
  operatorId: string
  timestamp: number
  reason: string
  riskScore: number
  duration: number
  outcome: 'success' | 'failed' | 'cancelled'
  aiAnalysis?: string
}

export interface AIAnalysisRecord {
  id: string
  vehicleId: string
  timestamp: number
  incidentDescription: string
  riskFactors: Array<{
    factor: string
    severity: string
    value: any
  }>
  recommendations: string[]
  confidence: number
}

export interface OperatorRecord {
  id: string
  name: string
  status: 'idle' | 'busy' | 'offline'
  assignedVehicles: string[]
  totalTakeovers: number
  successRate: number
  averageResponseTime: number
}

export interface DatabaseStats {
  totalVehicles: number
  totalTakeovers: number
  totalAIAnalyses: number
  totalOperators: number
  dateRange: {
    start: number
    end: number
  }
}

// ======================== IndexedDB 配置 ========================

const DB_NAME = 'FSM_Pilot_DB'
const DB_VERSION = 1

const STORES = {
  VEHICLES: 'vehicles',
  TAKEOVERS: 'takeover_events',
  AI_ANALYSES: 'ai_analyses',
  OPERATORS: 'operators',
  TELEMETRY: 'telemetry'
}

// ======================== 数据库服务类 ========================

class DatabaseService {
  private db: IDBDatabase | null = null
  private initialized = false

  /**
   * 初始化数据库
   */
  async initialize(): Promise<void> {
    if (this.initialized) return

    return new Promise((resolve, reject) => {
      const request = indexedDB.open(DB_NAME, DB_VERSION)

      request.onerror = () => {
        console.error('[DB] Failed to open database:', request.error)
        reject(request.error)
      }

      request.onsuccess = () => {
        this.db = request.result
        this.initialized = true
        console.log('[DB] Database initialized successfully')
        resolve()
      }

      request.onupgradeneeded = (event) => {
        const db = (event.target as IDBOpenDBRequest).result

        // 创建 vehicles 表
        if (!db.objectStoreNames.contains(STORES.VEHICLES)) {
          const vehicleStore = db.createObjectStore(STORES.VEHICLES, { keyPath: 'id' })
          vehicleStore.createIndex('timestamp', 'timestamp', { unique: false })
          vehicleStore.createIndex('urgencyLevel', 'urgencyLevel', { unique: false })
          vehicleStore.createIndex('scenario', 'scenario', { unique: false })
        }

        // 创建 takeover_events 表
        if (!db.objectStoreNames.contains(STORES.TAKEOVERS)) {
          const takeoverStore = db.createObjectStore(STORES.TAKEOVERS, { keyPath: 'id' })
          takeoverStore.createIndex('vehicleId', 'vehicleId', { unique: false })
          takeoverStore.createIndex('operatorId', 'operatorId', { unique: false })
          takeoverStore.createIndex('timestamp', 'timestamp', { unique: false })
          takeoverStore.createIndex('outcome', 'outcome', { unique: false })
        }

        // 创建 ai_analyses 表
        if (!db.objectStoreNames.contains(STORES.AI_ANALYSES)) {
          const aiStore = db.createObjectStore(STORES.AI_ANALYSES, { keyPath: 'id' })
          aiStore.createIndex('vehicleId', 'vehicleId', { unique: false })
          aiStore.createIndex('timestamp', 'timestamp', { unique: false })
        }

        // 创建 operators 表
        if (!db.objectStoreNames.contains(STORES.OPERATORS)) {
          const operatorStore = db.createObjectStore(STORES.OPERATORS, { keyPath: 'id' })
          operatorStore.createIndex('status', 'status', { unique: false })
        }

        // 创建 telemetry 表
        if (!db.objectStoreNames.contains(STORES.TELEMETRY)) {
          const telemetryStore = db.createObjectStore(STORES.TELEMETRY, {
            keyPath: 'id',
            autoIncrement: true
          })
          telemetryStore.createIndex('vehicleId', 'vehicleId', { unique: false })
          telemetryStore.createIndex('timestamp', 'timestamp', { unique: false })
        }

        console.log('[DB] Database schema created')
      }
    })
  }

  /**
   * 保存车辆记录
   */
  async saveVehicle(vehicle: VehicleRecord): Promise<void> {
    await this.ensureInitialized()
    return this.put(STORES.VEHICLES, vehicle)
  }

  /**
   * 保存接管事件
   */
  async saveTakeoverEvent(event: TakeoverEvent): Promise<void> {
    await this.ensureInitialized()
    return this.put(STORES.TAKEOVERS, event)
  }

  /**
   * 保存 AI 分析记录
   */
  async saveAIAnalysis(analysis: AIAnalysisRecord): Promise<void> {
    await this.ensureInitialized()
    return this.put(STORES.AI_ANALYSES, analysis)
  }

  /**
   * 保存操作员记录
   */
  async saveOperator(operator: OperatorRecord): Promise<void> {
    await this.ensureInitialized()
    return this.put(STORES.OPERATORS, operator)
  }

  /**
   * 获取所有车辆记录
   */
  async getAllVehicles(): Promise<VehicleRecord[]> {
    await this.ensureInitialized()
    return this.getAll(STORES.VEHICLES)
  }

  /**
   * 获取所有接管事件
   */
  async getAllTakeovers(): Promise<TakeoverEvent[]> {
    await this.ensureInitialized()
    return this.getAll(STORES.TAKEOVERS)
  }

  /**
   * 获取所有 AI 分析
   */
  async getAllAIAnalyses(): Promise<AIAnalysisRecord[]> {
    await this.ensureInitialized()
    return this.getAll(STORES.AI_ANALYSES)
  }

  /**
   * 获取所有操作员
   */
  async getAllOperators(): Promise<OperatorRecord[]> {
    await this.ensureInitialized()
    return this.getAll(STORES.OPERATORS)
  }

  /**
   * 获取数据库统计信息
   */
  async getStats(): Promise<DatabaseStats> {
    await this.ensureInitialized()

    const [vehicles, takeovers, analyses, operators] = await Promise.all([
      this.getAllVehicles(),
      this.getAllTakeovers(),
      this.getAllAIAnalyses(),
      this.getAllOperators()
    ])

    const allTimestamps = [
      ...vehicles.map(v => v.timestamp),
      ...takeovers.map(t => t.timestamp),
      ...analyses.map(a => a.timestamp)
    ]

    return {
      totalVehicles: vehicles.length,
      totalTakeovers: takeovers.length,
      totalAIAnalyses: analyses.length,
      totalOperators: operators.length,
      dateRange: {
        start: allTimestamps.length > 0 ? Math.min(...allTimestamps) : Date.now(),
        end: allTimestamps.length > 0 ? Math.max(...allTimestamps) : Date.now()
      }
    }
  }

  /**
   * 导出所有数据为 JSON
   */
  async exportToJSON(): Promise<string> {
    await this.ensureInitialized()

    const [vehicles, takeovers, analyses, operators] = await Promise.all([
      this.getAllVehicles(),
      this.getAllTakeovers(),
      this.getAllAIAnalyses(),
      this.getAllOperators()
    ])

    const exportData = {
      version: '2.0',
      exportTime: new Date().toISOString(),
      data: {
        vehicles,
        takeovers,
        aiAnalyses: analyses,
        operators
      },
      stats: await this.getStats()
    }

    return JSON.stringify(exportData, null, 2)
  }

  /**
   * 从 JSON 导入数据
   */
  async importFromJSON(jsonString: string): Promise<void> {
    await this.ensureInitialized()

    try {
      const importData = JSON.parse(jsonString)

      if (!importData.data) {
        throw new Error('Invalid import data format')
      }

      const { vehicles, takeovers, aiAnalyses, operators } = importData.data

      // 批量导入数据
      if (vehicles) {
        for (const vehicle of vehicles) {
          await this.saveVehicle(vehicle)
        }
      }

      if (takeovers) {
        for (const takeover of takeovers) {
          await this.saveTakeoverEvent(takeover)
        }
      }

      if (aiAnalyses) {
        for (const analysis of aiAnalyses) {
          await this.saveAIAnalysis(analysis)
        }
      }

      if (operators) {
        for (const operator of operators) {
          await this.saveOperator(operator)
        }
      }

      console.log('[DB] Data imported successfully')
    } catch (error) {
      console.error('[DB] Import failed:', error)
      throw error
    }
  }

  /**
   * 清空所有数据
   */
  async clearAll(): Promise<void> {
    await this.ensureInitialized()

    const stores = [
      STORES.VEHICLES,
      STORES.TAKEOVERS,
      STORES.AI_ANALYSES,
      STORES.OPERATORS,
      STORES.TELEMETRY
    ]

    for (const storeName of stores) {
      await this.clear(storeName)
    }

    console.log('[DB] All data cleared')
  }

  // ======================== 私有辅助方法 ========================

  private async ensureInitialized(): Promise<void> {
    if (!this.initialized) {
      await this.initialize()
    }
  }

  private put(storeName: string, data: any): Promise<void> {
    return new Promise((resolve, reject) => {
      if (!this.db) {
        reject(new Error('Database not initialized'))
        return
      }

      const transaction = this.db.transaction([storeName], 'readwrite')
      const store = transaction.objectStore(storeName)
      const request = store.put(data)

      request.onsuccess = () => resolve()
      request.onerror = () => reject(request.error)
    })
  }

  private getAll(storeName: string): Promise<any[]> {
    return new Promise((resolve, reject) => {
      if (!this.db) {
        reject(new Error('Database not initialized'))
        return
      }

      const transaction = this.db.transaction([storeName], 'readonly')
      const store = transaction.objectStore(storeName)
      const request = store.getAll()

      request.onsuccess = () => resolve(request.result)
      request.onerror = () => reject(request.error)
    })
  }

  private clear(storeName: string): Promise<void> {
    return new Promise((resolve, reject) => {
      if (!this.db) {
        reject(new Error('Database not initialized'))
        return
      }

      const transaction = this.db.transaction([storeName], 'readwrite')
      const store = transaction.objectStore(storeName)
      const request = store.clear()

      request.onsuccess = () => resolve()
      request.onerror = () => reject(request.error)
    })
  }
}

// 创建单例
export const databaseService = new DatabaseService()

export default databaseService
