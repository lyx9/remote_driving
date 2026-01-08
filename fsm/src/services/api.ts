/**
 * FSM-Pilot REST API 服务
 * 用于与云端服务器进行HTTP通信
 */

export interface ApiConfig {
  baseUrl: string
  timeout?: number
}

export interface VehicleInfo {
  id: string
  name: string
  type: 'ROBO-TAXI' | 'LOGISTICS' | 'SECURITY'
  status: 'ACTIVE' | 'IDLE' | 'PATROL' | 'OFFLINE'
  location: { lat: number; lng: number }
  battery_level: number
  speed: number
  control_mode: 'MANUAL' | 'AUTONOMOUS' | 'REMOTE'
  is_connected: boolean
  last_update: number
}

export interface SchedulingQueueItem {
  vehicle_id: string
  priority_score: number
  emergency_level: number
  latency_ms: number
  task_status: string
  reason: string
}

export interface SchedulingConfig {
  enabled: boolean
  algorithm: string
  weights: {
    emergency: number
    latency: number
    distance: number
    battery: number
    task_priority: number
  }
}

export interface Alert {
  id: string
  vehicle_id: string
  type: string
  severity: string
  title: string
  message: string
  timestamp: number
  acknowledged: boolean
}

export interface PredictionConfig {
  enabled: boolean
  compensation_enabled: boolean
  model_endpoint: string
  prediction_horizon: number
}

class ApiService {
  private config: ApiConfig

  constructor(config: ApiConfig) {
    this.config = {
      timeout: 10000,
      ...config
    }
  }

  private async request<T>(
    method: string,
    path: string,
    data?: object
  ): Promise<T> {
    const url = `${this.config.baseUrl}${path}`

    const options: RequestInit = {
      method,
      headers: {
        'Content-Type': 'application/json'
      }
    }

    if (data) {
      options.body = JSON.stringify(data)
    }

    const controller = new AbortController()
    const timeoutId = setTimeout(() => controller.abort(), this.config.timeout)
    options.signal = controller.signal

    try {
      const response = await fetch(url, options)
      clearTimeout(timeoutId)

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`)
      }

      return await response.json()
    } catch (error) {
      clearTimeout(timeoutId)
      throw error
    }
  }

  // ============ 车辆管理 API ============

  /**
   * 获取所有车辆列表
   */
  async getVehicles(): Promise<VehicleInfo[]> {
    return this.request<VehicleInfo[]>('GET', '/api/v1/vehicles')
  }

  /**
   * 获取单个车辆详情
   */
  async getVehicle(vehicleId: string): Promise<VehicleInfo> {
    return this.request<VehicleInfo>('GET', `/api/v1/vehicles/${vehicleId}`)
  }

  /**
   * 获取车辆实时状态
   */
  async getVehicleStatus(vehicleId: string): Promise<{
    vehicle_id: string
    speed: number
    steering: number
    gear: string
    location: { lat: number; lng: number }
    latency_ms: number
    timestamp: number
  }> {
    return this.request('GET', `/api/v1/vehicles/${vehicleId}/status`)
  }

  /**
   * 建立WebRTC连接
   */
  async connectVehicle(vehicleId: string): Promise<{
    session_id: string
    signaling_url: string
    ice_servers: Array<{ urls: string; username?: string; credential?: string }>
  }> {
    return this.request('POST', `/api/v1/vehicles/${vehicleId}/connect`)
  }

  /**
   * 断开连接
   */
  async disconnectVehicle(vehicleId: string): Promise<void> {
    return this.request('DELETE', `/api/v1/vehicles/${vehicleId}/connect`)
  }

  // ============ 调度 API ============

  /**
   * 获取调度队列
   */
  async getSchedulingQueue(): Promise<{
    queue: SchedulingQueueItem[]
    algorithm: string
    scheduling_enabled: boolean
    timestamp: number
  }> {
    return this.request('GET', '/api/v1/scheduling/queue')
  }

  /**
   * 获取调度配置
   */
  async getSchedulingConfig(): Promise<SchedulingConfig> {
    return this.request('GET', '/api/v1/scheduling/config')
  }

  /**
   * 更新调度配置
   */
  async updateSchedulingConfig(config: Partial<SchedulingConfig>): Promise<SchedulingConfig> {
    return this.request('PUT', '/api/v1/scheduling/config', config)
  }

  /**
   * 启用/禁用调度
   */
  async setSchedulingEnabled(enabled: boolean): Promise<void> {
    return this.request('PUT', '/api/v1/scheduling/config', { enabled })
  }

  // ============ 告警 API ============

  /**
   * 获取告警列表
   */
  async getAlerts(params?: {
    vehicle_id?: string
    severity?: string
    acknowledged?: boolean
    limit?: number
  }): Promise<Alert[]> {
    const query = new URLSearchParams()
    if (params) {
      Object.entries(params).forEach(([key, value]) => {
        if (value !== undefined) {
          query.append(key, String(value))
        }
      })
    }
    const queryString = query.toString()
    const path = queryString ? `/api/v1/alerts?${queryString}` : '/api/v1/alerts'
    return this.request('GET', path)
  }

  /**
   * 确认告警
   */
  async acknowledgeAlert(alertId: string): Promise<void> {
    return this.request('POST', `/api/v1/alerts/${alertId}/ack`)
  }

  /**
   * 批量确认告警
   */
  async acknowledgeAlerts(alertIds: string[]): Promise<void> {
    return this.request('POST', '/api/v1/alerts/ack', { alert_ids: alertIds })
  }

  // ============ 预测模型 API ============

  /**
   * 获取预测配置
   */
  async getPredictionConfig(): Promise<PredictionConfig> {
    return this.request('GET', '/api/v1/prediction/config')
  }

  /**
   * 更新预测配置
   */
  async updatePredictionConfig(config: Partial<PredictionConfig>): Promise<PredictionConfig> {
    return this.request('PUT', '/api/v1/prediction/config', config)
  }

  /**
   * 获取预测轨迹
   */
  async getPredictedTrajectory(vehicleId: string): Promise<{
    vehicle_id: string
    points: Array<{
      x: number
      y: number
      velocity: number
      time_from_start: number
    }>
    confidence: number
    timestamp: number
  }> {
    return this.request('GET', `/api/v1/prediction/${vehicleId}/trajectory`)
  }

  // ============ 系统 API ============

  /**
   * 健康检查
   */
  async healthCheck(): Promise<{
    status: 'healthy' | 'degraded' | 'unhealthy'
    services: {
      signaling: boolean
      database: boolean
      turn_server: boolean
    }
    uptime_seconds: number
  }> {
    return this.request('GET', '/api/v1/health')
  }

  /**
   * 获取系统统计
   */
  async getSystemStats(): Promise<{
    connected_vehicles: number
    total_vehicles: number
    active_sessions: number
    total_bandwidth_mbps: number
    avg_latency_ms: number
  }> {
    return this.request('GET', '/api/v1/system/stats')
  }
}

// 单例实例
let apiInstance: ApiService | null = null

export function getApiService(config?: ApiConfig): ApiService {
  if (!apiInstance && config) {
    apiInstance = new ApiService(config)
  }
  if (!apiInstance) {
    throw new Error('API service not initialized')
  }
  return apiInstance
}

export function initApi(config: ApiConfig): ApiService {
  apiInstance = new ApiService(config)
  return apiInstance
}

export { ApiService }
