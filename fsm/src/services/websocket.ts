/**
 * FSM-Pilot WebSocket 服务
 * 用于与云端服务器实时通信
 */

import { useFleetStore } from '@/stores/fleet'
import { useSystemStore } from '@/stores/system'
import type { Vehicle } from '@/types'

export interface WebSocketConfig {
  url: string
  reconnectInterval?: number
  maxReconnectAttempts?: number
}

export interface VehicleStatusMessage {
  event: 'vehicle_status'
  data: {
    vehicle_id: string
    timestamp: number
    speed: number
    steering: number
    gear: string
    location: { lat: number; lng: number }
    latency_ms: number
    cpu_usage: number
    memory_usage: number
    battery_level: number
    control_mode: number
  }
}

export interface AlertMessage {
  event: 'alert'
  data: {
    id: string
    vehicle_id: string
    type: string
    severity: 'info' | 'warning' | 'error' | 'critical'
    title: string
    message: string
    timestamp: number
  }
}

export interface SchedulingUpdateMessage {
  event: 'scheduling_update'
  data: {
    queue: Array<{
      vehicle_id: string
      priority: number
      reason: string
      latency_ms: number
      emergency_level: number
    }>
  }
}

export interface LatencyMessage {
  event: 'latency_update'
  data: {
    vehicle_id: string
    rtt_ms: number
    video_latency_ms: number
    control_latency_ms: number
    jitter_ms: number
  }
}

export type WebSocketMessage =
  | VehicleStatusMessage
  | AlertMessage
  | SchedulingUpdateMessage
  | LatencyMessage

type MessageHandler = (message: WebSocketMessage) => void

export class WebSocketService {
  private ws: WebSocket | null = null
  private config: WebSocketConfig
  private reconnectAttempts = 0
  private reconnectTimer: number | null = null
  private handlers: Map<string, MessageHandler[]> = new Map()
  private isConnecting = false

  constructor(config: WebSocketConfig) {
    this.config = {
      reconnectInterval: 5000,
      maxReconnectAttempts: 10,
      ...config
    }
  }

  /**
   * 连接到WebSocket服务器
   */
  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      if (this.ws && this.ws.readyState === WebSocket.OPEN) {
        resolve()
        return
      }

      if (this.isConnecting) {
        reject(new Error('Already connecting'))
        return
      }

      this.isConnecting = true

      try {
        this.ws = new WebSocket(this.config.url)

        this.ws.onopen = () => {
          console.log('[WebSocket] Connected')
          this.isConnecting = false
          this.reconnectAttempts = 0
          this.emit({ event: 'connection', data: { status: 'connected' } } as any)
          resolve()
        }

        this.ws.onmessage = (event) => {
          try {
            const message: WebSocketMessage = JSON.parse(event.data)
            this.handleMessage(message)
          } catch (e) {
            console.error('[WebSocket] Failed to parse message:', e)
          }
        }

        this.ws.onerror = (error) => {
          console.error('[WebSocket] Error:', error)
          this.isConnecting = false
          reject(error)
        }

        this.ws.onclose = (event) => {
          console.log('[WebSocket] Disconnected:', event.code, event.reason)
          this.isConnecting = false
          this.emit({ event: 'connection', data: { status: 'disconnected' } } as any)
          this.scheduleReconnect()
        }
      } catch (error) {
        this.isConnecting = false
        reject(error)
      }
    })
  }

  /**
   * 断开连接
   */
  disconnect(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer)
      this.reconnectTimer = null
    }

    if (this.ws) {
      this.ws.close()
      this.ws = null
    }
  }

  /**
   * 发送消息
   */
  send(message: object): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message))
    } else {
      console.warn('[WebSocket] Not connected, message not sent')
    }
  }

  /**
   * 订阅消息事件
   */
  on(event: string, handler: MessageHandler): void {
    if (!this.handlers.has(event)) {
      this.handlers.set(event, [])
    }
    this.handlers.get(event)!.push(handler)
  }

  /**
   * 取消订阅
   */
  off(event: string, handler: MessageHandler): void {
    const handlers = this.handlers.get(event)
    if (handlers) {
      const index = handlers.indexOf(handler)
      if (index > -1) {
        handlers.splice(index, 1)
      }
    }
  }

  /**
   * 发送心跳
   */
  sendHeartbeat(): void {
    this.send({
      event: 'heartbeat',
      timestamp: Date.now()
    })
  }

  /**
   * 发送控制指令
   */
  sendControlCommand(vehicleId: string, command: {
    steering: number
    throttle: number
    brake: number
    gear?: number
    emergency?: boolean
  }): void {
    this.send({
      event: 'control_command',
      data: {
        vehicle_id: vehicleId,
        timestamp: Date.now(),
        ...command
      }
    })
  }

  /**
   * 请求切换车辆
   */
  requestVehicleSwitch(vehicleId: string): void {
    this.send({
      event: 'switch_vehicle',
      data: {
        vehicle_id: vehicleId,
        timestamp: Date.now()
      }
    })
  }

  private handleMessage(message: WebSocketMessage): void {
    // 触发特定事件处理器
    this.emit(message)

    // 更新stores
    const fleetStore = useFleetStore()
    const systemStore = useSystemStore()

    switch (message.event) {
      case 'vehicle_status':
        this.handleVehicleStatus(message as VehicleStatusMessage)
        break

      case 'alert':
        this.handleAlert(message as AlertMessage)
        break

      case 'scheduling_update':
        this.handleSchedulingUpdate(message as SchedulingUpdateMessage)
        break

      case 'latency_update':
        this.handleLatencyUpdate(message as LatencyMessage)
        break
    }
  }

  private handleVehicleStatus(message: VehicleStatusMessage): void {
    const fleetStore = useFleetStore()
    const { data } = message

    // 更新车辆状态
    const vehicle = fleetStore.vehicles.find(v => v.id === data.vehicle_id)
    if (vehicle) {
      vehicle.speed = data.speed
      vehicle.location = [data.location.lat, data.location.lng]
      // 更新其他状态...
    }

    // 如果是当前选中的车辆，更新详细状态
    if (fleetStore.currentVehicle.id === data.vehicle_id) {
      fleetStore.updateVehicleSpeed(data.speed)
      fleetStore.updateVehicleSteering(data.steering)
    }
  }

  private handleAlert(message: AlertMessage): void {
    const systemStore = useSystemStore()
    const { data } = message

    // 添加到日志
    const level = data.severity === 'critical' ? 'error' :
                  data.severity === 'warning' ? 'warning' : 'info'
    systemStore.addLog(`[${data.vehicle_id}] ${data.title}: ${data.message}`, level)

    // 触发告警回调
    this.emit(message)
  }

  private handleSchedulingUpdate(message: SchedulingUpdateMessage): void {
    // 更新调度队列显示
    // 可以添加专门的scheduling store
    console.log('[WebSocket] Scheduling update:', message.data.queue)
  }

  private handleLatencyUpdate(message: LatencyMessage): void {
    const { data } = message
    // 更新延迟显示
    console.log(`[WebSocket] Latency for ${data.vehicle_id}: RTT=${data.rtt_ms}ms`)
  }

  private emit(message: WebSocketMessage): void {
    const handlers = this.handlers.get(message.event)
    if (handlers) {
      handlers.forEach(handler => handler(message))
    }

    // 触发通用处理器
    const allHandlers = this.handlers.get('*')
    if (allHandlers) {
      allHandlers.forEach(handler => handler(message))
    }
  }

  private scheduleReconnect(): void {
    if (this.reconnectAttempts >= (this.config.maxReconnectAttempts || 10)) {
      console.error('[WebSocket] Max reconnect attempts reached')
      return
    }

    this.reconnectAttempts++
    console.log(`[WebSocket] Reconnecting in ${this.config.reconnectInterval}ms (attempt ${this.reconnectAttempts})`)

    this.reconnectTimer = window.setTimeout(() => {
      this.connect().catch(e => {
        console.error('[WebSocket] Reconnect failed:', e)
      })
    }, this.config.reconnectInterval)
  }

  get isConnected(): boolean {
    return this.ws !== null && this.ws.readyState === WebSocket.OPEN
  }
}

// 单例实例
let wsInstance: WebSocketService | null = null

export function getWebSocketService(config?: WebSocketConfig): WebSocketService {
  if (!wsInstance && config) {
    wsInstance = new WebSocketService(config)
  }
  if (!wsInstance) {
    throw new Error('WebSocket service not initialized')
  }
  return wsInstance
}

export function initWebSocket(config: WebSocketConfig): WebSocketService {
  wsInstance = new WebSocketService(config)
  return wsInstance
}
