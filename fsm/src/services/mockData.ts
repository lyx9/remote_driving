/**
 * FSM-Pilot Mock 数据服务
 * 用于演示模式，连接到 Mock 服务器获取模拟数据
 */

import { ref, onMounted, onUnmounted } from 'vue'
import { useFleetStore, type ExtendedVehicle } from '@/stores/fleet'
import { useSystemStore } from '@/stores/system'

// Mock 服务器配置
const MOCK_CONFIG = {
  HTTP_URL: 'http://localhost:3001',
  WS_URL: 'ws://localhost:3002',
  RECONNECT_INTERVAL: 3000,
}

export interface MockVideoFrame {
  camera_id: string
  data: string // Base64 encoded
  width: number
  height: number
  timestamp: number
}

export function useMockData() {
  const fleetStore = useFleetStore()
  const systemStore = useSystemStore()

  const isConnected = ref(false)
  const connectionState = ref<'disconnected' | 'connecting' | 'connected'>('disconnected')
  let ws: WebSocket | null = null
  let reconnectTimer: number | null = null

  // 连接到 Mock 服务器
  function connect() {
    if (ws && ws.readyState === WebSocket.OPEN) {
      return
    }

    connectionState.value = 'connecting'
    systemStore.addLog('Connecting to Mock server...', 'info')

    try {
      ws = new WebSocket(MOCK_CONFIG.WS_URL)

      ws.onopen = () => {
        isConnected.value = true
        connectionState.value = 'connected'
        systemStore.addLog('Connected to Mock server', 'info')

        // 清除重连定时器
        if (reconnectTimer) {
          clearTimeout(reconnectTimer)
          reconnectTimer = null
        }
      }

      ws.onclose = () => {
        isConnected.value = false
        connectionState.value = 'disconnected'
        systemStore.addLog('Disconnected from Mock server', 'warning')

        // 自动重连
        scheduleReconnect()
      }

      ws.onerror = (error) => {
        console.error('[Mock] WebSocket error:', error)
        systemStore.addLog('Mock server connection error', 'error')
      }

      ws.onmessage = (event) => {
        handleMessage(JSON.parse(event.data))
      }

    } catch (error) {
      console.error('[Mock] Connection error:', error)
      systemStore.addLog(`Mock connection failed: ${error}`, 'error')
      scheduleReconnect()
    }
  }

  // 断开连接
  function disconnect() {
    if (reconnectTimer) {
      clearTimeout(reconnectTimer)
      reconnectTimer = null
    }

    if (ws) {
      ws.close()
      ws = null
    }

    isConnected.value = false
    connectionState.value = 'disconnected'
  }

  // 安排重连
  function scheduleReconnect() {
    if (reconnectTimer) return

    reconnectTimer = window.setTimeout(() => {
      reconnectTimer = null
      if (!isConnected.value) {
        console.log('[Mock] Attempting reconnect...')
        connect()
      }
    }, MOCK_CONFIG.RECONNECT_INTERVAL)
  }

  // 处理消息
  function handleMessage(message: any) {
    switch (message.event) {
      case 'vehicles_list':
        handleVehiclesList(message.data)
        break

      case 'vehicle_status':
        handleVehicleStatus(message.data)
        break

      case 'scheduling_update':
        handleSchedulingUpdate(message.data)
        break

      case 'alert':
        handleAlert(message.data)
        break

      case 'latency_update':
        handleLatencyUpdate(message.data)
        break

      case 'emergency':
        handleEmergency(message.data)
        break
    }
  }

  // 处理车辆列表
  function handleVehiclesList(vehicles: any[]) {
    vehicles.forEach(v => {
      const existing = fleetStore.vehicles.find(ev => ev.id === v.id)
      if (!existing) {
        // 添加新车辆
        fleetStore.addVehicle({
          id: v.id,
          type: v.type as 'ROBO-TAXI' | 'LOGISTICS' | 'SECURITY',
          status: v.status === 'online' ? 'ACTIVE' : 'IDLE',
          money: 0,
          mode: 'ECO',
          location: [31.2304, 121.4737],
          path: [],
          speed: 0,
          gear: 'P',
          steering: 0,
          connectionState: 'disconnected',
          latency_ms: 0,
          battery_level: 100,
          emergency_level: 0,
          priority_score: 50,
          telemetry: {
            cpu_usage: 0,
            gpu_usage: 0,
            memory_usage: 0,
            network_quality: 'good',
            signal_strength: 80,
          },
          sensors: {
            cameras_online: 5,
            cameras_total: 5,
            lidar_online: true,
            gps_fix: true,
          },
        })
      }
    })
  }

  // 处理车辆状态更新
  function handleVehicleStatus(data: any) {
    const vehicle = fleetStore.vehicles.find(v => v.id === data.vehicle_id)
    if (vehicle) {
      vehicle.speed = data.speed || 0
      vehicle.steering = data.steering || 0
      vehicle.gear = data.gear || 'P'

      if (data.location) {
        vehicle.location = [data.location.lat, data.location.lng]
      }

      vehicle.latency_ms = data.latency_ms || 0
      vehicle.battery_level = data.battery_level || 100
      vehicle.connectionState = data.connection_state || 'connected'

      // 更新网络质量
      if (data.latency_ms < 50) {
        vehicle.telemetry.network_quality = 'excellent'
      } else if (data.latency_ms < 100) {
        vehicle.telemetry.network_quality = 'good'
      } else if (data.latency_ms < 200) {
        vehicle.telemetry.network_quality = 'fair'
      } else {
        vehicle.telemetry.network_quality = 'poor'
      }
    }
  }

  // 处理调度更新
  function handleSchedulingUpdate(data: any) {
    data.queue.forEach((item: any, index: number) => {
      const vehicle = fleetStore.vehicles.find(v => v.id === item.vehicle_id)
      if (vehicle) {
        vehicle.priority_score = item.priority
        vehicle.emergency_level = item.emergency_level || 0
      }
    })
  }

  // 处理告警
  function handleAlert(alert: any) {
    const level = alert.severity === 'critical' ? 'error' :
                  alert.severity === 'warning' ? 'warning' : 'info'
    systemStore.addLog(`[${alert.vehicle_id}] ${alert.message}`, level)
  }

  // 处理延迟更新
  function handleLatencyUpdate(data: any) {
    const vehicle = fleetStore.vehicles.find(v => v.id === data.vehicle_id)
    if (vehicle) {
      vehicle.latency_ms = data.rtt_ms
    }
  }

  // 处理紧急状态
  function handleEmergency(data: any) {
    const vehicle = fleetStore.vehicles.find(v => v.id === data.vehicle_id)
    if (vehicle) {
      if (data.active) {
        vehicle.emergency_level = 4
        vehicle.speed = 0
        vehicle.gear = 'P'
        systemStore.addLog(`[${data.vehicle_id}] EMERGENCY STOP ACTIVATED`, 'error')
      } else {
        vehicle.emergency_level = 0
        systemStore.addLog(`[${data.vehicle_id}] Emergency state released`, 'info')
      }
    }
  }

  // 发送控制指令
  function sendControl(vehicleId: string, command: {
    steering?: number
    throttle?: number
    brake?: number
    gear?: number
    turn_signal?: number
    emergency?: boolean
  }) {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({
        type: 'control',
        vehicle_id: vehicleId,
        ...command,
      }))
    }
  }

  // 订阅车辆
  function subscribeVehicle(vehicleId: string) {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({
        type: 'subscribe',
        vehicle_id: vehicleId,
      }))
    }
  }

  // 发送心跳
  function sendPing() {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({
        type: 'ping',
        sequence: Date.now(),
        timestamp: Date.now(),
      }))
    }
  }

  // 生命周期
  onMounted(() => {
    connect()

    // 定期发送心跳
    const pingInterval = setInterval(() => {
      if (isConnected.value) {
        sendPing()
      }
    }, 1000)

    onUnmounted(() => {
      clearInterval(pingInterval)
      disconnect()
    })
  })

  return {
    isConnected,
    connectionState,
    connect,
    disconnect,
    sendControl,
    subscribeVehicle,
  }
}

// ============== Mock API 调用 ==============

export const mockApi = {
  baseUrl: MOCK_CONFIG.HTTP_URL,

  async getVehicles() {
    const response = await fetch(`${this.baseUrl}/api/v1/vehicles`)
    return response.json()
  },

  async getVehicle(id: string) {
    const response = await fetch(`${this.baseUrl}/api/v1/vehicles/${id}`)
    return response.json()
  },

  async getVehicleStatus(id: string) {
    const response = await fetch(`${this.baseUrl}/api/v1/vehicles/${id}/status`)
    return response.json()
  },

  async getSchedulingQueue() {
    const response = await fetch(`${this.baseUrl}/api/v1/scheduling/queue`)
    return response.json()
  },

  async getAlerts() {
    const response = await fetch(`${this.baseUrl}/api/v1/alerts`)
    return response.json()
  },

  async simulateNetwork(type: 'delay' | 'jitter' | 'disconnect' | 'reset', params?: any) {
    const response = await fetch(`${this.baseUrl}/mock/simulate`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ type, ...params }),
    })
    return response.json()
  },

  async triggerEmergency(vehicleId: string, action: 'trigger' | 'release') {
    const response = await fetch(`${this.baseUrl}/mock/emergency`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ vehicle_id: vehicleId, action }),
    })
    return response.json()
  },

  async sendControl(vehicleId: string, command: any) {
    const response = await fetch(`${this.baseUrl}/mock/control`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ vehicle_id: vehicleId, ...command }),
    })
    return response.json()
  },
}

// ============== Mock 视频生成 ==============

/**
 * 生成模拟视频帧 (用于演示)
 * 返回一个带有简单动画的 Canvas
 */
export function createMockVideoStream(cameraId: string): HTMLCanvasElement {
  const canvas = document.createElement('canvas')
  canvas.width = 640
  canvas.height = 480
  const ctx = canvas.getContext('2d')!

  let frameCount = 0

  function drawFrame() {
    frameCount++

    // 背景渐变 (模拟道路)
    const gradient = ctx.createLinearGradient(0, 0, 0, canvas.height)
    gradient.addColorStop(0, '#87CEEB')  // 天空蓝
    gradient.addColorStop(0.5, '#87CEEB')
    gradient.addColorStop(0.5, '#555555')  // 道路灰
    gradient.addColorStop(1, '#333333')
    ctx.fillStyle = gradient
    ctx.fillRect(0, 0, canvas.width, canvas.height)

    // 道路标线 (动画)
    ctx.strokeStyle = '#FFFFFF'
    ctx.lineWidth = 4
    ctx.setLineDash([30, 20])
    ctx.lineDashOffset = -frameCount * 2
    ctx.beginPath()
    ctx.moveTo(canvas.width / 2, canvas.height / 2)
    ctx.lineTo(canvas.width / 2, canvas.height)
    ctx.stroke()
    ctx.setLineDash([])

    // 侧边线
    ctx.strokeStyle = '#FFFF00'
    ctx.lineWidth = 3
    ctx.beginPath()
    ctx.moveTo(50, canvas.height / 2 + 20)
    ctx.lineTo(0, canvas.height)
    ctx.stroke()
    ctx.beginPath()
    ctx.moveTo(canvas.width - 50, canvas.height / 2 + 20)
    ctx.lineTo(canvas.width, canvas.height)
    ctx.stroke()

    // 摄像头 ID
    ctx.fillStyle = '#FFFFFF'
    ctx.font = '16px monospace'
    ctx.fillText(cameraId.toUpperCase(), 10, 25)

    // 时间戳
    const now = new Date()
    ctx.fillText(now.toLocaleTimeString(), canvas.width - 80, 25)

    // 帧计数
    ctx.fillStyle = 'rgba(0, 255, 0, 0.8)'
    ctx.fillText(`Frame: ${frameCount}`, 10, canvas.height - 10)

    requestAnimationFrame(drawFrame)
  }

  drawFrame()
  return canvas
}

/**
 * 生成多路摄像头 Mock 流
 */
export function createMockCameraStreams(): Map<string, HTMLCanvasElement> {
  const cameras = new Map<string, HTMLCanvasElement>()

  const cameraIds = [
    'cam_front_center',
    'cam_front_left',
    'cam_front_right',
    'cam_rear_left',
    'cam_rear_right',
  ]

  cameraIds.forEach(id => {
    cameras.set(id, createMockVideoStream(id))
  })

  return cameras
}
