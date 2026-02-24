/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      Remote Control Composable
 * @description 远程控制模块，整合WebSocket、WebRTC和API服务，提供统一的远程控制接口
 *              注意：此模块需要在 main.ts 中初始化服务后使用
 */

import { ref, onMounted, onUnmounted, watch } from 'vue'
import { useFleetStore } from '@/stores/fleet'
import { useSystemStore } from '@/stores/system'
import { WebSocketService, type WebSocketConfig } from '@/services/websocket'
import { ApiService, type VehicleInfo, type SchedulingQueueItem, type SchedulingConfig } from '@/services/api'
import { webrtcManager, type LatencyInfo, type ConnectionState } from '@/services/webrtc'

// 扩展消息类型以匹配实际数据结构
interface ExtendedVehicleStatusMessage {
  event: 'vehicle_status'
  vehicle_id: string
  timestamp: number
  speed: number
  steering: number
  gear: string
  location?: { lat: number; lng: number }
  latency_ms?: number
}

interface ExtendedAlertMessage {
  event: 'alert'
  vehicle_id: string
  severity: 'info' | 'warning' | 'error' | 'critical'
  message: string
}

export interface SchedulingQueue {
  vehicles: SchedulingQueueItem[]
  algorithm: string
  updated_at: string
}

export interface RemoteControlState {
  isConnected: boolean
  connectionState: ConnectionState
  latency: LatencyInfo
  schedulingQueue: SchedulingQueue | null
}

export function useRemoteControl() {
  const fleetStore = useFleetStore()
  const systemStore = useSystemStore()

  // 服务实例 (懒初始化)
  let webSocketService: WebSocketService | null = null
  let apiService: ApiService | null = null

  // 状态
  const isConnected = ref(false)
  const connectionState = ref<ConnectionState>('disconnected')
  const latency = ref<LatencyInfo>({
    rtt_ms: 0,
    video_latency_ms: 0,
    control_latency_ms: 0,
    jitter_ms: 0
  })
  const schedulingQueue = ref<SchedulingQueue | null>(null)
  const isSchedulingEnabled = ref(true)

  // WebRTC 连接管理
  const webrtcConnections = new Map<string, ReturnType<typeof webrtcManager.createConnection>>()

  // 初始化服务
  function initServices() {
    const wsConfig: WebSocketConfig = {
      url: `wss://${window.location.hostname}:8080/ws`,
      reconnectInterval: 5000,
      maxReconnectAttempts: 10
    }

    webSocketService = new WebSocketService(wsConfig)
    apiService = new ApiService({
      baseUrl: `https://${window.location.hostname}:8080`
    })
  }

  // 初始化连接
  async function initialize() {
    try {
      initServices()

      if (!webSocketService) {
        throw new Error('WebSocket service not initialized')
      }

      // 1. 连接 WebSocket
      await webSocketService.connect()
      isConnected.value = true
      systemStore.addLog('WebSocket connected to cloud server', 'info')

      // 2. 设置 WebSocket 回调
      setupWebSocketCallbacks()

      // 3. 加载车辆列表
      await loadVehicles()

      // 4. 加载调度队列
      await loadSchedulingQueue()

      systemStore.addLog('Remote control system initialized', 'info')
    } catch (error) {
      systemStore.addLog(`Failed to initialize: ${error}`, 'error')
      isConnected.value = false
    }
  }

  // 设置 WebSocket 回调
  function setupWebSocketCallbacks() {
    if (!webSocketService) return

    webSocketService.on('vehicle_status', (msg) => {
      const status = msg as unknown as ExtendedVehicleStatusMessage
      updateVehicleFromStatus(status)
    })

    webSocketService.on('alert', (msg) => {
      const alert = msg as unknown as ExtendedAlertMessage
      handleAlert(alert)
    })

    webSocketService.on('scheduling_update', (msg) => {
      const data = msg as { data: { queue: Array<{ vehicle_id: string; priority: number; task_status?: string; reason?: string }> } }
      schedulingQueue.value = {
        vehicles: data.data.queue.map(item => ({
          vehicle_id: item.vehicle_id,
          priority_score: item.priority,
          emergency_level: 0,
          latency_ms: 0,
          task_status: item.task_status || 'pending',
          reason: item.reason || ''
        })),
        algorithm: 'weighted_priority',
        updated_at: new Date().toISOString()
      }
      systemStore.addLog('Scheduling queue updated', 'info')
    })

    webSocketService.on('latency_update', (msg) => {
      const info = msg as { data: { rtt_ms: number; video_latency_ms?: number; control_latency_ms?: number; jitter_ms?: number } }
      latency.value = {
        rtt_ms: info.data.rtt_ms,
        video_latency_ms: info.data.video_latency_ms || 0,
        control_latency_ms: info.data.control_latency_ms || 0,
        jitter_ms: info.data.jitter_ms || 0
      }
    })
  }

  // 从状态消息更新车辆
  function updateVehicleFromStatus(status: ExtendedVehicleStatusMessage) {
    const vehicleIndex = fleetStore.vehicles.findIndex(v => v.id === status.vehicle_id)
    if (vehicleIndex >= 0) {
      const vehicle = fleetStore.vehicles[vehicleIndex]
      vehicle.speed = status.speed
      vehicle.steering = status.steering
      vehicle.gear = status.gear as 'P' | 'R' | 'N' | 'D'
      if (status.location) {
        vehicle.location = [status.location.lat, status.location.lng]
      }
    }
  }

  // 处理告警
  function handleAlert(alert: ExtendedAlertMessage) {
    const level = alert.severity === 'critical' ? 'error' :
                  alert.severity === 'warning' ? 'warning' : 'info'
    systemStore.addLog(`[${alert.vehicle_id}] ${alert.message}`, level)
  }

  // 加载车辆列表
  async function loadVehicles() {
    if (!apiService) return

    try {
      const vehicles = await apiService.getVehicles()
      // 更新 store 中的车辆信息
      vehicles.forEach((vehicleInfo: VehicleInfo) => {
        const existing = fleetStore.vehicles.find(v => v.id === vehicleInfo.id)
        if (!existing) {
          fleetStore.addVehicle({
            id: vehicleInfo.id,
            type: vehicleInfo.type as 'ROBO-TAXI' | 'LOGISTICS' | 'SECURITY',
            status: vehicleInfo.status,
            money: 0,
            mode: 'ECO',
            location: vehicleInfo.location ? [vehicleInfo.location.lat, vehicleInfo.location.lng] : [31.2304, 121.4737],
            path: [],
            speed: vehicleInfo.speed || 0,
            gear: 'P',
            steering: 0,
            // Extended properties required by ExtendedVehicle
            connectionState: vehicleInfo.is_connected ? 'connected' : 'disconnected',
            latency_ms: 0,
            battery_level: vehicleInfo.battery_level || 100,
            emergency_level: 0,
            priority_score: 50,
            telemetry: {
              cpu_usage: 0,
              gpu_usage: 0,
              memory_usage: 0,
              network_quality: 'good',
              signal_strength: 80
            },
            sensors: {
              cameras_online: 5,
              cameras_total: 5,
              lidar_online: true,
              gps_fix: true
            }
          })
        }
      })
    } catch (error) {
      systemStore.addLog(`Failed to load vehicles: ${error}`, 'error')
    }
  }

  // 加载调度队列
  async function loadSchedulingQueue() {
    if (!apiService) return

    try {
      const queue = await apiService.getSchedulingQueue()
      schedulingQueue.value = queue as unknown as SchedulingQueue
    } catch (error) {
      systemStore.addLog(`Failed to load scheduling queue: ${error}`, 'error')
    }
  }

  // 连接到车辆 (WebRTC)
  async function connectToVehicle(vehicleId: string) {
    try {
      systemStore.addLog(`Connecting to vehicle ${vehicleId}...`, 'info')

      // 获取信令 URL
      const signalingUrl = `wss://${window.location.hostname}:8080/signaling`

      // 创建 WebRTC 连接
      const connection = webrtcManager.createConnection({
        vehicleId,
        signalingUrl,
        iceServers: [
          { urls: 'stun:stun.l.google.com:19302' },
          { urls: 'stun:stun.aliyun.com:3478' }
        ]
      })

      // 设置回调
      connection.onConnectionState((state) => {
        connectionState.value = state
        if (state === 'connected') {
          systemStore.addLog(`Connected to vehicle ${vehicleId}`, 'info')
        } else if (state === 'failed') {
          systemStore.addLog(`Connection to ${vehicleId} failed`, 'error')
        }
      })

      connection.onVideoStream((stream) => {
        // 发出视频流事件
        window.dispatchEvent(new CustomEvent('vehicle-video-stream', {
          detail: { vehicleId, cameraId: stream.cameraId, stream: stream.stream }
        }))
      })

      connection.onTelemetry((data) => {
        updateVehicleFromStatus({
          event: 'vehicle_status',
          vehicle_id: vehicleId,
          timestamp: data.timestamp,
          speed: data.speed || 0,
          steering: data.steering || 0,
          gear: data.gear || 'P',
          location: data.location,
          latency_ms: latency.value.rtt_ms
        })
      })

      connection.onLatencyUpdate((info) => {
        latency.value = info
      })

      // 连接
      await connection.connect()
      webrtcConnections.set(vehicleId, connection)

      // 切换活动车辆
      await webrtcManager.switchActiveVehicle(vehicleId)

      return true
    } catch (error) {
      systemStore.addLog(`Failed to connect to ${vehicleId}: ${error}`, 'error')
      return false
    }
  }

  // 断开车辆连接
  function disconnectFromVehicle(vehicleId: string) {
    const connection = webrtcConnections.get(vehicleId)
    if (connection) {
      connection.disconnect()
      webrtcConnections.delete(vehicleId)
      systemStore.addLog(`Disconnected from vehicle ${vehicleId}`, 'info')
    }
  }

  // 发送控制指令
  function sendControl(command: {
    steering: number
    throttle: number
    brake: number
    gear?: number
    turn_signal?: number
    emergency?: boolean
  }) {
    const activeConnection = webrtcManager.getActiveConnection()
    if (activeConnection && activeConnection.isConnected) {
      activeConnection.sendControl(command)
    } else if (webSocketService) {
      // 回退到 WebSocket
      const currentVehicle = fleetStore.currentVehicle
      if (currentVehicle) {
        webSocketService.sendControlCommand(currentVehicle.id, command)
      }
    }
  }

  // 紧急停车
  function emergencyStop() {
    sendControl({
      steering: 0,
      throttle: 0,
      brake: 1.0,
      emergency: true
    })
    systemStore.addLog('EMERGENCY STOP ACTIVATED', 'error')
  }

  // 更新调度配置
  async function updateSchedulingConfig(config: Partial<SchedulingConfig>) {
    if (!apiService) return

    try {
      await apiService.updateSchedulingConfig(config)
      if (config.enabled !== undefined) {
        isSchedulingEnabled.value = config.enabled
      }
      systemStore.addLog('Scheduling config updated', 'info')
    } catch (error) {
      systemStore.addLog(`Failed to update scheduling config: ${error}`, 'error')
    }
  }

  // 切换活动车辆
  async function switchVehicle(vehicleId: string) {
    const index = fleetStore.vehicles.findIndex(v => v.id === vehicleId)
    if (index >= 0) {
      fleetStore.selectVehicle(index)

      // 如果有 WebRTC 连接，切换
      if (webrtcConnections.has(vehicleId)) {
        await webrtcManager.switchActiveVehicle(vehicleId)
      } else {
        // 自动连接
        await connectToVehicle(vehicleId)
      }
    }
  }

  // 清理
  function cleanup() {
    if (webSocketService) {
      webSocketService.disconnect()
    }
    webrtcManager.disconnectAll()
    webrtcConnections.clear()
    isConnected.value = false
    connectionState.value = 'disconnected'
  }

  // 监听当前车辆变化
  watch(() => fleetStore.currentVehicleIndex, async () => {
    // 可选: 自动连接新选中的车辆
  })

  // 生命周期
  onMounted(() => {
    // 不自动初始化，需要手动调用 initialize()
  })

  onUnmounted(() => {
    cleanup()
  })

  return {
    // 状态
    isConnected,
    connectionState,
    latency,
    schedulingQueue,
    isSchedulingEnabled,

    // 方法
    initialize,
    connectToVehicle,
    disconnectFromVehicle,
    switchVehicle,
    sendControl,
    emergencyStop,
    updateSchedulingConfig,
    loadSchedulingQueue,
    cleanup
  }
}
