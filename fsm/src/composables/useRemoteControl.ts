/**
 * FSM-Pilot 远程控制 Composable
 * 整合 WebSocket、WebRTC 和 API 服务，提供统一的远程控制接口
 */

import { ref, computed, onMounted, onUnmounted, watch } from 'vue'
import { useFleetStore } from '@/stores/fleet'
import { useSystemStore } from '@/stores/system'
import { webSocketService, type VehicleStatusMessage, type AlertMessage } from '@/services/websocket'
import { apiService, type VehicleInfo, type SchedulingQueue } from '@/services/api'
import { webrtcManager, type LatencyInfo, type ConnectionState } from '@/services/webrtc'

export interface RemoteControlState {
  isConnected: boolean
  connectionState: ConnectionState
  latency: LatencyInfo
  schedulingQueue: SchedulingQueue | null
}

export function useRemoteControl() {
  const fleetStore = useFleetStore()
  const systemStore = useSystemStore()

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

  // 初始化连接
  async function initialize() {
    try {
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
    webSocketService.onVehicleStatus((status: VehicleStatusMessage) => {
      updateVehicleFromStatus(status)
    })

    webSocketService.onAlert((alert: AlertMessage) => {
      handleAlert(alert)
    })

    webSocketService.onSchedulingUpdate((queue) => {
      schedulingQueue.value = {
        vehicles: queue.queue.map(item => ({
          vehicle_id: item.vehicle_id,
          priority_score: item.priority,
          emergency_level: 0,
          latency_ms: 0,
          distance_to_target: 0,
          battery_level: 100
        })),
        algorithm: 'weighted_priority',
        updated_at: new Date().toISOString()
      }
      systemStore.addLog('Scheduling queue updated', 'info')
    })

    webSocketService.onLatencyUpdate((info) => {
      latency.value = {
        rtt_ms: info.rtt_ms,
        video_latency_ms: info.video_latency_ms || 0,
        control_latency_ms: info.control_latency_ms || 0,
        jitter_ms: info.jitter_ms || 0
      }
    })
  }

  // 从状态消息更新车辆
  function updateVehicleFromStatus(status: VehicleStatusMessage) {
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
  function handleAlert(alert: AlertMessage) {
    const level = alert.severity === 'critical' ? 'error' :
                  alert.severity === 'warning' ? 'warning' : 'info'
    systemStore.addLog(`[${alert.vehicle_id}] ${alert.message}`, level)
  }

  // 加载车辆列表
  async function loadVehicles() {
    try {
      const vehicles = await apiService.getVehicles()
      // 更新 store 中的车辆信息
      vehicles.forEach((vehicleInfo: VehicleInfo) => {
        const existing = fleetStore.vehicles.find(v => v.id === vehicleInfo.id)
        if (!existing) {
          fleetStore.vehicles.push({
            id: vehicleInfo.id,
            type: vehicleInfo.type as 'ROBO-TAXI' | 'LOGISTICS' | 'SECURITY',
            status: vehicleInfo.status === 'online' ? 'ACTIVE' : 'IDLE',
            money: 0,
            mode: 'ECO',
            location: vehicleInfo.location ? [vehicleInfo.location.lat, vehicleInfo.location.lng] : [31.2304, 121.4737],
            path: [],
            speed: 0,
            gear: 'P',
            steering: 0
          })
        }
      })
    } catch (error) {
      systemStore.addLog(`Failed to load vehicles: ${error}`, 'error')
    }
  }

  // 加载调度队列
  async function loadSchedulingQueue() {
    try {
      schedulingQueue.value = await apiService.getSchedulingQueue()
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
    } else {
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
  async function updateSchedulingConfig(config: {
    algorithm?: string
    weights?: Record<string, number>
    enabled?: boolean
  }) {
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
    webSocketService.disconnect()
    webrtcManager.disconnectAll()
    webrtcConnections.clear()
    isConnected.value = false
    connectionState.value = 'disconnected'
  }

  // 监听当前车辆变化
  watch(() => fleetStore.currentVehicleIndex, async (newIndex) => {
    const vehicle = fleetStore.vehicles[newIndex]
    if (vehicle && !webrtcConnections.has(vehicle.id)) {
      // 可选: 自动连接新选中的车辆
      // await connectToVehicle(vehicle.id)
    }
  })

  // 生命周期
  onMounted(() => {
    initialize()
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
