/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      WebRTC Service
 * @description WebRTC服务，用于与车端建立P2P视频/数据连接
 *              支持阿里云TURN服务器和多摄像头传输
 */

// ======================== 阿里云 TURN 配置 ========================

/**
 * 阿里云 TURN 服务器配置
 * 实际部署时需要从后端获取临时凭证
 */
export const ALIYUN_ICE_SERVERS: RTCIceServer[] = [
  // 公共 STUN 服务器
  { urls: 'stun:stun.l.google.com:19302' },
  { urls: 'stun:stun1.l.google.com:19302' },
  // 阿里云 TURN (需配置)
  {
    urls: [
      'turn:turn.fsm-pilot.aliyuncs.com:3478?transport=udp',
      'turn:turn.fsm-pilot.aliyuncs.com:3478?transport=tcp',
      'turns:turn.fsm-pilot.aliyuncs.com:443?transport=tcp'
    ],
    username: 'fsm-turn-user',
    credential: 'temp-credential'
  }
]

// ======================== 摄像头类型定义 ========================

export interface CameraConfig {
  id: string
  name: string
  type: 'front' | 'rear' | 'left' | 'right' | 'interior' | 'surround'
  resolution: { width: number; height: number }
  frameRate: number
  bitrate: number
  enabled: boolean
}

export const DEFAULT_CAMERAS: CameraConfig[] = [
  { id: 'front-main', name: '前视主摄', type: 'front', resolution: { width: 1920, height: 1080 }, frameRate: 30, bitrate: 4000000, enabled: true },
  { id: 'front-wide', name: '前视广角', type: 'front', resolution: { width: 1280, height: 720 }, frameRate: 30, bitrate: 2000000, enabled: true },
  { id: 'rear-main', name: '后视摄像', type: 'rear', resolution: { width: 1280, height: 720 }, frameRate: 25, bitrate: 1500000, enabled: true },
  { id: 'left-mirror', name: '左后视镜', type: 'left', resolution: { width: 640, height: 480 }, frameRate: 25, bitrate: 800000, enabled: false },
  { id: 'right-mirror', name: '右后视镜', type: 'right', resolution: { width: 640, height: 480 }, frameRate: 25, bitrate: 800000, enabled: false },
  { id: 'surround-360', name: '360环视', type: 'surround', resolution: { width: 1920, height: 1080 }, frameRate: 20, bitrate: 3000000, enabled: false }
]

// ======================== 基础类型定义 ========================

export interface WebRTCConfig {
  iceServers: RTCIceServer[]
  signalingUrl: string
  vehicleId: string
  cameras?: CameraConfig[]
}

export interface VideoStream {
  cameraId: string
  stream: MediaStream
}

export type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'reconnecting' | 'failed'

export interface LatencyInfo {
  rtt_ms: number
  video_latency_ms: number
  control_latency_ms: number
  jitter_ms: number
}

type ConnectionStateCallback = (state: ConnectionState) => void
type VideoStreamCallback = (stream: VideoStream) => void
type TelemetryCallback = (data: any) => void
type LatencyCallback = (info: LatencyInfo) => void

export class WebRTCService {
  private config: WebRTCConfig
  private peerConnection: RTCPeerConnection | null = null
  private dataChannel: RTCDataChannel | null = null
  private signalingWs: WebSocket | null = null
  private state: ConnectionState = 'disconnected'

  // 回调
  private connectionCallbacks: ConnectionStateCallback[] = []
  private videoCallbacks: VideoStreamCallback[] = []
  private telemetryCallbacks: TelemetryCallback[] = []
  private latencyCallbacks: LatencyCallback[] = []

  // 延迟测量
  private pingTimestamps: Map<number, number> = new Map()
  private latencyInfo: LatencyInfo = {
    rtt_ms: 0,
    video_latency_ms: 0,
    control_latency_ms: 0,
    jitter_ms: 0
  }

  // 序列号
  private commandSequence = 0
  private pingSequence = 0

  constructor(config: WebRTCConfig) {
    this.config = config
  }

  /**
   * 连接到车辆
   */
  async connect(): Promise<void> {
    if (this.state === 'connected' || this.state === 'connecting') {
      return
    }

    this.setState('connecting')

    try {
      // 1. 连接信令服务器
      await this.connectSignaling()

      // 2. 创建PeerConnection
      this.createPeerConnection()

      // 3. 发送连接请求
      this.sendSignaling({
        type: 'connect',
        vehicle_id: this.config.vehicleId
      })

    } catch (error) {
      this.setState('failed')
      throw error
    }
  }

  /**
   * 断开连接
   */
  disconnect(): void {
    // 关闭数据通道
    if (this.dataChannel) {
      this.dataChannel.close()
      this.dataChannel = null
    }

    // 关闭PeerConnection
    if (this.peerConnection) {
      this.peerConnection.close()
      this.peerConnection = null
    }

    // 关闭信令WebSocket
    if (this.signalingWs) {
      this.signalingWs.close()
      this.signalingWs = null
    }

    this.setState('disconnected')
  }

  /**
   * 发送控制指令
   */
  sendControl(command: {
    steering: number
    throttle: number
    brake: number
    gear?: number
    turn_signal?: number
    emergency?: boolean
  }): void {
    if (!this.dataChannel || this.dataChannel.readyState !== 'open') {
      console.warn('[WebRTC] Data channel not open')
      return
    }

    const message = {
      type: 'control',
      sequence: ++this.commandSequence,
      timestamp: Date.now(),
      ...command
    }

    this.dataChannel.send(JSON.stringify(message))
  }

  /**
   * 发送心跳 (用于延迟测量)
   */
  sendPing(): void {
    if (!this.dataChannel || this.dataChannel.readyState !== 'open') {
      return
    }

    const sequence = ++this.pingSequence
    const timestamp = Date.now()
    this.pingTimestamps.set(sequence, timestamp)

    this.dataChannel.send(JSON.stringify({
      type: 'ping',
      sequence,
      timestamp
    }))
  }

  /**
   * 获取延迟信息
   */
  getLatencyInfo(): LatencyInfo {
    return { ...this.latencyInfo }
  }

  /**
   * 注册连接状态回调
   */
  onConnectionState(callback: ConnectionStateCallback): void {
    this.connectionCallbacks.push(callback)
  }

  /**
   * 注册视频流回调
   */
  onVideoStream(callback: VideoStreamCallback): void {
    this.videoCallbacks.push(callback)
  }

  /**
   * 注册遥测数据回调
   */
  onTelemetry(callback: TelemetryCallback): void {
    this.telemetryCallbacks.push(callback)
  }

  /**
   * 注册延迟更新回调
   */
  onLatencyUpdate(callback: LatencyCallback): void {
    this.latencyCallbacks.push(callback)
  }

  private async connectSignaling(): Promise<void> {
    return new Promise((resolve, reject) => {
      this.signalingWs = new WebSocket(this.config.signalingUrl)

      this.signalingWs.onopen = () => {
        console.log('[WebRTC] Signaling connected')
        resolve()
      }

      this.signalingWs.onerror = (error) => {
        console.error('[WebRTC] Signaling error:', error)
        reject(error)
      }

      this.signalingWs.onclose = () => {
        console.log('[WebRTC] Signaling disconnected')
        if (this.state === 'connected') {
          this.setState('reconnecting')
          this.scheduleReconnect()
        }
      }

      this.signalingWs.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data)
          this.handleSignalingMessage(message)
        } catch (e) {
          console.error('[WebRTC] Failed to parse signaling message:', e)
        }
      }
    })
  }

  private createPeerConnection(): void {
    this.peerConnection = new RTCPeerConnection({
      iceServers: this.config.iceServers
    })

    // ICE候选
    this.peerConnection.onicecandidate = (event) => {
      if (event.candidate) {
        this.sendSignaling({
          type: 'ice-candidate',
          candidate: event.candidate
        })
      }
    }

    // ICE连接状态
    this.peerConnection.oniceconnectionstatechange = () => {
      const state = this.peerConnection?.iceConnectionState
      console.log('[WebRTC] ICE connection state:', state)

      if (state === 'connected') {
        this.setState('connected')
        this.startLatencyMeasurement()
      } else if (state === 'disconnected' || state === 'failed') {
        this.setState('disconnected')
      }
    }

    // 接收视频轨道
    this.peerConnection.ontrack = (event) => {
      console.log('[WebRTC] Received track:', event.track.kind)

      if (event.track.kind === 'video') {
        // 从stream ID提取摄像头ID
        const streamId = event.streams[0]?.id || 'unknown'
        const cameraId = this.extractCameraId(streamId)

        this.videoCallbacks.forEach(cb => cb({
          cameraId,
          stream: event.streams[0]
        }))
      }
    }

    // 数据通道
    this.peerConnection.ondatachannel = (event) => {
      this.setupDataChannel(event.channel)
    }
  }

  private setupDataChannel(channel: RTCDataChannel): void {
    this.dataChannel = channel

    channel.onopen = () => {
      console.log('[WebRTC] Data channel opened')
    }

    channel.onclose = () => {
      console.log('[WebRTC] Data channel closed')
    }

    channel.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data)
        this.handleDataChannelMessage(data)
      } catch (e) {
        console.error('[WebRTC] Failed to parse data channel message:', e)
      }
    }
  }

  private handleSignalingMessage(message: any): void {
    switch (message.type) {
      case 'offer':
        this.handleOffer(message.sdp)
        break

      case 'answer':
        this.handleAnswer(message.sdp)
        break

      case 'ice-candidate':
        this.handleIceCandidate(message.candidate)
        break

      case 'error':
        console.error('[WebRTC] Signaling error:', message.message)
        this.setState('failed')
        break
    }
  }

  private async handleOffer(sdp: string): Promise<void> {
    if (!this.peerConnection) return

    await this.peerConnection.setRemoteDescription({
      type: 'offer',
      sdp
    })

    const answer = await this.peerConnection.createAnswer()
    await this.peerConnection.setLocalDescription(answer)

    this.sendSignaling({
      type: 'answer',
      sdp: answer.sdp
    })
  }

  private async handleAnswer(sdp: string): Promise<void> {
    if (!this.peerConnection) return

    await this.peerConnection.setRemoteDescription({
      type: 'answer',
      sdp
    })
  }

  private async handleIceCandidate(candidate: RTCIceCandidateInit): Promise<void> {
    if (!this.peerConnection) return

    await this.peerConnection.addIceCandidate(candidate)
  }

  private handleDataChannelMessage(data: any): void {
    switch (data.type) {
      case 'telemetry':
        this.telemetryCallbacks.forEach(cb => cb(data))
        break

      case 'pong':
        this.handlePong(data)
        break

      case 'system_status':
        // 处理系统状态
        break
    }
  }

  private handlePong(data: { sequence: number; timestamp: number; original_timestamp: number }): void {
    const sentTime = this.pingTimestamps.get(data.sequence)
    if (sentTime) {
      const rtt = Date.now() - sentTime
      this.pingTimestamps.delete(data.sequence)

      // 更新延迟信息
      this.latencyInfo.rtt_ms = rtt

      // 通知回调
      this.latencyCallbacks.forEach(cb => cb(this.latencyInfo))
    }
  }

  private sendSignaling(message: object): void {
    if (this.signalingWs && this.signalingWs.readyState === WebSocket.OPEN) {
      this.signalingWs.send(JSON.stringify(message))
    }
  }

  private setState(state: ConnectionState): void {
    if (this.state !== state) {
      this.state = state
      this.connectionCallbacks.forEach(cb => cb(state))
    }
  }

  private extractCameraId(streamId: string): string {
    // 从stream ID提取摄像头ID
    // 例如: "vehicle_FSM-01_cam_front_center" -> "cam_front_center"
    const parts = streamId.split('_')
    if (parts.length >= 3) {
      return parts.slice(2).join('_')
    }
    return streamId
  }

  private startLatencyMeasurement(): void {
    // 每秒发送一次ping
    setInterval(() => {
      if (this.state === 'connected') {
        this.sendPing()
      }
    }, 1000)
  }

  private scheduleReconnect(): void {
    setTimeout(() => {
      if (this.state === 'reconnecting') {
        this.connect().catch(e => {
          console.error('[WebRTC] Reconnect failed:', e)
        })
      }
    }, 5000)
  }

  get connectionState(): ConnectionState {
    return this.state
  }

  get isConnected(): boolean {
    return this.state === 'connected'
  }
}

// 管理多个车辆的WebRTC连接
class WebRTCManager {
  private connections: Map<string, WebRTCService> = new Map()
  private activeVehicleId: string | null = null

  createConnection(config: WebRTCConfig): WebRTCService {
    const service = new WebRTCService(config)
    this.connections.set(config.vehicleId, service)
    return service
  }

  getConnection(vehicleId: string): WebRTCService | undefined {
    return this.connections.get(vehicleId)
  }

  async switchActiveVehicle(vehicleId: string): Promise<void> {
    this.activeVehicleId = vehicleId
    console.log(`[WebRTC] Switched active vehicle to: ${vehicleId}`)
  }

  getActiveConnection(): WebRTCService | undefined {
    if (this.activeVehicleId) {
      return this.connections.get(this.activeVehicleId)
    }
    return undefined
  }

  disconnectAll(): void {
    this.connections.forEach(conn => conn.disconnect())
    this.connections.clear()
    this.activeVehicleId = null
  }
}

export const webrtcManager = new WebRTCManager()

// ======================== Vue 组合式 API 封装 ========================

import { ref, reactive, computed, onUnmounted } from 'vue'

export interface MultiCameraState {
  streams: Map<string, MediaStream>
  stats: Map<string, StreamStats>
}

export interface StreamStats {
  bytesReceived: number
  packetsReceived: number
  packetsLost: number
  frameRate: number
  resolution: { width: number; height: number }
  bitrate: number
}

/**
 * 多摄像头 WebRTC 组合式 API
 */
export function useMultiCameraWebRTC(vehicleId: string, signalingUrl?: string) {
  // 状态
  const isConnected = ref(false)
  const isConnecting = ref(false)
  const error = ref<string | null>(null)
  const latency = ref(0)
  const bandwidth = ref(0)

  // 摄像头配置
  const cameras = ref<CameraConfig[]>(DEFAULT_CAMERAS.map(c => ({ ...c })))

  // 视频流
  const streams = reactive<Map<string, MediaStream>>(new Map())
  const streamStats = reactive<Map<string, StreamStats>>(new Map())

  // WebRTC 服务实例
  let webrtcService: WebRTCService | null = null
  let statsInterval: number | null = null

  /**
   * 连接到车辆
   */
  const connect = async (): Promise<boolean> => {
    if (isConnected.value || isConnecting.value) return false

    isConnecting.value = true
    error.value = null

    try {
      const config: WebRTCConfig = {
        iceServers: ALIYUN_ICE_SERVERS,
        signalingUrl: signalingUrl || `wss://fsm-pilot.example.com/signaling/${vehicleId}`,
        vehicleId,
        cameras: cameras.value.filter(c => c.enabled)
      }

      webrtcService = webrtcManager.createConnection(config)

      // 注册回调
      webrtcService.onConnectionState((state) => {
        isConnected.value = state === 'connected'
        isConnecting.value = state === 'connecting'
        if (state === 'failed') {
          error.value = '连接失败'
        }
      })

      webrtcService.onVideoStream(({ cameraId, stream }) => {
        streams.set(cameraId, stream)
        initStreamStats(cameraId)
      })

      webrtcService.onLatencyUpdate((info) => {
        latency.value = info.rtt_ms
      })

      await webrtcService.connect()

      // 启动统计收集
      startStatsCollection()

      return true
    } catch (e) {
      error.value = `连接错误: ${e}`
      isConnecting.value = false
      return false
    }
  }

  /**
   * 断开连接
   */
  const disconnect = () => {
    stopStatsCollection()
    if (webrtcService) {
      webrtcService.disconnect()
      webrtcService = null
    }
    streams.clear()
    streamStats.clear()
    isConnected.value = false
    isConnecting.value = false
  }

  /**
   * 切换摄像头
   */
  const toggleCamera = (cameraId: string) => {
    const camera = cameras.value.find(c => c.id === cameraId)
    if (camera) {
      camera.enabled = !camera.enabled
      // TODO: 发送信令通知车端
    }
  }

  /**
   * 设置摄像头质量
   */
  const setCameraQuality = (cameraId: string, quality: 'low' | 'medium' | 'high') => {
    const camera = cameras.value.find(c => c.id === cameraId)
    if (!camera) return

    const presets = {
      low: { width: 640, height: 360, frameRate: 15, bitrate: 500000 },
      medium: { width: 1280, height: 720, frameRate: 25, bitrate: 1500000 },
      high: { width: 1920, height: 1080, frameRate: 30, bitrate: 4000000 }
    }

    const preset = presets[quality]
    camera.resolution = { width: preset.width, height: preset.height }
    camera.frameRate = preset.frameRate
    camera.bitrate = preset.bitrate
  }

  /**
   * 获取摄像头流
   */
  const getStream = (cameraId: string): MediaStream | undefined => {
    return streams.get(cameraId)
  }

  /**
   * 初始化流统计
   */
  const initStreamStats = (cameraId: string) => {
    streamStats.set(cameraId, {
      bytesReceived: 0,
      packetsReceived: 0,
      packetsLost: 0,
      frameRate: 0,
      resolution: { width: 0, height: 0 },
      bitrate: 0
    })
  }

  /**
   * 启动统计收集
   */
  const startStatsCollection = () => {
    if (statsInterval) return

    statsInterval = window.setInterval(() => {
      // 计算总带宽
      let totalBandwidth = 0
      streamStats.forEach(stat => {
        totalBandwidth += stat.bitrate
      })
      bandwidth.value = totalBandwidth / 1000 // kbps
    }, 1000)
  }

  /**
   * 停止统计收集
   */
  const stopStatsCollection = () => {
    if (statsInterval) {
      clearInterval(statsInterval)
      statsInterval = null
    }
  }

  // 计算属性
  const enabledCameras = computed(() => cameras.value.filter(c => c.enabled))
  const connectedCameras = computed(() => cameras.value.filter(c => streams.has(c.id)))
  const networkQuality = computed(() => {
    if (!isConnected.value) return 'disconnected'
    if (latency.value < 50) return 'excellent'
    if (latency.value < 100) return 'good'
    if (latency.value < 200) return 'fair'
    return 'poor'
  })

  // 清理
  onUnmounted(() => {
    disconnect()
  })

  return {
    // 状态
    isConnected,
    isConnecting,
    error,
    latency,
    bandwidth,
    cameras,
    streams,
    streamStats,

    // 计算属性
    enabledCameras,
    connectedCameras,
    networkQuality,

    // 方法
    connect,
    disconnect,
    toggleCamera,
    setCameraQuality,
    getStream
  }
}

// ======================== 阿里云 TURN 凭证获取 ========================

/**
 * 从后端获取阿里云 TURN 临时凭证
 */
export async function fetchAliyunTurnCredentials(apiUrl: string): Promise<RTCIceServer[]> {
  try {
    const response = await fetch(apiUrl, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ service: 'fsm-pilot', ttl: 86400 })
    })

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`)
    }

    const data = await response.json()
    return data.iceServers || ALIYUN_ICE_SERVERS
  } catch (e) {
    console.warn('[WebRTC] Failed to fetch TURN credentials, using defaults:', e)
    return ALIYUN_ICE_SERVERS
  }
}

// ======================== Mock WebRTC (演示用) ========================

/**
 * Mock 多摄像头 WebRTC 服务
 * 用于前端演示，模拟视频流
 */
export function useMockMultiCameraWebRTC(vehicleId: string) {
  const isConnected = ref(false)
  const isConnecting = ref(false)
  const latency = ref(45)
  const bandwidth = ref(8500)
  const cameras = ref<CameraConfig[]>(DEFAULT_CAMERAS.map(c => ({ ...c })))
  const error = ref<string | null>(null)

  let latencyInterval: number | null = null

  const connect = async (): Promise<boolean> => {
    isConnecting.value = true
    console.log('[MockWebRTC] Connecting to vehicle:', vehicleId)

    await new Promise(resolve => setTimeout(resolve, 1200))

    isConnected.value = true
    isConnecting.value = false

    // 模拟延迟波动
    latencyInterval = window.setInterval(() => {
      latency.value = 35 + Math.random() * 30
      bandwidth.value = 7000 + Math.random() * 3000
    }, 2000)

    return true
  }

  const disconnect = () => {
    isConnected.value = false
    isConnecting.value = false
    if (latencyInterval) {
      clearInterval(latencyInterval)
      latencyInterval = null
    }
  }

  const toggleCamera = (cameraId: string) => {
    const camera = cameras.value.find(c => c.id === cameraId)
    if (camera) {
      camera.enabled = !camera.enabled
    }
  }

  const enabledCameras = computed(() => cameras.value.filter(c => c.enabled))
  const networkQuality = computed(() => {
    if (!isConnected.value) return 'disconnected'
    if (latency.value < 50) return 'excellent'
    if (latency.value < 100) return 'good'
    return 'fair'
  })

  onUnmounted(() => {
    disconnect()
  })

  return {
    isConnected,
    isConnecting,
    latency,
    bandwidth,
    cameras,
    error,
    enabledCameras,
    networkQuality,
    connect,
    disconnect,
    toggleCamera
  }
}
