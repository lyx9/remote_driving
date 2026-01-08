/**
 * FSM-Pilot WebRTC 服务
 * 用于与车端建立P2P视频/数据连接
 */

export interface WebRTCConfig {
  iceServers: RTCIceServer[]
  signalingUrl: string
  vehicleId: string
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
