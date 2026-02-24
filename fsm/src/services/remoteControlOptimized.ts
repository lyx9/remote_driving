/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      Remote Control Service (Optimized)
 * @description 优化版远程控制服务
 *              修复连接问题、添加命令确认、改进重连机制
 */

import { ref, reactive, computed, readonly, onUnmounted } from 'vue'

// ======================== 配置常量 ========================

/** 默认信令服务器配置 */
export interface SignalingConfig {
  host: string
  port: number
  path: string
  secure: boolean
}

/** 默认 ICE 服务器配置 */
export interface IceServerConfig {
  urls: string | string[]
  username?: string
  credential?: string
}

/** 远程控制配置 */
export interface RemoteControlConfig {
  signaling: SignalingConfig
  iceServers: IceServerConfig[]
  turnCredentialApi?: string
  reconnect: {
    maxAttempts: number
    baseDelayMs: number
    maxDelayMs: number
  }
  control: {
    ackTimeoutMs: number
    maxPendingCommands: number
    sendIntervalMs: number
  }
  latency: {
    sampleSize: number
    pingIntervalMs: number
  }
}

/** 默认配置 */
export const DEFAULT_CONFIG: RemoteControlConfig = {
  signaling: {
    host: 'localhost',
    port: 8080,
    path: '/signaling',
    secure: false
  },
  iceServers: [
    { urls: 'stun:stun.l.google.com:19302' },
    { urls: 'stun:stun1.l.google.com:19302' }
  ],
  turnCredentialApi: undefined,
  reconnect: {
    maxAttempts: 10,
    baseDelayMs: 1000,
    maxDelayMs: 30000
  },
  control: {
    ackTimeoutMs: 500,
    maxPendingCommands: 10,
    sendIntervalMs: 50 // 20Hz
  },
  latency: {
    sampleSize: 20,
    pingIntervalMs: 1000
  }
}

// ======================== 类型定义 ========================

export interface ControlCommand {
  seq: number
  timestamp: number
  steering: number    // -1.0 to 1.0
  throttle: number    // 0 to 1.0
  brake: number       // 0 to 1.0
  gear?: number       // 0=P, 1=R, 2=N, 3=D
  turnSignal?: number // 0=off, 1=left, 2=right, 3=hazard
  emergency?: boolean
}

export interface CommandAck {
  seq: number
  receivedAt: number
  applied: boolean
  error?: string
}

export interface LatencyStats {
  rtt: number
  jitter: number
  controlLatency: number
  videoLatency: number
  samples: number[]
}

export interface ConnectionState {
  status: 'disconnected' | 'connecting' | 'connected' | 'reconnecting' | 'failed'
  vehicleId: string | null
  signalingConnected: boolean
  webrtcConnected: boolean
  dataChannelOpen: boolean
  lastError: string | null
  reconnectAttempt: number
}

export interface VehicleTelemetry {
  timestamp: number
  speed: number
  steering: number
  gear: number
  location: { lat: number; lng: number }
  heading: number
  batteryLevel: number
  fuelLevel?: number
  cpuUsage: number
  memoryUsage: number
  controlMode: 'manual' | 'remote' | 'autonomous'
}

// ======================== 验证函数 ========================

function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value))
}

function validateControlCommand(cmd: Partial<ControlCommand>): ControlCommand {
  return {
    seq: cmd.seq ?? 0,
    timestamp: cmd.timestamp ?? Date.now(),
    steering: clamp(cmd.steering ?? 0, -1, 1),
    throttle: clamp(cmd.throttle ?? 0, 0, 1),
    brake: clamp(cmd.brake ?? 0, 0, 1),
    gear: cmd.gear,
    turnSignal: cmd.turnSignal,
    emergency: cmd.emergency ?? false
  }
}

// ======================== 延迟统计计算 ========================

function calculateLatencyStats(samples: number[]): { mean: number; jitter: number } {
  if (samples.length === 0) return { mean: 0, jitter: 0 }

  const mean = samples.reduce((a, b) => a + b, 0) / samples.length

  if (samples.length < 2) return { mean, jitter: 0 }

  // 计算抖动 (相邻样本差的平均值)
  let jitterSum = 0
  for (let i = 1; i < samples.length; i++) {
    jitterSum += Math.abs(samples[i] - samples[i - 1])
  }
  const jitter = jitterSum / (samples.length - 1)

  return { mean, jitter }
}

// ======================== 指数退避重连 ========================

function calculateReconnectDelay(
  attempt: number,
  baseDelay: number,
  maxDelay: number
): number {
  // 指数退避 + 随机抖动
  const exponentialDelay = baseDelay * Math.pow(2, attempt)
  const jitter = Math.random() * 0.3 * exponentialDelay
  return Math.min(exponentialDelay + jitter, maxDelay)
}

// ======================== 优化版远程控制服务 ========================

export function useRemoteControlOptimized(config: Partial<RemoteControlConfig> = {}) {
  // 合并配置
  const finalConfig: RemoteControlConfig = {
    ...DEFAULT_CONFIG,
    ...config,
    signaling: { ...DEFAULT_CONFIG.signaling, ...config.signaling },
    reconnect: { ...DEFAULT_CONFIG.reconnect, ...config.reconnect },
    control: { ...DEFAULT_CONFIG.control, ...config.control },
    latency: { ...DEFAULT_CONFIG.latency, ...config.latency }
  }

  // ========== 状态 ==========
  const connectionState = reactive<ConnectionState>({
    status: 'disconnected',
    vehicleId: null,
    signalingConnected: false,
    webrtcConnected: false,
    dataChannelOpen: false,
    lastError: null,
    reconnectAttempt: 0
  })

  const latencyStats = reactive<LatencyStats>({
    rtt: 0,
    jitter: 0,
    controlLatency: 0,
    videoLatency: 0,
    samples: []
  })

  const telemetry = ref<VehicleTelemetry | null>(null)
  const isControlEnabled = ref(false)

  // ========== 内部状态 ==========
  let signalingWs: WebSocket | null = null
  let peerConnection: RTCPeerConnection | null = null
  let dataChannel: RTCDataChannel | null = null
  let iceServers: RTCIceServer[] = []

  // 命令管理
  let commandSeq = 0
  const pendingCommands = new Map<number, {
    command: ControlCommand
    sentAt: number
    timeout: ReturnType<typeof setTimeout>
  }>()

  // 定时器
  let pingInterval: ReturnType<typeof setInterval> | null = null
  let reconnectTimeout: ReturnType<typeof setTimeout> | null = null
  let controlInterval: ReturnType<typeof setInterval> | null = null

  // 待发送的控制命令
  let pendingControl: Partial<ControlCommand> | null = null

  // 回调
  const videoStreamCallbacks: Array<(stream: MediaStream, cameraId: string) => void> = []
  const telemetryCallbacks: Array<(data: VehicleTelemetry) => void> = []

  // ========== 公共方法 ==========

  /**
   * 连接到车辆
   */
  async function connect(vehicleId: string): Promise<boolean> {
    if (connectionState.status === 'connected' || connectionState.status === 'connecting') {
      console.warn('[RemoteControl] Already connected or connecting')
      return false
    }

    connectionState.status = 'connecting'
    connectionState.vehicleId = vehicleId
    connectionState.lastError = null
    connectionState.reconnectAttempt = 0

    try {
      // 1. 获取 TURN 凭证 (如果配置了)
      if (finalConfig.turnCredentialApi) {
        iceServers = await fetchTurnCredentials(finalConfig.turnCredentialApi)
      } else {
        iceServers = finalConfig.iceServers as RTCIceServer[]
      }

      // 2. 连接信令服务器
      await connectSignaling(vehicleId)

      // 3. 创建 WebRTC 连接
      createPeerConnection()

      // 4. 发起连接请求
      sendSignaling({
        type: 'connect',
        vehicleId,
        timestamp: Date.now()
      })

      return true
    } catch (error) {
      const errorMsg = `连接失败: ${error}`
      connectionState.status = 'failed'
      connectionState.lastError = errorMsg
      console.error('[RemoteControl]', errorMsg)
      return false
    }
  }

  /**
   * 断开连接
   */
  function disconnect(): void {
    cleanup()
    connectionState.status = 'disconnected'
    connectionState.vehicleId = null
    console.log('[RemoteControl] Disconnected')
  }

  /**
   * 发送控制命令 (带确认)
   */
  function sendControl(command: Partial<ControlCommand>): void {
    if (!isControlEnabled.value) {
      console.warn('[RemoteControl] Control not enabled')
      return
    }

    // 缓存最新控制指令，由定时器发送
    pendingControl = command
  }

  /**
   * 紧急停车
   */
  function emergencyStop(): void {
    const cmd = validateControlCommand({
      steering: 0,
      throttle: 0,
      brake: 1,
      emergency: true
    })

    // 紧急停车立即发送，跳过节流
    sendControlImmediate(cmd)
  }

  /**
   * 启用控制
   */
  function enableControl(): void {
    if (!connectionState.dataChannelOpen) {
      console.warn('[RemoteControl] Data channel not open')
      return
    }

    isControlEnabled.value = true
    startControlInterval()

    // 通知车端
    sendDataChannelMessage({
      type: 'control_enable',
      timestamp: Date.now()
    })
  }

  /**
   * 禁用控制
   */
  function disableControl(): void {
    isControlEnabled.value = false
    stopControlInterval()

    // 发送停止命令
    if (connectionState.dataChannelOpen) {
      sendControlImmediate(validateControlCommand({
        steering: 0,
        throttle: 0,
        brake: 0.5
      }))

      sendDataChannelMessage({
        type: 'control_disable',
        timestamp: Date.now()
      })
    }
  }

  /**
   * 注册视频流回调
   */
  function onVideoStream(callback: (stream: MediaStream, cameraId: string) => void): void {
    videoStreamCallbacks.push(callback)
  }

  /**
   * 注册遥测数据回调
   */
  function onTelemetry(callback: (data: VehicleTelemetry) => void): void {
    telemetryCallbacks.push(callback)
  }

  // ========== 信令处理 ==========

  async function connectSignaling(vehicleId: string): Promise<void> {
    return new Promise((resolve, reject) => {
      const { host, port, path, secure } = finalConfig.signaling
      const protocol = secure ? 'wss' : 'ws'
      const url = `${protocol}://${host}:${port}${path}?vehicle=${vehicleId}`

      signalingWs = new WebSocket(url)

      const timeout = setTimeout(() => {
        reject(new Error('信令服务器连接超时'))
      }, 10000)

      signalingWs.onopen = () => {
        clearTimeout(timeout)
        connectionState.signalingConnected = true
        console.log('[RemoteControl] Signaling connected')
        resolve()
      }

      signalingWs.onerror = (error) => {
        clearTimeout(timeout)
        connectionState.signalingConnected = false
        reject(error)
      }

      signalingWs.onclose = (event) => {
        connectionState.signalingConnected = false
        console.log('[RemoteControl] Signaling closed:', event.code, event.reason)

        if (connectionState.status === 'connected') {
          scheduleReconnect()
        }
      }

      signalingWs.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data)
          handleSignalingMessage(message)
        } catch (e) {
          console.error('[RemoteControl] Failed to parse signaling message:', e)
        }
      }
    })
  }

  function sendSignaling(message: object): void {
    if (signalingWs && signalingWs.readyState === WebSocket.OPEN) {
      signalingWs.send(JSON.stringify(message))
    } else {
      console.warn('[RemoteControl] Signaling not connected')
    }
  }

  function handleSignalingMessage(message: any): void {
    switch (message.type) {
      case 'offer':
        handleOffer(message.sdp)
        break
      case 'answer':
        handleAnswer(message.sdp)
        break
      case 'ice-candidate':
        handleIceCandidate(message.candidate)
        break
      case 'vehicle_ready':
        console.log('[RemoteControl] Vehicle ready for connection')
        break
      case 'error':
        console.error('[RemoteControl] Signaling error:', message.message)
        connectionState.lastError = message.message
        break
    }
  }

  // ========== WebRTC 处理 ==========

  function createPeerConnection(): void {
    peerConnection = new RTCPeerConnection({ iceServers })

    peerConnection.onicecandidate = (event) => {
      if (event.candidate) {
        sendSignaling({
          type: 'ice-candidate',
          candidate: event.candidate
        })
      }
    }

    peerConnection.oniceconnectionstatechange = () => {
      const state = peerConnection?.iceConnectionState
      console.log('[RemoteControl] ICE state:', state)

      if (state === 'connected') {
        connectionState.webrtcConnected = true
        connectionState.status = 'connected'
        connectionState.reconnectAttempt = 0
        startPingInterval()
      } else if (state === 'disconnected' || state === 'failed') {
        connectionState.webrtcConnected = false
        if (connectionState.status === 'connected') {
          scheduleReconnect()
        }
      }
    }

    peerConnection.ontrack = (event) => {
      if (event.track.kind === 'video' && event.streams[0]) {
        const streamId = event.streams[0].id
        const cameraId = extractCameraId(streamId)
        console.log('[RemoteControl] Received video stream:', cameraId)
        videoStreamCallbacks.forEach(cb => cb(event.streams[0], cameraId))
      }
    }

    peerConnection.ondatachannel = (event) => {
      setupDataChannel(event.channel)
    }
  }

  async function handleOffer(sdp: string): Promise<void> {
    if (!peerConnection) return

    await peerConnection.setRemoteDescription({ type: 'offer', sdp })
    const answer = await peerConnection.createAnswer()
    await peerConnection.setLocalDescription(answer)

    sendSignaling({
      type: 'answer',
      sdp: answer.sdp
    })
  }

  async function handleAnswer(sdp: string): Promise<void> {
    if (!peerConnection) return
    await peerConnection.setRemoteDescription({ type: 'answer', sdp })
  }

  async function handleIceCandidate(candidate: RTCIceCandidateInit): Promise<void> {
    if (!peerConnection) return
    await peerConnection.addIceCandidate(candidate)
  }

  // ========== 数据通道处理 ==========

  function setupDataChannel(channel: RTCDataChannel): void {
    dataChannel = channel

    channel.onopen = () => {
      connectionState.dataChannelOpen = true
      console.log('[RemoteControl] Data channel opened')
    }

    channel.onclose = () => {
      connectionState.dataChannelOpen = false
      isControlEnabled.value = false
      console.log('[RemoteControl] Data channel closed')
    }

    channel.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data)
        handleDataChannelMessage(data)
      } catch (e) {
        console.error('[RemoteControl] Failed to parse data channel message:', e)
      }
    }
  }

  function sendDataChannelMessage(message: object): void {
    if (dataChannel && dataChannel.readyState === 'open') {
      dataChannel.send(JSON.stringify(message))
    }
  }

  function handleDataChannelMessage(data: any): void {
    switch (data.type) {
      case 'telemetry':
        handleTelemetry(data)
        break
      case 'pong':
        handlePong(data)
        break
      case 'control_ack':
        handleControlAck(data)
        break
      case 'system_status':
        console.log('[RemoteControl] System status:', data)
        break
    }
  }

  function handleTelemetry(data: any): void {
    telemetry.value = {
      timestamp: data.timestamp,
      speed: data.speed,
      steering: data.steering,
      gear: data.gear,
      location: data.location,
      heading: data.heading,
      batteryLevel: data.battery_level,
      cpuUsage: data.cpu_usage,
      memoryUsage: data.memory_usage,
      controlMode: data.control_mode
    }
    telemetryCallbacks.forEach(cb => cb(telemetry.value!))
  }

  function handleControlAck(ack: CommandAck): void {
    const pending = pendingCommands.get(ack.seq)
    if (pending) {
      clearTimeout(pending.timeout)
      pendingCommands.delete(ack.seq)

      // 更新控制延迟
      const controlLatency = Date.now() - pending.sentAt
      latencyStats.controlLatency = controlLatency

      if (!ack.applied) {
        console.warn('[RemoteControl] Command not applied:', ack.error)
      }
    }
  }

  // ========== 控制命令发送 ==========

  function startControlInterval(): void {
    if (controlInterval) return

    controlInterval = setInterval(() => {
      if (pendingControl && isControlEnabled.value) {
        const cmd = validateControlCommand({
          ...pendingControl,
          seq: ++commandSeq,
          timestamp: Date.now()
        })
        sendControlWithAck(cmd)
        pendingControl = null
      }
    }, finalConfig.control.sendIntervalMs)
  }

  function stopControlInterval(): void {
    if (controlInterval) {
      clearInterval(controlInterval)
      controlInterval = null
    }
  }

  function sendControlWithAck(command: ControlCommand): void {
    // 检查待确认命令数量
    if (pendingCommands.size >= finalConfig.control.maxPendingCommands) {
      // 清理最老的命令
      const oldestSeq = Math.min(...pendingCommands.keys())
      const oldest = pendingCommands.get(oldestSeq)
      if (oldest) {
        clearTimeout(oldest.timeout)
        pendingCommands.delete(oldestSeq)
        console.warn('[RemoteControl] Dropped unacked command:', oldestSeq)
      }
    }

    // 发送命令
    sendDataChannelMessage({
      type: 'control',
      ...command
    })

    // 设置超时
    const timeout = setTimeout(() => {
      if (pendingCommands.has(command.seq)) {
        pendingCommands.delete(command.seq)
        console.warn('[RemoteControl] Command timeout:', command.seq)
      }
    }, finalConfig.control.ackTimeoutMs)

    pendingCommands.set(command.seq, {
      command,
      sentAt: Date.now(),
      timeout
    })
  }

  function sendControlImmediate(command: ControlCommand): void {
    command.seq = ++commandSeq
    command.timestamp = Date.now()
    sendDataChannelMessage({
      type: 'control',
      ...command
    })
  }

  // ========== 延迟测量 ==========

  function startPingInterval(): void {
    if (pingInterval) return

    let pingSeq = 0
    const pingTimestamps = new Map<number, number>()

    pingInterval = setInterval(() => {
      if (!connectionState.dataChannelOpen) return

      const seq = ++pingSeq
      const now = Date.now()
      pingTimestamps.set(seq, now)

      sendDataChannelMessage({
        type: 'ping',
        seq,
        timestamp: now
      })

      // 清理旧的 ping
      for (const [s, t] of pingTimestamps) {
        if (now - t > 5000) {
          pingTimestamps.delete(s)
        }
      }
    }, finalConfig.latency.pingIntervalMs)
  }

  function handlePong(data: { seq: number; timestamp: number }): void {
    const rtt = Date.now() - data.timestamp

    // 添加样本
    latencyStats.samples.push(rtt)
    if (latencyStats.samples.length > finalConfig.latency.sampleSize) {
      latencyStats.samples.shift()
    }

    // 计算统计
    const stats = calculateLatencyStats(latencyStats.samples)
    latencyStats.rtt = stats.mean
    latencyStats.jitter = stats.jitter
  }

  function stopPingInterval(): void {
    if (pingInterval) {
      clearInterval(pingInterval)
      pingInterval = null
    }
  }

  // ========== 重连机制 ==========

  function scheduleReconnect(): void {
    if (connectionState.reconnectAttempt >= finalConfig.reconnect.maxAttempts) {
      connectionState.status = 'failed'
      connectionState.lastError = '重连次数超过上限'
      console.error('[RemoteControl] Max reconnect attempts reached')
      return
    }

    connectionState.status = 'reconnecting'
    connectionState.reconnectAttempt++

    const delay = calculateReconnectDelay(
      connectionState.reconnectAttempt - 1,
      finalConfig.reconnect.baseDelayMs,
      finalConfig.reconnect.maxDelayMs
    )

    console.log(`[RemoteControl] Reconnecting in ${Math.round(delay)}ms (attempt ${connectionState.reconnectAttempt})`)

    reconnectTimeout = setTimeout(async () => {
      if (connectionState.vehicleId) {
        cleanup(false)
        try {
          await connect(connectionState.vehicleId)
        } catch (e) {
          console.error('[RemoteControl] Reconnect failed:', e)
          scheduleReconnect()
        }
      }
    }, delay)
  }

  // ========== 工具函数 ==========

  async function fetchTurnCredentials(apiUrl: string): Promise<RTCIceServer[]> {
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
      return data.iceServers || finalConfig.iceServers as RTCIceServer[]
    } catch (e) {
      console.warn('[RemoteControl] Failed to fetch TURN credentials:', e)
      return finalConfig.iceServers as RTCIceServer[]
    }
  }

  function extractCameraId(streamId: string): string {
    const parts = streamId.split('_')
    if (parts.length >= 3) {
      return parts.slice(2).join('_')
    }
    return streamId
  }

  function cleanup(resetState = true): void {
    stopControlInterval()
    stopPingInterval()

    if (reconnectTimeout) {
      clearTimeout(reconnectTimeout)
      reconnectTimeout = null
    }

    // 清理待确认命令
    pendingCommands.forEach(p => clearTimeout(p.timeout))
    pendingCommands.clear()

    if (dataChannel) {
      dataChannel.close()
      dataChannel = null
    }

    if (peerConnection) {
      peerConnection.close()
      peerConnection = null
    }

    if (signalingWs) {
      signalingWs.close()
      signalingWs = null
    }

    if (resetState) {
      connectionState.signalingConnected = false
      connectionState.webrtcConnected = false
      connectionState.dataChannelOpen = false
      isControlEnabled.value = false
    }
  }

  // ========== 生命周期 ==========

  onUnmounted(() => {
    disconnect()
  })

  // ========== 返回值 ==========

  return {
    // 状态 (只读)
    connectionState: readonly(connectionState),
    latencyStats: readonly(latencyStats),
    telemetry: readonly(telemetry),
    isControlEnabled: readonly(isControlEnabled),

    // 计算属性
    isConnected: computed(() => connectionState.status === 'connected'),
    isConnecting: computed(() =>
      connectionState.status === 'connecting' ||
      connectionState.status === 'reconnecting'
    ),

    // 方法
    connect,
    disconnect,
    sendControl,
    emergencyStop,
    enableControl,
    disableControl,
    onVideoStream,
    onTelemetry
  }
}

export default useRemoteControlOptimized
