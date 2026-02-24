/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      Remote Control Tests
 * @description 远程控制功能测试
 */

import { describe, it, expect, vi } from 'vitest'

// ======================== 控制命令验证测试 ========================

describe('Control Command Validation', () => {
  interface ControlCommand {
    seq: number
    timestamp: number
    steering: number
    throttle: number
    brake: number
    gear?: number
    turnSignal?: number
    emergency?: boolean
  }

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

  it('should clamp steering to valid range [-1, 1]', () => {
    expect(validateControlCommand({ steering: -2 }).steering).toBe(-1)
    expect(validateControlCommand({ steering: 2 }).steering).toBe(1)
    expect(validateControlCommand({ steering: 0.5 }).steering).toBe(0.5)
    expect(validateControlCommand({ steering: -0.5 }).steering).toBe(-0.5)
  })

  it('should clamp throttle to valid range [0, 1]', () => {
    expect(validateControlCommand({ throttle: -1 }).throttle).toBe(0)
    expect(validateControlCommand({ throttle: 2 }).throttle).toBe(1)
    expect(validateControlCommand({ throttle: 0.7 }).throttle).toBe(0.7)
  })

  it('should clamp brake to valid range [0, 1]', () => {
    expect(validateControlCommand({ brake: -0.5 }).brake).toBe(0)
    expect(validateControlCommand({ brake: 1.5 }).brake).toBe(1)
    expect(validateControlCommand({ brake: 0.3 }).brake).toBe(0.3)
  })

  it('should set default values for missing fields', () => {
    const cmd = validateControlCommand({})
    expect(cmd.steering).toBe(0)
    expect(cmd.throttle).toBe(0)
    expect(cmd.brake).toBe(0)
    expect(cmd.emergency).toBe(false)
  })

  it('should preserve optional fields when provided', () => {
    const cmd = validateControlCommand({
      gear: 3,
      turnSignal: 1,
      emergency: true
    })
    expect(cmd.gear).toBe(3)
    expect(cmd.turnSignal).toBe(1)
    expect(cmd.emergency).toBe(true)
  })
})

// ======================== 延迟统计计算测试 ========================

describe('Latency Statistics', () => {
  function calculateLatencyStats(samples: number[]): { mean: number; jitter: number } {
    if (samples.length === 0) return { mean: 0, jitter: 0 }

    const mean = samples.reduce((a, b) => a + b, 0) / samples.length

    if (samples.length < 2) return { mean, jitter: 0 }

    let jitterSum = 0
    for (let i = 1; i < samples.length; i++) {
      jitterSum += Math.abs(samples[i] - samples[i - 1])
    }
    const jitter = jitterSum / (samples.length - 1)

    return { mean, jitter }
  }

  it('should calculate mean correctly', () => {
    expect(calculateLatencyStats([10, 20, 30]).mean).toBe(20)
    expect(calculateLatencyStats([50, 50, 50]).mean).toBe(50)
    expect(calculateLatencyStats([100]).mean).toBe(100)
  })

  it('should calculate jitter correctly', () => {
    // Stable connection: no variation
    expect(calculateLatencyStats([50, 50, 50]).jitter).toBe(0)

    // Variable connection: |20-10| + |30-20| = 10+10 = 20, jitter = 20/2 = 10
    expect(calculateLatencyStats([10, 20, 30]).jitter).toBe(10)
  })

  it('should handle empty samples', () => {
    const stats = calculateLatencyStats([])
    expect(stats.mean).toBe(0)
    expect(stats.jitter).toBe(0)
  })

  it('should handle single sample', () => {
    const stats = calculateLatencyStats([42])
    expect(stats.mean).toBe(42)
    expect(stats.jitter).toBe(0)
  })
})

// ======================== 指数退避重连测试 ========================

describe('Exponential Backoff Reconnection', () => {
  function calculateReconnectDelay(
    attempt: number,
    baseDelay: number,
    maxDelay: number
  ): number {
    const exponentialDelay = baseDelay * Math.pow(2, attempt)
    // 不包含随机抖动的确定性版本用于测试
    return Math.min(exponentialDelay, maxDelay)
  }

  it('should increase delay exponentially', () => {
    const baseDelay = 1000
    const maxDelay = 30000

    expect(calculateReconnectDelay(0, baseDelay, maxDelay)).toBe(1000)
    expect(calculateReconnectDelay(1, baseDelay, maxDelay)).toBe(2000)
    expect(calculateReconnectDelay(2, baseDelay, maxDelay)).toBe(4000)
    expect(calculateReconnectDelay(3, baseDelay, maxDelay)).toBe(8000)
    expect(calculateReconnectDelay(4, baseDelay, maxDelay)).toBe(16000)
  })

  it('should cap delay at maxDelay', () => {
    const baseDelay = 1000
    const maxDelay = 30000

    expect(calculateReconnectDelay(5, baseDelay, maxDelay)).toBe(30000)
    expect(calculateReconnectDelay(10, baseDelay, maxDelay)).toBe(30000)
  })
})

// ======================== 命令确认超时测试 ========================

describe('Command Acknowledgment', () => {
  interface PendingCommand {
    seq: number
    sentAt: number
    timeout: ReturnType<typeof setTimeout>
  }

  it('should track pending commands', () => {
    const pendingCommands = new Map<number, PendingCommand>()
    const ACK_TIMEOUT = 500

    // Send command
    const seq = 1
    const sentAt = Date.now()
    const timeout = setTimeout(() => {
      pendingCommands.delete(seq)
    }, ACK_TIMEOUT)

    pendingCommands.set(seq, { seq, sentAt, timeout })
    expect(pendingCommands.has(seq)).toBe(true)

    // Receive ACK
    clearTimeout(pendingCommands.get(seq)!.timeout)
    pendingCommands.delete(seq)
    expect(pendingCommands.has(seq)).toBe(false)
  })

  it('should limit max pending commands', () => {
    const MAX_PENDING = 10
    const pendingCommands = new Map<number, { seq: number }>()

    function addCommand(seq: number) {
      if (pendingCommands.size >= MAX_PENDING) {
        const oldestSeq = Math.min(...pendingCommands.keys())
        pendingCommands.delete(oldestSeq)
      }
      pendingCommands.set(seq, { seq })
    }

    // Add 15 commands
    for (let i = 1; i <= 15; i++) {
      addCommand(i)
    }

    expect(pendingCommands.size).toBe(MAX_PENDING)
    expect(pendingCommands.has(6)).toBe(true)  // First 5 evicted
    expect(pendingCommands.has(5)).toBe(false)
    expect(pendingCommands.has(15)).toBe(true)
  })
})

// ======================== 连接状态机测试 ========================

describe('Connection State Machine', () => {
  type ConnectionStatus = 'disconnected' | 'connecting' | 'connected' | 'reconnecting' | 'failed'

  interface ConnectionState {
    status: ConnectionStatus
    signalingConnected: boolean
    webrtcConnected: boolean
    dataChannelOpen: boolean
    reconnectAttempt: number
  }

  function createInitialState(): ConnectionState {
    return {
      status: 'disconnected',
      signalingConnected: false,
      webrtcConnected: false,
      dataChannelOpen: false,
      reconnectAttempt: 0
    }
  }

  it('should transition through connecting states', () => {
    const state = createInitialState()

    // Start connection
    state.status = 'connecting'
    expect(state.status).toBe('connecting')

    // Signaling connected
    state.signalingConnected = true
    expect(state.signalingConnected).toBe(true)

    // WebRTC connected
    state.webrtcConnected = true
    state.status = 'connected'
    expect(state.status).toBe('connected')

    // Data channel opened
    state.dataChannelOpen = true
    expect(state.dataChannelOpen).toBe(true)
  })

  it('should handle disconnection', () => {
    const state = createInitialState()
    state.status = 'connected'
    state.signalingConnected = true
    state.webrtcConnected = true
    state.dataChannelOpen = true

    // Disconnect
    state.status = 'disconnected'
    state.signalingConnected = false
    state.webrtcConnected = false
    state.dataChannelOpen = false

    expect(state.status).toBe('disconnected')
    expect(state.signalingConnected).toBe(false)
  })

  it('should track reconnect attempts', () => {
    const state = createInitialState()
    const MAX_ATTEMPTS = 10

    state.status = 'reconnecting'
    for (let i = 1; i <= MAX_ATTEMPTS; i++) {
      state.reconnectAttempt = i
    }

    expect(state.reconnectAttempt).toBe(MAX_ATTEMPTS)

    // Should fail after max attempts
    if (state.reconnectAttempt >= MAX_ATTEMPTS) {
      state.status = 'failed'
    }
    expect(state.status).toBe('failed')
  })
})

// ======================== WebSocket 消息处理测试 ========================

describe('WebSocket Message Handling', () => {
  interface VehicleStatusMessage {
    event: 'vehicle_status'
    data: {
      vehicle_id: string
      speed: number
      steering: number
    }
  }

  interface AlertMessage {
    event: 'alert'
    data: {
      type: string
      severity: 'info' | 'warning' | 'error'
      message: string
    }
  }

  type Message = VehicleStatusMessage | AlertMessage | { event: string; data: unknown }

  function parseMessage(json: string): Message | null {
    try {
      const msg = JSON.parse(json)
      if (!msg.event) return null
      return msg
    } catch {
      return null
    }
  }

  it('should parse vehicle status messages', () => {
    const json = JSON.stringify({
      event: 'vehicle_status',
      data: { vehicle_id: 'FSM-01', speed: 30, steering: 0.1 }
    })

    const msg = parseMessage(json) as VehicleStatusMessage
    expect(msg?.event).toBe('vehicle_status')
    expect(msg?.data.speed).toBe(30)
  })

  it('should parse alert messages', () => {
    const json = JSON.stringify({
      event: 'alert',
      data: { type: 'obstacle', severity: 'warning', message: 'Obstacle detected' }
    })

    const msg = parseMessage(json) as AlertMessage
    expect(msg?.event).toBe('alert')
    expect(msg?.data.severity).toBe('warning')
  })

  it('should return null for invalid JSON', () => {
    expect(parseMessage('not json')).toBeNull()
    expect(parseMessage('{invalid}')).toBeNull()
  })

  it('should return null for messages without event', () => {
    expect(parseMessage('{"data": {}}')).toBeNull()
  })
})

// ======================== ICE 服务器配置测试 ========================

describe('ICE Server Configuration', () => {
  interface IceServerConfig {
    urls: string | string[]
    username?: string
    credential?: string
  }

  function mergeIceServers(defaults: IceServerConfig[], custom: IceServerConfig[]): RTCIceServer[] {
    const merged = new Map<string, RTCIceServer>()

    for (const server of [...defaults, ...custom]) {
      const urls = Array.isArray(server.urls) ? server.urls : [server.urls]
      const key = urls.sort().join(',')
      merged.set(key, server as RTCIceServer)
    }

    return Array.from(merged.values())
  }

  it('should merge default and custom ICE servers', () => {
    const defaults: IceServerConfig[] = [
      { urls: 'stun:stun.l.google.com:19302' }
    ]

    const custom: IceServerConfig[] = [
      { urls: 'turn:turn.example.com:3478', username: 'user', credential: 'pass' }
    ]

    const merged = mergeIceServers(defaults, custom)
    expect(merged.length).toBe(2)
  })

  it('should override duplicate servers', () => {
    const defaults: IceServerConfig[] = [
      { urls: 'stun:stun.l.google.com:19302' }
    ]

    const custom: IceServerConfig[] = [
      { urls: 'stun:stun.l.google.com:19302' } // Duplicate
    ]

    const merged = mergeIceServers(defaults, custom)
    expect(merged.length).toBe(1)
  })
})

// ======================== 控制频率节流测试 ========================

describe('Control Throttling', () => {
  it('should throttle control commands', () => {
    vi.useFakeTimers()

    const INTERVAL = 50 // 20 Hz
    let lastSent = 0
    let sentCount = 0

    function sendControl() {
      const now = Date.now()
      if (now - lastSent >= INTERVAL) {
        lastSent = now
        sentCount++
      }
    }

    // Try to send 100 times in 100ms
    for (let i = 0; i < 100; i++) {
      sendControl()
      vi.advanceTimersByTime(1)
    }

    // At 20 Hz (50ms interval), only ~2 commands should be sent in 100ms
    expect(sentCount).toBeLessThanOrEqual(3)

    vi.useRealTimers()
  })
})

// ======================== 紧急停车测试 ========================

describe('Emergency Stop', () => {
  interface ControlCommand {
    steering: number
    throttle: number
    brake: number
    emergency: boolean
  }

  function createEmergencyStopCommand(): ControlCommand {
    return {
      steering: 0,
      throttle: 0,
      brake: 1,
      emergency: true
    }
  }

  it('should set full brake on emergency', () => {
    const cmd = createEmergencyStopCommand()
    expect(cmd.brake).toBe(1)
    expect(cmd.throttle).toBe(0)
    expect(cmd.emergency).toBe(true)
  })

  it('should center steering on emergency', () => {
    const cmd = createEmergencyStopCommand()
    expect(cmd.steering).toBe(0)
  })
})

// ======================== 网络质量评估测试 ========================

describe('Network Quality Assessment', () => {
  type NetworkQuality = 'excellent' | 'good' | 'fair' | 'poor' | 'disconnected'

  function assessNetworkQuality(
    isConnected: boolean,
    latency: number,
    packetLoss: number
  ): NetworkQuality {
    if (!isConnected) return 'disconnected'
    if (packetLoss > 10) return 'poor'
    if (latency < 50 && packetLoss < 1) return 'excellent'
    if (latency < 100 && packetLoss < 5) return 'good'
    if (latency < 200 && packetLoss < 10) return 'fair'
    return 'poor'
  }

  it('should return disconnected when not connected', () => {
    expect(assessNetworkQuality(false, 0, 0)).toBe('disconnected')
  })

  it('should return excellent for low latency and no packet loss', () => {
    expect(assessNetworkQuality(true, 30, 0)).toBe('excellent')
    expect(assessNetworkQuality(true, 49, 0.5)).toBe('excellent')
  })

  it('should return good for moderate conditions', () => {
    expect(assessNetworkQuality(true, 60, 2)).toBe('good')
    expect(assessNetworkQuality(true, 99, 4)).toBe('good')
  })

  it('should return fair for higher latency', () => {
    expect(assessNetworkQuality(true, 150, 5)).toBe('fair')
    expect(assessNetworkQuality(true, 199, 9)).toBe('fair')
  })

  it('should return poor for bad conditions', () => {
    expect(assessNetworkQuality(true, 300, 5)).toBe('poor')
    expect(assessNetworkQuality(true, 50, 15)).toBe('poor')
  })
})
