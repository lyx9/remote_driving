/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      Alibaba Cloud Backend Server
 * @description 阿里云后端服务器，提供信令、API、TURN凭证等功能
 */

import express from 'express'
import { WebSocketServer, WebSocket } from 'ws'
import cors from 'cors'
import { createServer } from 'http'
import crypto from 'crypto'

// ======================== 配置 ========================

const config = {
  port: parseInt(process.env.PORT || '3000'),
  wsPort: parseInt(process.env.WS_PORT || '8080'),

  // 数据库
  database: {
    url: process.env.DATABASE_URL || 'mysql://localhost:3306/fsmpilot'
  },

  // Redis
  redis: {
    url: process.env.REDIS_URL || 'redis://localhost:6379'
  },

  // JWT
  jwt: {
    secret: process.env.JWT_SECRET || 'your-jwt-secret-change-in-production',
    expiresIn: '24h'
  },

  // TURN 服务器
  turn: {
    host: process.env.TURN_HOST || 'turn.fsm-pilot.com',
    port: parseInt(process.env.TURN_PORT || '3478'),
    username: process.env.TURN_USERNAME || 'turnuser',
    credential: process.env.TURN_CREDENTIAL || 'turnpassword',
    ttl: 86400 // 24小时
  },

  // 阿里云
  aliyun: {
    accessKeyId: process.env.ALIYUN_ACCESS_KEY_ID || '',
    accessKeySecret: process.env.ALIYUN_ACCESS_KEY_SECRET || '',
    region: process.env.ALIYUN_REGION || 'cn-hongkong'
  }
}

// ======================== Express 应用 ========================

const app = express()
const server = createServer(app)

// 中间件
app.use(cors())
app.use(express.json())

// 健康检查
app.get('/api/health', (req, res) => {
  res.json({
    status: 'ok',
    timestamp: Date.now(),
    uptime: process.uptime(),
    version: '2.0.0'
  })
})

// ======================== TURN 凭证 API ========================

/**
 * 生成 TURN 临时凭证
 * 使用 HMAC-SHA1 算法生成时间限制的凭证
 */
function generateTurnCredential(username: string, secret: string, ttl: number): {
  username: string
  credential: string
  ttl: number
} {
  const timestamp = Math.floor(Date.now() / 1000) + ttl
  const turnUsername = `${timestamp}:${username}`

  const hmac = crypto.createHmac('sha1', secret)
  hmac.update(turnUsername)
  const credential = hmac.digest('base64')

  return {
    username: turnUsername,
    credential,
    ttl
  }
}

app.post('/api/turn/credentials', (req, res) => {
  const { username = 'webrtc-user', ttl = config.turn.ttl } = req.body

  try {
    const credentials = generateTurnCredential(
      username,
      config.turn.credential,
      Math.min(ttl, 86400) // 最大24小时
    )

    const iceServers = [
      // STUN 服务器
      { urls: 'stun:stun.l.google.com:19302' },
      { urls: 'stun:stun1.l.google.com:19302' },
      // TURN 服务器
      {
        urls: [
          `turn:${config.turn.host}:${config.turn.port}?transport=udp`,
          `turn:${config.turn.host}:${config.turn.port}?transport=tcp`,
          `turns:${config.turn.host}:443?transport=tcp`
        ],
        username: credentials.username,
        credential: credentials.credential
      }
    ]

    res.json({
      success: true,
      iceServers,
      ttl: credentials.ttl
    })
  } catch (error) {
    res.status(500).json({
      success: false,
      error: 'Failed to generate TURN credentials'
    })
  }
})

// ======================== 车辆管理 API ========================

interface Vehicle {
  id: string
  name: string
  status: 'online' | 'offline' | 'busy' | 'error'
  location: { lat: number; lng: number }
  lastSeen: number
}

const vehicles = new Map<string, Vehicle>()

// 初始化测试车辆
vehicles.set('FSM-01', {
  id: 'FSM-01',
  name: 'FSM Test Vehicle 1',
  status: 'online',
  location: { lat: 22.3193, lng: 114.1694 },
  lastSeen: Date.now()
})

app.get('/api/vehicles', (req, res) => {
  res.json({
    success: true,
    vehicles: Array.from(vehicles.values())
  })
})

app.get('/api/vehicles/:id', (req, res) => {
  const vehicle = vehicles.get(req.params.id)
  if (!vehicle) {
    return res.status(404).json({
      success: false,
      error: 'Vehicle not found'
    })
  }
  res.json({
    success: true,
    vehicle
  })
})

app.post('/api/vehicles/:id/command', (req, res) => {
  const { id } = req.params
  const { command, data } = req.body

  const vehicle = vehicles.get(id)
  if (!vehicle) {
    return res.status(404).json({
      success: false,
      error: 'Vehicle not found'
    })
  }

  // 发送命令到车辆 (通过 WebSocket)
  const vehicleWs = vehicleConnections.get(id)
  if (vehicleWs && vehicleWs.readyState === WebSocket.OPEN) {
    vehicleWs.send(JSON.stringify({ type: 'command', command, data }))
    res.json({ success: true, message: 'Command sent' })
  } else {
    res.status(503).json({
      success: false,
      error: 'Vehicle not connected'
    })
  }
})

// ======================== 会话管理 API ========================

interface Session {
  id: string
  vehicleId: string
  operatorId: string
  startTime: number
  endTime?: number
  status: 'active' | 'ended' | 'failed'
}

const sessions = new Map<string, Session>()

app.post('/api/sessions', (req, res) => {
  const { vehicleId, operatorId } = req.body

  // 检查车辆是否可用
  const vehicle = vehicles.get(vehicleId)
  if (!vehicle) {
    return res.status(404).json({
      success: false,
      error: 'Vehicle not found'
    })
  }

  if (vehicle.status !== 'online') {
    return res.status(400).json({
      success: false,
      error: 'Vehicle is not available'
    })
  }

  // 创建会话
  const sessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`
  const session: Session = {
    id: sessionId,
    vehicleId,
    operatorId,
    startTime: Date.now(),
    status: 'active'
  }

  sessions.set(sessionId, session)
  vehicle.status = 'busy'

  res.json({
    success: true,
    session
  })
})

app.delete('/api/sessions/:id', (req, res) => {
  const session = sessions.get(req.params.id)
  if (!session) {
    return res.status(404).json({
      success: false,
      error: 'Session not found'
    })
  }

  session.status = 'ended'
  session.endTime = Date.now()

  // 释放车辆
  const vehicle = vehicles.get(session.vehicleId)
  if (vehicle) {
    vehicle.status = 'online'
  }

  res.json({
    success: true,
    session
  })
})

// ======================== WebSocket 信令服务器 ========================

const wss = new WebSocketServer({ server, path: '/signaling' })

// 连接映射
const operatorConnections = new Map<string, WebSocket>()
const vehicleConnections = new Map<string, WebSocket>()

interface SignalingMessage {
  type: string
  vehicleId?: string
  operatorId?: string
  sdp?: string
  candidate?: RTCIceCandidateInit
  data?: unknown
}

wss.on('connection', (ws, req) => {
  const url = new URL(req.url || '', `ws://${req.headers.host}`)
  const clientType = url.searchParams.get('type') // 'operator' | 'vehicle'
  const clientId = url.searchParams.get('id')

  console.log(`[Signaling] Client connected: ${clientType} ${clientId}`)

  if (!clientType || !clientId) {
    ws.close(4000, 'Missing client type or id')
    return
  }

  // 注册连接
  if (clientType === 'operator') {
    operatorConnections.set(clientId, ws)
  } else if (clientType === 'vehicle') {
    vehicleConnections.set(clientId, ws)
    // 更新车辆状态
    const vehicle = vehicles.get(clientId)
    if (vehicle) {
      vehicle.status = 'online'
      vehicle.lastSeen = Date.now()
    }
  }

  ws.on('message', (data) => {
    try {
      const message: SignalingMessage = JSON.parse(data.toString())
      handleSignalingMessage(ws, clientType, clientId, message)
    } catch (e) {
      console.error('[Signaling] Failed to parse message:', e)
    }
  })

  ws.on('close', () => {
    console.log(`[Signaling] Client disconnected: ${clientType} ${clientId}`)

    if (clientType === 'operator') {
      operatorConnections.delete(clientId)
    } else if (clientType === 'vehicle') {
      vehicleConnections.delete(clientId)
      // 更新车辆状态
      const vehicle = vehicles.get(clientId)
      if (vehicle) {
        vehicle.status = 'offline'
      }
    }
  })

  ws.on('error', (error) => {
    console.error(`[Signaling] WebSocket error:`, error)
  })

  // 发送连接确认
  ws.send(JSON.stringify({
    type: 'connected',
    clientType,
    clientId,
    timestamp: Date.now()
  }))
})

function handleSignalingMessage(
  ws: WebSocket,
  clientType: string,
  clientId: string,
  message: SignalingMessage
): void {
  switch (message.type) {
    case 'connect':
      // 操作员请求连接车辆
      if (clientType === 'operator' && message.vehicleId) {
        const vehicleWs = vehicleConnections.get(message.vehicleId)
        if (vehicleWs && vehicleWs.readyState === WebSocket.OPEN) {
          vehicleWs.send(JSON.stringify({
            type: 'connect_request',
            operatorId: clientId,
            timestamp: Date.now()
          }))
          ws.send(JSON.stringify({
            type: 'vehicle_ready',
            vehicleId: message.vehicleId
          }))
        } else {
          ws.send(JSON.stringify({
            type: 'error',
            message: 'Vehicle not connected'
          }))
        }
      }
      break

    case 'offer':
    case 'answer':
    case 'ice-candidate':
      // 转发 SDP/ICE 消息
      forwardSignalingMessage(clientType, clientId, message)
      break

    case 'heartbeat':
      // 心跳响应
      ws.send(JSON.stringify({
        type: 'heartbeat_ack',
        timestamp: Date.now()
      }))
      break

    case 'telemetry':
      // 车辆遥测数据
      if (clientType === 'vehicle') {
        // 更新车辆状态
        const vehicle = vehicles.get(clientId)
        if (vehicle && message.data) {
          const telemetry = message.data as { location?: { lat: number; lng: number } }
          if (telemetry.location) {
            vehicle.location = telemetry.location
          }
          vehicle.lastSeen = Date.now()
        }

        // 转发给相关操作员
        broadcastToOperators(clientId, message)
      }
      break
  }
}

function forwardSignalingMessage(
  clientType: string,
  clientId: string,
  message: SignalingMessage
): void {
  if (clientType === 'operator' && message.vehicleId) {
    // 操作员 -> 车辆
    const vehicleWs = vehicleConnections.get(message.vehicleId)
    if (vehicleWs && vehicleWs.readyState === WebSocket.OPEN) {
      vehicleWs.send(JSON.stringify({
        ...message,
        operatorId: clientId
      }))
    }
  } else if (clientType === 'vehicle' && message.operatorId) {
    // 车辆 -> 操作员
    const operatorWs = operatorConnections.get(message.operatorId)
    if (operatorWs && operatorWs.readyState === WebSocket.OPEN) {
      operatorWs.send(JSON.stringify({
        ...message,
        vehicleId: clientId
      }))
    }
  }
}

function broadcastToOperators(vehicleId: string, message: SignalingMessage): void {
  // 找到所有连接到该车辆的操作员并广播
  operatorConnections.forEach((ws, operatorId) => {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({
        ...message,
        vehicleId
      }))
    }
  })
}

// ======================== 启动服务器 ========================

server.listen(config.port, () => {
  console.log(`[Server] HTTP/WebSocket server running on port ${config.port}`)
  console.log(`[Server] Signaling endpoint: ws://localhost:${config.port}/signaling`)
  console.log(`[Server] API endpoint: http://localhost:${config.port}/api`)
})

// 优雅关闭
process.on('SIGTERM', () => {
  console.log('[Server] SIGTERM received, shutting down...')

  wss.clients.forEach(client => {
    client.close(1001, 'Server shutting down')
  })

  server.close(() => {
    console.log('[Server] Server closed')
    process.exit(0)
  })
})

export { app, server }
