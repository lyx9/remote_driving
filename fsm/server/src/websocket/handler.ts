/**
 * FSM-Pilot V2.0 - WebSocket Handler
 *
 * @description WebSocket 消息处理器
 */

import { WebSocketServer, WebSocket } from 'ws'
import { v4 as uuidv4 } from 'uuid'
import { logger } from '../utils/logger.js'
import { EventEmitter } from 'events'

interface Client {
  id: string
  ws: WebSocket
  connectedAt: number
  vehicleId?: string
}

export class WebSocketHandler extends EventEmitter {
  private clients: Map<string, Client> = new Map()

  constructor(private wss: WebSocketServer) {
    super()
    this.setupWebSocketServer()
  }

  private setupWebSocketServer() {
    this.wss.on('connection', (ws: WebSocket) => {
      const clientId = uuidv4()
      const client: Client = {
        id: clientId,
        ws,
        connectedAt: Date.now()
      }

      this.clients.set(clientId, client)
      logger.info(`WebSocket client connected: ${clientId} (Total: ${this.clients.size})`)

      // 发送欢迎消息
      this.sendToClient(clientId, {
        type: 'connected',
        data: {
          client_id: clientId,
          server_time: Date.now()
        }
      })

      ws.on('message', (data: Buffer) => {
        try {
          const message = JSON.parse(data.toString())
          this.handleMessage(clientId, message)
        } catch (error) {
          logger.error(`Failed to parse WebSocket message from ${clientId}:`, error)
        }
      })

      ws.on('close', () => {
        this.clients.delete(clientId)
        logger.info(`WebSocket client disconnected: ${clientId} (Remaining: ${this.clients.size})`)
      })

      ws.on('error', (error) => {
        logger.error(`WebSocket error for client ${clientId}:`, error)
      })

      // 心跳
      const heartbeat = setInterval(() => {
        if (ws.readyState === WebSocket.OPEN) {
          ws.ping()
        } else {
          clearInterval(heartbeat)
        }
      }, 30000)

      ws.on('pong', () => {
        // 客户端存活
      })
    })
  }

  private handleMessage(clientId: string, message: any) {
    const { type, data } = message

    switch (type) {
      case 'control_command':
        this.emit('control_command', clientId, data)
        break

      case 'get_vehicles':
        this.emit('get_vehicles', clientId)
        break

      case 'subscribe_vehicle':
        const client = this.clients.get(clientId)
        if (client) {
          client.vehicleId = data.vehicle_id
          logger.info(`Client ${clientId} subscribed to vehicle ${data.vehicle_id}`)
        }
        break

      case 'ping':
        this.sendToClient(clientId, {
          type: 'pong',
          data: { timestamp: Date.now() }
        })
        break

      default:
        logger.warn(`Unknown message type from ${clientId}: ${type}`)
    }
  }

  public sendToClient(clientId: string, message: any) {
    const client = this.clients.get(clientId)
    if (client && client.ws.readyState === WebSocket.OPEN) {
      client.ws.send(JSON.stringify(message))
    }
  }

  public broadcast(message: any, excludeClientId?: string) {
    const payload = JSON.stringify(message)
    this.clients.forEach((client, id) => {
      if (id !== excludeClientId && client.ws.readyState === WebSocket.OPEN) {
        client.ws.send(payload)
      }
    })
  }

  public broadcastToVehicleSubscribers(vehicleId: string, message: any) {
    const payload = JSON.stringify(message)
    this.clients.forEach((client) => {
      if (client.vehicleId === vehicleId && client.ws.readyState === WebSocket.OPEN) {
        client.ws.send(payload)
      }
    })
  }

  public getConnectedClients() {
    return this.clients.size
  }
}
