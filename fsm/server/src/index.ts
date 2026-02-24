/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      Backend Server Entry Point
 * @description WebSocket + REST API服务器，支持远程控制和实时通信
 */

import express from 'express'
import cors from 'cors'
import { WebSocketServer } from 'ws'
import { createServer } from 'http'
import { WebSocketHandler } from './websocket/handler.js'
import { VehicleSimulator, type VehicleConfig } from './simulator/vehicle.js'
import { SchedulingService } from './services/scheduling.js'
import { logger } from './utils/logger.js'

const PORT = process.env.PORT || 8080
const WS_PORT = process.env.WS_PORT || 8081

class FsmPilotServer {
  private app: express.Application
  private httpServer: ReturnType<typeof createServer>
  private wss: WebSocketServer
  private wsHandler: WebSocketHandler
  private scheduling: SchedulingService
  private vehicles: Map<string, VehicleSimulator> = new Map()

  constructor() {
    this.app = express()
    this.httpServer = createServer(this.app)
    this.wss = new WebSocketServer({ server: this.httpServer })
    this.wsHandler = new WebSocketHandler(this.wss)
    this.scheduling = new SchedulingService()

    this.setupMiddleware()
    this.setupRoutes()
    this.setupWebSocket()
    this.initializeMockVehicles()
  }

  private setupMiddleware() {
    this.app.use(cors())
    this.app.use(express.json())

    // 请求日志
    this.app.use((req, res, next) => {
      logger.info(`${req.method} ${req.path}`)
      next()
    })
  }

  private setupRoutes() {
    // 健康检查
    this.app.get('/api/v1/health', (req, res) => {
      res.json({
        status: 'healthy',
        services: {
          signaling: true,
          database: true,
          turn_server: true
        },
        uptime_seconds: process.uptime()
      })
    })

    // 获取所有车辆
    this.app.get('/api/v1/vehicles', (req, res) => {
      const vehicles = Array.from(this.vehicles.values()).map(v => v.getInfo())
      res.json(vehicles)
    })

    // 获取单个车辆
    this.app.get('/api/v1/vehicles/:id', (req, res) => {
      const vehicle = this.vehicles.get(req.params.id)
      if (!vehicle) {
        return res.status(404).json({ error: 'Vehicle not found' })
      }
      res.json(vehicle.getInfo())
    })

    // 获取车辆状态
    this.app.get('/api/v1/vehicles/:id/status', (req, res) => {
      const vehicle = this.vehicles.get(req.params.id)
      if (!vehicle) {
        return res.status(404).json({ error: 'Vehicle not found' })
      }
      res.json(vehicle.getStatus())
    })

    // 连接车辆 (WebRTC)
    this.app.post('/api/v1/vehicles/:id/connect', (req, res) => {
      const vehicle = this.vehicles.get(req.params.id)
      if (!vehicle) {
        return res.status(404).json({ error: 'Vehicle not found' })
      }

      res.json({
        session_id: `session_${Date.now()}`,
        signaling_url: `ws://localhost:${WS_PORT}`,
        ice_servers: [
          { urls: 'stun:stun.l.google.com:19302' }
        ]
      })
    })

    // 断开车辆
    this.app.delete('/api/v1/vehicles/:id/connect', (req, res) => {
      const vehicle = this.vehicles.get(req.params.id)
      if (!vehicle) {
        return res.status(404).json({ error: 'Vehicle not found' })
      }
      vehicle.disconnect()
      res.json({ message: 'Disconnected' })
    })

    // 调度队列
    this.app.get('/api/v1/scheduling/queue', (req, res) => {
      const queue = this.scheduling.getQueue()
      res.json({
        queue,
        algorithm: 'weighted_priority',
        scheduling_enabled: true,
        timestamp: Date.now()
      })
    })

    // 调度配置
    this.app.get('/api/v1/scheduling/config', (req, res) => {
      res.json(this.scheduling.getConfig())
    })

    this.app.put('/api/v1/scheduling/config', (req, res) => {
      this.scheduling.updateConfig(req.body)
      res.json(this.scheduling.getConfig())
    })

    // 告警列表
    this.app.get('/api/v1/alerts', (req, res) => {
      const alerts = Array.from(this.vehicles.values())
        .flatMap(v => v.getAlerts())
        .slice(0, Number(req.query.limit) || 50)
      res.json(alerts)
    })

    // 系统统计
    this.app.get('/api/v1/system/stats', (req, res) => {
      const vehicles = Array.from(this.vehicles.values())
      const connectedVehicles = vehicles.filter(v => v.getInfo().is_connected)

      res.json({
        connected_vehicles: connectedVehicles.length,
        total_vehicles: vehicles.length,
        active_sessions: connectedVehicles.length,
        total_bandwidth_mbps: connectedVehicles.length * 2.5,
        avg_latency_ms: connectedVehicles.reduce((sum, v) =>
          sum + v.getStatus().latency_ms, 0) / (connectedVehicles.length || 1)
      })
    })
  }

  private setupWebSocket() {
    this.wsHandler.on('control_command', (clientId, data) => {
      // 转发控制命令到对应车辆
      const vehicle = this.vehicles.get(data.vehicle_id)
      if (vehicle) {
        vehicle.handleControlCommand(data)

        // 广播车辆状态更新
        this.wsHandler.broadcast({
          type: 'vehicle_status',
          data: vehicle.getStatus()
        })
      }
    })

    this.wsHandler.on('get_vehicles', (clientId) => {
      const vehicles = Array.from(this.vehicles.values()).map(v => v.getInfo())
      this.wsHandler.sendToClient(clientId, {
        type: 'vehicles_list',
        data: vehicles
      })
    })
  }

  private initializeMockVehicles() {
    // 创建 3 个模拟车辆
    const vehicleConfigs: VehicleConfig[] = [
      { id: 'FSM-01', type: 'ROBO-TAXI', lat: 31.2304, lng: 121.4737 },
      { id: 'FSM-02', type: 'LOGISTICS', lat: 31.235, lng: 121.480 },
      { id: 'FSM-03', type: 'SECURITY', lat: 31.220, lng: 121.460 }
    ]

    for (const config of vehicleConfigs) {
      const vehicle = new VehicleSimulator(config)
      this.vehicles.set(config.id, vehicle)

      // 启动车辆模拟器
      vehicle.start()

      // 监听车辆状态变化
      vehicle.on('status_update', (status) => {
        this.wsHandler.broadcast({
          type: 'vehicle_status',
          data: status
        })

        // 更新调度队列
        this.scheduling.updateVehicle(config.id, status)
      })

      vehicle.on('alert', (alert) => {
        this.wsHandler.broadcast({
          type: 'alert',
          data: alert
        })
      })

      logger.info(`Initialized mock vehicle: ${config.id}`)
    }
  }

  public async start() {
    return new Promise<void>((resolve) => {
      this.httpServer.listen(PORT, () => {
        logger.info('═══════════════════════════════════════════════════════')
        logger.info('  FSM-Pilot Backend Server v2.0')
        logger.info('  远程驾驶平台后端服务')
        logger.info('═══════════════════════════════════════════════════════')
        logger.info(`  HTTP Server:       http://localhost:${PORT}`)
        logger.info(`  WebSocket Server:  ws://localhost:${PORT}`)
        logger.info(`  API Docs:          http://localhost:${PORT}/api/v1`)
        logger.info('═══════════════════════════════════════════════════════')
        logger.info(`  Mock Vehicles: ${this.vehicles.size} initialized`)
        logger.info('═══════════════════════════════════════════════════════')
        resolve()
      })
    })
  }

  public async stop() {
    // 停止所有车辆模拟器
    for (const vehicle of this.vehicles.values()) {
      vehicle.stop()
    }

    this.wss.close()
    this.httpServer.close()
    logger.info('Server stopped')
  }
}

// 启动服务器
const server = new FsmPilotServer()

server.start().catch(err => {
  logger.error('Failed to start server:', err)
  process.exit(1)
})

// 优雅关闭
process.on('SIGINT', async () => {
  logger.info('Received SIGINT, shutting down...')
  await server.stop()
  process.exit(0)
})

process.on('SIGTERM', async () => {
  logger.info('Received SIGTERM, shutting down...')
  await server.stop()
  process.exit(0)
})
