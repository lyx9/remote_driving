/**
 * FSM-Pilot V2.0 - Vehicle Simulator
 *
 * @description Mock 车辆模拟器，用于演示和测试
 */

import { EventEmitter } from 'events'
import { logger } from '../utils/logger.js'

export interface VehicleConfig {
  id: string
  type: 'ROBO-TAXI' | 'LOGISTICS' | 'SECURITY'
  lat: number
  lng: number
}

export interface VehicleInfo {
  id: string
  name: string
  type: 'ROBO-TAXI' | 'LOGISTICS' | 'SECURITY'
  status: 'ACTIVE' | 'IDLE' | 'PATROL' | 'OFFLINE'
  location: { lat: number; lng: number }
  battery_level: number
  speed: number
  control_mode: 'MANUAL' | 'AUTONOMOUS' | 'REMOTE'
  is_connected: boolean
  last_update: number
}

export interface VehicleStatus {
  vehicle_id: string
  speed: number
  steering: number
  gear: string
  location: { lat: number; lng: number }
  latency_ms: number
  timestamp: number
}

export interface ControlCommand {
  vehicle_id: string
  steering?: number
  throttle?: number
  brake?: number
  gear?: 'P' | 'R' | 'N' | 'D'
  emergency?: boolean
}

export class VehicleSimulator extends EventEmitter {
  private config: VehicleConfig
  private status: VehicleStatus
  private info: VehicleInfo
  private updateInterval: NodeJS.Timeout | null = null
  private physics: VehiclePhysics

  // 控制输入
  private currentSteering = 0 // -1 to 1
  private currentThrottle = 0 // 0 to 1
  private currentBrake = 0 // 0 to 1
  private currentGear: 'P' | 'R' | 'N' | 'D' = 'P'

  constructor(config: VehicleConfig) {
    super()
    this.config = config
    this.physics = new VehiclePhysics()

    this.info = {
      id: config.id,
      name: config.id,
      type: config.type,
      status: 'IDLE',
      location: { lat: config.lat, lng: config.lng },
      battery_level: 85 + Math.random() * 15,
      speed: 0,
      control_mode: 'AUTONOMOUS',
      is_connected: false,
      last_update: Date.now()
    }

    this.status = {
      vehicle_id: config.id,
      speed: 0,
      steering: 0,
      gear: 'P',
      location: { lat: config.lat, lng: config.lng },
      latency_ms: 50 + Math.random() * 50,
      timestamp: Date.now()
    }
  }

  public start() {
    logger.info(`Starting vehicle simulator: ${this.config.id}`)

    // 更新循环 (50Hz)
    this.updateInterval = setInterval(() => {
      this.update()
    }, 20)

    this.info.status = 'ACTIVE'
  }

  public stop() {
    if (this.updateInterval) {
      clearInterval(this.updateInterval)
      this.updateInterval = null
    }
    this.info.status = 'OFFLINE'
    logger.info(`Stopped vehicle simulator: ${this.config.id}`)
  }

  private update() {
    // 更新物理模拟
    this.physics.update(
      this.currentThrottle,
      this.currentBrake,
      this.currentSteering,
      this.currentGear,
      0.02 // 20ms = 0.02s
    )

    // 更新状态
    this.status.speed = this.physics.speed
    this.status.steering = this.currentSteering * 30 // 转换为角度
    this.status.gear = this.currentGear
    this.status.latency_ms = 50 + Math.random() * 30
    this.status.timestamp = Date.now()

    // 更新位置 (简化GPS模拟)
    const heading = this.physics.heading
    const distanceKm = (this.physics.speed / 3600) * 0.02 // 20ms的移动距离
    this.status.location.lat += Math.cos(heading) * distanceKm / 111.32
    this.status.location.lng += Math.sin(heading) * distanceKm / (111.32 * Math.cos(this.status.location.lat * Math.PI / 180))

    // 更新info
    this.info.speed = this.physics.speed
    this.info.location = this.status.location
    this.info.last_update = Date.now()
    this.info.battery_level -= 0.0001 // 缓慢消耗电量

    // 定期发送状态更新 (5Hz)
    if (Date.now() % 200 < 20) {
      this.emit('status_update', this.status)
    }

    // 模拟告警 (低电量)
    if (this.info.battery_level < 20 && Math.random() < 0.01) {
      this.emit('alert', {
        id: `alert_${Date.now()}`,
        vehicle_id: this.config.id,
        type: 'LOW_BATTERY',
        severity: 'WARNING',
        title: 'Low Battery',
        message: `Battery level at ${this.info.battery_level.toFixed(1)}%`,
        timestamp: Date.now(),
        acknowledged: false
      })
    }
  }

  public handleControlCommand(command: ControlCommand) {
    if (command.steering !== undefined) {
      this.currentSteering = Math.max(-1, Math.min(1, command.steering))
    }

    if (command.throttle !== undefined) {
      this.currentThrottle = Math.max(0, Math.min(1, command.throttle))
    }

    if (command.brake !== undefined) {
      this.currentBrake = Math.max(0, Math.min(1, command.brake))
    }

    if (command.gear) {
      this.currentGear = command.gear
    }

    if (command.emergency) {
      this.currentThrottle = 0
      this.currentBrake = 1
      logger.warn(`Emergency stop activated on ${this.config.id}`)
    }

    // 切换到远程控制模式
    this.info.control_mode = 'REMOTE'
    this.info.is_connected = true
  }

  public connect() {
    this.info.is_connected = true
    this.info.control_mode = 'REMOTE'
    logger.info(`Vehicle ${this.config.id} connected`)
  }

  public disconnect() {
    this.info.is_connected = false
    this.info.control_mode = 'AUTONOMOUS'
    this.currentThrottle = 0
    this.currentBrake = 1
    logger.info(`Vehicle ${this.config.id} disconnected`)
  }

  public getInfo(): VehicleInfo {
    return { ...this.info }
  }

  public getStatus(): VehicleStatus {
    return { ...this.status }
  }

  public getAlerts() {
    // 简化实现，实际应该维护告警列表
    return []
  }
}

/**
 * 简化的车辆物理模型
 */
class VehiclePhysics {
  public speed = 0 // km/h
  public heading = 0 // radians
  private acceleration = 0 // m/s^2

  private readonly MAX_SPEED = 120 // km/h
  private readonly MAX_ACCELERATION = 3 // m/s^2
  private readonly MAX_DECELERATION = 8 // m/s^2
  private readonly DRAG_COEFFICIENT = 0.3

  public update(
    throttle: number,
    brake: number,
    steering: number,
    gear: string,
    dt: number
  ) {
    // 计算目标加速度
    let targetAccel = 0

    if (gear === 'D') {
      targetAccel = throttle * this.MAX_ACCELERATION
    } else if (gear === 'R') {
      targetAccel = -throttle * this.MAX_ACCELERATION * 0.5
    }

    // 应用刹车
    if (brake > 0) {
      targetAccel = -brake * this.MAX_DECELERATION
    }

    // 空气阻力
    const speedMs = this.speed / 3.6
    const dragForce = this.DRAG_COEFFICIENT * speedMs * speedMs
    targetAccel -= dragForce

    // 更新加速度 (带平滑)
    this.acceleration = this.acceleration * 0.9 + targetAccel * 0.1

    // 更新速度
    this.speed += this.acceleration * dt * 3.6 // 转换为 km/h
    this.speed = Math.max(-30, Math.min(this.MAX_SPEED, this.speed))

    // 更新航向
    if (Math.abs(this.speed) > 1) {
      const turnRate = steering * 0.5 * (this.speed / 50)
      this.heading += turnRate * dt
      this.heading = (this.heading + Math.PI * 2) % (Math.PI * 2)
    }

    // 停车时速度归零
    if (Math.abs(this.speed) < 0.1 && gear === 'P') {
      this.speed = 0
      this.acceleration = 0
    }
  }
}
