/**
 * FSM-Pilot - 阿里云 IoT MQTT 服务
 *
 * 通过 MQTT over WebSocket 与车端通信:
 *   - 订阅 6 路摄像头 JPEG 帧
 *   - 订阅遥测数据 (速度/位置/状态)
 *   - 发布控制指令
 *
 * 依赖: npm install mqtt
 */

import mqtt, { type MqttClient, type IClientOptions } from 'mqtt'
import { ref, reactive, readonly } from 'vue'
import logger from '@/utils/logger'

// ======================== Topic 定义 ========================

export function makeTopics(vehicleId: string) {
  const base = `fsm/${vehicleId}`
  return {
    cam:        Array.from({ length: 6 }, (_, i) => `${base}/cam/${i}`),
    telemetry:  `${base}/telemetry`,
    status:     `${base}/status`,
    control:    `${base}/control`,
    controlAck: `${base}/control_ack`,
    cmd:        `${base}/cmd`,
  }
}

// ======================== 类型定义 ========================

export interface MQTTConfig {
  /** 阿里云 IoT WebSocket endpoint, 例如 wss://xxx.iot-as-mqtt.cn-shanghai.aliyuncs.com/mqtt */
  brokerUrl: string
  clientId: string
  username: string
  password: string
  vehicleId: string
}

export interface VehicleTelemetry {
  vehicle_id: string
  speed: number       // km/h
  steering: number    // rad
  gear: 'P' | 'R' | 'N' | 'D'
  lat: number
  lng: number
  heading: number
  battery: number
  timestamp: number
}

export interface ControlCommand {
  seq: number
  steering: number    // -1.0 ~ 1.0
  throttle: number    // 0 ~ 1.0
  brake: number       // 0 ~ 1.0
  gear?: number       // 0=P,1=R,2=N,3=D
  emergency?: boolean
}

export type ConnectionStatus = 'disconnected' | 'connecting' | 'connected' | 'error'

// ======================== MQTT 服务 ========================

export class MQTTVehicleService {
  private client: MqttClient | null = null
  private config: MQTTConfig
  private topics: ReturnType<typeof makeTopics>
  private cmdSeq = 0

  // 响应式状态
  public status = ref<ConnectionStatus>('disconnected')
  public telemetry = ref<VehicleTelemetry | null>(null)
  public vehicleOnline = ref(false)
  public latency = ref(0)

  // 6路摄像头 ObjectURL (用于 <img> 显示)
  public cameraUrls = reactive<string[]>(Array(6).fill(''))

  // 回调
  private _onCameraFrame: ((idx: number, blob: Blob) => void)[] = []
  private _onTelemetry: ((data: VehicleTelemetry) => void)[] = []
  private _onStatusChange: ((status: ConnectionStatus) => void)[] = []

  // 帧统计
  private _frameCount = Array(6).fill(0)
  private _lastFpsTime = Date.now()
  public cameraFps = reactive<number[]>(Array(6).fill(0))

  constructor(config: MQTTConfig) {
    this.config = config
    this.topics = makeTopics(config.vehicleId)
  }

  // ======================== 连接 ========================

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      if (this.client?.connected) { resolve(); return }

      this.status.value = 'connecting'
      this._notifyStatus('connecting')

      const options: IClientOptions = {
        clientId: this.config.clientId,
        username: this.config.username,
        password: this.config.password,
        clean: true,
        reconnectPeriod: 5000,
        connectTimeout: 15000,
        // 阿里云 IoT 需要 WebSocket 子协议
        wsOptions: { rejectUnauthorized: false },
      }

      this.client = mqtt.connect(this.config.brokerUrl, options)

      this.client.on('connect', () => {
        logger.mqtt(`Connected to Aliyun IoT: ${this.config.brokerUrl}`, 'success')
        this.status.value = 'connected'
        this._notifyStatus('connected')
        this._subscribe()
        resolve()
      })

      this.client.on('error', (err) => {
        logger.mqtt(`Connection error: ${err.message}`, 'error', String(err))
        this.status.value = 'error'
        this._notifyStatus('error')
        reject(err)
      })

      this.client.on('offline', () => {
        logger.mqtt(`Offline / disconnected from broker`, 'warning')
        this.status.value = 'disconnected'
        this._notifyStatus('disconnected')
        this.vehicleOnline.value = false
      })

      this.client.on('message', (topic, payload) => {
        this._handleMessage(topic, payload)
      })

      // FPS 计算定时器
      setInterval(() => this._calcFps(), 1000)
    })
  }

  disconnect() {
    this.client?.end(true)
    this.client = null
    this.status.value = 'disconnected'
    this.vehicleOnline.value = false
  }

  // ======================== 订阅 ========================

  private _subscribe() {
    if (!this.client) return

    // 6路摄像头 (QoS 0, 高频)
    this.topics.cam.forEach(t => this.client!.subscribe(t, { qos: 0 }))

    // 遥测 + 状态 (QoS 0)
    this.client.subscribe(this.topics.telemetry, { qos: 0 })
    this.client.subscribe(this.topics.status, { qos: 1 })
    this.client.subscribe(this.topics.controlAck, { qos: 0 })

    logger.mqtt(`Subscribed to vehicle topics [${this.config.vehicleId}]: cam×6, telemetry, status, control_ack`, 'success')
  }

  // ======================== 消息处理 ========================

  private _handleMessage(topic: string, payload: Buffer) {
    // 摄像头帧 (二进制 JPEG)
    const camIdx = this.topics.cam.indexOf(topic)
    if (camIdx >= 0) {
      this._handleCameraFrame(camIdx, payload)
      return
    }

    // JSON 消息
    try {
      const data = JSON.parse(payload.toString())

      if (topic === this.topics.telemetry) {
        this.telemetry.value = data as VehicleTelemetry
        this._onTelemetry.forEach(cb => cb(data))
        // 低频遥测日志 (每30条记一次，避免刷屏)
        if (this.cmdSeq % 30 === 0) {
          logger.telemetry(`speed=${data.speed?.toFixed(1)}km/h gear=${data.gear} bat=${data.battery}%`)
        }
      } else if (topic === this.topics.status) {
        const wasOnline = this.vehicleOnline.value
        this.vehicleOnline.value = data.online === true
        if (wasOnline !== this.vehicleOnline.value) {
          logger.vehicle(
            `Vehicle ${this.config.vehicleId} ${data.online ? 'came online' : 'went offline'}`,
            data.online ? 'success' : 'warning'
          )
        }
      } else if (topic === this.topics.controlAck) {
        if (data.ts) {
          this.latency.value = Date.now() - data.ts
        }
      }
    } catch (e) {
      // 非 JSON 消息忽略
    }
  }

  private _handleCameraFrame(idx: number, payload: Buffer) {
    if (this.cameraUrls[idx]) {
      URL.revokeObjectURL(this.cameraUrls[idx])
    }
    const blob = new Blob([payload], { type: 'image/jpeg' })
    this.cameraUrls[idx] = URL.createObjectURL(blob)
    this._frameCount[idx]++
    this._onCameraFrame.forEach(cb => cb(idx, blob))
  }

  private _calcFps() {
    const now = Date.now()
    const elapsed = (now - this._lastFpsTime) / 1000
    let anyDead = false
    this._frameCount.forEach((count, i) => {
      const fps = Math.round(count / elapsed)
      const prev = this.cameraFps[i]
      this.cameraFps[i] = fps
      this._frameCount[i] = 0
      // 摄像头掉线/恢复日志
      if (prev > 0 && fps === 0) {
        logger.camera(`CAM${i} signal lost (was ${prev}fps)`, 'warning')
        anyDead = true
      } else if (prev === 0 && fps > 0) {
        logger.camera(`CAM${i} signal restored (${fps}fps)`, 'success')
      }
    })
    this._lastFpsTime = now
  }

  // ======================== 控制指令发布 ========================

  sendControl(cmd: Omit<ControlCommand, 'seq'>) {
    if (!this.client?.connected) {
      logger.control('Control dropped: MQTT not connected', 'warning')
      return
    }

    const message: ControlCommand = {
      seq: ++this.cmdSeq,
      ...cmd,
    }

    this.client.publish(
      this.topics.control,
      JSON.stringify(message),
      { qos: 0 }  // 控制指令用 QoS 0 保证低延迟
    )
  }

  emergencyStop() {
    this.sendControl({ steering: 0, throttle: 0, brake: 1.0, emergency: true })
    logger.control('EMERGENCY STOP sent', 'error')
  }

  sendCmd(cmd: string, params?: object) {
    this.client?.publish(
      this.topics.cmd,
      JSON.stringify({ cmd, params, ts: Date.now() }),
      { qos: 1 }
    )
  }

  // ======================== 回调注册 ========================

  onCameraFrame(cb: (idx: number, blob: Blob) => void) {
    this._onCameraFrame.push(cb)
  }

  onTelemetry(cb: (data: VehicleTelemetry) => void) {
    this._onTelemetry.push(cb)
  }

  onStatusChange(cb: (status: ConnectionStatus) => void) {
    this._onStatusChange.push(cb)
  }

  private _notifyStatus(s: ConnectionStatus) {
    this._onStatusChange.forEach(cb => cb(s))
  }

  // ======================== 只读状态 ========================

  get isConnected() { return this.status.value === 'connected' }
  get vehicleId() { return this.config.vehicleId }
}

// ======================== 单例管理 ========================

let _instance: MQTTVehicleService | null = null

export function getMQTTService(config?: MQTTConfig): MQTTVehicleService {
  if (!_instance && config) {
    _instance = new MQTTVehicleService(config)
  }
  if (!_instance) throw new Error('MQTT service not initialized')
  return _instance
}

export function initMQTTService(config: MQTTConfig): MQTTVehicleService {
  _instance?.disconnect()
  _instance = new MQTTVehicleService(config)
  return _instance
}

// ======================== Vue Composable ========================

export function useMQTTVehicle(config?: MQTTConfig) {
  const service = config ? initMQTTService(config) : getMQTTService()

  return {
    status:       readonly(service.status),
    telemetry:    readonly(service.telemetry),
    vehicleOnline: readonly(service.vehicleOnline),
    latency:      readonly(service.latency),
    cameraUrls:   service.cameraUrls,
    cameraFps:    service.cameraFps,

    connect:       () => service.connect(),
    disconnect:    () => service.disconnect(),
    sendControl:   (cmd: Omit<ControlCommand, 'seq'>) => service.sendControl(cmd),
    emergencyStop: () => service.emergencyStop(),
    onCameraFrame: (cb: (idx: number, blob: Blob) => void) => service.onCameraFrame(cb),
    onTelemetry:   (cb: (data: VehicleTelemetry) => void) => service.onTelemetry(cb),
  }
}
