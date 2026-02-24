/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      VehicleDispatchService
 * @description 车队调度服务 - 支持 100+ 车辆的智能接管调度
 */

import { ref, computed, readonly } from 'vue'

// ======================== 类型定义 ========================

/**
 * 优先级等级
 */
export enum Priority {
  EMERGENCY = 0,    // 紧急情况（碰撞预警、系统故障）
  HIGH = 1,         // 高优先级（复杂路况、交通违章）
  MEDIUM = 2,       // 中等优先级（停车、掉头）
  LOW = 3,          // 低优先级（路线优化咨询）
  ROUTINE = 4       // 常规（定期检查）
}

/**
 * 事件类型
 */
export enum EventType {
  COLLISION_WARNING = 'collision_warning',      // 碰撞预警
  TRAFFIC_VIOLATION = 'traffic_violation',      // 交通违章
  SYSTEM_ERROR = 'system_error',                // 系统错误
  ROUTE_BLOCKED = 'route_blocked',              // 路径阻塞
  PASSENGER_REQUEST = 'passenger_request',      // 乘客请求
  SCHEDULED_HANDOVER = 'scheduled_handover',    // 计划交接
  NETWORK_DEGRADATION = 'network_degradation',  // 网络降级
  SENSOR_FAILURE = 'sensor_failure'             // 传感器故障
}

/**
 * 车辆状态
 */
export enum VehicleStatus {
  ONLINE = 'online',              // 在线（自动驾驶中）
  REQUESTING = 'requesting',      // 请求接管中
  CONNECTED = 'connected',        // 已接入远程
  OFFLINE = 'offline',            // 离线
  MAINTENANCE = 'maintenance'     // 维护中
}

/**
 * 地理位置
 */
export interface Location {
  latitude: number
  longitude: number
  city: string
  region: string
}

/**
 * 网络质量
 */
export interface NetworkMetrics {
  latency: number        // 延迟 (ms)
  bandwidth: number      // 带宽 (Mbps)
  packetLoss: number     // 丢包率 (%)
  jitter: number         // 抖动 (ms)
}

/**
 * 车辆信息
 */
export interface Vehicle {
  id: string
  name: string
  status: VehicleStatus
  location: Location
  network: NetworkMetrics
  priority: Priority
  event: EventType | null
  requestTime: number        // 请求时间戳
  estimatedWaitTime: number  // 预计等待时间 (ms)
  assignedOperator: string | null
}

/**
 * 运营商信息
 */
export interface Operator {
  id: string
  name: string
  location: Location
  capacity: number           // 最大并发接管数
  currentLoad: number        // 当前接管数
  activeVehicles: string[]   // 正在接管的车辆 ID
  averageLatency: number     // 平均延迟
}

/**
 * 调度配置
 */
export interface DispatchConfig {
  maxFleetSize: number           // 最大车队规模
  maxConcurrentHandovers: number // 最大并发接管数
  priorityWeights: {
    priority: number             // 优先级权重
    latency: number              // 延迟权重
    location: number             // 位置权重
    event: number                // 事件权重
  }
  latencyThresholds: {
    excellent: number            // 优秀 (< 50ms)
    good: number                 // 良好 (< 100ms)
    acceptable: number           // 可接受 (< 200ms)
    poor: number                 // 较差 (> 200ms)
  }
}

/**
 * 调度统计
 */
export interface DispatchStats {
  totalVehicles: number          // 总车辆数
  onlineVehicles: number         // 在线车辆数
  requestingVehicles: number     // 请求中车辆数
  connectedVehicles: number      // 已接入车辆数
  queueLength: number            // 队列长度
  averageWaitTime: number        // 平均等待时间 (ms)
  successRate: number            // 接管成功率 (%)
  totalHandovers: number         // 总接管次数
  operatorUtilization: number    // 运营商利用率 (%)
}

// ======================== 默认配置 ========================

const DEFAULT_CONFIG: DispatchConfig = {
  maxFleetSize: 100,
  maxConcurrentHandovers: 10,
  priorityWeights: {
    priority: 0.5,    // 50%
    latency: 0.2,     // 20%
    location: 0.2,    // 20%
    event: 0.1        // 10%
  },
  latencyThresholds: {
    excellent: 50,
    good: 100,
    acceptable: 200,
    poor: 300
  }
}

// ======================== 事件优先级映射 ========================

const EVENT_PRIORITY_MAP: Record<EventType, Priority> = {
  [EventType.COLLISION_WARNING]: Priority.EMERGENCY,
  [EventType.SYSTEM_ERROR]: Priority.EMERGENCY,
  [EventType.SENSOR_FAILURE]: Priority.EMERGENCY,
  [EventType.TRAFFIC_VIOLATION]: Priority.HIGH,
  [EventType.NETWORK_DEGRADATION]: Priority.HIGH,
  [EventType.ROUTE_BLOCKED]: Priority.MEDIUM,
  [EventType.PASSENGER_REQUEST]: Priority.LOW,
  [EventType.SCHEDULED_HANDOVER]: Priority.ROUTINE
}

// ======================== 调度服务 ========================

export function useVehicleDispatchService() {
  // 状态
  const config = ref<DispatchConfig>(DEFAULT_CONFIG)
  const vehicles = ref<Map<string, Vehicle>>(new Map())
  const operators = ref<Map<string, Operator>>(new Map())
  const requestQueue = ref<string[]>([]) // 车辆 ID 队列

  // 统计
  const stats = ref<DispatchStats>({
    totalVehicles: 0,
    onlineVehicles: 0,
    requestingVehicles: 0,
    connectedVehicles: 0,
    queueLength: 0,
    averageWaitTime: 0,
    successRate: 100,
    totalHandovers: 0,
    operatorUtilization: 0
  })

  // 计算属性
  const sortedQueue = computed(() => {
    return requestQueue.value
      .map(id => vehicles.value.get(id))
      .filter((v): v is Vehicle => v !== undefined)
      .sort((a, b) => calculatePriority(b) - calculatePriority(a))
  })

  const availableOperators = computed(() => {
    return Array.from(operators.value.values())
      .filter(op => op.currentLoad < op.capacity)
      .sort((a, b) => a.currentLoad - b.currentLoad)
  })

  // ======================== 核心调度算法 ========================

  /**
   * 计算车辆调度优先级分数
   * 分数越高，优先级越高
   */
  function calculatePriority(vehicle: Vehicle): number {
    const weights = config.value.priorityWeights

    // 1. 优先级分数 (0-1，值越小优先级越高，需要反转)
    const priorityScore = (4 - vehicle.priority) / 4

    // 2. 延迟分数 (0-1，延迟越低分数越高)
    const latency = vehicle.network.latency
    let latencyScore = 0
    if (latency < config.value.latencyThresholds.excellent) {
      latencyScore = 1.0
    } else if (latency < config.value.latencyThresholds.good) {
      latencyScore = 0.8
    } else if (latency < config.value.latencyThresholds.acceptable) {
      latencyScore = 0.5
    } else {
      latencyScore = 0.2
    }

    // 3. 位置分数 (0-1，基于最近运营商的距离)
    const locationScore = calculateLocationScore(vehicle)

    // 4. 事件分数 (0-1，紧急事件分数更高)
    let eventScore = 0
    if (vehicle.event) {
      const eventPriority = EVENT_PRIORITY_MAP[vehicle.event]
      eventScore = (4 - eventPriority) / 4
    }

    // 加权求和
    const totalScore =
      priorityScore * weights.priority +
      latencyScore * weights.latency +
      locationScore * weights.location +
      eventScore * weights.event

    return totalScore
  }

  /**
   * 计算位置分数
   */
  function calculateLocationScore(vehicle: Vehicle): number {
    const nearestOperator = findNearestOperator(vehicle.location)
    if (!nearestOperator) return 0.5

    // 简化的距离计算（实际应使用 Haversine 公式）
    const distance = Math.sqrt(
      Math.pow(vehicle.location.latitude - nearestOperator.location.latitude, 2) +
      Math.pow(vehicle.location.longitude - nearestOperator.location.longitude, 2)
    )

    // 距离越近分数越高
    return Math.max(0, 1 - distance / 10)
  }

  /**
   * 查找最近的运营商
   */
  function findNearestOperator(location: Location): Operator | null {
    let nearest: Operator | null = null
    let minDistance = Infinity

    for (const operator of operators.value.values()) {
      const distance = Math.sqrt(
        Math.pow(location.latitude - operator.location.latitude, 2) +
        Math.pow(location.longitude - operator.location.longitude, 2)
      )

      if (distance < minDistance) {
        minDistance = distance
        nearest = operator
      }
    }

    return nearest
  }

  /**
   * 为车辆分配运营商
   */
  function assignOperator(vehicleId: string): Operator | null {
    const vehicle = vehicles.value.get(vehicleId)
    if (!vehicle) return null

    // 查找可用运营商
    const available = availableOperators.value

    if (available.length === 0) return null

    // 选择负载最低且延迟可接受的运营商
    const bestOperator = available.reduce((best, current) => {
      // 考虑延迟和负载
      const currentScore =
        (1 - current.currentLoad / current.capacity) * 0.6 +
        (1 - current.averageLatency / 200) * 0.4

      const bestScore =
        (1 - best.currentLoad / best.capacity) * 0.6 +
        (1 - best.averageLatency / 200) * 0.4

      return currentScore > bestScore ? current : best
    })

    return bestOperator
  }

  // ======================== 车辆管理 ========================

  /**
   * 添加车辆
   */
  function addVehicle(vehicle: Vehicle) {
    vehicles.value.set(vehicle.id, vehicle)
    updateStats()
    console.log(`[Dispatch] Vehicle added: ${vehicle.id}`)
  }

  /**
   * 移除车辆
   */
  function removeVehicle(vehicleId: string) {
    vehicles.value.delete(vehicleId)
    requestQueue.value = requestQueue.value.filter(id => id !== vehicleId)
    updateStats()
    console.log(`[Dispatch] Vehicle removed: ${vehicleId}`)
  }

  /**
   * 更新车辆状态
   */
  function updateVehicleStatus(vehicleId: string, status: VehicleStatus) {
    const vehicle = vehicles.value.get(vehicleId)
    if (vehicle) {
      vehicle.status = status
      updateStats()
    }
  }

  /**
   * 请求接管
   */
  function requestHandover(
    vehicleId: string,
    priority: Priority = Priority.MEDIUM,
    event: EventType | null = null
  ): boolean {
    const vehicle = vehicles.value.get(vehicleId)
    if (!vehicle) return false

    // 更新车辆信息
    vehicle.status = VehicleStatus.REQUESTING
    vehicle.priority = priority
    vehicle.event = event
    vehicle.requestTime = Date.now()

    // 如果是紧急事件，立即尝试分配
    if (priority === Priority.EMERGENCY) {
      return processHandover(vehicleId)
    }

    // 否则加入队列
    if (!requestQueue.value.includes(vehicleId)) {
      requestQueue.value.push(vehicleId)
    }

    updateStats()
    console.log(`[Dispatch] Handover requested for ${vehicleId}, priority: ${priority}`)

    // 尝试处理队列
    processQueue()

    return true
  }

  /**
   * 处理接管请求
   */
  function processHandover(vehicleId: string): boolean {
    const vehicle = vehicles.value.get(vehicleId)
    if (!vehicle) return false

    // 分配运营商
    const operator = assignOperator(vehicleId)
    if (!operator) {
      console.warn(`[Dispatch] No operator available for ${vehicleId}`)
      return false
    }

    // 更新车辆状态
    vehicle.status = VehicleStatus.CONNECTED
    vehicle.assignedOperator = operator.id

    // 更新运营商负载
    operator.currentLoad++
    operator.activeVehicles.push(vehicleId)

    // 从队列移除
    requestQueue.value = requestQueue.value.filter(id => id !== vehicleId)

    // 更新统计
    stats.value.totalHandovers++
    updateStats()

    console.log(`[Dispatch] Handover successful: ${vehicleId} → ${operator.name}`)

    return true
  }

  /**
   * 处理队列
   */
  function processQueue() {
    // 按优先级排序
    const sorted = sortedQueue.value

    // 尝试处理队列中的车辆
    for (const vehicle of sorted) {
      if (vehicle && availableOperators.value.length > 0) {
        processHandover(vehicle.id)
      } else {
        break // 没有可用运营商，停止处理
      }
    }
  }

  /**
   * 释放接管
   */
  function releaseHandover(vehicleId: string) {
    const vehicle = vehicles.value.get(vehicleId)
    if (!vehicle || !vehicle.assignedOperator) return

    const operator = operators.value.get(vehicle.assignedOperator)
    if (operator) {
      operator.currentLoad--
      operator.activeVehicles = operator.activeVehicles.filter(id => id !== vehicleId)
    }

    vehicle.status = VehicleStatus.ONLINE
    vehicle.assignedOperator = null

    updateStats()

    // 尝试处理队列中的下一个
    processQueue()

    console.log(`[Dispatch] Handover released for ${vehicleId}`)
  }

  // ======================== 运营商管理 ========================

  /**
   * 添加运营商
   */
  function addOperator(operator: Operator) {
    operators.value.set(operator.id, operator)
    console.log(`[Dispatch] Operator added: ${operator.name}`)
  }

  /**
   * 移除运营商
   */
  function removeOperator(operatorId: string) {
    const operator = operators.value.get(operatorId)
    if (operator) {
      // 释放所有正在接管的车辆
      for (const vehicleId of operator.activeVehicles) {
        releaseHandover(vehicleId)
      }
      operators.value.delete(operatorId)
      console.log(`[Dispatch] Operator removed: ${operator.name}`)
    }
  }

  // ======================== 统计更新 ========================

  function updateStats() {
    const vehicleList = Array.from(vehicles.value.values())

    stats.value.totalVehicles = vehicleList.length
    stats.value.onlineVehicles = vehicleList.filter(v => v.status === VehicleStatus.ONLINE).length
    stats.value.requestingVehicles = vehicleList.filter(v => v.status === VehicleStatus.REQUESTING).length
    stats.value.connectedVehicles = vehicleList.filter(v => v.status === VehicleStatus.CONNECTED).length
    stats.value.queueLength = requestQueue.value.length

    // 平均等待时间
    const now = Date.now()
    const waitTimes = requestQueue.value
      .map(id => vehicles.value.get(id))
      .filter(v => v !== undefined)
      .map(v => now - v!.requestTime)

    stats.value.averageWaitTime = waitTimes.length > 0
      ? waitTimes.reduce((sum, t) => sum + t, 0) / waitTimes.length
      : 0

    // 运营商利用率
    const totalCapacity = Array.from(operators.value.values())
      .reduce((sum, op) => sum + op.capacity, 0)
    const totalLoad = Array.from(operators.value.values())
      .reduce((sum, op) => sum + op.currentLoad, 0)

    stats.value.operatorUtilization = totalCapacity > 0
      ? (totalLoad / totalCapacity) * 100
      : 0
  }

  // ======================== 模拟数据生成 ========================

  /**
   * 生成模拟车队
   */
  function generateMockFleet(count: number = 100) {
    const cities = ['Hong Kong', 'Shenzhen', 'Guangzhou', 'Shanghai', 'Beijing']
    const regions = ['Central', 'Eastern', 'Western', 'Northern', 'Southern']

    for (let i = 0; i < count; i++) {
      const vehicle: Vehicle = {
        id: `vehicle_${i.toString().padStart(3, '0')}`,
        name: `AV-${i.toString().padStart(3, '0')}`,
        status: VehicleStatus.ONLINE,
        location: {
          latitude: 22.3 + Math.random() * 0.5,
          longitude: 114.1 + Math.random() * 0.5,
          city: cities[Math.floor(Math.random() * cities.length)],
          region: regions[Math.floor(Math.random() * regions.length)]
        },
        network: {
          latency: 50 + Math.random() * 100,
          bandwidth: 10 + Math.random() * 20,
          packetLoss: Math.random() * 2,
          jitter: Math.random() * 20
        },
        priority: Priority.ROUTINE,
        event: null,
        requestTime: 0,
        estimatedWaitTime: 0,
        assignedOperator: null
      }

      addVehicle(vehicle)
    }

    console.log(`[Dispatch] Generated ${count} mock vehicles`)
  }

  /**
   * 生成模拟运营商
   */
  function generateMockOperators(count: number = 5) {
    const cities = ['Hong Kong', 'Shenzhen', 'Guangzhou', 'Shanghai', 'Beijing']

    for (let i = 0; i < count; i++) {
      const operator: Operator = {
        id: `operator_${i}`,
        name: `Operator ${i + 1}`,
        location: {
          latitude: 22.3 + Math.random() * 0.5,
          longitude: 114.1 + Math.random() * 0.5,
          city: cities[i % cities.length],
          region: 'Central'
        },
        capacity: 10,
        currentLoad: 0,
        activeVehicles: [],
        averageLatency: 60 + Math.random() * 40
      }

      addOperator(operator)
    }

    console.log(`[Dispatch] Generated ${count} mock operators`)
  }

  return {
    // 只读状态
    config: readonly(config),
    vehicles: readonly(vehicles),
    operators: readonly(operators),
    stats: readonly(stats),
    sortedQueue: readonly(sortedQueue),
    availableOperators: readonly(availableOperators),

    // 车辆管理
    addVehicle,
    removeVehicle,
    updateVehicleStatus,
    requestHandover,
    releaseHandover,

    // 运营商管理
    addOperator,
    removeOperator,

    // 工具方法
    calculatePriority,
    assignOperator,

    // 模拟数据
    generateMockFleet,
    generateMockOperators
  }
}

// 创建全局单例
let dispatchServiceInstance: ReturnType<typeof useVehicleDispatchService> | null = null

export function getDispatchService() {
  if (!dispatchServiceInstance) {
    dispatchServiceInstance = useVehicleDispatchService()
  }
  return dispatchServiceInstance
}

export default useVehicleDispatchService
