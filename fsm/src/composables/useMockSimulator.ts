/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      Mock Simulator
 * @description Mock模拟器服务，模拟车辆、道路、障碍物，用于演示和测试
 */

import { ref, reactive, computed } from 'vue'

// ======================== 类型定义 ========================

export interface Vector2 {
  x: number
  y: number
}

export interface Vector3 extends Vector2 {
  z: number
}

export interface MockVehicle {
  id: string
  name: string
  position: Vector3            // 世界坐标 (米)
  rotation: number             // 航向角 (弧度)
  velocity: Vector3            // 速度 (m/s)
  speed: number               // 标量速度 (m/s)
  steering: number            // 转向角 [-1, 1]
  throttle: number            // 油门 [0, 1]
  brake: number               // 刹车 [0, 1]
  gear: 'P' | 'R' | 'N' | 'D'
  turnSignal: -1 | 0 | 1
  hazard: boolean
  emergency: boolean

  // 物理参数
  wheelbase: number           // 轴距 (米)
  maxSpeed: number            // 最大速度 (m/s)
  maxSteeringAngle: number    // 最大转向角 (弧度)
  acceleration: number        // 加速度 (m/s²)
  deceleration: number        // 减速度 (m/s²)

  // 传感器状态
  sensors: {
    cameras: number           // 摄像头数量
    lidar: boolean
    gps: boolean
  }

  // 系统状态
  battery: number             // 电量 %
  cpuUsage: number
  memoryUsage: number
}

export interface RoadSegment {
  id: string
  type: 'straight' | 'curve' | 'intersection' | 'roundabout'
  startPoint: Vector2
  endPoint: Vector2
  width: number               // 道路宽度
  lanes: number               // 车道数
  speedLimit: number          // 限速 (m/s)
  controlPoints?: Vector2[]   // 贝塞尔曲线控制点
}

export interface Obstacle {
  id: string
  type: 'vehicle' | 'pedestrian' | 'cyclist' | 'static' | 'cone' | 'barrier'
  position: Vector3
  rotation: number
  dimensions: Vector3         // 尺寸 (长宽高)
  velocity?: Vector3          // 动态障碍物速度

  // 动态属性
  path?: Vector2[]            // 移动路径
  pathIndex?: number          // 当前路径点
  speed?: number              // 移动速度
}

export interface MockScenario {
  id: string
  name: string
  description: string
  vehicles: MockVehicle[]
  roads: RoadSegment[]
  obstacles: Obstacle[]
  spawnPoint: Vector3         // 车辆出生点
  targetPoint?: Vector3       // 目标点
}

// ======================== 物理仿真 ========================

class VehiclePhysics {
  // Ackermann 转向模型
  static updateVehicle(vehicle: MockVehicle, dt: number): void {
    // 获取输入
    const throttleInput = vehicle.throttle
    const brakeInput = vehicle.brake
    const steeringInput = vehicle.steering

    // 计算加速度
    let accel = 0
    if (brakeInput > 0) {
      accel = -vehicle.deceleration * brakeInput
    } else if (throttleInput > 0) {
      // 档位影响
      if (vehicle.gear === 'D') {
        accel = vehicle.acceleration * throttleInput
      } else if (vehicle.gear === 'R') {
        accel = -vehicle.acceleration * throttleInput * 0.5 // 倒车减速
      } else {
        accel = 0 // P/N档不动
      }
    } else {
      // 阻力减速
      accel = -vehicle.speed * 0.5
    }

    // 紧急停车
    if (vehicle.emergency) {
      accel = -vehicle.deceleration * 2
    }

    // 更新速度
    vehicle.speed = Math.max(0, Math.min(vehicle.maxSpeed, vehicle.speed + accel * dt))

    // 阿克曼转向
    const steeringAngle = steeringInput * vehicle.maxSteeringAngle

    if (Math.abs(steeringAngle) > 0.001 && vehicle.speed > 0.1) {
      // 转弯半径
      const turningRadius = vehicle.wheelbase / Math.tan(Math.abs(steeringAngle))
      // 角速度
      const angularVelocity = vehicle.speed / turningRadius
      // 更新航向
      vehicle.rotation += Math.sign(steeringAngle) * angularVelocity * dt
    }

    // 归一化航向角 [-π, π]
    while (vehicle.rotation > Math.PI) vehicle.rotation -= 2 * Math.PI
    while (vehicle.rotation < -Math.PI) vehicle.rotation += 2 * Math.PI

    // 更新位置
    const direction = vehicle.gear === 'R' ? -1 : 1
    vehicle.position.x += Math.cos(vehicle.rotation) * vehicle.speed * dt * direction
    vehicle.position.y += Math.sin(vehicle.rotation) * vehicle.speed * dt * direction

    // 更新速度向量
    vehicle.velocity.x = Math.cos(vehicle.rotation) * vehicle.speed * direction
    vehicle.velocity.y = Math.sin(vehicle.rotation) * vehicle.speed * direction
    vehicle.velocity.z = 0
  }

  // 更新动态障碍物
  static updateObstacle(obstacle: Obstacle, dt: number): void {
    if (!obstacle.path || obstacle.path.length < 2 || !obstacle.speed) return

    const currentTarget = obstacle.path[obstacle.pathIndex || 0]
    const dx = currentTarget.x - obstacle.position.x
    const dy = currentTarget.y - obstacle.position.y
    const distance = Math.sqrt(dx * dx + dy * dy)

    if (distance < 1.0) {
      // 到达当前点，移动到下一个
      obstacle.pathIndex = ((obstacle.pathIndex || 0) + 1) % obstacle.path.length
    } else {
      // 向目标点移动
      const speed = obstacle.speed || 5
      obstacle.position.x += (dx / distance) * speed * dt
      obstacle.position.y += (dy / distance) * speed * dt
      obstacle.rotation = Math.atan2(dy, dx)

      if (obstacle.velocity) {
        obstacle.velocity.x = (dx / distance) * speed
        obstacle.velocity.y = (dy / distance) * speed
      }
    }
  }
}

// ======================== 预置场景 ========================

const PRESET_SCENARIOS: MockScenario[] = [
  {
    id: 'highway',
    name: '高速公路',
    description: '双向四车道高速公路场景',
    vehicles: [],
    roads: [
      {
        id: 'road_main',
        type: 'straight',
        startPoint: { x: -500, y: 0 },
        endPoint: { x: 500, y: 0 },
        width: 14,
        lanes: 4,
        speedLimit: 33.3, // 120 km/h
      }
    ],
    obstacles: [
      {
        id: 'obs_1',
        type: 'vehicle',
        position: { x: 50, y: 3.5, z: 0 },
        rotation: 0,
        dimensions: { x: 4.5, y: 1.8, z: 1.5 },
        path: [{ x: 50, y: 3.5 }, { x: 200, y: 3.5 }],
        pathIndex: 0,
        speed: 25,
      },
      {
        id: 'obs_2',
        type: 'vehicle',
        position: { x: 100, y: -3.5, z: 0 },
        rotation: Math.PI,
        dimensions: { x: 4.5, y: 1.8, z: 1.5 },
        path: [{ x: 100, y: -3.5 }, { x: -100, y: -3.5 }],
        pathIndex: 0,
        speed: 28,
      },
    ],
    spawnPoint: { x: 0, y: 3.5, z: 0 },
  },
  {
    id: 'city_intersection',
    name: '城市十字路口',
    description: '带信号灯的十字路口',
    vehicles: [],
    roads: [
      {
        id: 'road_ns',
        type: 'straight',
        startPoint: { x: 0, y: -100 },
        endPoint: { x: 0, y: 100 },
        width: 10,
        lanes: 2,
        speedLimit: 13.9, // 50 km/h
      },
      {
        id: 'road_ew',
        type: 'straight',
        startPoint: { x: -100, y: 0 },
        endPoint: { x: 100, y: 0 },
        width: 10,
        lanes: 2,
        speedLimit: 13.9,
      },
    ],
    obstacles: [
      {
        id: 'ped_1',
        type: 'pedestrian',
        position: { x: -20, y: 8, z: 0 },
        rotation: Math.PI / 2,
        dimensions: { x: 0.5, y: 0.5, z: 1.7 },
        path: [{ x: -20, y: 8 }, { x: -20, y: -8 }],
        pathIndex: 0,
        speed: 1.4, // 步行速度
      },
      {
        id: 'cyclist_1',
        type: 'cyclist',
        position: { x: 30, y: 2, z: 0 },
        rotation: Math.PI,
        dimensions: { x: 1.8, y: 0.6, z: 1.5 },
        path: [{ x: 30, y: 2 }, { x: -50, y: 2 }],
        pathIndex: 0,
        speed: 5,
      },
    ],
    spawnPoint: { x: 0, y: -50, z: 0 },
    targetPoint: { x: 0, y: 50, z: 0 },
  },
  {
    id: 'parking_lot',
    name: '停车场',
    description: '室外停车场场景',
    vehicles: [],
    roads: [
      {
        id: 'parking_lane',
        type: 'straight',
        startPoint: { x: -50, y: 0 },
        endPoint: { x: 50, y: 0 },
        width: 6,
        lanes: 1,
        speedLimit: 5.6, // 20 km/h
      },
    ],
    obstacles: [
      // 停放的车辆
      { id: 'parked_1', type: 'vehicle', position: { x: -30, y: 8, z: 0 }, rotation: Math.PI / 2, dimensions: { x: 4.5, y: 1.8, z: 1.5 } },
      { id: 'parked_2', type: 'vehicle', position: { x: -20, y: 8, z: 0 }, rotation: Math.PI / 2, dimensions: { x: 4.5, y: 1.8, z: 1.5 } },
      { id: 'parked_3', type: 'vehicle', position: { x: 0, y: 8, z: 0 }, rotation: Math.PI / 2, dimensions: { x: 4.5, y: 1.8, z: 1.5 } },
      { id: 'parked_4', type: 'vehicle', position: { x: 20, y: 8, z: 0 }, rotation: Math.PI / 2, dimensions: { x: 4.5, y: 1.8, z: 1.5 } },
      // 锥桶
      { id: 'cone_1', type: 'cone', position: { x: 10, y: 0, z: 0 }, rotation: 0, dimensions: { x: 0.3, y: 0.3, z: 0.5 } },
      { id: 'cone_2', type: 'cone', position: { x: 12, y: 0, z: 0 }, rotation: 0, dimensions: { x: 0.3, y: 0.3, z: 0.5 } },
    ],
    spawnPoint: { x: -40, y: 0, z: 0 },
    targetPoint: { x: 10, y: 8, z: 0 },
  },
  {
    id: 'emergency_test',
    name: '紧急避障测试',
    description: '突然出现障碍物的紧急情况',
    vehicles: [],
    roads: [
      {
        id: 'road_main',
        type: 'straight',
        startPoint: { x: -100, y: 0 },
        endPoint: { x: 100, y: 0 },
        width: 7,
        lanes: 2,
        speedLimit: 16.7, // 60 km/h
      },
    ],
    obstacles: [
      // 静态障碍物 (模拟突然停下的车)
      {
        id: 'stopped_car',
        type: 'vehicle',
        position: { x: 50, y: 1.75, z: 0 },
        rotation: 0,
        dimensions: { x: 4.5, y: 1.8, z: 1.5 },
      },
      // 路边障碍
      { id: 'barrier_1', type: 'barrier', position: { x: 45, y: 4.5, z: 0 }, rotation: 0, dimensions: { x: 2, y: 0.5, z: 1 } },
    ],
    spawnPoint: { x: -50, y: 1.75, z: 0 },
  },
]

// ======================== Mock 模拟器 ========================

export function useMockSimulator() {
  // 当前场景
  const currentScenario = ref<MockScenario | null>(null)

  // 主控车辆
  const mainVehicle = ref<MockVehicle | null>(null)

  // 所有障碍物
  const obstacles = ref<Obstacle[]>([])

  // 仿真状态
  const isRunning = ref(false)
  const simulationTime = ref(0)
  const fps = ref(0)

  // 仿真配置
  const config = reactive({
    timeScale: 1.0,           // 时间缩放
    physicsFps: 60,           // 物理仿真帧率
    enableCollision: true,    // 启用碰撞检测
    enableDynamicObstacles: true, // 启用动态障碍物
  })

  // 统计
  const stats = reactive({
    distance: 0,              // 行驶距离
    maxSpeed: 0,              // 最大速度
    collisions: 0,            // 碰撞次数
    nearMisses: 0,            // 险情次数
  })

  let animationId: number | null = null
  let lastTime = 0

  // 创建默认车辆
  const createDefaultVehicle = (id: string, position: Vector3): MockVehicle => ({
    id,
    name: `FSM-${id.slice(-2)}`,
    position: { ...position },
    rotation: 0,
    velocity: { x: 0, y: 0, z: 0 },
    speed: 0,
    steering: 0,
    throttle: 0,
    brake: 0,
    gear: 'P',
    turnSignal: 0,
    hazard: false,
    emergency: false,
    wheelbase: 2.8,
    maxSpeed: 33.3,           // 120 km/h
    maxSteeringAngle: Math.PI / 6, // 30度
    acceleration: 3.0,
    deceleration: 5.0,
    sensors: {
      cameras: 5,
      lidar: true,
      gps: true,
    },
    battery: 85,
    cpuUsage: 35,
    memoryUsage: 45,
  })

  // 加载场景
  const loadScenario = (scenarioId: string) => {
    const scenario = PRESET_SCENARIOS.find(s => s.id === scenarioId)
    if (!scenario) {
      console.error(`Scenario ${scenarioId} not found`)
      return false
    }

    currentScenario.value = { ...scenario }
    obstacles.value = scenario.obstacles.map(o => ({ ...o, position: { ...o.position } }))

    // 创建主控车辆
    mainVehicle.value = createDefaultVehicle('main-01', scenario.spawnPoint)

    // 重置统计
    stats.distance = 0
    stats.maxSpeed = 0
    stats.collisions = 0
    stats.nearMisses = 0
    simulationTime.value = 0

    return true
  }

  // 应用控制指令
  const applyControl = (command: {
    throttle: number
    brake: number
    steering: number
    gear: 'P' | 'R' | 'N' | 'D'
    turnSignal: -1 | 0 | 1
    hazard: boolean
    emergency: boolean
  }) => {
    if (!mainVehicle.value) return

    mainVehicle.value.throttle = command.throttle
    mainVehicle.value.brake = command.brake
    mainVehicle.value.steering = command.steering
    mainVehicle.value.gear = command.gear
    mainVehicle.value.turnSignal = command.turnSignal
    mainVehicle.value.hazard = command.hazard
    mainVehicle.value.emergency = command.emergency
  }

  // 碰撞检测 (简化的AABB)
  const checkCollision = (vehicle: MockVehicle, obstacle: Obstacle): boolean => {
    const vHalfW = 2.0 // 车辆半宽
    const vHalfL = 2.5 // 车辆半长
    const oHalfW = obstacle.dimensions.y / 2
    const oHalfL = obstacle.dimensions.x / 2

    const dx = Math.abs(vehicle.position.x - obstacle.position.x)
    const dy = Math.abs(vehicle.position.y - obstacle.position.y)

    return dx < (vHalfL + oHalfL) && dy < (vHalfW + oHalfW)
  }

  // 仿真循环
  const simulationLoop = (timestamp: number) => {
    if (!isRunning.value) return

    // 计算 delta time
    const dt = Math.min((timestamp - lastTime) / 1000, 0.1) * config.timeScale
    lastTime = timestamp

    // 计算帧率
    fps.value = Math.round(1 / (dt / config.timeScale))

    // 更新主车辆
    if (mainVehicle.value) {
      const prevPos = { ...mainVehicle.value.position }

      VehiclePhysics.updateVehicle(mainVehicle.value, dt)

      // 更新统计
      const moved = Math.sqrt(
        Math.pow(mainVehicle.value.position.x - prevPos.x, 2) +
        Math.pow(mainVehicle.value.position.y - prevPos.y, 2)
      )
      stats.distance += moved
      stats.maxSpeed = Math.max(stats.maxSpeed, mainVehicle.value.speed)

      // 碰撞检测
      if (config.enableCollision) {
        for (const obs of obstacles.value) {
          if (checkCollision(mainVehicle.value, obs)) {
            stats.collisions++
            // 停车
            mainVehicle.value.speed = 0
            mainVehicle.value.emergency = true
          }
        }
      }
    }

    // 更新动态障碍物
    if (config.enableDynamicObstacles) {
      for (const obs of obstacles.value) {
        if (obs.path && obs.path.length > 0) {
          VehiclePhysics.updateObstacle(obs, dt)
        }
      }
    }

    // 更新仿真时间
    simulationTime.value += dt

    // 继续循环
    animationId = requestAnimationFrame(simulationLoop)
  }

  // 启动仿真
  const start = () => {
    if (isRunning.value) return

    isRunning.value = true
    lastTime = performance.now()
    animationId = requestAnimationFrame(simulationLoop)
  }

  // 暂停仿真
  const pause = () => {
    isRunning.value = false
    if (animationId) {
      cancelAnimationFrame(animationId)
      animationId = null
    }
  }

  // 重置仿真
  const reset = () => {
    pause()

    if (currentScenario.value) {
      loadScenario(currentScenario.value.id)
    }
  }

  // 添加障碍物
  const addObstacle = (obstacle: Obstacle) => {
    obstacles.value.push(obstacle)
  }

  // 移除障碍物
  const removeObstacle = (id: string) => {
    const index = obstacles.value.findIndex(o => o.id === id)
    if (index >= 0) {
      obstacles.value.splice(index, 1)
    }
  }

  // 获取车辆状态 (用于前端显示)
  const getVehicleStatus = computed(() => {
    if (!mainVehicle.value) return null

    return {
      id: mainVehicle.value.id,
      name: mainVehicle.value.name,
      speed: mainVehicle.value.speed * 3.6, // m/s -> km/h
      steering: mainVehicle.value.steering * 100, // [-1,1] -> [-100,100]
      gear: mainVehicle.value.gear,
      throttle: mainVehicle.value.throttle * 100,
      brake: mainVehicle.value.brake * 100,
      position: mainVehicle.value.position,
      heading: mainVehicle.value.rotation * 180 / Math.PI,
      battery: mainVehicle.value.battery,
      emergency: mainVehicle.value.emergency,
      turnSignal: mainVehicle.value.turnSignal,
      hazard: mainVehicle.value.hazard,
    }
  })

  // 获取可用场景列表
  const availableScenarios = PRESET_SCENARIOS.map(s => ({
    id: s.id,
    name: s.name,
    description: s.description,
  }))

  return {
    // 状态
    currentScenario,
    mainVehicle,
    obstacles,
    isRunning,
    simulationTime,
    fps,
    config,
    stats,

    // 计算属性
    vehicleStatus: getVehicleStatus,
    availableScenarios,

    // 方法
    loadScenario,
    applyControl,
    start,
    pause,
    reset,
    addObstacle,
    removeObstacle,
  }
}
