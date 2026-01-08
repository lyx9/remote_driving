import { defineStore } from 'pinia'
import type { Vehicle } from '@/types'

// 扩展车辆类型以支持远程控制功能
export interface ExtendedVehicle extends Vehicle {
  // 远程控制相关
  connectionState: 'disconnected' | 'connecting' | 'connected' | 'reconnecting' | 'failed'
  latency_ms: number
  battery_level: number
  emergency_level: number
  priority_score: number

  // 遥测数据
  telemetry: {
    cpu_usage: number
    gpu_usage: number
    memory_usage: number
    network_quality: 'excellent' | 'good' | 'fair' | 'poor'
    signal_strength: number
  }

  // 传感器状态
  sensors: {
    cameras_online: number
    cameras_total: number
    lidar_online: boolean
    gps_fix: boolean
  }
}

export const useFleetStore = defineStore('fleet', {
  state: () => ({
    vehicles: [
      {
        id: 'FSM-01',
        type: 'ROBO-TAXI' as const,
        status: 'ACTIVE' as const,
        money: 1245.50,
        mode: 'ECO' as const,
        location: [31.2304, 121.4737] as [number, number],
        path: [[31.225, 121.470], [31.228, 121.472], [31.2304, 121.4737]] as Array<[number, number]>,
        speed: 0,
        gear: 'D' as const,
        steering: 0,
        // 扩展属性
        connectionState: 'disconnected' as const,
        latency_ms: 0,
        battery_level: 85,
        emergency_level: 0,
        priority_score: 60,
        telemetry: {
          cpu_usage: 35,
          gpu_usage: 45,
          memory_usage: 60,
          network_quality: 'excellent' as const,
          signal_strength: 95
        },
        sensors: {
          cameras_online: 5,
          cameras_total: 5,
          lidar_online: true,
          gps_fix: true
        }
      },
      {
        id: 'FSM-02',
        type: 'LOGISTICS' as const,
        status: 'IDLE' as const,
        money: 450.20,
        mode: 'SPORT' as const,
        location: [31.235, 121.480] as [number, number],
        path: [[31.240, 121.485], [31.238, 121.482], [31.235, 121.480]] as Array<[number, number]>,
        speed: 0,
        gear: 'P' as const,
        steering: 0,
        connectionState: 'disconnected' as const,
        latency_ms: 0,
        battery_level: 45,
        emergency_level: 0,
        priority_score: 40,
        telemetry: {
          cpu_usage: 20,
          gpu_usage: 15,
          memory_usage: 40,
          network_quality: 'good' as const,
          signal_strength: 80
        },
        sensors: {
          cameras_online: 5,
          cameras_total: 5,
          lidar_online: true,
          gps_fix: true
        }
      },
      {
        id: 'FSM-03',
        type: 'SECURITY' as const,
        status: 'PATROL' as const,
        money: 0.00,
        mode: 'AI-HUNT' as const,
        location: [31.220, 121.460] as [number, number],
        path: [[31.215, 121.455], [31.218, 121.458], [31.220, 121.460]] as Array<[number, number]>,
        speed: 0,
        gear: 'D' as const,
        steering: 0,
        connectionState: 'disconnected' as const,
        latency_ms: 0,
        battery_level: 92,
        emergency_level: 3,
        priority_score: 85,
        telemetry: {
          cpu_usage: 55,
          gpu_usage: 60,
          memory_usage: 70,
          network_quality: 'fair' as const,
          signal_strength: 65
        },
        sensors: {
          cameras_online: 4,
          cameras_total: 5,
          lidar_online: true,
          gps_fix: true
        }
      }
    ] as ExtendedVehicle[],
    currentVehicleIndex: 0,

    // 调度相关
    schedulingEnabled: true,
    schedulingAlgorithm: 'weighted_priority' as 'weighted_priority' | 'emergency_first' | 'latency_based'
  }),

  getters: {
    currentVehicle: (state): ExtendedVehicle => state.vehicles[state.currentVehicleIndex],
    activeVehicles: (state): ExtendedVehicle[] => state.vehicles.filter(v => v.status === 'ACTIVE'),
    connectedVehicles: (state): ExtendedVehicle[] => state.vehicles.filter(v => v.connectionState === 'connected'),

    // 按优先级排序的调度队列
    schedulingQueue: (state): ExtendedVehicle[] => {
      return [...state.vehicles].sort((a, b) => b.priority_score - a.priority_score)
    },

    // 获取当前车辆的网络质量颜色
    currentLatencyColor: (state): string => {
      const latency = state.vehicles[state.currentVehicleIndex].latency_ms
      if (latency < 100) return '#00ff88'
      if (latency < 200) return '#ffaa00'
      return '#ff4444'
    }
  },

  actions: {
    selectVehicle(index: number) {
      if (index >= 0 && index < this.vehicles.length) {
        this.currentVehicleIndex = index
      }
    },

    selectVehicleById(id: string) {
      const index = this.vehicles.findIndex(v => v.id === id)
      if (index >= 0) {
        this.currentVehicleIndex = index
      }
    },

    updateVehicleSpeed(speed: number) {
      if (this.currentVehicle) {
        this.currentVehicle.speed = speed
      }
    },

    updateVehicleGear(gear: 'P' | 'R' | 'N' | 'D') {
      if (this.currentVehicle) {
        this.currentVehicle.gear = gear
      }
    },

    updateVehicleSteering(angle: number) {
      if (this.currentVehicle) {
        this.currentVehicle.steering = angle
      }
    },

    addProfit(amount: number) {
      if (this.currentVehicle) {
        this.currentVehicle.money += amount
      }
    },

    // 更新车辆连接状态
    updateConnectionState(vehicleId: string, state: ExtendedVehicle['connectionState']) {
      const vehicle = this.vehicles.find(v => v.id === vehicleId)
      if (vehicle) {
        vehicle.connectionState = state
      }
    },

    // 更新车辆延迟
    updateLatency(vehicleId: string, latency_ms: number) {
      const vehicle = this.vehicles.find(v => v.id === vehicleId)
      if (vehicle) {
        vehicle.latency_ms = latency_ms
        // 根据延迟更新网络质量
        if (latency_ms < 50) {
          vehicle.telemetry.network_quality = 'excellent'
        } else if (latency_ms < 100) {
          vehicle.telemetry.network_quality = 'good'
        } else if (latency_ms < 200) {
          vehicle.telemetry.network_quality = 'fair'
        } else {
          vehicle.telemetry.network_quality = 'poor'
        }
      }
    },

    // 更新车辆遥测数据
    updateTelemetry(vehicleId: string, telemetry: Partial<ExtendedVehicle['telemetry']>) {
      const vehicle = this.vehicles.find(v => v.id === vehicleId)
      if (vehicle) {
        Object.assign(vehicle.telemetry, telemetry)
      }
    },

    // 更新车辆完整状态
    updateVehicleState(vehicleId: string, state: Partial<ExtendedVehicle>) {
      const vehicle = this.vehicles.find(v => v.id === vehicleId)
      if (vehicle) {
        Object.assign(vehicle, state)
      }
    },

    // 更新优先级分数
    updatePriorityScore(vehicleId: string, score: number) {
      const vehicle = this.vehicles.find(v => v.id === vehicleId)
      if (vehicle) {
        vehicle.priority_score = score
      }
    },

    // 设置紧急级别
    setEmergencyLevel(vehicleId: string, level: number) {
      const vehicle = this.vehicles.find(v => v.id === vehicleId)
      if (vehicle) {
        vehicle.emergency_level = Math.max(0, Math.min(4, level))
        // 紧急级别会影响优先级
        if (level >= 3) {
          vehicle.priority_score = Math.max(vehicle.priority_score, 90)
        }
      }
    },

    // 添加新车辆
    addVehicle(vehicle: ExtendedVehicle) {
      const existing = this.vehicles.find(v => v.id === vehicle.id)
      if (!existing) {
        this.vehicles.push(vehicle)
      }
    },

    // 移除车辆
    removeVehicle(vehicleId: string) {
      const index = this.vehicles.findIndex(v => v.id === vehicleId)
      if (index >= 0) {
        this.vehicles.splice(index, 1)
        if (this.currentVehicleIndex >= this.vehicles.length) {
          this.currentVehicleIndex = Math.max(0, this.vehicles.length - 1)
        }
      }
    },

    // 设置调度算法
    setSchedulingAlgorithm(algorithm: 'weighted_priority' | 'emergency_first' | 'latency_based') {
      this.schedulingAlgorithm = algorithm
    },

    // 切换调度开关
    toggleScheduling() {
      this.schedulingEnabled = !this.schedulingEnabled
    }
  }
})
