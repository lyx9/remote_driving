/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 */

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
      // Hong Kong Fleet - Active Vehicles (AI Dispatch Demo)
      {
        id: 'HK-001',
        type: 'ROBO-TAXI' as const,
        status: 'ACTIVE' as const,
        money: 2845.50,
        mode: 'ECO' as const,
        location: [22.3193, 114.1694] as [number, number], // Tsim Sha Tsui
        path: [[22.3150, 114.1650], [22.3170, 114.1670], [22.3193, 114.1694]] as Array<[number, number]>,
        speed: 35,
        gear: 'D' as const,
        steering: -5.2,
        connectionState: 'connected' as const,
        latency_ms: 45,
        battery_level: 85,
        emergency_level: 0,
        priority_score: 75,
        telemetry: {
          cpu_usage: 35,
          gpu_usage: 45,
          memory_usage: 60,
          network_quality: 'excellent' as const,
          signal_strength: 95
        },
        sensors: {
          cameras_online: 6,
          cameras_total: 6,
          lidar_online: true,
          gps_fix: true
        }
      },
      {
        id: 'HK-002',
        type: 'ROBO-TAXI' as const,
        status: 'ACTIVE' as const,
        money: 1950.30,
        mode: 'ECO' as const,
        location: [22.2783, 114.1747] as [number, number], // Central
        path: [[22.2750, 114.1720], [22.2765, 114.1735], [22.2783, 114.1747]] as Array<[number, number]>,
        speed: 28,
        gear: 'D' as const,
        steering: 3.1,
        connectionState: 'connected' as const,
        latency_ms: 52,
        battery_level: 78,
        emergency_level: 0,
        priority_score: 70,
        telemetry: {
          cpu_usage: 40,
          gpu_usage: 50,
          memory_usage: 65,
          network_quality: 'excellent' as const,
          signal_strength: 92
        },
        sensors: {
          cameras_online: 6,
          cameras_total: 6,
          lidar_online: true,
          gps_fix: true
        }
      },
      {
        id: 'HK-003',
        type: 'LOGISTICS' as const,
        status: 'ACTIVE' as const,
        money: 3120.80,
        mode: 'SPORT' as const,
        location: [22.3364, 114.1733] as [number, number], // Mong Kok
        path: [[22.3330, 114.1700], [22.3345, 114.1715], [22.3364, 114.1733]] as Array<[number, number]>,
        speed: 42,
        gear: 'D' as const,
        steering: -2.8,
        connectionState: 'connected' as const,
        latency_ms: 38,
        battery_level: 92,
        emergency_level: 0,
        priority_score: 80,
        telemetry: {
          cpu_usage: 45,
          gpu_usage: 55,
          memory_usage: 70,
          network_quality: 'excellent' as const,
          signal_strength: 98
        },
        sensors: {
          cameras_online: 6,
          cameras_total: 6,
          lidar_online: true,
          gps_fix: true
        }
      },
      {
        id: 'HK-004',
        type: 'ROBO-TAXI' as const,
        status: 'ACTIVE' as const,
        money: 1675.20,
        mode: 'ECO' as const,
        location: [22.2800, 114.1580] as [number, number], // Sheung Wan
        path: [[22.2770, 114.1550], [22.2785, 114.1565], [22.2800, 114.1580]] as Array<[number, number]>,
        speed: 31,
        gear: 'D' as const,
        steering: 1.5,
        connectionState: 'connected' as const,
        latency_ms: 48,
        battery_level: 81,
        emergency_level: 0,
        priority_score: 68,
        telemetry: {
          cpu_usage: 38,
          gpu_usage: 48,
          memory_usage: 62,
          network_quality: 'excellent' as const,
          signal_strength: 94
        },
        sensors: {
          cameras_online: 6,
          cameras_total: 6,
          lidar_online: true,
          gps_fix: true
        }
      },
      {
        id: 'HK-005',
        type: 'SECURITY' as const,
        status: 'ACTIVE' as const,
        money: 890.50,
        mode: 'AI-HUNT' as const,
        location: [22.3200, 114.2100] as [number, number], // Kwun Tong
        path: [[22.3170, 114.2070], [22.3185, 114.2085], [22.3200, 114.2100]] as Array<[number, number]>,
        speed: 25,
        gear: 'D' as const,
        steering: -1.2,
        connectionState: 'connected' as const,
        latency_ms: 55,
        battery_level: 88,
        emergency_level: 1,
        priority_score: 72,
        telemetry: {
          cpu_usage: 50,
          gpu_usage: 60,
          memory_usage: 75,
          network_quality: 'good' as const,
          signal_strength: 88
        },
        sensors: {
          cameras_online: 6,
          cameras_total: 6,
          lidar_online: true,
          gps_fix: true
        }
      },
      {
        id: 'HK-006',
        type: 'ROBO-TAXI' as const,
        status: 'ACTIVE' as const,
        money: 2230.90,
        mode: 'ECO' as const,
        location: [22.2850, 114.1950] as [number, number], // Causeway Bay
        path: [[22.2820, 114.1920], [22.2835, 114.1935], [22.2850, 114.1950]] as Array<[number, number]>,
        speed: 38,
        gear: 'D' as const,
        steering: 4.3,
        connectionState: 'connected' as const,
        latency_ms: 42,
        battery_level: 76,
        emergency_level: 0,
        priority_score: 74,
        telemetry: {
          cpu_usage: 42,
          gpu_usage: 52,
          memory_usage: 68,
          network_quality: 'excellent' as const,
          signal_strength: 96
        },
        sensors: {
          cameras_online: 6,
          cameras_total: 6,
          lidar_online: true,
          gps_fix: true
        }
      },
      {
        id: 'HK-007',
        type: 'LOGISTICS' as const,
        status: 'ACTIVE' as const,
        money: 4150.60,
        mode: 'SPORT' as const,
        location: [22.3700, 114.1100] as [number, number], // Sha Tin
        path: [[22.3670, 114.1070], [22.3685, 114.1085], [22.3700, 114.1100]] as Array<[number, number]>,
        speed: 45,
        gear: 'D' as const,
        steering: -3.5,
        connectionState: 'connected' as const,
        latency_ms: 62,
        battery_level: 94,
        emergency_level: 0,
        priority_score: 82,
        telemetry: {
          cpu_usage: 48,
          gpu_usage: 58,
          memory_usage: 72,
          network_quality: 'good' as const,
          signal_strength: 90
        },
        sensors: {
          cameras_online: 6,
          cameras_total: 6,
          lidar_online: true,
          gps_fix: true
        }
      },
      {
        id: 'HK-008',
        type: 'ROBO-TAXI' as const,
        status: 'ACTIVE' as const,
        money: 1580.40,
        mode: 'ECO' as const,
        location: [22.2500, 114.1650] as [number, number], // Aberdeen
        path: [[22.2470, 114.1620], [22.2485, 114.1635], [22.2500, 114.1650]] as Array<[number, number]>,
        speed: 33,
        gear: 'D' as const,
        steering: 2.1,
        connectionState: 'connected' as const,
        latency_ms: 58,
        battery_level: 83,
        emergency_level: 0,
        priority_score: 69,
        telemetry: {
          cpu_usage: 36,
          gpu_usage: 46,
          memory_usage: 64,
          network_quality: 'good' as const,
          signal_strength: 91
        },
        sensors: {
          cameras_online: 6,
          cameras_total: 6,
          lidar_online: true,
          gps_fix: true
        }
      },
      {
        id: 'HK-009',
        type: 'ROBO-TAXI' as const,
        status: 'ACTIVE' as const,
        money: 2095.70,
        mode: 'ECO' as const,
        location: [22.3100, 114.2250] as [number, number], // Tseung Kwan O
        path: [[22.3070, 114.2220], [22.3085, 114.2235], [22.3100, 114.2250]] as Array<[number, number]>,
        speed: 36,
        gear: 'D' as const,
        steering: -1.8,
        connectionState: 'connected' as const,
        latency_ms: 51,
        battery_level: 79,
        emergency_level: 0,
        priority_score: 71,
        telemetry: {
          cpu_usage: 39,
          gpu_usage: 49,
          memory_usage: 66,
          network_quality: 'excellent' as const,
          signal_strength: 93
        },
        sensors: {
          cameras_online: 6,
          cameras_total: 6,
          lidar_online: true,
          gps_fix: true
        }
      },
      {
        id: 'HK-010',
        type: 'SECURITY' as const,
        status: 'ACTIVE' as const,
        money: 1120.30,
        mode: 'AI-HUNT' as const,
        location: [22.4200, 114.2070] as [number, number], // Tai Po
        path: [[22.4170, 114.2040], [22.4185, 114.2055], [22.4200, 114.2070]] as Array<[number, number]>,
        speed: 27,
        gear: 'D' as const,
        steering: 0.8,
        connectionState: 'connected' as const,
        latency_ms: 68,
        battery_level: 91,
        emergency_level: 2,
        priority_score: 78,
        telemetry: {
          cpu_usage: 52,
          gpu_usage: 62,
          memory_usage: 78,
          network_quality: 'good' as const,
          signal_strength: 87
        },
        sensors: {
          cameras_online: 6,
          cameras_total: 6,
          lidar_online: true,
          gps_fix: true
        }
      },
      {
        id: 'HK-011',
        type: 'LOGISTICS' as const,
        status: 'ACTIVE' as const,
        money: 3680.90,
        mode: 'SPORT' as const,
        location: [22.3950, 113.9750] as [number, number], // Tuen Mun
        path: [[22.3920, 113.9720], [22.3935, 113.9735], [22.3950, 113.9750]] as Array<[number, number]>,
        speed: 48,
        gear: 'D' as const,
        steering: -4.2,
        connectionState: 'connected' as const,
        latency_ms: 72,
        battery_level: 87,
        emergency_level: 0,
        priority_score: 76,
        telemetry: {
          cpu_usage: 46,
          gpu_usage: 56,
          memory_usage: 74,
          network_quality: 'good' as const,
          signal_strength: 85
        },
        sensors: {
          cameras_online: 6,
          cameras_total: 6,
          lidar_online: true,
          gps_fix: true
        }
      },
      // Idle Vehicles
      {
        id: 'HK-012',
        type: 'ROBO-TAXI' as const,
        status: 'IDLE' as const,
        money: 890.20,
        mode: 'ECO' as const,
        location: [22.3000, 114.1700] as [number, number], // Wan Chai
        path: [[22.2980, 114.1680], [22.2990, 114.1690], [22.3000, 114.1700]] as Array<[number, number]>,
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
          cameras_online: 6,
          cameras_total: 6,
          lidar_online: true,
          gps_fix: true
        }
      },
      {
        id: 'HK-013',
        type: 'LOGISTICS' as const,
        status: 'IDLE' as const,
        money: 1250.50,
        mode: 'ECO' as const,
        location: [22.3550, 114.1350] as [number, number], // Kowloon Tong
        path: [[22.3530, 114.1330], [22.3540, 114.1340], [22.3550, 114.1350]] as Array<[number, number]>,
        speed: 0,
        gear: 'P' as const,
        steering: 0,
        connectionState: 'disconnected' as const,
        latency_ms: 0,
        battery_level: 52,
        emergency_level: 0,
        priority_score: 45,
        telemetry: {
          cpu_usage: 18,
          gpu_usage: 12,
          memory_usage: 38,
          network_quality: 'good' as const,
          signal_strength: 82
        },
        sensors: {
          cameras_online: 6,
          cameras_total: 6,
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
