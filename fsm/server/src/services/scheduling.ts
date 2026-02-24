/**
 * FSM-Pilot V2.0 - Scheduling Service
 *
 * @description 车辆调度服务
 */

import { logger } from '../utils/logger.js'

export interface SchedulingQueueItem {
  vehicle_id: string
  priority_score: number
  emergency_level: number
  latency_ms: number
  task_status: string
  reason: string
  last_update_time: number
  battery_level?: number
}

export interface SchedulingConfig {
  enabled: boolean
  algorithm: string
  weights: {
    emergency: number
    latency: number
    distance: number
    battery: number
    task_priority: number
  }
}

export class SchedulingService {
  private queue: Map<string, SchedulingQueueItem> = new Map()
  private config: SchedulingConfig = {
    enabled: true,
    algorithm: 'weighted_priority',
    weights: {
      emergency: 0.4,
      latency: 0.2,
      distance: 0.15,
      battery: 0.15,
      task_priority: 0.1
    }
  }

  public updateVehicle(vehicleId: string, status: any) {
    const existing = this.queue.get(vehicleId)
    const item: SchedulingQueueItem = {
      vehicle_id: vehicleId,
      priority_score: this.calculatePriority(status, existing),
      emergency_level: status.emergency_level || 0,
      latency_ms: status.latency_ms || 0,
      task_status: 'active',
      reason: 'Normal operation',
      last_update_time: Date.now(),
      battery_level: status.battery_level
    }

    this.queue.set(vehicleId, item)
  }

  private calculatePriority(status: any, existing?: SchedulingQueueItem): number {
    const w = this.config.weights

    // 基础分数
    let score = 50

    // 紧急程度
    score += (status.emergency_level || 0) * 10 * w.emergency

    // 延迟 (低延迟优先)
    const latencyScore = Math.max(0, 100 - (status.latency_ms || 0) / 10)
    score += latencyScore * w.latency

    // 电池 (高电量优先)
    score += (status.battery_level || 50) * w.battery

    // 保持现有优先级的连续性
    if (existing) {
      score = score * 0.3 + existing.priority_score * 0.7
    }

    return Math.max(0, Math.min(100, score))
  }

  public getQueue(): SchedulingQueueItem[] {
    const items = Array.from(this.queue.values())
    return items.sort((a, b) => b.priority_score - a.priority_score)
  }

  public getConfig(): SchedulingConfig {
    return { ...this.config }
  }

  public updateConfig(update: Partial<SchedulingConfig>) {
    if (update.enabled !== undefined) {
      this.config.enabled = update.enabled
    }
    if (update.algorithm) {
      this.config.algorithm = update.algorithm
    }
    if (update.weights) {
      this.config.weights = { ...this.config.weights, ...update.weights }
    }
    logger.info('Scheduling config updated:', this.config)
  }
}
