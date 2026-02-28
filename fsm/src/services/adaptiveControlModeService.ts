import logger from '@/utils/logger'
/**
 * FSM-Pilot V2.0 - Adaptive Control Mode Service
 *
 * Automatically switches between three control modes based on network conditions:
 * 1. Direct Control - Low latency, high bandwidth (full vehicle control)
 * 2. Trajectory Confirmation - Medium latency, medium bandwidth (waypoint approval)
 * 3. Semantic Instruction - High latency, low bandwidth (high-level commands)
 *
 * Enables optimal resource efficiency and seamless mode transitions
 *
 * @author Li Yixiang
 * @institution City University of Hong Kong
 */

export type ControlMode = 'direct' | 'trajectory' | 'semantic'

export interface ControlModeRequirements {
  mode: ControlMode
  minBandwidth: number         // Mbps
  maxLatency: number          // ms
  maxPacketLoss: number       // percentage
  description: string
  controlFrequency: number    // Hz
  dataSizePerUpdate: number   // KB
}

export interface NetworkCondition {
  latency: number             // ms
  bandwidth: number           // Mbps
  packetLoss: number         // percentage
  jitter: number             // ms
  connectionStability: number // 0-1
  timestamp: number
}

export interface ControlCommand {
  mode: ControlMode
  timestamp: number
  data: DirectControlData | TrajectoryData | SemanticInstructionData
}

// Direct control: Full vehicle control (steering, throttle, brake)
export interface DirectControlData {
  steering: number            // -100 to 100
  throttle: number           // 0 to 100
  brake: number              // 0 to 100
  gear: 'P' | 'R' | 'N' | 'D'
  turnSignal: 'none' | 'left' | 'right' | 'hazard'
}

// Trajectory control: Waypoint-based navigation
export interface TrajectoryData {
  waypoints: Array<{
    lat: number
    lon: number
    speed: number            // Target speed (km/h)
    timestamp: number        // When to reach this waypoint
  }>
  confirmationRequired: boolean
  trajectoryId: string
}

// Semantic instruction: High-level commands
export interface SemanticInstructionData {
  instruction: string
  type: 'navigation' | 'maneuver' | 'safety' | 'parking'
  parameters: Record<string, any>
  expectedDuration: number   // ms
  requiresAck: boolean
}

export interface ModeTransition {
  fromMode: ControlMode
  toMode: ControlMode
  reason: string
  timestamp: number
  networkCondition: NetworkCondition
  transitionDuration: number  // ms
  success: boolean
}

export interface ControlModeState {
  vehicleId: string
  currentMode: ControlMode
  previousMode: ControlMode | null
  modeStartTime: number
  transitions: ModeTransition[]
  modeDurations: Record<ControlMode, number>  // Total time in each mode (ms)
  performanceMetrics: {
    commandLatency: number
    commandSuccessRate: number
    networkUtilization: number
    userSatisfaction: number
  }
}

/**
 * Control mode requirements and specifications
 */
const MODE_REQUIREMENTS: Record<ControlMode, ControlModeRequirements> = {
  direct: {
    mode: 'direct',
    minBandwidth: 5,           // 5 Mbps for video + control
    maxLatency: 100,           // 100ms max for responsive control
    maxPacketLoss: 1,          // 1% max packet loss
    description: 'Direct vehicle control with real-time steering, throttle, and brake',
    controlFrequency: 20,      // 20 Hz control updates
    dataSizePerUpdate: 2       // 2 KB per update
  },
  trajectory: {
    mode: 'trajectory',
    minBandwidth: 2,           // 2 Mbps for compressed video + waypoints
    maxLatency: 300,           // 300ms acceptable for waypoint confirmation
    maxPacketLoss: 3,          // 3% packet loss acceptable
    description: 'Waypoint-based trajectory control with operator confirmation',
    controlFrequency: 1,       // 1 Hz trajectory updates
    dataSizePerUpdate: 5       // 5 KB per trajectory
  },
  semantic: {
    mode: 'semantic',
    minBandwidth: 0.5,         // 0.5 Mbps for low-res video + commands
    maxLatency: 1000,          // 1s latency acceptable for high-level commands
    maxPacketLoss: 5,          // 5% packet loss acceptable
    description: 'High-level semantic instructions (e.g., "Turn left at next intersection")',
    controlFrequency: 0.1,     // 0.1 Hz (once per 10 seconds)
    dataSizePerUpdate: 1       // 1 KB per instruction
  }
}

/**
 * Adaptive Control Mode Service
 */
class AdaptiveControlModeService {
  private modeStates: Map<string, ControlModeState> = new Map()
  private networkHistory: Map<string, NetworkCondition[]> = new Map()
  private transitionCallbacks: Array<(vehicleId: string, transition: ModeTransition) => void> = []

  // Hysteresis thresholds to prevent mode flapping
  private hysteresis = {
    latency: 20,              // 20ms hysteresis
    bandwidth: 0.5,           // 0.5 Mbps hysteresis
    packetLoss: 0.5,          // 0.5% hysteresis
    stabilityWindow: 5000     // 5s stability required before transition
  }

  constructor() {
    logger.info('[Adaptive Control] Service initialized', 'vehicle')
  }

  /**
   * Initialize control mode state for vehicle
   */
  initializeVehicle(vehicleId: string, initialMode: ControlMode = 'trajectory') {
    this.modeStates.set(vehicleId, {
      vehicleId,
      currentMode: initialMode,
      previousMode: null,
      modeStartTime: Date.now(),
      transitions: [],
      modeDurations: {
        direct: 0,
        trajectory: 0,
        semantic: 0
      },
      performanceMetrics: {
        commandLatency: 0,
        commandSuccessRate: 1.0,
        networkUtilization: 0.5,
        userSatisfaction: 0.8
      }
    })

    this.networkHistory.set(vehicleId, [])
    logger.info(`[Adaptive Control] Vehicle ${vehicleId} initialized in ${initialMode} mode`, 'vehicle')
  }

  /**
   * Update network condition and potentially trigger mode switch
   */
  updateNetworkCondition(vehicleId: string, condition: NetworkCondition): ControlMode {
    const state = this.modeStates.get(vehicleId)
    if (!state) {
      logger.warn(`Vehicle ${vehicleId} not initialized`, 'vehicle')
      return 'trajectory'  // Default fallback
    }

    // Store network history
    const history = this.networkHistory.get(vehicleId) || []
    history.push(condition)
    if (history.length > 50) {
      history.shift()  // Keep last 50 samples
    }
    this.networkHistory.set(vehicleId, history)

    // Determine optimal mode
    const optimalMode = this.determineOptimalMode(condition, state.currentMode)

    // Check if mode switch is needed
    if (optimalMode !== state.currentMode) {
      const canSwitch = this.canSwitchMode(vehicleId, optimalMode, condition)

      if (canSwitch) {
        this.switchMode(vehicleId, optimalMode, condition)
      }
    }

    return state.currentMode
  }

  /**
   * Determine optimal control mode based on network conditions
   */
  private determineOptimalMode(condition: NetworkCondition, currentMode: ControlMode): ControlMode {
    // Apply hysteresis to prevent rapid switching
    const effectiveLatency = currentMode === 'direct'
      ? condition.latency - this.hysteresis.latency
      : condition.latency + this.hysteresis.latency

    const effectiveBandwidth = currentMode === 'direct'
      ? condition.bandwidth + this.hysteresis.bandwidth
      : condition.bandwidth - this.hysteresis.bandwidth

    const effectivePacketLoss = currentMode === 'direct'
      ? condition.packetLoss + this.hysteresis.packetLoss
      : condition.packetLoss - this.hysteresis.packetLoss

    // Check direct control mode
    if (
      effectiveBandwidth >= MODE_REQUIREMENTS.direct.minBandwidth &&
      effectiveLatency <= MODE_REQUIREMENTS.direct.maxLatency &&
      effectivePacketLoss <= MODE_REQUIREMENTS.direct.maxPacketLoss &&
      condition.connectionStability > 0.8
    ) {
      return 'direct'
    }

    // Check trajectory mode
    if (
      effectiveBandwidth >= MODE_REQUIREMENTS.trajectory.minBandwidth &&
      effectiveLatency <= MODE_REQUIREMENTS.trajectory.maxLatency &&
      effectivePacketLoss <= MODE_REQUIREMENTS.trajectory.maxPacketLoss
    ) {
      return 'trajectory'
    }

    // Fall back to semantic mode
    return 'semantic'
  }

  /**
   * Check if mode switch is allowed (stability window)
   */
  private canSwitchMode(
    vehicleId: string,
    targetMode: ControlMode,
    condition: NetworkCondition
  ): boolean {
    const state = this.modeStates.get(vehicleId)
    if (!state) return false

    // Must be in current mode for minimum stability window
    const timeInCurrentMode = Date.now() - state.modeStartTime
    if (timeInCurrentMode < this.hysteresis.stabilityWindow) {
      return false
    }

    // Check if network has been stable for target mode
    const history = this.networkHistory.get(vehicleId) || []
    if (history.length < 5) {
      return false  // Need at least 5 samples
    }

    const recentHistory = history.slice(-5)
    const targetReq = MODE_REQUIREMENTS[targetMode]

    // All recent samples must meet target mode requirements
    return recentHistory.every(nc =>
      nc.bandwidth >= targetReq.minBandwidth &&
      nc.latency <= targetReq.maxLatency &&
      nc.packetLoss <= targetReq.maxPacketLoss
    )
  }

  /**
   * Perform mode switch
   */
  private switchMode(vehicleId: string, newMode: ControlMode, condition: NetworkCondition) {
    const state = this.modeStates.get(vehicleId)
    if (!state) return

    const startTime = Date.now()
    const oldMode = state.currentMode

    // Update mode durations
    const durationInOldMode = startTime - state.modeStartTime
    state.modeDurations[oldMode] += durationInOldMode

    // Perform transition
    state.previousMode = oldMode
    state.currentMode = newMode
    state.modeStartTime = startTime

    const transition: ModeTransition = {
      fromMode: oldMode,
      toMode: newMode,
      reason: this.getTransitionReason(oldMode, newMode, condition),
      timestamp: startTime,
      networkCondition: condition,
      transitionDuration: Date.now() - startTime,
      success: true
    }

    state.transitions.push(transition)

    logger.info(`[Adaptive Control] Vehicle ${vehicleId}: ${oldMode} → ${newMode}`, 'vehicle')
    logger.info(`  Reason: ${transition.reason}`, 'vehicle')
    logger.info(`  Network: ${condition.latency}ms latency, ${condition.bandwidth.toFixed(1)} Mbps, ${condition.packetLoss.toFixed(1)}% loss`, 'vehicle')

    // Notify callbacks
    for (const callback of this.transitionCallbacks) {
      callback(vehicleId, transition)
    }
  }

  /**
   * Generate human-readable transition reason
   */
  private getTransitionReason(fromMode: ControlMode, toMode: ControlMode, condition: NetworkCondition): string {
    if (fromMode === 'semantic' && toMode === 'trajectory') {
      return `Network improved (latency: ${condition.latency}ms, bandwidth: ${condition.bandwidth.toFixed(1)} Mbps)`
    }
    if (fromMode === 'trajectory' && toMode === 'direct') {
      return `Network optimal for direct control (latency: ${condition.latency}ms)`
    }
    if (fromMode === 'direct' && toMode === 'trajectory') {
      return `Network degraded (latency: ${condition.latency}ms, packet loss: ${condition.packetLoss.toFixed(1)}%)`
    }
    if (fromMode === 'trajectory' && toMode === 'semantic') {
      return `Poor network quality (latency: ${condition.latency}ms, bandwidth: ${condition.bandwidth.toFixed(1)} Mbps)`
    }
    if (fromMode === 'direct' && toMode === 'semantic') {
      return `Severe network degradation (latency: ${condition.latency}ms)`
    }
    if (fromMode === 'semantic' && toMode === 'direct') {
      return `Network significantly improved - switching to direct control`
    }
    return `Network conditions changed`
  }

  /**
   * Get current mode for vehicle
   */
  getCurrentMode(vehicleId: string): ControlMode | null {
    const state = this.modeStates.get(vehicleId)
    return state ? state.currentMode : null
  }

  /**
   * Get mode state
   */
  getModeState(vehicleId: string): ControlModeState | null {
    return this.modeStates.get(vehicleId) || null
  }

  /**
   * Get mode requirements
   */
  getModeRequirements(mode: ControlMode): ControlModeRequirements {
    return MODE_REQUIREMENTS[mode]
  }

  /**
   * Calculate bandwidth usage for current mode
   */
  calculateBandwidthUsage(mode: ControlMode, includeVideo: boolean = true): number {
    const req = MODE_REQUIREMENTS[mode]
    let bandwidth = req.controlFrequency * req.dataSizePerUpdate / 1000  // Mbps

    if (includeVideo) {
      // Add video stream bandwidth
      const videoMap: Record<ControlMode, number> = {
        direct: 4,        // High quality video: 4 Mbps
        trajectory: 1.5,  // Medium quality: 1.5 Mbps
        semantic: 0.3     // Low quality: 0.3 Mbps
      }
      bandwidth += videoMap[mode]
    }

    return bandwidth
  }

  /**
   * Subscribe to mode transitions
   */
  onModeTransition(callback: (vehicleId: string, transition: ModeTransition) => void) {
    this.transitionCallbacks.push(callback)
  }

  /**
   * Get statistics across all vehicles
   */
  getStatistics() {
    const stats = {
      totalVehicles: this.modeStates.size,
      byMode: {
        direct: 0,
        trajectory: 0,
        semantic: 0
      },
      avgTransitionsPerVehicle: 0,
      totalBandwidthUsage: 0,
      modeDistribution: {
        direct: 0,
        trajectory: 0,
        semantic: 0
      }
    }

    let totalTransitions = 0

    for (const [vehicleId, state] of this.modeStates.entries()) {
      stats.byMode[state.currentMode]++
      totalTransitions += state.transitions.length
      stats.totalBandwidthUsage += this.calculateBandwidthUsage(state.currentMode, true)

      // Calculate mode distribution by time
      const totalTime = Object.values(state.modeDurations).reduce((sum, dur) => sum + dur, 0)
      if (totalTime > 0) {
        for (const mode of ['direct', 'trajectory', 'semantic'] as ControlMode[]) {
          stats.modeDistribution[mode] += state.modeDurations[mode] / totalTime
        }
      }
    }

    stats.avgTransitionsPerVehicle = stats.totalVehicles > 0
      ? totalTransitions / stats.totalVehicles
      : 0

    // Normalize mode distribution
    if (stats.totalVehicles > 0) {
      for (const mode of ['direct', 'trajectory', 'semantic'] as ControlMode[]) {
        stats.modeDistribution[mode] /= stats.totalVehicles
      }
    }

    return stats
  }

  /**
   * Generate mock network condition
   */
  generateMockNetworkCondition(quality: 'excellent' | 'good' | 'fair' | 'poor'): NetworkCondition {
    const conditions: Record<typeof quality, NetworkCondition> = {
      excellent: {
        latency: 40 + Math.random() * 20,
        bandwidth: 8 + Math.random() * 4,
        packetLoss: Math.random() * 0.5,
        jitter: 5 + Math.random() * 10,
        connectionStability: 0.95 + Math.random() * 0.05,
        timestamp: Date.now()
      },
      good: {
        latency: 80 + Math.random() * 40,
        bandwidth: 4 + Math.random() * 3,
        packetLoss: 0.5 + Math.random() * 1,
        jitter: 15 + Math.random() * 15,
        connectionStability: 0.8 + Math.random() * 0.15,
        timestamp: Date.now()
      },
      fair: {
        latency: 150 + Math.random() * 100,
        bandwidth: 1.5 + Math.random() * 2,
        packetLoss: 1.5 + Math.random() * 2,
        jitter: 30 + Math.random() * 20,
        connectionStability: 0.6 + Math.random() * 0.2,
        timestamp: Date.now()
      },
      poor: {
        latency: 300 + Math.random() * 200,
        bandwidth: 0.5 + Math.random() * 1,
        packetLoss: 3 + Math.random() * 3,
        jitter: 50 + Math.random() * 50,
        connectionStability: 0.3 + Math.random() * 0.3,
        timestamp: Date.now()
      }
    }

    return conditions[quality]
  }

  /**
   * Create example control command for mode
   */
  createExampleCommand(mode: ControlMode, vehicleId: string): ControlCommand {
    const timestamp = Date.now()

    switch (mode) {
      case 'direct':
        return {
          mode: 'direct',
          timestamp,
          data: {
            steering: 0,
            throttle: 30,
            brake: 0,
            gear: 'D',
            turnSignal: 'none'
          }
        }

      case 'trajectory':
        return {
          mode: 'trajectory',
          timestamp,
          data: {
            waypoints: [
              { lat: 22.3, lon: 114.2, speed: 40, timestamp: timestamp + 5000 },
              { lat: 22.301, lon: 114.201, speed: 45, timestamp: timestamp + 10000 },
              { lat: 22.302, lon: 114.202, speed: 50, timestamp: timestamp + 15000 }
            ],
            confirmationRequired: true,
            trajectoryId: `traj-${vehicleId}-${timestamp}`
          }
        }

      case 'semantic':
        return {
          mode: 'semantic',
          timestamp,
          data: {
            instruction: 'Continue straight for 500 meters, then turn right at the traffic light',
            type: 'navigation',
            parameters: {
              distance: 500,
              action: 'turn_right',
              landmark: 'traffic_light'
            },
            expectedDuration: 30000,
            requiresAck: true
          }
        }
    }
  }
}

// Singleton instance
let adaptiveControlModeService: AdaptiveControlModeService | null = null

export function getAdaptiveControlModeService(): AdaptiveControlModeService {
  if (!adaptiveControlModeService) {
    adaptiveControlModeService = new AdaptiveControlModeService()
  }
  return adaptiveControlModeService
}

export { AdaptiveControlModeService, MODE_REQUIREMENTS }
