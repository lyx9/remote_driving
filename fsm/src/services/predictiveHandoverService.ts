/**
 * FSM-Pilot V2.0 - Predictive Handover Service
 *
 * Predictive compensation system that pre-establishes connections
 * before vehicle disengagement to minimize handover latency
 *
 * Uses machine learning to predict:
 * - When vehicle will require manual intervention
 * - Optimal pre-connection timing
 * - Network setup requirements
 *
 * @author Li Yixiang
 * @institution City University of Hong Kong
 */

import type { RiskScore, VehicleRiskFeatures } from './riskScoringService'
import type { Operator } from './bipartiteMatchingService'

export interface VehicleState {
  vehicleId: string
  currentMode: 'autonomous' | 'supervised' | 'manual'
  autonomyConfidence: number    // 0-1, AD system confidence
  riskFeatures: VehicleRiskFeatures
  riskScore: RiskScore
  networkMetrics: {
    latency: number
    bandwidth: number            // Mbps
    packetLoss: number          // percentage
    jitter: number              // ms
  }
  trajectory: {
    speed: number
    heading: number
    predictedPath: Array<{ lat: number; lon: number; timestamp: number }>
  }
  history: {
    recentDisengagements: Array<{ timestamp: number; reason: string }>
    avgTimeBetweenDisengagements: number  // seconds
    lastDisengagement?: number
  }
}

export interface PredictionResult {
  vehicleId: string
  disengagementProbability: number     // 0-1, probability in next 60s
  predictedDisengagementTime: number   // ms from now
  confidence: number                   // 0-1, prediction confidence
  triggerFactors: string[]             // Key factors triggering prediction
  recommendedAction: 'pre-connect' | 'monitor' | 'none'
  connectionSetupTime: number          // Estimated ms to establish connection
  optimalPreConnectionTime: number     // ms before predicted disengagement
}

export interface ConnectionState {
  vehicleId: string
  operatorId: string | null
  status: 'disconnected' | 'pre-connecting' | 'ready' | 'active'
  connectionQuality: number            // 0-1
  setupStartTime?: number
  readyTime?: number
  activationTime?: number
  preConnectionBufferMs: number        // Time buffer before disengagement
}

/**
 * Predictive model for disengagement forecasting
 */
class DisengagementPredictor {
  // Learned thresholds (simplified ML model)
  private thresholds = {
    criticalRisk: 0.75,
    highRisk: 0.6,
    mediumRisk: 0.4,
    confidenceDrop: 0.3,        // AD confidence drop threshold
    frequentDisengagement: 120   // seconds between disengagements
  }

  /**
   * Predict probability of disengagement in next 60 seconds
   */
  predict(state: VehicleState): PredictionResult {
    const factors: string[] = []
    let probability = 0.0
    let predictedTime = 60000  // Default: 60 seconds

    // Factor 1: Risk score (40% weight)
    const riskFactor = state.riskScore.overallScore
    probability += riskFactor * 0.4

    if (riskFactor > this.thresholds.criticalRisk) {
      factors.push('Critical risk level detected')
      predictedTime = Math.min(predictedTime, 5000)  // 5 seconds
    } else if (riskFactor > this.thresholds.highRisk) {
      factors.push('High risk level')
      predictedTime = Math.min(predictedTime, 15000)  // 15 seconds
    }

    // Factor 2: Autonomy confidence (30% weight)
    const confidenceFactor = 1 - state.autonomyConfidence
    probability += confidenceFactor * 0.3

    if (state.autonomyConfidence < 0.5) {
      factors.push('Low AD confidence')
      predictedTime = Math.min(predictedTime, 10000)
    }

    // Factor 3: Recent disengagement pattern (20% weight)
    if (state.history.avgTimeBetweenDisengagements > 0 &&
        state.history.avgTimeBetweenDisengagements < this.thresholds.frequentDisengagement) {
      const patternFactor = 1 - (state.history.avgTimeBetweenDisengagements / this.thresholds.frequentDisengagement)
      probability += patternFactor * 0.2
      factors.push('Frequent disengagement pattern')
      predictedTime = Math.min(predictedTime, state.history.avgTimeBetweenDisengagements * 1000 * 0.8)
    }

    // Factor 4: Network quality (10% weight)
    const networkScore = this.calculateNetworkScore(state.networkMetrics)
    const networkFactor = 1 - networkScore
    probability += networkFactor * 0.1

    if (state.networkMetrics.latency > 200 || state.networkMetrics.packetLoss > 3) {
      factors.push('Poor network quality')
      // Don't reduce predicted time for network issues
    }

    // Factor 5: Trajectory complexity
    const trajectoryComplexity = this.analyzeTrajectoryComplexity(state)
    if (trajectoryComplexity > 0.7) {
      factors.push('Complex trajectory ahead')
      probability += 0.1
      predictedTime = Math.min(predictedTime, 20000)
    }

    // Normalize probability
    probability = Math.min(1, Math.max(0, probability))

    // Confidence based on available data
    const confidence = this.calculatePredictionConfidence(state)

    // Recommendation
    let recommendedAction: PredictionResult['recommendedAction'] = 'none'
    if (probability > 0.7 && confidence > 0.6) {
      recommendedAction = 'pre-connect'
    } else if (probability > 0.5) {
      recommendedAction = 'monitor'
    }

    return {
      vehicleId: state.vehicleId,
      disengagementProbability: probability,
      predictedDisengagementTime: predictedTime,
      confidence,
      triggerFactors: factors,
      recommendedAction,
      connectionSetupTime: this.estimateConnectionSetupTime(state),
      optimalPreConnectionTime: this.calculateOptimalPreConnectionTime(
        predictedTime,
        this.estimateConnectionSetupTime(state)
      )
    }
  }

  private calculateNetworkScore(metrics: VehicleState['networkMetrics']): number {
    let score = 1.0

    // Latency penalty
    if (metrics.latency > 300) score -= 0.4
    else if (metrics.latency > 200) score -= 0.3
    else if (metrics.latency > 100) score -= 0.15

    // Packet loss penalty
    if (metrics.packetLoss > 5) score -= 0.3
    else if (metrics.packetLoss > 3) score -= 0.2
    else if (metrics.packetLoss > 1) score -= 0.1

    // Bandwidth penalty
    if (metrics.bandwidth < 2) score -= 0.2
    else if (metrics.bandwidth < 5) score -= 0.1

    // Jitter penalty
    if (metrics.jitter > 50) score -= 0.2
    else if (metrics.jitter > 30) score -= 0.1

    return Math.max(0, score)
  }

  private analyzeTrajectoryComplexity(state: VehicleState): number {
    let complexity = 0

    // High speed increases complexity
    if (state.trajectory.speed > 80) complexity += 0.3
    else if (state.trajectory.speed > 60) complexity += 0.2

    // Rapid heading changes
    const path = state.trajectory.predictedPath
    if (path.length >= 2) {
      let totalHeadingChange = 0
      for (let i = 1; i < path.length; i++) {
        const dLat = path[i].lat - path[i - 1].lat
        const dLon = path[i].lon - path[i - 1].lon
        const heading = Math.atan2(dLon, dLat) * 180 / Math.PI
        if (i > 1) {
          const prevDLat = path[i - 1].lat - path[i - 2].lat
          const prevDLon = path[i - 1].lon - path[i - 2].lon
          const prevHeading = Math.atan2(prevDLon, prevDLat) * 180 / Math.PI
          totalHeadingChange += Math.abs(heading - prevHeading)
        }
      }
      const avgHeadingChange = totalHeadingChange / (path.length - 1)
      if (avgHeadingChange > 15) complexity += 0.4
      else if (avgHeadingChange > 8) complexity += 0.2
    }

    return Math.min(1, complexity)
  }

  private calculatePredictionConfidence(state: VehicleState): number {
    let confidence = 1.0

    // Reduce confidence if insufficient history
    if (state.history.recentDisengagements.length < 3) {
      confidence -= 0.2
    }

    // Reduce confidence if risk score computation was slow (unreliable)
    if (state.riskScore.computeTimeMs > 15) {
      confidence -= 0.15
    }

    // Reduce confidence if trajectory data is sparse
    if (state.trajectory.predictedPath.length < 5) {
      confidence -= 0.15
    }

    return Math.max(0.3, confidence)
  }

  private estimateConnectionSetupTime(state: VehicleState): number {
    // Base setup time
    let setupTime = 2000  // 2 seconds baseline

    // Network latency increases setup time
    setupTime += state.networkMetrics.latency * 2

    // Poor network quality increases setup time
    const networkScore = this.calculateNetworkScore(state.networkMetrics)
    if (networkScore < 0.5) {
      setupTime += 1000
    }

    // High packet loss increases retry time
    setupTime += state.networkMetrics.packetLoss * 200

    return setupTime
  }

  private calculateOptimalPreConnectionTime(
    predictedDisengagementTime: number,
    connectionSetupTime: number
  ): number {
    // Add 20% buffer for safety
    const buffer = connectionSetupTime * 0.2
    const optimalTime = connectionSetupTime + buffer

    // Ensure we don't pre-connect too early (max 30s before)
    return Math.min(30000, Math.max(1000, predictedDisengagementTime - optimalTime))
  }
}

/**
 * Predictive Handover Service
 */
class PredictiveHandoverService {
  private predictor: DisengagementPredictor
  private vehicleStates: Map<string, VehicleState> = new Map()
  private connectionStates: Map<string, ConnectionState> = new Map()
  private predictionInterval: number | null = null
  private updateCallbacks: Array<(vehicleId: string, prediction: PredictionResult) => void> = []

  constructor() {
    this.predictor = new DisengagementPredictor()
  }

  /**
   * Start predictive monitoring
   */
  startMonitoring(intervalMs: number = 1000) {
    if (this.predictionInterval) {
      this.stopMonitoring()
    }

    this.predictionInterval = window.setInterval(() => {
      this.runPredictionCycle()
    }, intervalMs)

    console.log(`Predictive handover monitoring started (interval: ${intervalMs}ms)`)
  }

  /**
   * Stop predictive monitoring
   */
  stopMonitoring() {
    if (this.predictionInterval) {
      clearInterval(this.predictionInterval)
      this.predictionInterval = null
      console.log('Predictive handover monitoring stopped')
    }
  }

  /**
   * Update vehicle state
   */
  updateVehicleState(state: VehicleState) {
    this.vehicleStates.set(state.vehicleId, state)

    // Initialize connection state if needed
    if (!this.connectionStates.has(state.vehicleId)) {
      this.connectionStates.set(state.vehicleId, {
        vehicleId: state.vehicleId,
        operatorId: null,
        status: 'disconnected',
        connectionQuality: 0,
        preConnectionBufferMs: 5000
      })
    }
  }

  /**
   * Run prediction cycle for all vehicles
   */
  private runPredictionCycle() {
    for (const [vehicleId, state] of this.vehicleStates.entries()) {
      const prediction = this.predictor.predict(state)
      const connectionState = this.connectionStates.get(vehicleId)

      if (!connectionState) continue

      // Handle pre-connection based on prediction
      if (prediction.recommendedAction === 'pre-connect' &&
          connectionState.status === 'disconnected') {
        this.initiatePreConnection(vehicleId, prediction)
      }

      // Notify callbacks
      for (const callback of this.updateCallbacks) {
        callback(vehicleId, prediction)
      }
    }
  }

  /**
   * Initiate pre-connection for vehicle
   */
  private async initiatePreConnection(vehicleId: string, prediction: PredictionResult) {
    const connectionState = this.connectionStates.get(vehicleId)
    if (!connectionState) return

    console.log(`[Predictive Handover] Pre-connecting for vehicle ${vehicleId}`)
    console.log(`  Predicted disengagement in ${(prediction.predictedDisengagementTime / 1000).toFixed(1)}s`)
    console.log(`  Factors: ${prediction.triggerFactors.join(', ')}`)

    connectionState.status = 'pre-connecting'
    connectionState.setupStartTime = Date.now()

    // Simulate connection setup
    setTimeout(() => {
      if (connectionState.status === 'pre-connecting') {
        connectionState.status = 'ready'
        connectionState.readyTime = Date.now()
        connectionState.connectionQuality = 0.9

        const setupDuration = connectionState.readyTime - (connectionState.setupStartTime || 0)
        console.log(`[Predictive Handover] Connection ready for ${vehicleId} (setup: ${setupDuration}ms)`)
      }
    }, prediction.connectionSetupTime)
  }

  /**
   * Activate pre-established connection
   */
  activateConnection(vehicleId: string, operatorId: string): boolean {
    const connectionState = this.connectionStates.get(vehicleId)

    if (!connectionState) {
      console.error(`No connection state for vehicle ${vehicleId}`)
      return false
    }

    if (connectionState.status !== 'ready') {
      console.warn(`Connection not ready for vehicle ${vehicleId} (status: ${connectionState.status})`)
      return false
    }

    connectionState.status = 'active'
    connectionState.operatorId = operatorId
    connectionState.activationTime = Date.now()

    const handoverLatency = connectionState.activationTime - (connectionState.setupStartTime || 0)
    console.log(`[Predictive Handover] Connection activated for ${vehicleId} → ${operatorId}`)
    console.log(`  Total handover latency: ${handoverLatency}ms`)

    return true
  }

  /**
   * Get prediction for specific vehicle
   */
  getPrediction(vehicleId: string): PredictionResult | null {
    const state = this.vehicleStates.get(vehicleId)
    if (!state) return null

    return this.predictor.predict(state)
  }

  /**
   * Get connection state
   */
  getConnectionState(vehicleId: string): ConnectionState | null {
    return this.connectionStates.get(vehicleId) || null
  }

  /**
   * Subscribe to prediction updates
   */
  onPredictionUpdate(callback: (vehicleId: string, prediction: PredictionResult) => void) {
    this.updateCallbacks.push(callback)
  }

  /**
   * Get statistics
   */
  getStatistics() {
    const stats = {
      totalVehicles: this.vehicleStates.size,
      byConnectionStatus: {
        disconnected: 0,
        preConnecting: 0,
        ready: 0,
        active: 0
      },
      avgPredictionProbability: 0,
      vehiclesRequiringAttention: 0
    }

    let totalProbability = 0

    for (const [vehicleId, connectionState] of this.connectionStates.entries()) {
      const status = connectionState.status === 'pre-connecting' ? 'preConnecting' : connectionState.status
      if (status in stats.byConnectionStatus) {
        stats.byConnectionStatus[status as keyof typeof stats.byConnectionStatus]++
      }

      const state = this.vehicleStates.get(vehicleId)
      if (state) {
        const prediction = this.predictor.predict(state)
        totalProbability += prediction.disengagementProbability

        if (prediction.disengagementProbability > 0.5) {
          stats.vehiclesRequiringAttention++
        }
      }
    }

    stats.avgPredictionProbability = stats.totalVehicles > 0
      ? totalProbability / stats.totalVehicles
      : 0

    return stats
  }

  /**
   * Generate mock vehicle state for testing
   */
  generateMockVehicleState(vehicleId: string, scenario: 'stable' | 'risky' | 'critical'): VehicleState {
    const baseState: VehicleState = {
      vehicleId,
      currentMode: 'autonomous',
      autonomyConfidence: 0.85,
      riskFeatures: {
        speed: 60,
        speedVariance: 5,
        acceleration: 0,
        locationComplexity: 0.5,
        trafficDensity: 5,
        weatherSeverity: 0.2,
        roadType: 'urban',
        timeOfDay: 14,
        isDaylight: true,
        systemHealth: 0.95,
        sensorStatus: 0.95,
        networkLatency: 80,
        networkQuality: 0.9,
        trajectoryDeviation: 0.5,
        emergencyBrakeCount: 0,
        laneChangeFrequency: 2,
        steeringVariance: 5,
        operatorExperience: 0.7,
        operatorFatigue: 0.3,
        operatorWorkload: 0.5,
        recentIncidents: 0,
        avgResponseTime: 2
      },
      riskScore: {
        overallScore: 0.3,
        urgencyLevel: 'low',
        components: { vehicleRisk: 0.2, environmentRisk: 0.3, systemRisk: 0.1, operatorRisk: 0.2 },
        computeTimeMs: 5,
        timestamp: Date.now(),
        recommendation: 'Normal operation'
      },
      networkMetrics: {
        latency: 80,
        bandwidth: 10,
        packetLoss: 0.5,
        jitter: 10
      },
      trajectory: {
        speed: 60,
        heading: 45,
        predictedPath: [
          { lat: 22.3, lon: 114.2, timestamp: Date.now() },
          { lat: 22.301, lon: 114.201, timestamp: Date.now() + 1000 },
          { lat: 22.302, lon: 114.202, timestamp: Date.now() + 2000 }
        ]
      },
      history: {
        recentDisengagements: [],
        avgTimeBetweenDisengagements: 300
      }
    }

    if (scenario === 'risky') {
      baseState.autonomyConfidence = 0.55
      baseState.riskScore.overallScore = 0.65
      baseState.riskScore.urgencyLevel = 'high'
      baseState.networkMetrics.latency = 180
      baseState.history.avgTimeBetweenDisengagements = 90
      baseState.history.recentDisengagements = [
        { timestamp: Date.now() - 120000, reason: 'Lane deviation' },
        { timestamp: Date.now() - 60000, reason: 'Traffic complexity' }
      ]
    } else if (scenario === 'critical') {
      baseState.autonomyConfidence = 0.35
      baseState.currentMode = 'supervised'
      baseState.riskScore.overallScore = 0.85
      baseState.riskScore.urgencyLevel = 'critical'
      baseState.networkMetrics.latency = 280
      baseState.networkMetrics.packetLoss = 4
      baseState.riskFeatures.emergencyBrakeCount = 2
      baseState.history.avgTimeBetweenDisengagements = 45
      baseState.history.recentDisengagements = [
        { timestamp: Date.now() - 90000, reason: 'Sensor failure' },
        { timestamp: Date.now() - 45000, reason: 'Emergency brake' },
        { timestamp: Date.now() - 20000, reason: 'System error' }
      ]
    }

    return baseState
  }
}

// Singleton instance
let predictiveHandoverService: PredictiveHandoverService | null = null

export function getPredictiveHandoverService(): PredictiveHandoverService {
  if (!predictiveHandoverService) {
    predictiveHandoverService = new PredictiveHandoverService()
  }
  return predictiveHandoverService
}

export { PredictiveHandoverService }
