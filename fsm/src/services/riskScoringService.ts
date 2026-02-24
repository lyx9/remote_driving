/**
 * FSM-Pilot V2.0 - Risk Scoring Service
 *
 * XGBoost-based vehicle risk assessment for intelligent dispatch
 * Quantifies task urgency in milliseconds for real-time decision making
 *
 * @author Li Yixiang
 * @institution City University of Hong Kong
 */

export interface VehicleRiskFeatures {
  // Vehicle state
  speed: number                    // Current speed (km/h)
  speedVariance: number            // Speed stability over last 10s
  acceleration: number             // Current acceleration (m/s²)

  // Environment complexity
  locationComplexity: number       // Urban density score (0-1)
  trafficDensity: number          // Surrounding vehicles count
  weatherSeverity: number         // Weather condition score (0-1)
  roadType: 'highway' | 'urban' | 'residential' | 'parking'

  // Time factors
  timeOfDay: number               // Hour (0-23)
  isDaylight: boolean

  // System health
  systemHealth: number            // Overall system status (0-1)
  sensorStatus: number            // Sensor health (0-1)
  networkLatency: number          // Current latency (ms)
  networkQuality: number          // Network stability (0-1)

  // Driving behavior
  trajectoryDeviation: number     // Deviation from planned path (m)
  emergencyBrakeCount: number     // Emergency brakes in last 5 min
  laneChangeFrequency: number     // Lane changes per minute
  steeringVariance: number        // Steering wheel variance

  // Operator factors
  operatorExperience: number      // Experience score (0-1)
  operatorFatigue: number         // Fatigue estimate (0-1)
  operatorWorkload: number        // Current workload (0-1)

  // Historical data
  recentIncidents: number         // Incidents in last hour
  avgResponseTime: number         // Historical response time (s)
}

export interface RiskScore {
  overallScore: number            // Final risk score (0-1, higher = more urgent)
  urgencyLevel: 'critical' | 'high' | 'medium' | 'low'
  components: {
    vehicleRisk: number
    environmentRisk: number
    systemRisk: number
    operatorRisk: number
  }
  computeTimeMs: number
  timestamp: number
  recommendation: string
}

// Simplified XGBoost decision tree node
interface TreeNode {
  feature?: keyof VehicleRiskFeatures
  threshold?: number
  value?: number
  left?: TreeNode
  right?: TreeNode
}

/**
 * XGBoost-inspired ensemble model
 * Simplified for browser execution with <10ms latency requirement
 */
class XGBoostEnsemble {
  private trees: TreeNode[] = []
  private learningRate = 0.1

  constructor() {
    this.initializeTrees()
  }

  private initializeTrees() {
    // Tree 1: System health focused
    this.trees.push({
      feature: 'systemHealth',
      threshold: 0.7,
      left: {
        feature: 'sensorStatus',
        threshold: 0.5,
        left: { value: 0.9 },  // Critical: poor system + sensor
        right: { value: 0.7 }   // High: poor system only
      },
      right: {
        feature: 'networkLatency',
        threshold: 200,
        left: { value: 0.3 },   // Low: good system + network
        right: { value: 0.5 }   // Medium: good system, poor network
      }
    })

    // Tree 2: Emergency behavior focused
    this.trees.push({
      feature: 'emergencyBrakeCount',
      threshold: 2,
      left: {
        feature: 'trajectoryDeviation',
        threshold: 2.0,
        left: { value: 0.2 },   // Low: no emergency, on track
        right: { value: 0.5 }   // Medium: deviation without emergency
      },
      right: {
        feature: 'speed',
        threshold: 60,
        left: { value: 0.7 },   // High: emergency at low speed
        right: { value: 0.95 }  // Critical: emergency at high speed
      }
    })

    // Tree 3: Environment complexity
    this.trees.push({
      feature: 'locationComplexity',
      threshold: 0.7,
      left: {
        feature: 'trafficDensity',
        threshold: 5,
        left: { value: 0.1 },   // Low: simple area, low traffic
        right: { value: 0.4 }   // Medium: simple area, high traffic
      },
      right: {
        feature: 'weatherSeverity',
        threshold: 0.5,
        left: { value: 0.6 },   // High: complex area, good weather
        right: { value: 0.85 }  // Critical: complex area, bad weather
      }
    })

    // Tree 4: Speed and control
    this.trees.push({
      feature: 'speedVariance',
      threshold: 15,
      left: {
        feature: 'steeringVariance',
        threshold: 10,
        left: { value: 0.15 },  // Low: stable speed + steering
        right: { value: 0.45 }  // Medium: stable speed, erratic steering
      },
      right: {
        feature: 'speed',
        threshold: 80,
        left: { value: 0.55 },  // High: unstable at medium speed
        right: { value: 0.8 }   // Critical: unstable at high speed
      }
    })

    // Tree 5: Operator state
    this.trees.push({
      feature: 'operatorFatigue',
      threshold: 0.6,
      left: {
        feature: 'operatorWorkload',
        threshold: 0.7,
        left: { value: 0.2 },   // Low: alert + light workload
        right: { value: 0.5 }   // Medium: alert + heavy workload
      },
      right: {
        feature: 'operatorExperience',
        threshold: 0.5,
        left: { value: 0.8 },   // Critical: fatigued + inexperienced
        right: { value: 0.65 }  // High: fatigued + experienced
      }
    })

    // Tree 6: Time and conditions
    this.trees.push({
      feature: 'isDaylight',
      threshold: 0.5,
      left: {  // Night
        feature: 'weatherSeverity',
        threshold: 0.3,
        left: { value: 0.4 },   // Medium: night, good weather
        right: { value: 0.75 }  // High: night, bad weather
      },
      right: {  // Day
        feature: 'trafficDensity',
        threshold: 10,
        left: { value: 0.2 },   // Low: day, low traffic
        right: { value: 0.5 }   // Medium: day, high traffic
      }
    })

    // Tree 7: Network quality
    this.trees.push({
      feature: 'networkQuality',
      threshold: 0.6,
      left: {
        feature: 'networkLatency',
        threshold: 300,
        left: { value: 0.6 },   // High: poor network + high latency
        right: { value: 0.85 }  // Critical: very poor network
      },
      right: {
        feature: 'networkLatency',
        threshold: 100,
        left: { value: 0.1 },   // Low: good network + low latency
        right: { value: 0.35 }  // Medium: good network, moderate latency
      }
    })

    // Tree 8: Historical risk
    this.trees.push({
      feature: 'recentIncidents',
      threshold: 1,
      left: {
        feature: 'avgResponseTime',
        threshold: 3,
        left: { value: 0.15 },  // Low: no incidents, fast response
        right: { value: 0.35 }  // Medium: no incidents, slow response
      },
      right: {
        feature: 'recentIncidents',
        threshold: 3,
        left: { value: 0.65 },  // High: few incidents
        right: { value: 0.9 }   // Critical: many incidents
      }
    })
  }

  private predictTree(tree: TreeNode, features: VehicleRiskFeatures): number {
    if (tree.value !== undefined) {
      return tree.value
    }

    if (!tree.feature || tree.threshold === undefined) {
      return 0.5  // Default
    }

    const featureValue = features[tree.feature]
    const numericValue = typeof featureValue === 'boolean' ? (featureValue ? 1 : 0) : Number(featureValue)

    if (numericValue < tree.threshold) {
      return tree.left ? this.predictTree(tree.left, features) : 0.5
    } else {
      return tree.right ? this.predictTree(tree.right, features) : 0.5
    }
  }

  predict(features: VehicleRiskFeatures): number {
    let score = 0.5  // Base score

    for (const tree of this.trees) {
      const treeScore = this.predictTree(tree, features)
      score += this.learningRate * treeScore
    }

    // Normalize to [0, 1]
    return Math.max(0, Math.min(1, score))
  }
}

/**
 * Risk Scoring Service
 */
class RiskScoringService {
  private model: XGBoostEnsemble

  constructor() {
    this.model = new XGBoostEnsemble()
  }

  /**
   * Calculate comprehensive risk score
   * Target: <10ms computation time
   */
  calculateRiskScore(features: VehicleRiskFeatures): RiskScore {
    const startTime = performance.now()

    // Component scores
    const vehicleRisk = this.calculateVehicleRisk(features)
    const environmentRisk = this.calculateEnvironmentRisk(features)
    const systemRisk = this.calculateSystemRisk(features)
    const operatorRisk = this.calculateOperatorRisk(features)

    // XGBoost ensemble prediction
    const modelScore = this.model.predict(features)

    // Weighted combination (model has highest weight)
    const overallScore =
      modelScore * 0.5 +
      vehicleRisk * 0.2 +
      environmentRisk * 0.15 +
      systemRisk * 0.1 +
      operatorRisk * 0.05

    const urgencyLevel = this.getUrgencyLevel(overallScore)
    const recommendation = this.generateRecommendation(overallScore, features)

    const computeTimeMs = performance.now() - startTime

    return {
      overallScore,
      urgencyLevel,
      components: {
        vehicleRisk,
        environmentRisk,
        systemRisk,
        operatorRisk
      },
      computeTimeMs,
      timestamp: Date.now(),
      recommendation
    }
  }

  private calculateVehicleRisk(features: VehicleRiskFeatures): number {
    let risk = 0

    // Speed instability
    if (features.speedVariance > 20) risk += 0.3
    else if (features.speedVariance > 10) risk += 0.15

    // High speed
    if (features.speed > 100) risk += 0.25
    else if (features.speed > 80) risk += 0.15

    // Emergency braking
    risk += Math.min(0.3, features.emergencyBrakeCount * 0.1)

    // Trajectory deviation
    if (features.trajectoryDeviation > 3) risk += 0.2
    else if (features.trajectoryDeviation > 1.5) risk += 0.1

    // Steering instability
    if (features.steeringVariance > 15) risk += 0.15

    return Math.min(1, risk)
  }

  private calculateEnvironmentRisk(features: VehicleRiskFeatures): number {
    let risk = 0

    // Location complexity
    risk += features.locationComplexity * 0.3

    // Traffic density
    if (features.trafficDensity > 15) risk += 0.3
    else if (features.trafficDensity > 8) risk += 0.15

    // Weather
    risk += features.weatherSeverity * 0.25

    // Night time
    if (!features.isDaylight) risk += 0.15

    // Road type
    const roadRisk = {
      highway: 0.1,
      urban: 0.3,
      residential: 0.2,
      parking: 0.05
    }
    risk += roadRisk[features.roadType]

    return Math.min(1, risk)
  }

  private calculateSystemRisk(features: VehicleRiskFeatures): number {
    let risk = 0

    // System health
    risk += (1 - features.systemHealth) * 0.4

    // Sensor status
    risk += (1 - features.sensorStatus) * 0.3

    // Network quality
    risk += (1 - features.networkQuality) * 0.2

    // Network latency
    if (features.networkLatency > 300) risk += 0.3
    else if (features.networkLatency > 200) risk += 0.2
    else if (features.networkLatency > 100) risk += 0.1

    return Math.min(1, risk)
  }

  private calculateOperatorRisk(features: VehicleRiskFeatures): number {
    let risk = 0

    // Fatigue
    risk += features.operatorFatigue * 0.4

    // Inexperience
    risk += (1 - features.operatorExperience) * 0.2

    // Workload
    risk += features.operatorWorkload * 0.25

    // Recent incidents
    risk += Math.min(0.3, features.recentIncidents * 0.1)

    // Response time
    if (features.avgResponseTime > 5) risk += 0.15
    else if (features.avgResponseTime > 3) risk += 0.1

    return Math.min(1, risk)
  }

  private getUrgencyLevel(score: number): 'critical' | 'high' | 'medium' | 'low' {
    if (score >= 0.8) return 'critical'
    if (score >= 0.6) return 'high'
    if (score >= 0.4) return 'medium'
    return 'low'
  }

  private generateRecommendation(score: number, features: VehicleRiskFeatures): string {
    if (score >= 0.8) {
      if (features.systemHealth < 0.5) {
        return 'CRITICAL: System failure detected. Immediate intervention required.'
      }
      if (features.emergencyBrakeCount > 2) {
        return 'CRITICAL: Multiple emergency brakes. Take over control immediately.'
      }
      return 'CRITICAL: High-risk situation. Priority dispatch to experienced operator.'
    }

    if (score >= 0.6) {
      if (features.networkLatency > 200) {
        return 'HIGH: Poor network quality. Consider adaptive control mode.'
      }
      if (features.weatherSeverity > 0.7) {
        return 'HIGH: Severe weather conditions. Enhanced monitoring required.'
      }
      return 'HIGH: Elevated risk. Dispatch to available operator soon.'
    }

    if (score >= 0.4) {
      return 'MEDIUM: Moderate risk. Queue for next available operator.'
    }

    return 'LOW: Routine situation. Standard queue processing.'
  }

  /**
   * Generate mock features for testing
   */
  generateMockFeatures(scenario: 'critical' | 'high' | 'medium' | 'low' | 'random'): VehicleRiskFeatures {
    if (scenario === 'random') {
      scenario = ['critical', 'high', 'medium', 'low'][Math.floor(Math.random() * 4)] as any
    }

    const baseFeatures: VehicleRiskFeatures = {
      speed: 50,
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
    }

    switch (scenario) {
      case 'critical':
        return {
          ...baseFeatures,
          speed: 95,
          speedVariance: 25,
          systemHealth: 0.4,
          sensorStatus: 0.3,
          networkLatency: 350,
          networkQuality: 0.3,
          emergencyBrakeCount: 4,
          trajectoryDeviation: 4.5,
          weatherSeverity: 0.8,
          locationComplexity: 0.9,
          trafficDensity: 18,
          operatorFatigue: 0.8,
          isDaylight: false,
          recentIncidents: 3
        }

      case 'high':
        return {
          ...baseFeatures,
          speed: 75,
          speedVariance: 18,
          systemHealth: 0.65,
          networkLatency: 220,
          networkQuality: 0.55,
          emergencyBrakeCount: 2,
          trajectoryDeviation: 2.5,
          weatherSeverity: 0.6,
          locationComplexity: 0.75,
          trafficDensity: 12,
          operatorFatigue: 0.6,
          recentIncidents: 1
        }

      case 'medium':
        return {
          ...baseFeatures,
          speed: 60,
          speedVariance: 12,
          systemHealth: 0.8,
          networkLatency: 150,
          trajectoryDeviation: 1.8,
          weatherSeverity: 0.4,
          locationComplexity: 0.6,
          trafficDensity: 8,
          operatorFatigue: 0.45
        }

      case 'low':
        return {
          ...baseFeatures,
          speed: 40,
          speedVariance: 3,
          systemHealth: 0.98,
          networkLatency: 50,
          networkQuality: 0.95,
          trajectoryDeviation: 0.3,
          weatherSeverity: 0.1,
          locationComplexity: 0.3,
          trafficDensity: 3,
          operatorFatigue: 0.2,
          roadType: 'highway'
        }

      default:
        return baseFeatures
    }
  }
}

// Singleton instance
let riskScoringService: RiskScoringService | null = null

export function getRiskScoringService(): RiskScoringService {
  if (!riskScoringService) {
    riskScoringService = new RiskScoringService()
  }
  return riskScoringService
}

export { RiskScoringService }
