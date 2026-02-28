import logger from '@/utils/logger'
/**
 * FSM-Pilot V2.0 - Bipartite Matching Service
 *
 * Geographic bipartite graph matching algorithm
 * Optimizes vehicle-operator assignment based on:
 * - Minimum cognitive switching cost
 * - Skill matching
 * - Geographic proximity
 * - Load balancing
 *
 * @author Li Yixiang
 * @institution City University of Hong Kong
 */

import type { RiskScore } from './riskScoringService'

export interface GeographicLocation {
  latitude: number
  longitude: number
  region: string           // Geographic shard ID
  city?: string
  district?: string
}

export interface OperatorSkills {
  experience: number       // Years of experience
  scenarioExpertise: {
    highway: number        // 0-1 proficiency
    urban: number
    residential: number
    parking: number
    rural: number
  }
  weatherExpertise: {
    clear: number
    rainy: number
    foggy: number
    snowy: number
  }
  certifications: string[]
  handledVehicles: number  // Total vehicles handled
  successRate: number      // Successful handovers (0-1)
  averageResponseTime: number  // Average response time in seconds
}

export interface Operator {
  id: string
  name: string
  status: 'idle' | 'busy' | 'offline'
  skills: OperatorSkills
  currentLoad: number      // 0-1, percentage of capacity
  maxConcurrentVehicles: number
  maxCapacity: number      // Same as maxConcurrentVehicles for compatibility
  assignedVehicles: string[]
  location: GeographicLocation
  fatigue: number          // 0-1
  lastSwitchTime: number   // Timestamp of last vehicle switch
  preferredRegions: string[]
  performance: {
    avgResponseTime: number
    avgHandoverTime: number
    recentErrors: number
  }
}

export interface VehicleRequest {
  vehicleId: string
  location: GeographicLocation
  riskScore: RiskScore
  scenario: 'highway' | 'urban' | 'residential' | 'parking' | 'rural'
  weather: 'clear' | 'rainy' | 'foggy' | 'snowy'
  requiredSkills: string[]
  priority: number         // From risk score
  timestamp: number
}

export interface MatchResult {
  vehicleId: string
  operatorId: string
  matchScore: number       // 0-1, higher is better
  breakdown: {
    skillMatch: number
    proximityScore: number
    loadBalance: number
    switchingCost: number
    urgencyBonus: number
  }
  estimatedHandoverTime: number  // ms
  recommendation: string
}

/**
 * Geographic sharding for efficient matching
 */
class GeographicShard {
  id: string
  bounds: {
    minLat: number
    maxLat: number
    minLon: number
    maxLon: number
  }
  operators: Set<string> = new Set()
  vehicles: Set<string> = new Set()

  constructor(id: string, bounds: typeof GeographicShard.prototype.bounds) {
    this.id = id
    this.bounds = bounds
  }

  contains(location: GeographicLocation): boolean {
    return (
      location.latitude >= this.bounds.minLat &&
      location.latitude <= this.bounds.maxLat &&
      location.longitude >= this.bounds.minLon &&
      location.longitude <= this.bounds.maxLon
    )
  }

  getNeighborShards(allShards: GeographicShard[]): GeographicShard[] {
    // Return adjacent shards for expanding search
    return allShards.filter(shard => {
      if (shard.id === this.id) return false
      // Check if shards are adjacent (simplified)
      const latOverlap =
        Math.abs(shard.bounds.minLat - this.bounds.maxLat) < 0.5 ||
        Math.abs(shard.bounds.maxLat - this.bounds.minLat) < 0.5
      const lonOverlap =
        Math.abs(shard.bounds.minLon - this.bounds.maxLon) < 0.5 ||
        Math.abs(shard.bounds.maxLon - this.bounds.minLon) < 0.5
      return latOverlap && lonOverlap
    })
  }
}

/**
 * Bipartite Matching Service
 */
class BipartiteMatchingService {
  private shards: GeographicShard[] = []
  private operators: Map<string, Operator> = new Map()
  private pendingRequests: Map<string, VehicleRequest> = new Map()

  // Weights for matching algorithm
  private weights = {
    skillMatch: 0.35,
    proximity: 0.25,
    loadBalance: 0.20,
    switchingCost: 0.15,
    urgencyBonus: 0.05
  }

  constructor() {
    this.initializeShards()
  }

  /**
   * Initialize geographic shards (Hong Kong example)
   * Can be configured for different regions
   */
  private initializeShards() {
    // Hong Kong region divided into 9 shards (3x3 grid)
    const hkBounds = {
      minLat: 22.15,
      maxLat: 22.57,
      minLon: 113.83,
      maxLon: 114.41
    }

    const latStep = (hkBounds.maxLat - hkBounds.minLat) / 3
    const lonStep = (hkBounds.maxLon - hkBounds.minLon) / 3

    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        const shard = new GeographicShard(`HK-${i}-${j}`, {
          minLat: hkBounds.minLat + i * latStep,
          maxLat: hkBounds.minLat + (i + 1) * latStep,
          minLon: hkBounds.minLon + j * lonStep,
          maxLon: hkBounds.minLon + (j + 1) * lonStep
        })
        this.shards.push(shard)
      }
    }
  }

  /**
   * Register operator in the system
   */
  registerOperator(operator: Operator) {
    this.operators.set(operator.id, operator)

    // Add to geographic shard
    const shard = this.findShard(operator.location)
    if (shard) {
      shard.operators.add(operator.id)
    }
  }

  /**
   * Find optimal operator for vehicle request
   * Uses bipartite matching with minimum cognitive switching cost
   */
  findOptimalMatch(request: VehicleRequest): MatchResult | null {
    const startTime = performance.now()

    // Find vehicle's shard
    const vehicleShard = this.findShard(request.location)
    if (!vehicleShard) {
      logger.warn('Vehicle location outside known shards', 'vehicle')
      return null
    }

    // Get candidate operators (same shard + neighbors)
    const candidateOperators = this.getCandidateOperators(vehicleShard)

    if (candidateOperators.length === 0) {
      logger.warn('No available operators in region', 'vehicle')
      return null
    }

    // Calculate match scores for all candidates
    const matches = candidateOperators.map(operator =>
      this.calculateMatchScore(request, operator)
    )

    // Sort by match score (descending)
    matches.sort((a, b) => b.matchScore - a.matchScore)

    const bestMatch = matches[0]
    bestMatch.estimatedHandoverTime = performance.now() - startTime

    return bestMatch
  }

  /**
   * Batch matching for multiple requests (Hungarian algorithm inspired)
   */
  batchMatch(requests: VehicleRequest[]): Map<string, MatchResult> {
    const results = new Map<string, MatchResult>()

    // Sort requests by priority (risk score)
    const sortedRequests = [...requests].sort((a, b) =>
      b.riskScore.overallScore - a.riskScore.overallScore
    )

    // Greedy assignment with preference for high-priority requests
    const assignedOperators = new Set<string>()

    for (const request of sortedRequests) {
      const match = this.findOptimalMatch(request)

      if (match) {
        const operator = this.operators.get(match.operatorId)

        // Check if operator can take more vehicles
        if (operator &&
            operator.status !== 'offline' &&
            operator.assignedVehicles.length < operator.maxConcurrentVehicles &&
            !assignedOperators.has(match.operatorId)) {

          results.set(request.vehicleId, match)
          assignedOperators.add(match.operatorId)

          // Update operator state (simulated)
          operator.assignedVehicles.push(request.vehicleId)
          operator.currentLoad = operator.assignedVehicles.length / operator.maxConcurrentVehicles
        }
      }
    }

    return results
  }

  private findShard(location: GeographicLocation): GeographicShard | null {
    return this.shards.find(shard => shard.contains(location)) || null
  }

  private getCandidateOperators(shard: GeographicShard): Operator[] {
    const candidates: Operator[] = []

    // Operators in same shard
    for (const operatorId of shard.operators) {
      const operator = this.operators.get(operatorId)
      if (operator && operator.status !== 'offline') {
        candidates.push(operator)
      }
    }

    // If not enough operators, expand to neighbor shards
    if (candidates.length < 3) {
      const neighbors = shard.getNeighborShards(this.shards)
      for (const neighbor of neighbors) {
        for (const operatorId of neighbor.operators) {
          const operator = this.operators.get(operatorId)
          if (operator && operator.status !== 'offline' && !candidates.includes(operator)) {
            candidates.push(operator)
          }
        }
      }
    }

    return candidates
  }

  private calculateMatchScore(request: VehicleRequest, operator: Operator): MatchResult {
    const skillMatch = this.calculateSkillMatch(request, operator)
    const proximityScore = this.calculateProximityScore(request.location, operator.location)
    const loadBalance = this.calculateLoadBalance(operator)
    const switchingCost = this.calculateSwitchingCost(operator)
    const urgencyBonus = this.calculateUrgencyBonus(request.riskScore)

    // Weighted sum
    const matchScore =
      skillMatch * this.weights.skillMatch +
      proximityScore * this.weights.proximity +
      loadBalance * this.weights.loadBalance +
      switchingCost * this.weights.switchingCost +
      urgencyBonus * this.weights.urgencyBonus

    const recommendation = this.generateMatchRecommendation(
      matchScore,
      skillMatch,
      operator
    )

    return {
      vehicleId: request.vehicleId,
      operatorId: operator.id,
      matchScore,
      breakdown: {
        skillMatch,
        proximityScore,
        loadBalance,
        switchingCost,
        urgencyBonus
      },
      estimatedHandoverTime: 0,  // Will be set by caller
      recommendation
    }
  }

  private calculateSkillMatch(request: VehicleRequest, operator: Operator): number {
    let score = 0

    // Scenario expertise
    const scenarioScore = operator.skills.scenarioExpertise[request.scenario]
    score += scenarioScore * 0.4

    // Weather expertise
    const weatherScore = operator.skills.weatherExpertise[request.weather]
    score += weatherScore * 0.2

    // Experience factor
    const expScore = Math.min(1, operator.skills.experience / 10)  // Cap at 10 years
    score += expScore * 0.2

    // Success rate
    score += operator.skills.successRate * 0.2

    return Math.min(1, score)
  }

  private calculateProximityScore(vehicleLocation: GeographicLocation, operatorLocation: GeographicLocation): number {
    const distance = this.calculateDistance(vehicleLocation, operatorLocation)

    // Exponential decay with distance
    // 0 km = 1.0, 10 km = 0.5, 50 km = 0.1
    return Math.exp(-distance / 20)
  }

  private calculateDistance(loc1: GeographicLocation, loc2: GeographicLocation): number {
    // Haversine formula (simplified)
    const R = 6371  // Earth radius in km
    const dLat = (loc2.latitude - loc1.latitude) * Math.PI / 180
    const dLon = (loc2.longitude - loc1.longitude) * Math.PI / 180

    const a =
      Math.sin(dLat / 2) * Math.sin(dLat / 2) +
      Math.cos(loc1.latitude * Math.PI / 180) *
      Math.cos(loc2.latitude * Math.PI / 180) *
      Math.sin(dLon / 2) * Math.sin(dLon / 2)

    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a))
    return R * c
  }

  private calculateLoadBalance(operator: Operator): number {
    // Prefer operators with lower load
    // 0% load = 1.0, 50% load = 0.5, 100% load = 0.0
    return 1 - operator.currentLoad
  }

  private calculateSwitchingCost(operator: Operator): number {
    // Minimum cognitive switching cost
    // Prefer operators who haven't switched recently
    const now = Date.now()
    const timeSinceSwitch = now - operator.lastSwitchTime
    const minutesSinceSwitch = timeSinceSwitch / (60 * 1000)

    // Score increases with time since last switch
    // 0 min = 0.0, 5 min = 0.5, 15+ min = 1.0
    return Math.min(1, minutesSinceSwitch / 15)
  }

  private calculateUrgencyBonus(riskScore: RiskScore): number {
    // Bonus for matching urgent requests quickly
    return riskScore.overallScore
  }

  private generateMatchRecommendation(
    matchScore: number,
    skillMatch: number,
    operator: Operator
  ): string {
    if (matchScore >= 0.8 && skillMatch >= 0.8) {
      return `Excellent match. Operator ${operator.name} has optimal skills and availability.`
    }
    if (matchScore >= 0.6) {
      return `Good match. Operator ${operator.name} can handle this situation effectively.`
    }
    if (matchScore >= 0.4) {
      return `Acceptable match. Consider monitoring closely.`
    }
    return `Suboptimal match. No better operators available in region.`
  }

  /**
   * Get all available operators
   */
  getOperators(): Operator[] {
    return Array.from(this.operators.values())
  }

  /**
   * Update operator status
   */
  updateOperatorStatus(operatorId: string, status: Operator['status']) {
    const operator = this.operators.get(operatorId)
    if (operator) {
      operator.status = status
    }
  }

  /**
   * Generate mock operators for testing
   */
  generateMockOperators(count: number): Operator[] {
    const operators: Operator[] = []
    const names = ['Alice', 'Bob', 'Charlie', 'Diana', 'Eve', 'Frank', 'Grace', 'Henry']

    for (let i = 0; i < count; i++) {
      const shardIndex = i % this.shards.length
      const shard = this.shards[shardIndex]

      // Random location within shard
      const lat = shard.bounds.minLat + Math.random() * (shard.bounds.maxLat - shard.bounds.minLat)
      const lon = shard.bounds.minLon + Math.random() * (shard.bounds.maxLon - shard.bounds.minLon)

      const operator: Operator = {
        id: `OP-${String(i + 1).padStart(3, '0')}`,
        name: names[i % names.length] + ` ${Math.floor(i / names.length) + 1}`,
        status: Math.random() > 0.2 ? 'idle' : 'busy',
        skills: {
          experience: 2 + Math.random() * 8,
          scenarioExpertise: {
            highway: 0.5 + Math.random() * 0.5,
            urban: 0.5 + Math.random() * 0.5,
            residential: 0.5 + Math.random() * 0.5,
            parking: 0.5 + Math.random() * 0.5,
            rural: 0.5 + Math.random() * 0.5
          },
          weatherExpertise: {
            clear: 0.8 + Math.random() * 0.2,
            rainy: 0.5 + Math.random() * 0.5,
            snowy: 0.3 + Math.random() * 0.5,
            foggy: 0.4 + Math.random() * 0.5
          },
          certifications: ['Basic', Math.random() > 0.5 ? 'Advanced' : ''].filter(Boolean),
          handledVehicles: Math.floor(50 + Math.random() * 500),
          successRate: 0.85 + Math.random() * 0.15,
          averageResponseTime: 2 + Math.random() * 3
        },
        currentLoad: Math.random() * 0.7,
        maxConcurrentVehicles: 3,
        maxCapacity: 3,  // Same as maxConcurrentVehicles
        assignedVehicles: [],
        location: {
          latitude: lat,
          longitude: lon,
          region: shard.id
        },
        fatigue: Math.random() * 0.5,
        lastSwitchTime: Date.now() - Math.random() * 30 * 60 * 1000,
        preferredRegions: [shard.id],
        performance: {
          avgResponseTime: 2 + Math.random() * 3,
          avgHandoverTime: 5 + Math.random() * 10,
          recentErrors: Math.floor(Math.random() * 3)
        }
      }

      this.registerOperator(operator)
      operators.push(operator)
    }

    return operators
  }
}

// Singleton instance
let bipartiteMatchingService: BipartiteMatchingService | null = null

export function getBipartiteMatchingService(): BipartiteMatchingService {
  if (!bipartiteMatchingService) {
    bipartiteMatchingService = new BipartiteMatchingService()
  }
  return bipartiteMatchingService
}

export { BipartiteMatchingService }
