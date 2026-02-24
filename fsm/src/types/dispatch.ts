/**
 * FSM-Pilot V2.0 - Dispatch Type Definitions
 *
 * Shared type definitions for dispatch system components
 *
 * @author Li Yixiang
 * @institution City University of Hong Kong
 */

// Geographic Location
export interface GeographicLocation {
  longitude: number
  latitude: number
  street?: string
  district?: string
  region?: string
}

// Risk Score
export interface RiskScore {
  overallScore: number
  urgencyLevel: 'critical' | 'high' | 'medium' | 'low'
  riskFactors?: string[]
}

// Vehicle Risk Features
export interface VehicleRiskFeatures {
  speed: number
  trafficDensity: number
  weatherCondition: number
  roadCondition: number
  timeOfDay: number
  driverBehavior: number
  vehicleCondition: number
  historicalIncidents: number
  proximityToObstacles: number
  laneChangeFrequency: number
  brakingFrequency: number
  accelerationVariance: number
  steeringStability: number
  signalCompliance: number
  pedestrianDensity: number
  intersectionComplexity: number
  visibilityLevel: number
  networkLatency: number
  systemHealthScore: number
  operatorExperience: number
  geographicRisk: number
}

// Control Mode
export type ControlMode = 'direct' | 'trajectory' | 'semantic' | 'full-autonomous' | 'monitored' | 'partial-autonomous' | 'manual'

// Vehicle Telemetry
export interface VehicleTelemetry {
  speed: number
  acceleration: number
  heading: number
  batteryLevel?: number
  systemStatus?: string
}

// Vehicle
export interface Vehicle {
  id: string
  location: GeographicLocation
  riskScore: RiskScore
  scenario: 'highway' | 'urban' | 'residential' | 'parking' | 'rural'
  weather: 'clear' | 'rainy' | 'foggy' | 'snowy'
  controlMode: ControlMode
  telemetry?: VehicleTelemetry
  timestamp: number
}

// Operator Skills
export interface OperatorSkills {
  scenarioExpertise: Record<string, number>
  successRate: number
  averageResponseTime: number
  experience: number
  weatherExpertise?: Record<string, number>
  certifications?: string[]
  handledVehicles?: number
}

// Operator
export interface Operator {
  id: string
  name: string
  status: 'idle' | 'busy' | 'offline'
  location: GeographicLocation
  skills: OperatorSkills
  maxConcurrentVehicles: number
  currentLoad: number
  maxCapacity: number
  assignedVehicles: string[]
  matchScore?: number
}

// Vehicle Request
export interface VehicleRequest {
  vehicleId: string
  location: GeographicLocation
  urgency: 'critical' | 'high' | 'medium' | 'low'
  scenario: string
  requiredSkills?: string[]
  estimatedDuration?: number
}

// Match Result
export interface MatchResult {
  vehicleId: string
  operatorId: string
  matchScore: number
  estimatedDistance: number
  estimatedResponseTime: number
  timestamp: number
}

// Prediction Result
export interface PredictionResult {
  vehicleId: string
  disengagementProbability: number
  predictedDisengagementTime: number
  recommendedOperatorId: string | null
  confidenceScore: number
  timestamp: number
}
