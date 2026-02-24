<!--
  Guardian Mobility v0.0 - Intelligent Dispatch Demo

  Demonstrates advanced dispatch system with:
  - XGBoost-based risk scoring
  - Geographic bipartite matching
  - Predictive handover
  - Adaptive control modes
  - 100 concurrent vehicles simulation

  @author Li Yixiang
  @institution City University of Hong Kong
-->
<template>
  <div class="intelligent-dispatch-demo">
    <NavBar />

    <!-- Header -->
    <div class="demo-header">
      <h1>🧠 Intelligent Dispatch System</h1>
      <p>Time-Space Prediction + Dynamic Triage Architecture</p>

      <div class="simulation-controls">
        <button
          @click="toggleSimulation"
          :class="['btn-primary', isRunning ? 'btn-stop' : 'btn-start']"
        >
          {{ isRunning ? '⏸ Pause' : '▶ Start' }} Simulation
        </button>

        <button @click="addEmergencyVehicle" class="btn-emergency">
          🚨 Add Emergency Vehicle
        </button>

        <button @click="addBatchVehicles" class="btn-secondary">
          📦 Add 10 Vehicles
        </button>

        <button @click="resetSimulation" class="btn-reset">
          🔄 Reset
        </button>
      </div>
    </div>

    <!-- Statistics Cards -->
    <div class="stats-grid">
      <div class="stat-card">
        <div class="stat-icon">🚗</div>
        <div class="stat-content">
          <div class="stat-value">{{ stats.totalVehicles }}</div>
          <div class="stat-label">Total Vehicles</div>
        </div>
      </div>

      <div class="stat-card">
        <div class="stat-icon">👥</div>
        <div class="stat-content">
          <div class="stat-value">{{ stats.activeOperators }}</div>
          <div class="stat-label">Active Operators</div>
        </div>
      </div>

      <div class="stat-card">
        <div class="stat-icon">⏱️</div>
        <div class="stat-content">
          <div class="stat-value">{{ stats.avgMatchTime.toFixed(1) }}ms</div>
          <div class="stat-label">Avg Match Time</div>
        </div>
      </div>

      <div class="stat-card">
        <div class="stat-icon">📊</div>
        <div class="stat-content">
          <div class="stat-value">{{ (stats.matchSuccessRate * 100).toFixed(1) }}%</div>
          <div class="stat-label">Match Success</div>
        </div>
      </div>

      <div class="stat-card">
        <div class="stat-icon">🔮</div>
        <div class="stat-content">
          <div class="stat-value">{{ stats.preConnectedVehicles }}</div>
          <div class="stat-label">Pre-Connected</div>
        </div>
      </div>

      <div class="stat-card">
        <div class="stat-icon">🎯</div>
        <div class="stat-content">
          <div class="stat-value">{{ stats.totalMatches }}</div>
          <div class="stat-label">Total Matches</div>
        </div>
      </div>
    </div>

    <!-- Main Content -->
    <div class="demo-content">
      <!-- Left Column: Vehicle Queue -->
      <div class="left-column">
        <div class="panel">
          <div class="panel-header">
            <h2>🚦 Vehicle Request Queue ({{ vehicleQueue.length }})</h2>
            <div class="queue-filter">
              <button
                v-for="level in ['all', 'critical', 'high', 'medium', 'low']"
                :key="level"
                @click="queueFilter = level"
                :class="['filter-btn', { active: queueFilter === level }]"
              >
                {{ level }}
              </button>
            </div>
          </div>

          <div class="vehicle-queue">
            <div
              v-for="vehicle in filteredVehicleQueue"
              :key="vehicle.id"
              :class="['vehicle-card', `urgency-${vehicle.riskScore.urgencyLevel}`]"
              @click="selectVehicle(vehicle.id)"
            >
              <div class="vehicle-header">
                <span class="vehicle-id">{{ vehicle.id }}</span>
                <span :class="['urgency-badge', vehicle.riskScore.urgencyLevel]">
                  {{ vehicle.riskScore.urgencyLevel.toUpperCase() }}
                </span>
              </div>

              <div class="vehicle-info">
                <div class="info-row">
                  <span class="label">Risk Score:</span>
                  <div class="risk-bar">
                    <div
                      class="risk-fill"
                      :style="{ width: `${vehicle.riskScore.overallScore * 100}%` }"
                    ></div>
                  </div>
                  <span class="value">{{ (vehicle.riskScore.overallScore * 100).toFixed(0) }}%</span>
                </div>

                <div class="info-row">
                  <span class="label">Location:</span>
                  <span class="value">{{ vehicle.location.region }}</span>
                </div>

                <div class="info-row">
                  <span class="label">Scenario:</span>
                  <span class="value">{{ vehicle.scenario }}</span>
                </div>

                <div class="info-row">
                  <span class="label">Control Mode:</span>
                  <span :class="['mode-badge', vehicle.controlMode]">
                    {{ vehicle.controlMode }}
                  </span>
                </div>

                <div class="info-row">
                  <span class="label">Prediction:</span>
                  <span class="value">
                    {{ vehicle.prediction ? (vehicle.prediction.disengagementProbability * 100).toFixed(0) : 0 }}%
                    in {{ vehicle.prediction ? (vehicle.prediction.predictedDisengagementTime / 1000).toFixed(0) : 0 }}s
                  </span>
                </div>
              </div>

              <div v-if="vehicle.match" class="match-info">
                <div class="match-header">✅ Matched</div>
                <div class="match-details">
                  <span>Operator: {{ vehicle.match.operatorId }}</span>
                  <span>Score: {{ (vehicle.match.matchScore * 100).toFixed(0) }}%</span>
                </div>
              </div>

              <!-- Takeover Action Button -->
              <button
                v-if="vehicle.riskScore.urgencyLevel === 'critical' || vehicle.riskScore.urgencyLevel === 'high'"
                @click.stop="openTakeoverDialog(vehicle.id)"
                :class="['takeover-btn', vehicle.riskScore.urgencyLevel]"
              >
                <span class="btn-icon">🚨</span>
                <span class="btn-text">Takeover Vehicle</span>
              </button>
            </div>

            <div v-if="filteredVehicleQueue.length === 0" class="empty-state">
              No vehicles in queue
            </div>
          </div>
        </div>
      </div>

      <!-- Center Column: Map & Visualization -->
      <div class="center-column">
        <!-- Amap Vehicle Location -->
        <div class="panel map-panel">
          <div class="panel-header">
            <h2>🗺️ Amap - Real-time Vehicle Locations</h2>
          </div>

          <div class="map-container">
            <AmapVehicleLocation
              :vehicles="amapVehicles"
              :operators="amapOperators"
              :center="mapCenter"
              :zoom="12"
            />
          </div>
        </div>

        <!-- AI Suggestion Panel -->
        <div class="panel ai-panel">
          <AISuggestionPanel
            :context="selectedVehicleContext"
            :auto-refresh="true"
            :refresh-interval="30000"
          />
        </div>

        <!-- Control Mode Distribution -->
        <div class="panel mode-panel">
          <div class="panel-header">
            <h2>🎮 Control Mode Distribution</h2>
          </div>

          <div class="mode-stats">
            <div class="mode-stat">
              <div class="mode-label">Direct Control</div>
              <div class="mode-bar">
                <div
                  class="mode-fill direct"
                  :style="{ width: `${modeDistribution.direct * 100}%` }"
                ></div>
              </div>
              <div class="mode-value">{{ (modeDistribution.direct * 100).toFixed(1) }}%</div>
            </div>

            <div class="mode-stat">
              <div class="mode-label">Trajectory Confirmation</div>
              <div class="mode-bar">
                <div
                  class="mode-fill trajectory"
                  :style="{ width: `${modeDistribution.trajectory * 100}%` }"
                ></div>
              </div>
              <div class="mode-value">{{ (modeDistribution.trajectory * 100).toFixed(1) }}%</div>
            </div>

            <div class="mode-stat">
              <div class="mode-label">Semantic Instruction</div>
              <div class="mode-bar">
                <div
                  class="mode-fill semantic"
                  :style="{ width: `${modeDistribution.semantic * 100}%` }"
                ></div>
              </div>
              <div class="mode-value">{{ (modeDistribution.semantic * 100).toFixed(1) }}%</div>
            </div>
          </div>
        </div>
      </div>

      <!-- Right Column: Operators & Metrics -->
      <div class="right-column">
        <div class="panel">
          <div class="panel-header">
            <h2>👥 Operator Status ({{ operators.length }})</h2>
          </div>

          <div class="operator-list">
            <div
              v-for="operator in operators.slice(0, 15)"
              :key="operator.id"
              :class="['operator-card', operator.status]"
            >
              <div class="operator-header">
                <span class="operator-name">{{ operator.name }}</span>
                <span :class="['status-badge', operator.status]">
                  {{ operator.status }}
                </span>
              </div>

              <div class="operator-info">
                <div class="info-item">
                  <span class="label">Load:</span>
                  <div class="load-bar">
                    <div
                      class="load-fill"
                      :style="{ width: `${operator.currentLoad * 100}%` }"
                    ></div>
                  </div>
                  <span class="value">{{ (operator.currentLoad * 100).toFixed(0) }}%</span>
                </div>

                <div class="info-item">
                  <span class="label">Vehicles:</span>
                  <span class="value">{{ operator.assignedVehicles.length }}/{{ operator.maxConcurrentVehicles }}</span>
                </div>

                <div class="info-item">
                  <span class="label">Experience:</span>
                  <span class="value">{{ operator.skills.experience.toFixed(1) }}y</span>
                </div>

                <div class="info-item">
                  <span class="label">Success:</span>
                  <span class="value">{{ (operator.skills.successRate * 100).toFixed(0) }}%</span>
                </div>
              </div>
            </div>
          </div>
        </div>

        <!-- Algorithm Metrics -->
        <div class="panel metrics-panel">
          <div class="panel-header">
            <h2>📈 Algorithm Performance</h2>
          </div>

          <div class="metrics-grid">
            <div class="metric-item">
              <div class="metric-label">XGBoost Scoring</div>
              <div class="metric-value">{{ algorithmMetrics.avgScoringTime.toFixed(2) }}ms</div>
            </div>

            <div class="metric-item">
              <div class="metric-label">Bipartite Matching</div>
              <div class="metric-value">{{ algorithmMetrics.avgMatchingTime.toFixed(2) }}ms</div>
            </div>

            <div class="metric-item">
              <div class="metric-label">Prediction Accuracy</div>
              <div class="metric-value">{{ (algorithmMetrics.predictionAccuracy * 100).toFixed(1) }}%</div>
            </div>

            <div class="metric-item">
              <div class="metric-label">Handover Latency</div>
              <div class="metric-value">{{ algorithmMetrics.avgHandoverLatency.toFixed(0) }}ms</div>
            </div>

            <div class="metric-item">
              <div class="metric-label">Mode Transitions</div>
              <div class="metric-value">{{ algorithmMetrics.totalModeTransitions }}</div>
            </div>

            <div class="metric-item">
              <div class="metric-label">Bandwidth Saved</div>
              <div class="metric-value">{{ algorithmMetrics.bandwidthSaved.toFixed(1) }} Mbps</div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- Takeover Confirmation Dialog -->
    <TakeoverConfirmDialog
      :show="showTakeoverDialog"
      :vehicle="takeoverVehicle"
      :operators="operators"
      @confirm="handleTakeoverConfirm"
      @cancel="handleTakeoverCancel"
    />

    <!-- Remote Assistant Panel -->
    <RemoteAssistantPanel
      v-if="remoteAssistantEnabled"
      :vehicle="vehicleQueue.find(v => v.id === selectedVehicleId) || null"
      :auto-refresh="true"
      :refresh-interval="5000"
      @apply-suggestion="handleApplySuggestion"
      @request-emergency="handleRequestEmergency"
      @request-route-optimization="handleRequestRouteOptimization"
    />
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'
import NavBar from './NavBar.vue'
import AmapVehicleLocation from './AmapVehicleLocation.vue'
import AISuggestionPanel from './AISuggestionPanel.vue'
import TakeoverConfirmDialog from './TakeoverConfirmDialog.vue'
import RemoteAssistantPanel from './RemoteAssistantPanel.vue'
import { getRiskScoringService } from '@/services/riskScoringService'
import type { RiskScore, VehicleRiskFeatures } from '@/services/riskScoringService'
import { getBipartiteMatchingService } from '@/services/bipartiteMatchingService'
import type { Operator, VehicleRequest, MatchResult, GeographicLocation } from '@/services/bipartiteMatchingService'
import { getPredictiveHandoverService } from '@/services/predictiveHandoverService'
import type { PredictionResult } from '@/services/predictiveHandoverService'
import { getAdaptiveControlModeService } from '@/services/adaptiveControlModeService'
import type { ControlMode } from '@/services/adaptiveControlModeService'
import type { ScenarioContext } from '@/services/doubaoLLMService'

// Services
const riskScoring = getRiskScoringService()
const bipartiteMatching = getBipartiteMatchingService()
const predictiveHandover = getPredictiveHandoverService()
const adaptiveControl = getAdaptiveControlModeService()

// State
interface DemoVehicle {
  id: string
  location: GeographicLocation
  riskScore: RiskScore
  scenario: 'highway' | 'urban' | 'residential' | 'parking' | 'rural'
  weather: 'clear' | 'rainy' | 'foggy' | 'snowy'
  controlMode: ControlMode
  prediction: PredictionResult | null
  match: MatchResult | null
  timestamp: number
  telemetry?: {
    speed: number
    acceleration: number
    heading: number
  }
}

const isRunning = ref(false)
const vehicleQueue = ref<DemoVehicle[]>([])
const operators = ref<Operator[]>([])
const queueFilter = ref<string>('all')
const selectedVehicleId = ref<string | null>(null)
const mapCenter = ref({ longitude: 114.17, latitude: 22.32 })

// Takeover Dialog State
const showTakeoverDialog = ref(false)
const takeoverVehicle = ref<DemoVehicle | null>(null)

// Remote Assistant State
const remoteAssistantEnabled = ref(true)

let simulationInterval: number | null = null
let nextVehicleId = 1

// Statistics
const stats = ref({
  totalVehicles: 0,
  activeOperators: 0,
  avgMatchTime: 0,
  matchSuccessRate: 1.0,
  preConnectedVehicles: 0,
  totalMatches: 0
})

const algorithmMetrics = ref({
  avgScoringTime: 0,
  avgMatchingTime: 0,
  predictionAccuracy: 0.85,
  avgHandoverLatency: 0,
  totalModeTransitions: 0,
  bandwidthSaved: 0
})

const modeDistribution = ref({
  direct: 0.33,
  trajectory: 0.5,
  semantic: 0.17
})

// Computed
const filteredVehicleQueue = computed(() => {
  if (queueFilter.value === 'all') {
    return vehicleQueue.value
  }
  return vehicleQueue.value.filter(v => v.riskScore.urgencyLevel === queueFilter.value)
})

// Amap vehicles data
const amapVehicles = computed(() => {
  return vehicleQueue.value.map(v => ({
    id: v.id,
    longitude: v.location.longitude,
    latitude: v.location.latitude,
    urgency: v.riskScore.urgencyLevel,
    label: v.id
  }))
})

// Amap operators data
const amapOperators = computed(() => {
  return operators.value
    .filter(o => o.status !== 'offline')  // Filter out offline operators
    .map(o => ({
      id: o.id,
      longitude: o.location.longitude,
      latitude: o.location.latitude,
      name: o.name,
      status: o.status as 'idle' | 'busy'
    }))
})

// Selected vehicle context for AI analysis
const selectedVehicleContext = computed<ScenarioContext | undefined>(() => {
  const selectedVehicle = vehicleQueue.value.find(v => v.id === selectedVehicleId.value)
  if (!selectedVehicle) {
    // Auto-select highest risk vehicle
    const sortedVehicles = [...vehicleQueue.value].sort((a, b) =>
      b.riskScore.overallScore - a.riskScore.overallScore
    )
    const highestRisk = sortedVehicles[0]
    if (!highestRisk) return undefined

    return {
      vehicleId: highestRisk.id,
      location: `${highestRisk.location.region} (${highestRisk.location.latitude.toFixed(4)}, ${highestRisk.location.longitude.toFixed(4)})`,
      scenario: highestRisk.scenario,
      weather: highestRisk.weather,
      riskScore: highestRisk.riskScore,
      riskFeatures: generateRiskFeatures(highestRisk.scenario),
      controlMode: highestRisk.controlMode,
      speed: generateRiskFeatures(highestRisk.scenario).speed,
      trafficDensity: generateRiskFeatures(highestRisk.scenario).trafficDensity
    }
  }

  return {
    vehicleId: selectedVehicle.id,
    location: `${selectedVehicle.location.region} (${selectedVehicle.location.latitude.toFixed(4)}, ${selectedVehicle.location.longitude.toFixed(4)})`,
    scenario: selectedVehicle.scenario,
    weather: selectedVehicle.weather,
    riskScore: selectedVehicle.riskScore,
    riskFeatures: generateRiskFeatures(selectedVehicle.scenario),
    controlMode: selectedVehicle.controlMode,
    speed: generateRiskFeatures(selectedVehicle.scenario).speed,
    trafficDensity: generateRiskFeatures(selectedVehicle.scenario).trafficDensity
  }
})

// Methods
function toggleSimulation() {
  isRunning.value = !isRunning.value

  if (isRunning.value) {
    startSimulation()
  } else {
    stopSimulation()
  }
}

function startSimulation() {
  console.log('[Demo] Starting simulation')

  // Initialize operators
  if (operators.value.length === 0) {
    operators.value = bipartiteMatching.generateMockOperators(20)
  }

  // Initialize predictive handover
  predictiveHandover.startMonitoring(1000)

  // Add initial vehicles
  for (let i = 0; i < 30; i++) {
    addRandomVehicle()
  }

  // Start simulation loop
  simulationInterval = window.setInterval(() => {
    updateSimulation()
  }, 2000)
}

function stopSimulation() {
  console.log('[Demo] Stopping simulation')

  if (simulationInterval) {
    clearInterval(simulationInterval)
    simulationInterval = null
  }

  predictiveHandover.stopMonitoring()
}

function updateSimulation() {
  // Randomly add new vehicles
  if (Math.random() < 0.3 && vehicleQueue.value.length < 100) {
    addRandomVehicle()
  }

  // Process existing vehicles
  for (const vehicle of vehicleQueue.value) {
    // Update risk score
    const features = generateRiskFeatures(vehicle.scenario)
    vehicle.riskScore = riskScoring.calculateRiskScore(features)

    // Update prediction
    const vehicleState = predictiveHandover.generateMockVehicleState(
      vehicle.id,
      vehicle.riskScore.urgencyLevel === 'critical' ? 'critical' :
      vehicle.riskScore.urgencyLevel === 'high' ? 'risky' : 'stable'
    )
    predictiveHandover.updateVehicleState(vehicleState)
    vehicle.prediction = predictiveHandover.getPrediction(vehicle.id) || null

    // Update control mode
    const networkCondition = adaptiveControl.generateMockNetworkCondition(
      vehicle.riskScore.urgencyLevel === 'critical' ? 'poor' :
      vehicle.riskScore.urgencyLevel === 'high' ? 'fair' :
      vehicle.riskScore.urgencyLevel === 'medium' ? 'good' : 'excellent'
    )
    adaptiveControl.initializeVehicle(vehicle.id, vehicle.controlMode)
    vehicle.controlMode = adaptiveControl.updateNetworkCondition(vehicle.id, networkCondition)

    // Match vehicles without matches
    if (!vehicle.match && vehicle.riskScore.urgencyLevel !== 'low') {
      const request: VehicleRequest = {
        vehicleId: vehicle.id,
        location: vehicle.location,
        riskScore: vehicle.riskScore,
        scenario: vehicle.scenario,
        weather: vehicle.weather,
        requiredSkills: [],
        priority: vehicle.riskScore.overallScore,
        timestamp: Date.now()
      }

      const match = bipartiteMatching.findOptimalMatch(request)
      if (match) {
        vehicle.match = match
        stats.value.totalMatches++
      }
    }
  }

  // Remove old matched vehicles (simulate handover completion)
  vehicleQueue.value = vehicleQueue.value.filter(v => {
    if (v.match && Date.now() - v.timestamp > 10000) {
      return false  // Remove after 10 seconds
    }
    return true
  })

  // Update statistics
  updateStatistics()
}

function addRandomVehicle() {
  const scenarios: ('highway' | 'urban' | 'residential' | 'parking' | 'rural')[] = ['highway', 'urban', 'residential', 'parking', 'rural']
  const weathers: ('clear' | 'rainy' | 'foggy' | 'snowy')[] = ['clear', 'rainy', 'foggy', 'snowy']
  const urgencyLevels: ('critical' | 'high' | 'medium' | 'low')[] = ['critical', 'high', 'medium', 'low']

  const scenario = scenarios[Math.floor(Math.random() * scenarios.length)]
  const weather = weathers[Math.floor(Math.random() * weathers.length)]
  const urgency = urgencyLevels[Math.floor(Math.random() * urgencyLevels.length)]

  const features = generateRiskFeatures(scenario, urgency)
  const riskScore = riskScoring.calculateRiskScore(features)

  const vehicle: DemoVehicle = {
    id: `V-${String(nextVehicleId++).padStart(3, '0')}`,
    location: generateRandomLocation(),
    riskScore,
    scenario,
    weather,
    controlMode: 'trajectory',
    prediction: null,
    match: null,
    timestamp: Date.now(),
    telemetry: {
      speed: 30 + Math.random() * 70,
      acceleration: -2 + Math.random() * 4,
      heading: Math.random() * 360
    }
  }

  vehicleQueue.value.push(vehicle)
  console.log(`[Demo] Added vehicle ${vehicle.id} (${urgency})`)
}

function addEmergencyVehicle() {
  const features = riskScoring.generateMockFeatures('critical')
  const riskScore = riskScoring.calculateRiskScore(features)

  const vehicle: DemoVehicle = {
    id: `V-${String(nextVehicleId++).padStart(3, '0')}`,
    location: generateRandomLocation(),
    riskScore,
    scenario: 'urban',
    weather: 'rainy',
    controlMode: 'direct',
    prediction: null,
    match: null,
    timestamp: Date.now(),
    telemetry: {
      speed: 80 + Math.random() * 20,
      acceleration: -3,
      heading: Math.random() * 360
    }
  }

  vehicleQueue.value.unshift(vehicle)  // Add to front of queue
  console.log(`[Demo] Added emergency vehicle ${vehicle.id}`)
}

function addBatchVehicles() {
  for (let i = 0; i < 10; i++) {
    addRandomVehicle()
  }
}

function resetSimulation() {
  stopSimulation()
  vehicleQueue.value = []
  nextVehicleId = 1
  stats.value = {
    totalVehicles: 0,
    activeOperators: 0,
    avgMatchTime: 0,
    matchSuccessRate: 1.0,
    preConnectedVehicles: 0,
    totalMatches: 0
  }
}

function generateRiskFeatures(
  scenario: string,
  urgency?: 'critical' | 'high' | 'medium' | 'low'
): VehicleRiskFeatures {
  if (urgency) {
    return riskScoring.generateMockFeatures(urgency)
  }
  return riskScoring.generateMockFeatures('random')
}

function generateRandomLocation(): GeographicLocation {
  // Hong Kong region
  const lat = 22.15 + Math.random() * 0.42
  const lon = 113.83 + Math.random() * 0.58

  return {
    latitude: lat,
    longitude: lon,
    region: `HK-${Math.floor((lat - 22.15) / 0.14)}-${Math.floor((lon - 113.83) / 0.19)}`
  }
}

function selectVehicle(vehicleId: string) {
  console.log(`[Demo] Selected vehicle ${vehicleId}`)
  selectedVehicleId.value = vehicleId

  // Update map center to selected vehicle
  const vehicle = vehicleQueue.value.find(v => v.id === vehicleId)
  if (vehicle) {
    mapCenter.value = {
      longitude: vehicle.location.longitude,
      latitude: vehicle.location.latitude
    }
  }
}

// Takeover Dialog Functions
function openTakeoverDialog(vehicleId: string) {
  const vehicle = vehicleQueue.value.find(v => v.id === vehicleId)
  if (vehicle) {
    takeoverVehicle.value = vehicle
    showTakeoverDialog.value = true
    console.log(`[Demo] Opening takeover dialog for ${vehicleId}`)
  }
}

function handleTakeoverConfirm(vehicleId: string, operatorId: string | null) {
  console.log(`[Demo] Takeover confirmed for vehicle ${vehicleId} with operator ${operatorId}`)

  // Update vehicle control mode
  const vehicle = vehicleQueue.value.find(v => v.id === vehicleId)
  if (vehicle) {
    vehicle.controlMode = 'direct'

    // Assign operator if available
    if (operatorId) {
      const operator = operators.value.find(o => o.id === operatorId)
      if (operator && !operator.assignedVehicles.includes(vehicleId)) {
        operator.assignedVehicles.push(vehicleId)
        operator.currentLoad = operator.assignedVehicles.length / operator.maxConcurrentVehicles
        operator.status = 'busy'
      }
    }

    // Update statistics
    algorithmMetrics.value.totalModeTransitions++
    stats.value.totalMatches++
  }

  // Close dialog
  showTakeoverDialog.value = false
  takeoverVehicle.value = null
}

function handleTakeoverCancel() {
  console.log(`[Demo] Takeover cancelled`)
  showTakeoverDialog.value = false
  takeoverVehicle.value = null
}

// Remote Assistant Functions
function handleApplySuggestion(suggestion: any) {
  console.log(`[Demo] Applying suggestion:`, suggestion)
  // Implementation for applying suggestions
}

function handleRequestEmergency() {
  console.log(`[Demo] Emergency guidance requested`)
  // Open takeover dialog for current selected vehicle
  if (selectedVehicleId.value) {
    openTakeoverDialog(selectedVehicleId.value)
  }
}

function handleRequestRouteOptimization() {
  console.log(`[Demo] Route optimization requested`)
  // Implementation for route optimization
}

function updateStatistics() {
  stats.value.totalVehicles = vehicleQueue.value.length
  stats.value.activeOperators = operators.value.filter(o => o.status !== 'offline').length

  // Calculate avg match time
  const matchedVehicles = vehicleQueue.value.filter(v => v.match)
  if (matchedVehicles.length > 0) {
    stats.value.avgMatchTime = matchedVehicles.reduce((sum, v) =>
      sum + (v.match?.estimatedHandoverTime || 0), 0) / matchedVehicles.length
  }

  // Pre-connected vehicles
  stats.value.preConnectedVehicles = vehicleQueue.value.filter(v =>
    v.prediction && v.prediction.recommendedAction === 'pre-connect'
  ).length

  // Mode distribution
  const modeStats = adaptiveControl.getStatistics()
  modeDistribution.value = modeStats.modeDistribution

  // Algorithm metrics
  algorithmMetrics.value.avgScoringTime = vehicleQueue.value.length > 0
    ? vehicleQueue.value.reduce((sum, v) => sum + v.riskScore.computeTimeMs, 0) / vehicleQueue.value.length
    : 0

  algorithmMetrics.value.totalModeTransitions = modeStats.avgTransitionsPerVehicle * modeStats.totalVehicles
  algorithmMetrics.value.bandwidthSaved = (modeStats.modeDistribution.trajectory * 2 +
    modeStats.modeDistribution.semantic * 4) * modeStats.totalVehicles
}

// Lifecycle
onMounted(() => {
  console.log('[Demo] Component mounted')
})

onUnmounted(() => {
  stopSimulation()
})
</script>

<style scoped>
.intelligent-dispatch-demo {
  background: #0a0a12;
  min-height: 100vh;
  color: #fff;
  padding: 20px;
}

.demo-header {
  text-align: center;
  margin-bottom: 30px;
}

.demo-header h1 {
  font-size: 32px;
  margin: 0 0 10px 0;
  color: var(--primary, #ff5722);
}

.demo-header p {
  color: #888;
  font-size: 14px;
  margin: 0 0 20px 0;
}

.simulation-controls {
  display: flex;
  gap: 12px;
  justify-content: center;
}

.simulation-controls button {
  padding: 10px 20px;
  border: none;
  border-radius: 6px;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s;
}

.btn-primary {
  background: var(--primary, #ff5722);
  color: #0a0a12;
}

.btn-primary:hover {
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(255, 87, 34, 0.3);
}

.btn-stop {
  background: #ff9933;
}

.btn-emergency {
  background: #ff3333;
  color: #fff;
}

.btn-secondary {
  background: #3a3a4a;
  color: #fff;
}

.btn-reset {
  background: #2a2a3a;
  color: #888;
}

.stats-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
  gap: 16px;
  margin-bottom: 24px;
}

.stat-card {
  background: #14141f;
  border: 1px solid #2a2a3a;
  border-radius: 8px;
  padding: 16px;
  display: flex;
  align-items: center;
  gap: 16px;
}

.stat-icon {
  font-size: 32px;
}

.stat-value {
  font-size: 28px;
  font-weight: 700;
  color: var(--primary, #ff5722);
}

.stat-label {
  font-size: 12px;
  color: #888;
}

.demo-content {
  display: grid;
  grid-template-columns: 350px 1fr 350px;
  gap: 20px;
}

.panel {
  background: #14141f;
  border: 1px solid #2a2a3a;
  border-radius: 8px;
  overflow: hidden;
}

.panel-header {
  padding: 16px;
  border-bottom: 1px solid #2a2a3a;
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.panel-header h2 {
  font-size: 16px;
  margin: 0;
}

.queue-filter {
  display: flex;
  gap: 4px;
}

.filter-btn {
  padding: 4px 10px;
  font-size: 11px;
  background: transparent;
  border: 1px solid #3a3a4a;
  border-radius: 4px;
  color: #888;
  cursor: pointer;
  text-transform: uppercase;
}

.filter-btn.active {
  background: var(--primary, #ff5722);
  color: #0a0a12;
  border-color: var(--primary, #ff5722);
}

.vehicle-queue {
  max-height: 700px;
  overflow-y: auto;
  padding: 12px;
}

.vehicle-card {
  background: #1a1a2a;
  border: 1px solid #2a2a3a;
  border-radius: 6px;
  padding: 12px;
  margin-bottom: 8px;
  cursor: pointer;
  transition: all 0.2s;
}

.vehicle-card:hover {
  border-color: var(--primary, #ff5722);
}

.vehicle-card.urgency-critical {
  border-left: 3px solid #ff3333;
}

.vehicle-card.urgency-high {
  border-left: 3px solid #ff9933;
}

.vehicle-card.urgency-medium {
  border-left: 3px solid #ffdd33;
}

.vehicle-card.urgency-low {
  border-left: 3px solid #33ff99;
}

.vehicle-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
}

.vehicle-id {
  font-weight: 600;
  color: var(--primary, #ff5722);
}

.urgency-badge {
  font-size: 10px;
  font-weight: 700;
  padding: 2px 6px;
  border-radius: 4px;
}

.urgency-badge.critical {
  background: rgba(255, 51, 51, 0.2);
  color: #ff3333;
}

.urgency-badge.high {
  background: rgba(255, 153, 51, 0.2);
  color: #ff9933;
}

.urgency-badge.medium {
  background: rgba(255, 221, 51, 0.2);
  color: #ffdd33;
}

.urgency-badge.low {
  background: rgba(51, 255, 153, 0.2);
  color: #33ff99;
}

.info-row {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 11px;
  margin-bottom: 4px;
}

.info-row .label {
  color: #666;
  width: 80px;
}

.info-row .value {
  color: #ccc;
}

.risk-bar {
  flex: 1;
  height: 6px;
  background: #2a2a3a;
  border-radius: 3px;
  overflow: hidden;
}

.risk-fill {
  height: 100%;
  background: linear-gradient(to right, #33ff99, #ffdd33, #ff9933, #ff3333);
}

.mode-badge {
  font-size: 10px;
  padding: 2px 6px;
  border-radius: 4px;
  text-transform: uppercase;
}

.mode-badge.direct {
  background: rgba(51, 255, 153, 0.2);
  color: #33ff99;
}

.mode-badge.trajectory {
  background: rgba(255, 87, 34, 0.2);
  color: var(--primary, #ff5722);
}

.mode-badge.semantic {
  background: rgba(153, 153, 255, 0.2);
  color: #9999ff;
}

.match-info {
  margin-top: 8px;
  padding-top: 8px;
  border-top: 1px solid #2a2a3a;
}

.match-header {
  font-size: 11px;
  color: #33ff99;
  margin-bottom: 4px;
}

.match-details {
  display: flex;
  justify-content: space-between;
  font-size: 10px;
  color: #888;
}

.map-container {
  padding: 16px;
  display: flex;
  justify-content: center;
}

canvas {
  border: 1px solid #2a2a3a;
  border-radius: 4px;
  max-width: 100%;
  height: auto;
}

.map-legend {
  display: flex;
  gap: 16px;
  padding: 12px 16px;
  border-top: 1px solid #2a2a3a;
  flex-wrap: wrap;
}

.legend-item {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 11px;
  color: #888;
}

.legend-dot {
  width: 12px;
  height: 12px;
  border-radius: 50%;
}

.legend-dot.urgency-critical {
  background: #ff3333;
}

.legend-dot.urgency-high {
  background: #ff9933;
}

.legend-dot.urgency-medium {
  background: #ffdd33;
}

.legend-dot.urgency-low {
  background: #33ff99;
}

.legend-dot.operator {
  background: #ff5722;
  border: 2px solid #fff;
}

.mode-panel {
  margin-top: 20px;
}

.mode-stats {
  padding: 16px;
}

.mode-stat {
  margin-bottom: 16px;
}

.mode-label {
  font-size: 12px;
  color: #888;
  margin-bottom: 6px;
}

.mode-bar {
  height: 24px;
  background: #2a2a3a;
  border-radius: 4px;
  overflow: hidden;
  margin-bottom: 4px;
}

.mode-fill {
  height: 100%;
  transition: width 0.3s;
}

.mode-fill.direct {
  background: #33ff99;
}

.mode-fill.trajectory {
  background: var(--primary, #ff5722);
}

.mode-fill.semantic {
  background: #9999ff;
}

.mode-value {
  font-size: 12px;
  color: #ccc;
  text-align: right;
}

.operator-list {
  max-height: 500px;
  overflow-y: auto;
  padding: 12px;
}

.operator-card {
  background: #1a1a2a;
  border: 1px solid #2a2a3a;
  border-radius: 6px;
  padding: 10px;
  margin-bottom: 8px;
}

.operator-card.busy {
  border-left: 3px solid #ff9933;
}

.operator-card.idle {
  border-left: 3px solid #33ff99;
}

.operator-header {
  display: flex;
  justify-content: space-between;
  margin-bottom: 8px;
}

.operator-name {
  font-weight: 600;
  font-size: 13px;
}

.status-badge {
  font-size: 10px;
  padding: 2px 6px;
  border-radius: 4px;
  text-transform: uppercase;
}

.status-badge.idle {
  background: rgba(51, 255, 153, 0.2);
  color: #33ff99;
}

.status-badge.busy {
  background: rgba(255, 153, 51, 0.2);
  color: #ff9933;
}

.operator-info .info-item {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 10px;
  margin-bottom: 4px;
}

.operator-info .label {
  color: #666;
  width: 70px;
}

.operator-info .value {
  color: #ccc;
}

.load-bar {
  flex: 1;
  height: 4px;
  background: #2a2a3a;
  border-radius: 2px;
  overflow: hidden;
}

.load-fill {
  height: 100%;
  background: linear-gradient(to right, #33ff99, #ff9933);
}

.metrics-panel {
  margin-top: 20px;
}

.metrics-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 12px;
  padding: 16px;
}

.metric-item {
  background: #1a1a2a;
  padding: 12px;
  border-radius: 6px;
  border: 1px solid #2a2a3a;
}

.metric-label {
  font-size: 11px;
  color: #888;
  margin-bottom: 6px;
}

.metric-value {
  font-size: 18px;
  font-weight: 700;
  color: var(--primary, #ff5722);
}

.empty-state {
  text-align: center;
  padding: 40px 20px;
  color: #666;
  font-size: 14px;
}

/* AI Panel */
.ai-panel {
  margin-top: 20px;
}

.ai-panel .map-container {
  padding: 0;
}

/* Takeover Button */
.takeover-btn {
  width: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  margin-top: 12px;
  padding: 10px 16px;
  background: linear-gradient(135deg, rgba(255, 51, 51, 0.2), rgba(255, 0, 0, 0.1));
  border: 2px solid #ff3333;
  border-radius: 8px;
  color: #ff3333;
  font-size: 14px;
  font-weight: 700;
  cursor: pointer;
  transition: all 0.2s;
  animation: pulse-border 2s ease-in-out infinite;
}

@keyframes pulse-border {
  0%, 100% {
    box-shadow: 0 0 0 0 rgba(255, 51, 51, 0.4);
  }
  50% {
    box-shadow: 0 0 0 8px rgba(255, 51, 51, 0);
  }
}

.takeover-btn:hover {
  background: linear-gradient(135deg, rgba(255, 51, 51, 0.3), rgba(255, 0, 0, 0.2));
  border-color: #ff5555;
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(255, 51, 51, 0.4);
}

.takeover-btn.high {
  background: linear-gradient(135deg, rgba(255, 153, 51, 0.2), rgba(255, 100, 0, 0.1));
  border-color: #ff9933;
  color: #ff9933;
}

.takeover-btn.high:hover {
  background: linear-gradient(135deg, rgba(255, 153, 51, 0.3), rgba(255, 100, 0, 0.2));
  border-color: #ffaa55;
}

.btn-icon {
  font-size: 18px;
  animation: shake 2s ease-in-out infinite;
}

@keyframes shake {
  0%, 100% {
    transform: rotate(0deg);
  }
  10%, 30%, 50%, 70%, 90% {
    transform: rotate(-5deg);
  }
  20%, 40%, 60%, 80% {
    transform: rotate(5deg);
  }
}

.btn-text {
  font-size: 14px;
}
</style>
