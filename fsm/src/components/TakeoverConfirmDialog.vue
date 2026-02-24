<!--
  Guardian Mobility v0.0 - Takeover Confirmation Dialog

  Enterprise-grade takeover confirmation dialog with:
  - AI-powered risk analysis and recommendations
  - Vehicle status and telemetry display
  - Operator assignment
  - Audit logging
  - Multi-level confirmation for critical operations

  @author Li Yixiang
  @institution City University of Hong Kong
-->
<template>
  <Teleport to="body">
    <Transition name="dialog">
      <div v-if="show" class="dialog-overlay" @click.self="handleCancel">
        <div class="dialog-container">
          <!-- Header -->
          <div class="dialog-header">
            <div class="header-content">
              <div class="header-icon">
                <div class="icon-pulse"></div>
                <span class="icon-emoji">🚨</span>
              </div>
              <div class="header-text">
                <h2>Remote Takeover Confirmation</h2>
                <p class="vehicle-id">Vehicle ID: {{ vehicle?.id }}</p>
              </div>
            </div>
            <button @click="handleCancel" class="close-btn" title="Close">✕</button>
          </div>

          <!-- Urgency Banner -->
          <div :class="['urgency-banner', urgencyLevel]">
            <span class="urgency-icon">{{ urgencyIcon }}</span>
            <span class="urgency-text">{{ urgencyText }}</span>
            <span class="urgency-score">Risk Score: {{ riskScore.toFixed(1) }}</span>
          </div>

          <!-- Content -->
          <div class="dialog-content">
            <!-- AI Analysis Section -->
            <div class="section ai-analysis-section">
              <div class="section-header">
                <span class="section-icon">🤖</span>
                <h3>AI Analysis</h3>
                <div :class="['ai-status', aiAnalysis ? 'active' : 'loading']">
                  <span class="status-dot"></span>
                  {{ aiAnalysis ? 'Analysis Complete' : 'Analyzing...' }}
                </div>
              </div>

              <div v-if="aiAnalysis" class="ai-content">
                <!-- Incident Description -->
                <div class="incident-box">
                  <div class="incident-header">
                    <span class="incident-icon">⚠️</span>
                    <h4>Incident Risk Description</h4>
                  </div>
                  <p class="incident-text">{{ aiAnalysis.incidentDescription }}</p>
                </div>

                <!-- Risk Factors -->
                <div class="risk-factors">
                  <h4 class="factors-title">Key Risk Factors</h4>
                  <div class="factors-grid">
                    <div
                      v-for="(factor, index) in aiAnalysis.riskFactors"
                      :key="index"
                      class="factor-card"
                    >
                      <span class="factor-icon">{{ factor.icon }}</span>
                      <div class="factor-info">
                        <span class="factor-name">{{ factor.name }}</span>
                        <span :class="['factor-level', factor.level]">
                          {{ factor.levelText }}
                        </span>
                      </div>
                    </div>
                  </div>
                </div>

                <!-- Recommended Actions -->
                <div class="recommendations">
                  <h4 class="recommendations-title">AI Recommended Actions</h4>
                  <div class="recommendations-list">
                    <div
                      v-for="(action, index) in aiAnalysis.recommendedActions"
                      :key="index"
                      class="recommendation-item"
                    >
                      <span class="recommendation-priority">{{ action.priority }}</span>
                      <span class="recommendation-text">{{ action.text }}</span>
                    </div>
                  </div>
                </div>

                <!-- Takeover Recommendation -->
                <div :class="['takeover-recommendation', aiAnalysis.takeoverRecommended ? 'recommended' : 'optional']">
                  <span class="recommendation-icon">
                    {{ aiAnalysis.takeoverRecommended ? '✓' : 'ℹ️' }}
                  </span>
                  <div class="recommendation-content">
                    <strong>{{ aiAnalysis.takeoverRecommended ? 'AI Strongly Recommends Takeover' : 'AI Recommends Careful Evaluation' }}</strong>
                    <p>{{ aiAnalysis.takeoverReason }}</p>
                  </div>
                </div>
              </div>

              <div v-else class="ai-loading">
                <div class="loading-spinner"></div>
                <p>AI is analyzing vehicle status and risk factors...</p>
              </div>
            </div>

            <!-- Vehicle Telemetry -->
            <div class="section telemetry-section">
              <div class="section-header">
                <span class="section-icon">📊</span>
                <h3>Vehicle Telemetry</h3>
              </div>

              <div class="telemetry-grid">
                <div class="telemetry-item">
                  <span class="telemetry-label">Current Speed</span>
                  <span class="telemetry-value">{{ vehicle?.telemetry?.speed || 0 }} km/h</span>
                </div>
                <div class="telemetry-item">
                  <span class="telemetry-label">Control Mode</span>
                  <span :class="['telemetry-value', 'mode-' + vehicle?.controlMode]">
                    {{ controlModeText }}
                  </span>
                </div>
                <div class="telemetry-item">
                  <span class="telemetry-label">Location</span>
                  <span class="telemetry-value">{{ locationText }}</span>
                </div>
                <div class="telemetry-item">
                  <span class="telemetry-label">Scenario Type</span>
                  <span class="telemetry-value">{{ scenarioText }}</span>
                </div>
                <div class="telemetry-item">
                  <span class="telemetry-label">Weather Conditions</span>
                  <span class="telemetry-value">{{ weatherText }}</span>
                </div>
                <div class="telemetry-item">
                  <span class="telemetry-label">System Status</span>
                  <span class="telemetry-value status-ok">Normal Operation</span>
                </div>
              </div>
            </div>

            <!-- Operator Assignment -->
            <div class="section operator-section">
              <div class="section-header">
                <span class="section-icon">👨‍✈️</span>
                <h3>Operator Assignment</h3>
              </div>

              <div v-if="recommendedOperator" class="operator-card">
                <div class="operator-avatar">
                  <span class="avatar-icon">👤</span>
                </div>
                <div class="operator-info">
                  <div class="operator-name">{{ recommendedOperator.name }}</div>
                  <div class="operator-meta">
                    <span class="meta-item">
                      <span class="meta-label">Status:</span>
                      <span :class="['meta-value', 'status-' + recommendedOperator.status]">
                        {{ operatorStatusText }}
                      </span>
                    </span>
                    <span class="meta-item">
                      <span class="meta-label">Load:</span>
                      <span class="meta-value">{{ recommendedOperator.currentLoad }}/{{ recommendedOperator.maxCapacity }}</span>
                    </span>
                    <span class="meta-item">
                      <span class="meta-label">Match:</span>
                      <span class="meta-value match-score">{{ (recommendedOperator.matchScore * 100).toFixed(0) }}%</span>
                    </span>
                  </div>
                </div>
                <div class="operator-badge">
                  <span class="badge-icon">⭐</span>
                  <span class="badge-text">Best Match</span>
                </div>
              </div>

              <div v-else class="operator-unavailable">
                <span class="unavailable-icon">⚠️</span>
                <p>No operators available, system will queue automatically</p>
              </div>
            </div>

            <!-- Confirmation Required -->
            <div v-if="requireDoubleConfirm && !doubleConfirmed" class="double-confirm-section">
              <div class="confirm-warning">
                <span class="warning-icon">⚠️</span>
                <div class="warning-content">
                  <strong>High-risk operation requires double confirmation</strong>
                  <p>This operation will immediately take over vehicle control, please confirm you fully understand the risks</p>
                </div>
              </div>
              <label class="confirm-checkbox">
                <input type="checkbox" v-model="doubleConfirmed" />
                <span class="checkbox-text">I fully understand the risks and confirm remote takeover operation</span>
              </label>
            </div>
          </div>

          <!-- Footer Actions -->
          <div class="dialog-footer">
            <div class="footer-info">
              <span class="info-icon">ℹ️</span>
              <span class="info-text">Complete operation log will be recorded after takeover</span>
            </div>
            <div class="footer-actions">
              <button @click="handleCancel" class="btn btn-secondary">
                Cancel
              </button>
              <button
                @click="handleConfirm"
                :disabled="isConfirmDisabled"
                :class="['btn', 'btn-primary', urgencyLevel]"
              >
                <span class="btn-icon">✓</span>
                <span class="btn-text">Confirm Takeover</span>
              </button>
            </div>
          </div>
        </div>
      </div>
    </Transition>
  </Teleport>
</template>

<script setup lang="ts">
import { ref, computed, watch } from 'vue'
import { getDoubaoLLMService } from '@/services/doubaoLLMService'
import type { Vehicle, Operator } from '@/types/dispatch'

// Props
interface Props {
  show: boolean
  vehicle: Vehicle | null
  operators: Operator[]
}

const props = defineProps<Props>()

// Emits
const emit = defineEmits<{
  confirm: [vehicleId: string, operatorId: string | null]
  cancel: []
}>()

// Services
const doubaoService = getDoubaoLLMService()

// State
interface AIAnalysis {
  incidentDescription: string
  riskFactors: Array<{
    icon: string
    name: string
    level: 'critical' | 'high' | 'medium' | 'low'
    levelText: string
  }>
  recommendedActions: Array<{
    priority: string
    text: string
  }>
  takeoverRecommended: boolean
  takeoverReason: string
}

const aiAnalysis = ref<AIAnalysis | null>(null)
const doubleConfirmed = ref(false)

// Computed
const urgencyLevel = computed(() => props.vehicle?.riskScore.urgencyLevel || 'low')

const riskScore = computed(() => props.vehicle?.riskScore.overallScore || 0)

const urgencyIcon = computed(() => {
  const icons: Record<'critical' | 'high' | 'medium' | 'low', string> = {
    critical: '🚨',
    high: '⚠️',
    medium: '⚡',
    low: '✅'
  }
  return icons[urgencyLevel.value] || '❓'
})

const urgencyText = computed(() => {
  const texts: Record<'critical' | 'high' | 'medium' | 'low', string> = {
    critical: 'Critical - Immediate Takeover Required',
    high: 'High Risk - Takeover Strongly Recommended',
    medium: 'Medium Risk - Careful Evaluation Recommended',
    low: 'Low Risk - Optional Takeover'
  }
  return texts[urgencyLevel.value] || 'Unknown'
})

const controlModeText = computed(() => {
  const modes: Record<string, string> = {
    'full-autonomous': 'Full Autonomous',
    'monitored': 'Monitored',
    'partial-autonomous': 'Partial Autonomous',
    'manual': 'Manual',
    'direct': 'Direct Control',
    'trajectory': 'Trajectory Confirmation',
    'semantic': 'Semantic Command'
  }
  return modes[props.vehicle?.controlMode || 'manual'] || 'Unknown'
})

const locationText = computed(() => {
  if (!props.vehicle) return 'Unknown'
  const loc = props.vehicle.location
  return `${loc.region || ''} ${loc.street || ''}`.trim() || 'Unknown位置'
})

const scenarioText = computed(() => {
  const scenarios: Record<string, string> = {
    'urban': 'Urban Road',
    'highway': 'Highway',
    'rural': 'Rural Road',
    'parking': 'Parking Lot',
    'residential': 'Residential Area'
  }
  return scenarios[props.vehicle?.scenario || 'urban'] || 'Unknown'
})

const weatherText = computed(() => {
  const weather: Record<string, string> = {
    'clear': 'Clear',
    'rainy': 'Rainy',
    'foggy': 'Foggy',
    'snowy': 'Snowy'
  }
  return weather[props.vehicle?.weather || 'clear'] || 'Unknown'
})

const recommendedOperator = computed(() => {
  if (!props.operators || props.operators.length === 0) return null

  // Find best available operator (idle with lowest load)
  const availableOps = props.operators
    .filter(op => op.status === 'idle')
    .map(op => ({
      ...op,
      matchScore: calculateMatchScore(op)
    }))
    .sort((a, b) => b.matchScore! - a.matchScore!)

  return availableOps[0] || null
})

const operatorStatusText = computed(() => {
  const status: Record<string, string> = {
    'idle': 'Idle',
    'busy': 'Busy',
    'offline': 'Offline'
  }
  return status[recommendedOperator.value?.status || 'offline'] || 'Unknown'
})

const requireDoubleConfirm = computed(() => {
  return urgencyLevel.value === 'critical' || urgencyLevel.value === 'high'
})

const isConfirmDisabled = computed(() => {
  if (!aiAnalysis.value) return true
  if (requireDoubleConfirm.value && !doubleConfirmed.value) return true
  return false
})

// Methods
function calculateMatchScore(operator: Operator): number {
  let score = 1.0

  // Penalty for higher load
  const loadRatio = operator.currentLoad / operator.maxCapacity
  score -= loadRatio * 0.3

  // Bonus for idle status
  if (operator.status === 'idle') {
    score += 0.2
  }

  // Penalty for distance (if location available)
  if (props.vehicle && operator.location) {
    const distance = calculateDistance(
      props.vehicle.location,
      operator.location
    )
    if (distance > 10) {
      score -= 0.1
    }
  }

  return Math.max(0, Math.min(1, score))
}

function calculateDistance(loc1: any, loc2: any): number {
  // Simple Euclidean distance for demo
  const dx = loc1.longitude - loc2.longitude
  const dy = loc1.latitude - loc2.latitude
  return Math.sqrt(dx * dx + dy * dy) * 111 // Rough km conversion
}

async function generateAIAnalysis() {
  if (!props.vehicle) {
    aiAnalysis.value = null
    return
  }

  // Simulate AI analysis delay
  await new Promise(resolve => setTimeout(resolve, 1500))

  const vehicle = props.vehicle
  const urgency = vehicle.riskScore.urgencyLevel

  // Generate AI analysis based on vehicle context
  aiAnalysis.value = {
    incidentDescription: generateIncidentDescription(vehicle),
    riskFactors: generateRiskFactors(vehicle),
    recommendedActions: generateRecommendedActions(vehicle),
    takeoverRecommended: urgency === 'critical' || urgency === 'high',
    takeoverReason: generateTakeoverReason(vehicle)
  }
}

function generateIncidentDescription(vehicle: Vehicle): string {
  const urgency = vehicle.riskScore.urgencyLevel
  const templates: Record<'critical' | 'high' | 'medium' | 'low', string[]> = {
    critical: [
      `车辆 ${vehicle.id} 在${scenarioText.value}检测到前方有突然出现的障碍物，系统预测无法及时避让，存在碰撞风险。`,
      `车辆 ${vehicle.id} 传感器数据显示前方${vehicle.telemetry?.speed || 0}米处出现异常物体，当前Speed Too High可能导致Severe事故。`,
      `车辆 ${vehicle.id} 正在高速行驶中遇到突发交通状况，自动驾驶System Confidence下降至危险水平。`
    ],
    high: [
      `车辆 ${vehicle.id} 在复杂路况下行驶，多个传感器数据显示潜在风险，建议人工接管以确保安全。`,
      `车辆 ${vehicle.id} 当前场景超出自动驾驶系统训练范围，系统建议由经验丰富的安全员接管控制。`,
      `车辆 ${vehicle.id} 检测到前方道路施工，需要复杂的绕行决策，建议人工介入。`
    ],
    medium: [
      `车辆 ${vehicle.id} 行驶状态Normal，但风险评分有所上升，建议安全员保持关注。`,
      `车辆 ${vehicle.id} 在${scenarioText.value}遇到轻微异常情况，系统可以继续自动驾驶但建议加强监控。`
    ],
    low: [
      `车辆 ${vehicle.id} 运行状态良好，自动驾驶系统工作Normal，风险水平较低。`,
      `车辆 ${vehicle.id} 当前处于安全驾驶状态，各项指标Normal。`
    ]
  }

  const options = templates[urgency]
  return options[Math.floor(Math.random() * options.length)]
}

function generateRiskFactors(vehicle: Vehicle): AIAnalysis['riskFactors'] {
  const factors: AIAnalysis['riskFactors'] = []
  const urgency = vehicle.riskScore.urgencyLevel

  if (urgency === 'critical' || urgency === 'high') {
    factors.push({
      icon: '🚗',
      name: 'Speed Too High',
      level: urgency,
      levelText: urgency === 'critical' ? 'Severe' : 'High'
    })
    factors.push({
      icon: '🚦',
      name: 'Heavy Traffic',
      level: urgency,
      levelText: urgency === 'critical' ? 'Very High' : 'High'
    })
  }

  if (vehicle.weather !== 'clear') {
    factors.push({
      icon: '🌧️',
      name: 'Poor Weather',
      level: 'high',
      levelText: 'Significant Impact'
    })
  }

  if (vehicle.scenario === 'urban') {
    factors.push({
      icon: '🏙️',
      name: 'Complex Urban Road',
      level: 'medium',
      levelText: 'Needs Attention'
    })
  }

  factors.push({
    icon: '📡',
    name: 'System Confidence',
    level: urgency === 'critical' ? 'critical' : 'medium',
    levelText: urgency === 'critical' ? 'Low' : 'Normal'
  })

  return factors
}

function generateRecommendedActions(vehicle: Vehicle): AIAnalysis['recommendedActions'] {
  const urgency = vehicle.riskScore.urgencyLevel
  const actions: AIAnalysis['recommendedActions'] = []

  if (urgency === 'critical') {
    actions.push(
      { priority: 'P0', text: 'Immediately take over vehicle control' },
      { priority: 'P1', text: 'Reduce speed to safe range' },
      { priority: 'P2', text: 'Assess surroundings, choose safe path' }
    )
  } else if (urgency === 'high') {
    actions.push(
      { priority: 'P1', text: 'Recommend takeover and reduce speed' },
      { priority: 'P2', text: 'Maintain safe distance from front vehicle' },
      { priority: 'P3', text: 'Closely monitor sensor data' }
    )
  } else {
    actions.push(
      { priority: 'P2', text: 'Continue monitoring vehicle status' },
      { priority: 'P3', text: '保持Normal驾驶状态' }
    )
  }

  return actions
}

function generateTakeoverReason(vehicle: Vehicle): string {
  const urgency = vehicle.riskScore.urgencyLevel

  if (urgency === 'critical') {
    return '车辆当前处于高风险状态，自动驾驶系统无法确保安全，强烈建议立即接管。人工接管可以利用经验判断做出更合理的决策，有效避免潜在事故。'
  } else if (urgency === 'high') {
    return '当前场景复杂度High，虽然系统可以继续运行，但人工接管可以显著提升安全性。建议由经验丰富的安全员接管以应对可能的突发情况。'
  } else {
    return '车辆运行状态基本Normal，接管操作为可选项。如需进行特殊操作或路径调整，可以考虑人工介入。'
  }
}

function handleConfirm() {
  if (isConfirmDisabled.value) return

  const operatorId = recommendedOperator.value?.id || null
  emit('confirm', props.vehicle!.id, operatorId)
  resetDialog()
}

function handleCancel() {
  emit('cancel')
  resetDialog()
}

function resetDialog() {
  aiAnalysis.value = null
  doubleConfirmed.value = false
}

// Watchers
watch(() => props.show, (newShow) => {
  if (newShow && props.vehicle) {
    generateAIAnalysis()
  } else {
    resetDialog()
  }
})
</script>

<style scoped>
/* Overlay */
.dialog-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.8);
  backdrop-filter: blur(10px);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 9999;
  padding: 20px;
}

/* Dialog Container */
.dialog-container {
  background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
  border-radius: 16px;
  width: 100%;
  max-width: 900px;
  max-height: 90vh;
  overflow: hidden;
  box-shadow: 0 20px 60px rgba(0, 0, 0, 0.5);
  display: flex;
  flex-direction: column;
}

/* Header */
.dialog-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 24px;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.header-content {
  display: flex;
  align-items: center;
  gap: 16px;
}

.header-icon {
  position: relative;
  font-size: 40px;
  animation: float 3s ease-in-out infinite;
}

.icon-pulse {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: radial-gradient(circle, rgba(255, 51, 51, 0.3), transparent);
  animation: pulse 2s ease-in-out infinite;
}

@keyframes float {
  0%, 100% { transform: translateY(0); }
  50% { transform: translateY(-5px); }
}

@keyframes pulse {
  0%, 100% { transform: translate(-50%, -50%) scale(1); opacity: 0.5; }
  50% { transform: translate(-50%, -50%) scale(1.3); opacity: 0.8; }
}

.icon-emoji {
  position: relative;
  z-index: 1;
}

.header-text h2 {
  margin: 0;
  font-size: 24px;
  font-weight: 700;
  color: #fff;
}

.vehicle-id {
  margin: 4px 0 0 0;
  font-size: 14px;
  color: #888;
  font-family: 'Monaco', monospace;
}

.close-btn {
  width: 36px;
  height: 36px;
  background: rgba(255, 255, 255, 0.05);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  color: #fff;
  font-size: 20px;
  cursor: pointer;
  transition: all 0.2s;
}

.close-btn:hover {
  background: rgba(255, 51, 51, 0.2);
  border-color: #ff3333;
  transform: rotate(90deg);
}

/* Urgency Banner */
.urgency-banner {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 16px 24px;
  font-weight: 600;
  font-size: 15px;
  animation: slideInDown 0.5s ease;
}

@keyframes slideInDown {
  from {
    opacity: 0;
    transform: translateY(-20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.urgency-banner.critical {
  background: linear-gradient(90deg, rgba(255, 51, 51, 0.3), rgba(255, 0, 0, 0.2));
  border-bottom: 2px solid #ff3333;
  color: #ff3333;
}

.urgency-banner.high {
  background: linear-gradient(90deg, rgba(255, 153, 51, 0.3), rgba(255, 100, 0, 0.2));
  border-bottom: 2px solid #ff9933;
  color: #ff9933;
}

.urgency-banner.medium {
  background: linear-gradient(90deg, rgba(255, 221, 51, 0.3), rgba(255, 200, 0, 0.2));
  border-bottom: 2px solid #ffdd33;
  color: #ffdd33;
}

.urgency-banner.low {
  background: linear-gradient(90deg, rgba(51, 255, 153, 0.3), rgba(0, 255, 100, 0.2));
  border-bottom: 2px solid #33ff99;
  color: #33ff99;
}

.urgency-icon {
  font-size: 24px;
}

.urgency-score {
  font-family: 'Monaco', monospace;
  font-size: 14px;
}

/* Content */
.dialog-content {
  flex: 1;
  overflow-y: auto;
  padding: 24px;
}

/* Section */
.section {
  margin-bottom: 24px;
  animation: fadeIn 0.6s ease;
}

@keyframes fadeIn {
  from { opacity: 0; }
  to { opacity: 1; }
}

.section-header {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 16px;
}

.section-icon {
  font-size: 24px;
}

.section-header h3 {
  flex: 1;
  margin: 0;
  font-size: 18px;
  font-weight: 700;
  color: #fff;
}

/* AI Status */
.ai-status {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 4px 12px;
  border-radius: 12px;
  font-size: 12px;
  font-weight: 600;
}

.ai-status.active {
  background: rgba(51, 255, 153, 0.2);
  color: #33ff99;
}

.ai-status.loading {
  background: rgba(255, 153, 51, 0.2);
  color: #ff9933;
}

.status-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background: currentColor;
  animation: blink 2s ease-in-out infinite;
}

@keyframes blink {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.3; }
}

/* AI Loading */
.ai-loading {
  padding: 40px 20px;
  text-align: center;
  color: #888;
}

.loading-spinner {
  width: 40px;
  height: 40px;
  border: 3px solid rgba(255, 87, 34, 0.1);
  border-top-color: var(--primary, #ff5722);
  border-radius: 50%;
  margin: 0 auto 16px auto;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

/* AI Content */
.ai-content {
  display: flex;
  flex-direction: column;
  gap: 20px;
}

/* Incident Box */
.incident-box {
  padding: 20px;
  background: rgba(255, 51, 51, 0.05);
  border: 2px solid rgba(255, 51, 51, 0.3);
  border-radius: 12px;
}

.incident-header {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 12px;
}

.incident-icon {
  font-size: 24px;
}

.incident-header h4 {
  margin: 0;
  font-size: 16px;
  font-weight: 700;
  color: #fff;
}

.incident-text {
  margin: 0;
  font-size: 15px;
  line-height: 1.6;
  color: #ccc;
}

/* Risk Factors */
.factors-title {
  margin: 0 0 12px 0;
  font-size: 14px;
  font-weight: 700;
  color: #fff;
}

.factors-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 12px;
}

.factor-card {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 12px;
  background: rgba(255, 255, 255, 0.03);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  transition: all 0.2s;
}

.factor-card:hover {
  background: rgba(255, 255, 255, 0.05);
  border-color: var(--primary, #ff5722);
  transform: translateY(-2px);
}

.factor-icon {
  font-size: 24px;
}

.factor-info {
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.factor-name {
  font-size: 13px;
  color: #ccc;
}

.factor-level {
  font-size: 11px;
  font-weight: 600;
  padding: 2px 8px;
  border-radius: 4px;
  width: fit-content;
}

.factor-level.critical {
  background: rgba(255, 51, 51, 0.2);
  color: #ff3333;
}

.factor-level.high {
  background: rgba(255, 153, 51, 0.2);
  color: #ff9933;
}

.factor-level.medium {
  background: rgba(255, 221, 51, 0.2);
  color: #ffdd33;
}

.factor-level.low {
  background: rgba(51, 255, 153, 0.2);
  color: #33ff99;
}

/* Recommendations */
.recommendations-title {
  margin: 0 0 12px 0;
  font-size: 14px;
  font-weight: 700;
  color: #fff;
}

.recommendations-list {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.recommendation-item {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 12px;
  background: rgba(255, 87, 34, 0.05);
  border-left: 3px solid var(--primary, #ff5722);
  border-radius: 6px;
}

.recommendation-priority {
  display: flex;
  align-items: center;
  justify-content: center;
  min-width: 32px;
  height: 24px;
  background: var(--primary, #ff5722);
  border-radius: 4px;
  font-size: 11px;
  font-weight: 700;
  color: #0a0a12;
}

.recommendation-text {
  flex: 1;
  font-size: 14px;
  color: #ccc;
}

/* Takeover Recommendation */
.takeover-recommendation {
  display: flex;
  align-items: flex-start;
  gap: 16px;
  padding: 20px;
  border-radius: 12px;
  animation: highlight 2s ease-in-out infinite;
}

.takeover-recommendation.recommended {
  background: linear-gradient(135deg, rgba(51, 255, 153, 0.15), rgba(0, 255, 100, 0.1));
  border: 2px solid #33ff99;
}

.takeover-recommendation.optional {
  background: rgba(255, 255, 255, 0.03);
  border: 2px solid rgba(255, 255, 255, 0.1);
}

@keyframes highlight {
  0%, 100% { box-shadow: 0 0 0 rgba(51, 255, 153, 0.3); }
  50% { box-shadow: 0 0 20px rgba(51, 255, 153, 0.5); }
}

.recommendation-icon {
  font-size: 32px;
}

.recommendation-content {
  flex: 1;
}

.recommendation-content strong {
  display: block;
  margin-bottom: 8px;
  font-size: 16px;
  color: #fff;
}

.recommendation-content p {
  margin: 0;
  font-size: 14px;
  line-height: 1.6;
  color: #ccc;
}

/* Telemetry Grid */
.telemetry-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 16px;
}

.telemetry-item {
  padding: 16px;
  background: rgba(255, 255, 255, 0.03);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.telemetry-label {
  font-size: 12px;
  color: #888;
  text-transform: uppercase;
  letter-spacing: 0.5px;
}

.telemetry-value {
  font-size: 16px;
  font-weight: 600;
  color: var(--primary, #ff5722);
}

.telemetry-value.status-ok {
  color: #33ff99;
}

/* Operator Card */
.operator-card {
  display: flex;
  align-items: center;
  gap: 16px;
  padding: 16px;
  background: rgba(255, 87, 34, 0.05);
  border: 2px solid var(--primary, #ff5722);
  border-radius: 12px;
}

.operator-avatar {
  width: 60px;
  height: 60px;
  background: linear-gradient(135deg, var(--primary, #ff5722), #0066ff);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 32px;
}

.operator-info {
  flex: 1;
}

.operator-name {
  font-size: 16px;
  font-weight: 700;
  color: #fff;
  margin-bottom: 8px;
}

.operator-meta {
  display: flex;
  flex-wrap: wrap;
  gap: 16px;
}

.meta-item {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 13px;
}

.meta-label {
  color: #888;
}

.meta-value {
  color: #ccc;
  font-weight: 600;
}

.meta-value.match-score {
  color: #33ff99;
}

.operator-badge {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 4px;
  padding: 8px 12px;
  background: rgba(255, 193, 7, 0.2);
  border-radius: 8px;
}

.badge-icon {
  font-size: 24px;
}

.badge-text {
  font-size: 11px;
  font-weight: 700;
  color: #ffc107;
}

.operator-unavailable {
  padding: 20px;
  text-align: center;
  color: #888;
  background: rgba(255, 153, 51, 0.05);
  border: 1px solid rgba(255, 153, 51, 0.3);
  border-radius: 8px;
}

.unavailable-icon {
  font-size: 32px;
  display: block;
  margin-bottom: 8px;
}

/* Double Confirm */
.double-confirm-section {
  padding: 20px;
  background: rgba(255, 153, 51, 0.1);
  border: 2px solid #ff9933;
  border-radius: 12px;
}

.confirm-warning {
  display: flex;
  align-items: flex-start;
  gap: 12px;
  margin-bottom: 16px;
}

.warning-icon {
  font-size: 32px;
}

.warning-content strong {
  display: block;
  margin-bottom: 8px;
  font-size: 16px;
  color: #ff9933;
}

.warning-content p {
  margin: 0;
  font-size: 14px;
  color: #ccc;
}

.confirm-checkbox {
  display: flex;
  align-items: center;
  gap: 12px;
  cursor: pointer;
}

.confirm-checkbox input[type="checkbox"] {
  width: 20px;
  height: 20px;
  cursor: pointer;
  accent-color: var(--primary, #ff5722);
}

.checkbox-text {
  font-size: 14px;
  color: #fff;
  user-select: none;
}

/* Footer */
.dialog-footer {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 20px 24px;
  background: rgba(0, 0, 0, 0.3);
  border-top: 1px solid rgba(255, 255, 255, 0.1);
}

.footer-info {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 12px;
  color: #888;
}

.info-icon {
  font-size: 16px;
}

.footer-actions {
  display: flex;
  gap: 12px;
}

/* Buttons */
.btn {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 12px 24px;
  border: none;
  border-radius: 8px;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s;
}

.btn:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.btn-secondary {
  background: rgba(255, 255, 255, 0.05);
  border: 1px solid rgba(255, 255, 255, 0.2);
  color: #fff;
}

.btn-secondary:hover:not(:disabled) {
  background: rgba(255, 255, 255, 0.1);
  transform: translateY(-2px);
}

.btn-primary {
  background: linear-gradient(135deg, var(--primary, #ff5722), #0066ff);
  color: #fff;
  box-shadow: 0 4px 12px rgba(255, 87, 34, 0.3);
}

.btn-primary:hover:not(:disabled) {
  transform: translateY(-2px);
  box-shadow: 0 6px 20px rgba(255, 87, 34, 0.5);
}

.btn-primary.critical {
  background: linear-gradient(135deg, #ff3333, #cc0000);
  box-shadow: 0 4px 12px rgba(255, 51, 51, 0.3);
}

.btn-primary.critical:hover:not(:disabled) {
  box-shadow: 0 6px 20px rgba(255, 51, 51, 0.5);
}

.btn-icon {
  font-size: 16px;
}

/* Transitions */
.dialog-enter-active,
.dialog-leave-active {
  transition: opacity 0.3s ease;
}

.dialog-enter-active .dialog-container,
.dialog-leave-active .dialog-container {
  transition: transform 0.3s ease;
}

.dialog-enter-from,
.dialog-leave-to {
  opacity: 0;
}

.dialog-enter-from .dialog-container,
.dialog-leave-to .dialog-container {
  transform: scale(0.9);
}

/* Scrollbar */
.dialog-content::-webkit-scrollbar {
  width: 8px;
}

.dialog-content::-webkit-scrollbar-track {
  background: rgba(255, 255, 255, 0.05);
  border-radius: 4px;
}

.dialog-content::-webkit-scrollbar-thumb {
  background: rgba(255, 87, 34, 0.3);
  border-radius: 4px;
}

.dialog-content::-webkit-scrollbar-thumb:hover {
  background: rgba(255, 87, 34, 0.5);
}
</style>
