<!--
  FSM-Pilot V2.0 - Remote Assistant Panel

  Real-time AI assistant providing continuous driving suggestions
  for selected vehicle during operation.

  Features:
  - Real-time driving suggestions
  - Context-aware recommendations
  - Risk warnings
  - Performance metrics
  - Operator guidance

  @author Li Yixiang
  @institution City University of Hong Kong
-->
<template>
  <div :class="['remote-assistant-panel', isExpanded ? 'expanded' : 'collapsed']">
    <!-- Toggle Button -->
    <button @click="toggleExpand" class="toggle-btn" title="展开/收起远程助手">
      <span class="toggle-icon">{{ isExpanded ? '◀' : '▶' }}</span>
      <span v-if="!isExpanded" class="toggle-label">远程助手</span>
    </button>

    <!-- Panel Content -->
    <div v-if="isExpanded" class="panel-content">
      <!-- Header -->
      <div class="panel-header">
        <div class="header-left">
          <div class="assistant-icon">
            <div class="icon-pulse"></div>
            <span class="icon-emoji">🤖</span>
          </div>
          <div class="header-text">
            <h3>Remote Assistant</h3>
            <p class="header-subtitle">实时驾驶建议</p>
          </div>
        </div>
        <div :class="['status-badge', isActive ? 'active' : 'inactive']">
          <span class="status-dot"></span>
          {{ isActive ? '运行中' : '待机' }}
        </div>
      </div>

      <!-- Vehicle Info -->
      <div v-if="vehicle" class="vehicle-info">
        <div class="info-header">
          <span class="info-icon">🚗</span>
          <span class="info-label">当前监控车辆</span>
        </div>
        <div class="info-content">
          <div class="vehicle-id">{{ vehicle.id }}</div>
          <div class="vehicle-meta">
            <span class="meta-tag">{{ scenarioText }}</span>
            <span class="meta-tag">{{ weatherText }}</span>
            <span :class="['meta-tag', 'urgency-' + vehicle.riskScore.urgencyLevel]">
              风险: {{ vehicle.riskScore.overallScore.toFixed(1) }}
            </span>
          </div>
        </div>
      </div>

      <!-- Suggestions Stream -->
      <div class="suggestions-container">
        <div class="suggestions-header">
          <span class="suggestions-icon">💡</span>
          <h4>实时建议</h4>
          <button @click="refreshSuggestions" class="refresh-btn" title="刷新">
            <span class="refresh-icon">🔄</span>
          </button>
        </div>

        <div class="suggestions-list">
          <TransitionGroup name="suggestion">
            <div
              v-for="suggestion in recentSuggestions"
              :key="suggestion.id"
              :class="['suggestion-card', 'priority-' + suggestion.priority]"
            >
              <div class="suggestion-header">
                <span class="suggestion-time">{{ formatTime(suggestion.timestamp) }}</span>
                <span :class="['suggestion-priority', 'priority-' + suggestion.priority]">
                  {{ priorityText(suggestion.priority) }}
                </span>
              </div>
              <div class="suggestion-content">
                <span class="suggestion-icon">{{ suggestion.icon }}</span>
                <p class="suggestion-text">{{ suggestion.text }}</p>
              </div>
              <div v-if="suggestion.action" class="suggestion-action">
                <button @click="applySuggestion(suggestion)" class="action-btn">
                  {{ suggestion.action }}
                </button>
              </div>
            </div>
          </TransitionGroup>

          <div v-if="recentSuggestions.length === 0" class="empty-suggestions">
            <span class="empty-icon">📭</span>
            <p>{{ vehicle ? '暂无新建议' : '请选择车辆以接收实时建议' }}</p>
          </div>
        </div>
      </div>

      <!-- Metrics -->
      <div class="metrics-section">
        <div class="metrics-header">
          <span class="metrics-icon">📊</span>
          <h4>关键指标</h4>
        </div>
        <div class="metrics-grid">
          <div class="metric-card">
            <span class="metric-label">建议采纳率</span>
            <span class="metric-value">{{ metrics.adoptionRate }}%</span>
          </div>
          <div class="metric-card">
            <span class="metric-label">风险降低</span>
            <span class="metric-value">-{{ metrics.riskReduction }}%</span>
          </div>
          <div class="metric-card">
            <span class="metric-label">响应时间</span>
            <span class="metric-value">{{ metrics.responseTime }}ms</span>
          </div>
          <div class="metric-card">
            <span class="metric-label">今日建议</span>
            <span class="metric-value">{{ metrics.totalSuggestions }}</span>
          </div>
        </div>
      </div>

      <!-- Quick Actions -->
      <div class="quick-actions">
        <button @click="requestEmergencyGuidance" class="quick-action-btn emergency">
          <span class="action-icon">🚨</span>
          <span class="action-text">紧急指导</span>
        </button>
        <button @click="requestRouteOptimization" class="quick-action-btn">
          <span class="action-icon">🗺️</span>
          <span class="action-text">路线优化</span>
        </button>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'
import type { Vehicle } from '@/types/dispatch'

// Props
interface Props {
  vehicle: Vehicle | null
  autoRefresh?: boolean
  refreshInterval?: number
}

const props = withDefaults(defineProps<Props>(), {
  autoRefresh: true,
  refreshInterval: 5000 // 5 seconds
})

// Emits
const emit = defineEmits<{
  applySuggestion: [suggestion: Suggestion]
  requestEmergency: []
  requestRouteOptimization: []
}>()

// Types
interface Suggestion {
  id: string
  timestamp: number
  priority: 'critical' | 'high' | 'medium' | 'low'
  icon: string
  text: string
  action?: string
  context?: any
}

interface Metrics {
  adoptionRate: number
  riskReduction: number
  responseTime: number
  totalSuggestions: number
}

// State
const isExpanded = ref(true)
const isActive = ref(false)
const recentSuggestions = ref<Suggestion[]>([])
const metrics = ref<Metrics>({
  adoptionRate: 87,
  riskReduction: 23,
  responseTime: 320,
  totalSuggestions: 142
})

let suggestionInterval: number | null = null
let suggestionIdCounter = 0

// Computed
const scenarioText = computed(() => {
  const scenarios: Record<string, string> = {
    'urban': '城市道路',
    'highway': '高速公路',
    'rural': '乡村道路',
    'parking': '停车场',
    'residential': '居民区'
  }
  return scenarios[props.vehicle?.scenario || 'urban'] || '未知场景'
})

const weatherText = computed(() => {
  const weather: Record<string, string> = {
    'clear': '晴朗☀️',
    'rainy': '雨天🌧️',
    'foggy': '雾天🌫️',
    'snowy': '雪天❄️'
  }
  return weather[props.vehicle?.weather || 'clear'] || '未知天气'
})

// Methods
function toggleExpand() {
  isExpanded.value = !isExpanded.value
}

function formatTime(timestamp: number): string {
  const date = new Date(timestamp)
  return date.toLocaleTimeString('zh-CN', {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit'
  })
}

function priorityText(priority: Suggestion['priority']): string {
  const texts = {
    critical: '紧急',
    high: '高优先级',
    medium: '中等',
    low: '提示'
  }
  return texts[priority]
}

function generateSuggestion(): Suggestion | null {
  if (!props.vehicle) return null

  const vehicle = props.vehicle
  const urgency = vehicle.riskScore.urgencyLevel
  const speed = vehicle.telemetry?.speed || 0

  // Generate contextual suggestions
  const suggestions: Omit<Suggestion, 'id' | 'timestamp'>[] = []

  // Critical suggestions for high-risk scenarios
  if (urgency === 'critical') {
    suggestions.push(
      {
        priority: 'critical',
        icon: '🚨',
        text: '检测到紧急情况，建议立即降低车速并准备接管',
        action: '立即接管'
      },
      {
        priority: 'critical',
        icon: '⚠️',
        text: '前方存在潜在碰撞风险，请确认周围环境安全',
        action: '查看详情'
      }
    )
  }

  // High priority suggestions
  if (urgency === 'high' || urgency === 'critical') {
    suggestions.push(
      {
        priority: 'high',
        icon: '🚦',
        text: `当前车速 ${speed} km/h 偏高，建议降至 ${Math.max(20, speed - 20)} km/h`,
        action: '应用建议'
      },
      {
        priority: 'high',
        icon: '👁️',
        text: '交通状况复杂，建议加强对车辆的监控频率',
        action: '增强监控'
      }
    )
  }

  // Weather-based suggestions
  if (vehicle.weather === 'rainy') {
    suggestions.push({
      priority: 'medium',
      icon: '🌧️',
      text: '雨天路况，建议保持更大的安全距离',
      action: '调整距离'
    })
  } else if (vehicle.weather === 'foggy') {
    suggestions.push({
      priority: 'high',
      icon: '🌫️',
      text: '能见度较低，建议降低车速并开启雾灯',
      action: '应用建议'
    })
  }

  // Scenario-based suggestions
  if (vehicle.scenario === 'urban') {
    suggestions.push(
      {
        priority: 'medium',
        icon: '🏙️',
        text: '城市路况复杂，注意行人和非机动车'
      },
      {
        priority: 'low',
        icon: '📍',
        text: '前方500米有红绿灯，建议提前减速'
      }
    )
  } else if (vehicle.scenario === 'highway') {
    suggestions.push({
      priority: 'medium',
      icon: '🛣️',
      text: '高速行驶，保持车道稳定性'
    })
  }

  // Control mode suggestions
  if (vehicle.controlMode === 'monitored') {
    suggestions.push({
      priority: 'low',
      icon: '🤖',
      text: '车辆运行正常，自动驾驶系统工作稳定'
    })
  }

  // Performance optimization
  suggestions.push(
    {
      priority: 'low',
      icon: '⚡',
      text: '当前路况良好，可以优化能源消耗',
      action: '优化路线'
    },
    {
      priority: 'low',
      icon: '📊',
      text: '车辆性能数据已记录，运行状态良好'
    }
  )

  // Select random suggestion
  if (suggestions.length === 0) return null

  const selected = suggestions[Math.floor(Math.random() * suggestions.length)]

  return {
    ...selected,
    id: `suggestion-${++suggestionIdCounter}`,
    timestamp: Date.now()
  }
}

function addSuggestion(suggestion: Suggestion) {
  recentSuggestions.value.unshift(suggestion)

  // Keep only recent 10 suggestions
  if (recentSuggestions.value.length > 10) {
    recentSuggestions.value = recentSuggestions.value.slice(0, 10)
  }

  // Update metrics
  metrics.value.totalSuggestions++
}

function refreshSuggestions() {
  const suggestion = generateSuggestion()
  if (suggestion) {
    addSuggestion(suggestion)
  }
}

function applySuggestion(suggestion: Suggestion) {
  emit('applySuggestion', suggestion)

  // Update metrics
  metrics.value.adoptionRate = Math.min(100, metrics.value.adoptionRate + 1)

  // Mark as applied (visual feedback)
  const index = recentSuggestions.value.findIndex(s => s.id === suggestion.id)
  if (index !== -1) {
    recentSuggestions.value.splice(index, 1)
  }
}

function requestEmergencyGuidance() {
  emit('requestEmergency')

  // Add emergency suggestion
  addSuggestion({
    id: `suggestion-${++suggestionIdCounter}`,
    timestamp: Date.now(),
    priority: 'critical',
    icon: '🚨',
    text: 'AI 紧急分析已启动，正在评估最佳应对方案'
  })
}

function requestRouteOptimization() {
  emit('requestRouteOptimization')

  addSuggestion({
    id: `suggestion-${++suggestionIdCounter}`,
    timestamp: Date.now(),
    priority: 'medium',
    icon: '🗺️',
    text: '路线优化请求已提交，正在计算最优路径'
  })
}

function startSuggestionStream() {
  if (suggestionInterval) {
    clearInterval(suggestionInterval)
  }

  if (props.autoRefresh && props.vehicle) {
    isActive.value = true
    suggestionInterval = window.setInterval(() => {
      const suggestion = generateSuggestion()
      if (suggestion && Math.random() > 0.3) { // 70% chance to generate
        addSuggestion(suggestion)
      }
    }, props.refreshInterval)
  } else {
    isActive.value = false
  }
}

function stopSuggestionStream() {
  if (suggestionInterval) {
    clearInterval(suggestionInterval)
    suggestionInterval = null
  }
  isActive.value = false
}

// Watchers
watch(() => props.vehicle, (newVehicle) => {
  if (newVehicle) {
    startSuggestionStream()

    // Generate initial suggestion
    const suggestion = generateSuggestion()
    if (suggestion) {
      addSuggestion(suggestion)
    }
  } else {
    stopSuggestionStream()
    recentSuggestions.value = []
  }
}, { immediate: true })

watch(() => props.autoRefresh, () => {
  if (props.autoRefresh && props.vehicle) {
    startSuggestionStream()
  } else {
    stopSuggestionStream()
  }
})

// Lifecycle
onMounted(() => {
  if (props.vehicle) {
    startSuggestionStream()
  }
})

onUnmounted(() => {
  stopSuggestionStream()
})
</script>

<style scoped>
/* Panel Container */
.remote-assistant-panel {
  position: fixed;
  top: 80px;
  right: 0;
  height: calc(100vh - 80px);
  background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
  border-left: 2px solid rgba(255, 87, 34, 0.3);
  box-shadow: -4px 0 20px rgba(0, 0, 0, 0.5);
  transition: all 0.3s ease;
  z-index: 1000;
  display: flex;
  flex-direction: column;
}

.remote-assistant-panel.expanded {
  width: 400px;
}

.remote-assistant-panel.collapsed {
  width: 48px;
}

/* Toggle Button */
.toggle-btn {
  position: absolute;
  top: 20px;
  left: -48px;
  width: 48px;
  height: 48px;
  background: linear-gradient(135deg, var(--primary, #ff5722), #0066ff);
  border: none;
  border-radius: 8px 0 0 8px;
  color: #fff;
  font-size: 20px;
  cursor: pointer;
  transition: all 0.2s;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 4px;
  box-shadow: -2px 2px 8px rgba(0, 0, 0, 0.3);
}

.toggle-btn:hover {
  background: linear-gradient(135deg, #00d4e6, #0055dd);
  transform: translateX(-4px);
}

.toggle-icon {
  font-size: 16px;
}

.toggle-label {
  writing-mode: vertical-rl;
  font-size: 12px;
  font-weight: 600;
  letter-spacing: 2px;
}

/* Panel Content */
.panel-content {
  flex: 1;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

/* Panel Header */
.panel-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 20px;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.header-left {
  display: flex;
  align-items: center;
  gap: 12px;
}

.assistant-icon {
  position: relative;
  font-size: 32px;
  animation: float 3s ease-in-out infinite;
}

.icon-pulse {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  width: 48px;
  height: 48px;
  border-radius: 50%;
  background: radial-gradient(circle, rgba(255, 87, 34, 0.3), transparent);
  animation: pulse 2s ease-in-out infinite;
}

@keyframes float {
  0%, 100% { transform: translateY(0); }
  50% { transform: translateY(-5px); }
}

@keyframes pulse {
  0%, 100% { transform: translate(-50%, -50%) scale(1); opacity: 0.5; }
  50% { transform: translate(-50%, -50%) scale(1.2); opacity: 0.8; }
}

.icon-emoji {
  position: relative;
  z-index: 1;
}

.header-text h3 {
  margin: 0;
  font-size: 18px;
  font-weight: 700;
  color: #fff;
}

.header-subtitle {
  margin: 4px 0 0 0;
  font-size: 12px;
  color: #888;
}

.status-badge {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 4px 12px;
  border-radius: 12px;
  font-size: 11px;
  font-weight: 600;
}

.status-badge.active {
  background: rgba(51, 255, 153, 0.2);
  color: #33ff99;
}

.status-badge.inactive {
  background: rgba(136, 136, 136, 0.2);
  color: #888;
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

/* Vehicle Info */
.vehicle-info {
  padding: 16px 20px;
  background: rgba(255, 87, 34, 0.05);
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.info-header {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 12px;
  font-size: 12px;
  color: #888;
  text-transform: uppercase;
  letter-spacing: 0.5px;
}

.info-icon {
  font-size: 16px;
}

.info-content {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.vehicle-id {
  font-size: 16px;
  font-weight: 700;
  font-family: 'Monaco', monospace;
  color: var(--primary, #ff5722);
}

.vehicle-meta {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
}

.meta-tag {
  padding: 4px 8px;
  background: rgba(255, 255, 255, 0.05);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 4px;
  font-size: 11px;
  color: #ccc;
}

.meta-tag.urgency-critical {
  background: rgba(255, 51, 51, 0.2);
  border-color: #ff3333;
  color: #ff3333;
}

.meta-tag.urgency-high {
  background: rgba(255, 153, 51, 0.2);
  border-color: #ff9933;
  color: #ff9933;
}

.meta-tag.urgency-medium {
  background: rgba(255, 221, 51, 0.2);
  border-color: #ffdd33;
  color: #ffdd33;
}

.meta-tag.urgency-low {
  background: rgba(51, 255, 153, 0.2);
  border-color: #33ff99;
  color: #33ff99;
}

/* Suggestions Container */
.suggestions-container {
  flex: 1;
  display: flex;
  flex-direction: column;
  overflow: hidden;
  padding: 16px 20px;
}

.suggestions-header {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 12px;
}

.suggestions-icon {
  font-size: 20px;
}

.suggestions-header h4 {
  flex: 1;
  margin: 0;
  font-size: 14px;
  font-weight: 700;
  color: #fff;
}

.refresh-btn {
  width: 28px;
  height: 28px;
  background: rgba(255, 255, 255, 0.05);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 6px;
  color: #fff;
  font-size: 14px;
  cursor: pointer;
  transition: all 0.2s;
}

.refresh-btn:hover {
  background: rgba(255, 87, 34, 0.2);
  border-color: var(--primary, #ff5722);
  transform: rotate(180deg);
}

.refresh-icon {
  display: block;
}

/* Suggestions List */
.suggestions-list {
  flex: 1;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.suggestion-card {
  padding: 12px;
  background: rgba(255, 255, 255, 0.03);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  transition: all 0.2s;
  animation: slideInRight 0.3s ease;
}

@keyframes slideInRight {
  from {
    opacity: 0;
    transform: translateX(20px);
  }
  to {
    opacity: 1;
    transform: translateX(0);
  }
}

.suggestion-card:hover {
  background: rgba(255, 255, 255, 0.05);
  border-color: var(--primary, #ff5722);
  transform: translateX(-4px);
}

.suggestion-card.priority-critical {
  border-left: 3px solid #ff3333;
  background: rgba(255, 51, 51, 0.05);
}

.suggestion-card.priority-high {
  border-left: 3px solid #ff9933;
  background: rgba(255, 153, 51, 0.05);
}

.suggestion-card.priority-medium {
  border-left: 3px solid #ffdd33;
}

.suggestion-card.priority-low {
  border-left: 3px solid #33ff99;
}

.suggestion-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-bottom: 8px;
}

.suggestion-time {
  font-size: 10px;
  color: #666;
  font-family: 'Monaco', monospace;
}

.suggestion-priority {
  font-size: 10px;
  font-weight: 600;
  padding: 2px 6px;
  border-radius: 4px;
}

.suggestion-priority.priority-critical {
  background: rgba(255, 51, 51, 0.2);
  color: #ff3333;
}

.suggestion-priority.priority-high {
  background: rgba(255, 153, 51, 0.2);
  color: #ff9933;
}

.suggestion-priority.priority-medium {
  background: rgba(255, 221, 51, 0.2);
  color: #ffdd33;
}

.suggestion-priority.priority-low {
  background: rgba(51, 255, 153, 0.2);
  color: #33ff99;
}

.suggestion-content {
  display: flex;
  align-items: flex-start;
  gap: 8px;
}

.suggestion-icon {
  font-size: 20px;
  flex-shrink: 0;
}

.suggestion-text {
  flex: 1;
  margin: 0;
  font-size: 13px;
  line-height: 1.5;
  color: #ccc;
}

.suggestion-action {
  margin-top: 8px;
}

.action-btn {
  width: 100%;
  padding: 6px 12px;
  background: rgba(255, 87, 34, 0.1);
  border: 1px solid var(--primary, #ff5722);
  border-radius: 4px;
  color: var(--primary, #ff5722);
  font-size: 12px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s;
}

.action-btn:hover {
  background: rgba(255, 87, 34, 0.2);
  transform: scale(1.02);
}

/* Empty State */
.empty-suggestions {
  padding: 40px 20px;
  text-align: center;
  color: #666;
}

.empty-icon {
  font-size: 48px;
  display: block;
  margin-bottom: 12px;
  opacity: 0.3;
}

.empty-suggestions p {
  margin: 0;
  font-size: 13px;
}

/* Metrics Section */
.metrics-section {
  padding: 16px 20px;
  background: rgba(0, 0, 0, 0.2);
  border-top: 1px solid rgba(255, 255, 255, 0.1);
}

.metrics-header {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 12px;
}

.metrics-icon {
  font-size: 20px;
}

.metrics-header h4 {
  margin: 0;
  font-size: 14px;
  font-weight: 700;
  color: #fff;
}

.metrics-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 12px;
}

.metric-card {
  padding: 12px;
  background: rgba(255, 255, 255, 0.03);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 6px;
  display: flex;
  flex-direction: column;
  gap: 6px;
}

.metric-label {
  font-size: 11px;
  color: #888;
}

.metric-value {
  font-size: 18px;
  font-weight: 700;
  color: var(--primary, #ff5722);
  font-family: 'Monaco', monospace;
}

/* Quick Actions */
.quick-actions {
  padding: 16px 20px;
  background: rgba(0, 0, 0, 0.3);
  border-top: 1px solid rgba(255, 255, 255, 0.1);
  display: flex;
  gap: 12px;
}

.quick-action-btn {
  flex: 1;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 6px;
  padding: 12px;
  background: rgba(255, 87, 34, 0.1);
  border: 1px solid var(--primary, #ff5722);
  border-radius: 8px;
  color: var(--primary, #ff5722);
  font-size: 12px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s;
}

.quick-action-btn:hover {
  background: rgba(255, 87, 34, 0.2);
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(255, 87, 34, 0.3);
}

.quick-action-btn.emergency {
  background: rgba(255, 51, 51, 0.1);
  border-color: #ff3333;
  color: #ff3333;
}

.quick-action-btn.emergency:hover {
  background: rgba(255, 51, 51, 0.2);
  box-shadow: 0 4px 12px rgba(255, 51, 51, 0.3);
}

.action-icon {
  font-size: 24px;
}

.action-text {
  font-size: 11px;
}

/* Transitions */
.suggestion-enter-active,
.suggestion-leave-active {
  transition: all 0.3s ease;
}

.suggestion-enter-from {
  opacity: 0;
  transform: translateX(20px);
}

.suggestion-leave-to {
  opacity: 0;
  transform: translateX(-20px);
}

.suggestion-move {
  transition: transform 0.3s ease;
}

/* Scrollbar */
.suggestions-list::-webkit-scrollbar {
  width: 6px;
}

.suggestions-list::-webkit-scrollbar-track {
  background: rgba(255, 255, 255, 0.05);
  border-radius: 3px;
}

.suggestions-list::-webkit-scrollbar-thumb {
  background: rgba(255, 87, 34, 0.3);
  border-radius: 3px;
}

.suggestions-list::-webkit-scrollbar-thumb:hover {
  background: rgba(255, 87, 34, 0.5);
}
</style>
