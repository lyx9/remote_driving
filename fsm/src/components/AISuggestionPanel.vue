<!--
  FSM-Pilot V2.0 - AI Suggestion Panel

  Beautiful AI-powered suggestion panel with:
  - Doubao LLM integration for scenario analysis
  - Real-time driving suggestions
  - Risk factor visualization
  - Operator guidance

  @author Li Yixiang
  @institution City University of Hong Kong
-->
<template>
  <div :class="['ai-suggestion-panel', `urgency-${urgency}`]">
    <!-- Header -->
    <div class="panel-header">
      <div class="header-left">
        <div class="ai-icon">
          <div class="icon-pulse"></div>
          🤖
        </div>
        <div class="header-text">
          <h3>AI智能分析</h3>
          <p v-if="analysis">{{ llmEnabled ? '豆包大模型' : '本地分析' }}</p>
        </div>
      </div>

      <div class="header-right">
        <div :class="['status-indicator', llmEnabled ? 'online' : 'offline']">
          <span class="status-dot"></span>
          {{ llmEnabled ? '在线' : '离线' }}
        </div>
        <div v-if="analysis" class="processing-time">
          {{ analysis.processingTime.toFixed(0) }}ms
        </div>
      </div>
    </div>

    <!-- Loading State -->
    <div v-if="loading" class="loading-state">
      <div class="loading-spinner"></div>
      <p>AI分析中...</p>
    </div>

    <!-- Error State -->
    <div v-else-if="error" class="error-state">
      <div class="error-icon">⚠️</div>
      <p>{{ error }}</p>
      <button @click="retry" class="retry-btn">重试</button>
    </div>

    <!-- Analysis Content -->
    <div v-else-if="analysis" class="analysis-content">
      <!-- Urgency Badge -->
      <div :class="['urgency-badge', urgency]">
        <span class="urgency-icon">{{ urgencyIcon }}</span>
        <span class="urgency-text">{{ urgencyText }}</span>
      </div>

      <!-- Scenario Description -->
      <div class="section scenario-section">
        <div class="section-header">
          <span class="section-icon">🚗</span>
          <h4>场景描述</h4>
        </div>
        <p class="scenario-text">{{ analysis.scenarioDescription }}</p>
      </div>

      <!-- Risk Explanation -->
      <div class="section risk-section">
        <div class="section-header">
          <span class="section-icon">⚠️</span>
          <h4>风险分析</h4>
        </div>
        <p class="risk-text">{{ analysis.riskExplanation }}</p>

        <!-- Key Factors -->
        <div v-if="analysis.keyFactors.length > 0" class="key-factors">
          <div
            v-for="(factor, index) in analysis.keyFactors"
            :key="index"
            class="factor-tag"
          >
            {{ factor }}
          </div>
        </div>
      </div>

      <!-- Driving Suggestions -->
      <div class="section suggestions-section">
        <div class="section-header">
          <span class="section-icon">💡</span>
          <h4>驾驶建议</h4>
        </div>
        <div class="suggestions-list">
          <div
            v-for="(suggestion, index) in analysis.drivingSuggestions"
            :key="index"
            class="suggestion-item"
          >
            <span class="suggestion-number">{{ index + 1 }}</span>
            <span class="suggestion-text">{{ suggestion }}</span>
          </div>
        </div>
      </div>

      <!-- Operator Guidance -->
      <div class="section guidance-section">
        <div class="section-header">
          <span class="section-icon">👨‍✈️</span>
          <h4>安全员指导</h4>
        </div>
        <div class="guidance-box">
          <p class="guidance-text">{{ analysis.operatorGuidance }}</p>
        </div>
      </div>

      <!-- Footer -->
      <div class="panel-footer">
        <div class="update-time">
          更新于 {{ formatTime(analysis.timestamp) }}
        </div>
        <button @click="refresh" class="refresh-btn">
          🔄 刷新分析
        </button>
      </div>
    </div>

    <!-- Empty State -->
    <div v-else class="empty-state">
      <div class="empty-icon">🤖</div>
      <p>暂无分析数据</p>
      <p class="empty-hint">选择车辆以获取AI分析</p>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, watch, onUnmounted } from 'vue'
import { getDoubaoLLMService } from '@/services/doubaoLLMService'
import type { LLMAnalysis, ScenarioContext } from '@/services/doubaoLLMService'

// Props
interface Props {
  context?: ScenarioContext
  autoRefresh?: boolean
  refreshInterval?: number
}

const props = withDefaults(defineProps<Props>(), {
  autoRefresh: false,
  refreshInterval: 30000
})

// Services
const doubaoService = getDoubaoLLMService()

// State
const analysis = ref<LLMAnalysis | null>(null)
const loading = ref(false)
const error = ref<string | null>(null)
const llmEnabled = ref(doubaoService.isAvailable())

let refreshTimer: number | null = null

// Computed
const urgency = computed(() => analysis.value?.urgencyLevel || 'low')

const urgencyIcon = computed(() => {
  const icons = {
    critical: '🚨',
    high: '⚠️',
    medium: '⚡',
    low: '✅'
  }
  return icons[urgency.value]
})

const urgencyText = computed(() => {
  const texts = {
    critical: '紧急情况',
    high: '高风险',
    medium: '中等风险',
    low: '低风险'
  }
  return texts[urgency.value]
})

// Methods
async function analyzeContext() {
  if (!props.context) {
    analysis.value = null
    return
  }

  loading.value = true
  error.value = null

  try {
    analysis.value = await doubaoService.analyzeScenario(props.context)
  } catch (err) {
    console.error('[AI Panel] Analysis failed:', err)
    error.value = err instanceof Error ? err.message : 'AI分析失败'
  } finally {
    loading.value = false
  }
}

function retry() {
  error.value = null
  analyzeContext()
}

function refresh() {
  analyzeContext()
}

function formatTime(timestamp: number): string {
  const date = new Date(timestamp)
  return date.toLocaleTimeString('zh-CN', {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit'
  })
}

function startAutoRefresh() {
  if (refreshTimer) {
    clearInterval(refreshTimer)
  }

  if (props.autoRefresh && props.context) {
    refreshTimer = window.setInterval(() => {
      analyzeContext()
    }, props.refreshInterval)
  }
}

function stopAutoRefresh() {
  if (refreshTimer) {
    clearInterval(refreshTimer)
    refreshTimer = null
  }
}

// Watchers
watch(() => props.context, (newContext) => {
  if (newContext) {
    analyzeContext()
    startAutoRefresh()
  } else {
    analysis.value = null
    stopAutoRefresh()
  }
}, { immediate: true })

// Lifecycle
onUnmounted(() => {
  stopAutoRefresh()
})
</script>

<style scoped>
.ai-suggestion-panel {
  background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
  border-radius: 12px;
  overflow: hidden;
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
  transition: all 0.3s ease;
}

.ai-suggestion-panel.urgency-critical {
  border: 2px solid #ff3333;
  box-shadow: 0 8px 32px rgba(255, 51, 51, 0.4);
}

.ai-suggestion-panel.urgency-high {
  border: 2px solid #ff9933;
  box-shadow: 0 8px 32px rgba(255, 153, 51, 0.3);
}

.ai-suggestion-panel.urgency-medium {
  border: 2px solid #ffdd33;
}

.ai-suggestion-panel.urgency-low {
  border: 2px solid #33ff99;
}

.panel-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 20px;
  background: rgba(0, 0, 0, 0.3);
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.header-left {
  display: flex;
  align-items: center;
  gap: 12px;
}

.ai-icon {
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

.header-text h3 {
  margin: 0;
  font-size: 18px;
  font-weight: 700;
  color: #fff;
}

.header-text p {
  margin: 4px 0 0 0;
  font-size: 12px;
  color: #888;
}

.header-right {
  display: flex;
  align-items: center;
  gap: 12px;
}

.status-indicator {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 4px 12px;
  border-radius: 12px;
  font-size: 12px;
  font-weight: 600;
}

.status-indicator.online {
  background: rgba(51, 255, 153, 0.2);
  color: #33ff99;
}

.status-indicator.offline {
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

.processing-time {
  padding: 4px 12px;
  background: rgba(255, 87, 34, 0.1);
  border-radius: 8px;
  font-size: 11px;
  font-weight: 600;
  color: var(--primary, #ff5722);
}

.loading-state {
  padding: 60px 20px;
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

.error-state {
  padding: 60px 20px;
  text-align: center;
  color: #ff3333;
}

.error-icon {
  font-size: 48px;
  margin-bottom: 16px;
}

.retry-btn {
  margin-top: 16px;
  padding: 8px 24px;
  background: rgba(255, 51, 51, 0.2);
  border: 1px solid #ff3333;
  border-radius: 6px;
  color: #ff3333;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s;
}

.retry-btn:hover {
  background: rgba(255, 51, 51, 0.3);
  transform: translateY(-2px);
}

.analysis-content {
  padding: 20px;
}

.urgency-badge {
  display: inline-flex;
  align-items: center;
  gap: 8px;
  padding: 8px 16px;
  border-radius: 20px;
  font-size: 14px;
  font-weight: 700;
  margin-bottom: 20px;
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

.urgency-badge.critical {
  background: linear-gradient(135deg, rgba(255, 51, 51, 0.3), rgba(255, 0, 0, 0.2));
  border: 2px solid #ff3333;
  color: #ff3333;
  box-shadow: 0 0 20px rgba(255, 51, 51, 0.3);
}

.urgency-badge.high {
  background: linear-gradient(135deg, rgba(255, 153, 51, 0.3), rgba(255, 100, 0, 0.2));
  border: 2px solid #ff9933;
  color: #ff9933;
}

.urgency-badge.medium {
  background: linear-gradient(135deg, rgba(255, 221, 51, 0.3), rgba(255, 200, 0, 0.2));
  border: 2px solid #ffdd33;
  color: #ffdd33;
}

.urgency-badge.low {
  background: linear-gradient(135deg, rgba(51, 255, 153, 0.3), rgba(0, 255, 100, 0.2));
  border: 2px solid #33ff99;
  color: #33ff99;
}

.urgency-icon {
  font-size: 20px;
}

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
  margin-bottom: 12px;
}

.section-icon {
  font-size: 20px;
}

.section-header h4 {
  margin: 0;
  font-size: 16px;
  font-weight: 700;
  color: #fff;
}

.scenario-text {
  margin: 0;
  padding: 16px;
  background: rgba(255, 87, 34, 0.05);
  border-left: 3px solid var(--primary, #ff5722);
  border-radius: 6px;
  font-size: 15px;
  line-height: 1.6;
  color: #ccc;
}

.risk-text {
  margin: 0 0 12px 0;
  padding: 16px;
  background: rgba(255, 153, 51, 0.05);
  border-left: 3px solid #ff9933;
  border-radius: 6px;
  font-size: 14px;
  line-height: 1.6;
  color: #ccc;
}

.key-factors {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
}

.factor-tag {
  padding: 6px 12px;
  background: rgba(255, 255, 255, 0.05);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 12px;
  font-size: 12px;
  color: #888;
  transition: all 0.2s;
}

.factor-tag:hover {
  background: rgba(255, 255, 255, 0.1);
  border-color: var(--primary, #ff5722);
  color: var(--primary, #ff5722);
  transform: translateY(-2px);
}

.suggestions-list {
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.suggestion-item {
  display: flex;
  align-items: flex-start;
  gap: 12px;
  padding: 12px;
  background: rgba(51, 255, 153, 0.05);
  border-left: 3px solid #33ff99;
  border-radius: 6px;
  transition: all 0.2s;
}

.suggestion-item:hover {
  background: rgba(51, 255, 153, 0.1);
  transform: translateX(4px);
}

.suggestion-number {
  display: flex;
  align-items: center;
  justify-content: center;
  min-width: 24px;
  height: 24px;
  background: #33ff99;
  border-radius: 50%;
  font-size: 12px;
  font-weight: 700;
  color: #0a0a12;
}

.suggestion-text {
  flex: 1;
  font-size: 14px;
  line-height: 1.6;
  color: #ccc;
}

.guidance-box {
  padding: 16px;
  background: linear-gradient(135deg, rgba(255, 87, 34, 0.1), rgba(100, 100, 255, 0.05));
  border: 2px solid var(--primary, #ff5722);
  border-radius: 8px;
}

.guidance-text {
  margin: 0;
  font-size: 15px;
  font-weight: 600;
  line-height: 1.6;
  color: var(--primary, #ff5722);
}

.panel-footer {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px 0 0 0;
  border-top: 1px solid rgba(255, 255, 255, 0.1);
}

.update-time {
  font-size: 12px;
  color: #666;
}

.refresh-btn {
  padding: 8px 16px;
  background: rgba(255, 87, 34, 0.1);
  border: 1px solid var(--primary, #ff5722);
  border-radius: 6px;
  color: var(--primary, #ff5722);
  font-size: 12px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s;
}

.refresh-btn:hover {
  background: rgba(255, 87, 34, 0.2);
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(255, 87, 34, 0.3);
}

.empty-state {
  padding: 60px 20px;
  text-align: center;
  color: #666;
}

.empty-icon {
  font-size: 64px;
  margin-bottom: 16px;
  opacity: 0.3;
}

.empty-state p {
  margin: 8px 0;
  font-size: 14px;
}

.empty-hint {
  font-size: 12px;
  color: #444;
}
</style>
