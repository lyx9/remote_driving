<!--
  Guardian Mobility - AI Driving Suggestions Component

  @project     Guardian Mobility Remote Driving Platform
  @author      Li Yixiang
  @institution City University of Hong Kong
  @description AI real-time driving suggestions component, displaying intelligent driving tips and safety recommendations
-->
<template>
  <Transition name="slide-down">
    <div v-if="systemStore.ui.showAIBar" class="ai-suggestions">
      <!-- Vehicle ID and Status -->
      <div class="tag vehicle-id">
        <span class="label">ID:</span>
        <span class="value">{{ fleetStore.currentVehicle.id }}</span>
      </div>

      <div class="tag profit">
        <span class="label">PROFIT:</span>
        <span class="value">${{ fleetStore.currentVehicle.money.toFixed(2) }}</span>
      </div>

      <!-- AI Confidence Bar -->
      <div class="confidence-section">
        <span class="confidence-label">AI CONFIDENCE</span>
        <div class="confidence-track">
          <div
            class="confidence-fill"
            :style="{ width: systemStore.aiConfidence + '%' }"
            :class="confidenceClass"
          ></div>
        </div>
        <span class="confidence-value">{{ systemStore.aiConfidence.toFixed(0) }}%</span>
      </div>

      <!-- AI Driving Suggestions -->
      <div class="suggestions-container">
        <div class="suggestion-icon" :class="suggestionStatus">
          <span v-if="suggestionStatus === 'normal'">✓</span>
          <span v-else-if="suggestionStatus === 'warning'">⚠</span>
          <span v-else>ℹ</span>
        </div>
        <div class="suggestion-text">
          <span class="suggestion-message">{{ currentSuggestion }}</span>
        </div>
      </div>
    </div>
  </Transition>
</template>

<script setup lang="ts">
import { ref, computed, watch, onMounted, onUnmounted } from 'vue'
import { useFleetStore } from '@/stores/fleet'
import { useSystemStore } from '@/stores/system'

const fleetStore = useFleetStore()
const systemStore = useSystemStore()

// AI Suggestion state
const currentSuggestion = ref('System initializing...')
const suggestionStatus = ref<'normal' | 'warning' | 'info'>('info')

// Confidence class based on AI confidence level
const confidenceClass = computed(() => {
  const confidence = systemStore.aiConfidence
  if (confidence >= 80) return 'confidence-high'
  if (confidence >= 60) return 'confidence-medium'
  return 'confidence-low'
})

// AI Suggestions based on vehicle state
const generateSuggestion = () => {
  const vehicle = fleetStore.currentVehicle
  const speed = vehicle.speed
  const confidence = systemStore.aiConfidence

  // Check vehicle status
  if (vehicle.status !== 'ACTIVE') {
    currentSuggestion.value = 'Vehicle inactive, please activate remote driving system'
    suggestionStatus.value = 'info'
    return
  }

  // Check if receiving video/data
  const hasVideo = true // Assume video is streaming
  const hasData = speed !== undefined && speed >= 0

  if (!hasVideo || !hasData) {
    currentSuggestion.value = 'Waiting for video and telemetry data...'
    suggestionStatus.value = 'info'
    return
  }

  // Normal operation - provide driving suggestions
  if (confidence >= 85) {
    if (speed > 50) {
      currentSuggestion.value = 'System normal, high speed detected, drive carefully'
      suggestionStatus.value = 'warning'
    } else if (speed > 30) {
      currentSuggestion.value = 'System normal, maintain safe speed within limits'
      suggestionStatus.value = 'normal'
    } else if (speed > 0) {
      currentSuggestion.value = 'System normal, speed moderate, maintain safe driving'
      suggestionStatus.value = 'normal'
    } else {
      currentSuggestion.value = 'System normal, vehicle stationary, ready to start'
      suggestionStatus.value = 'normal'
    }
  } else if (confidence >= 70) {
    currentSuggestion.value = 'AI confidence medium, drive cautiously and stay alert'
    suggestionStatus.value = 'warning'
  } else {
    currentSuggestion.value = 'AI confidence low, reduce speed or switch to manual mode'
    suggestionStatus.value = 'warning'
  }
}

// Watch for changes in vehicle state
watch(
  () => [fleetStore.currentVehicle.status, fleetStore.currentVehicle.speed, systemStore.aiConfidence],
  () => {
    generateSuggestion()
  },
  { deep: true }
)

// Update suggestions periodically
let suggestionInterval: number | null = null

onMounted(() => {
  generateSuggestion()

  // Update suggestions every 3 seconds
  suggestionInterval = window.setInterval(() => {
    generateSuggestion()
  }, 3000)
})

onUnmounted(() => {
  if (suggestionInterval) {
    clearInterval(suggestionInterval)
  }
})
</script>

<style scoped>
.ai-suggestions {
  height: var(--ai-h);
  background: linear-gradient(135deg, #080c12 0%, #0f1419 100%);
  border-bottom: 1px solid var(--border);
  display: flex;
  align-items: center;
  padding: 0 1.5rem;
  gap: 1.5rem;
  flex-shrink: 0;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.3);
}

.tag {
  background: rgba(255, 87, 34, 0.1);
  border: 1px solid rgba(255, 87, 34, 0.3);
  padding: 0.4rem 1rem;
  border-radius: 6px;
  display: flex;
  gap: 0.5rem;
  align-items: center;
  font-size: 0.875rem;
  font-weight: 600;
  transition: all 0.3s ease;
}

.tag:hover {
  background: rgba(255, 87, 34, 0.15);
  border-color: rgba(255, 87, 34, 0.5);
}

.tag .label {
  color: var(--primary);
  font-weight: 700;
}

.tag .value {
  color: #e0e6ed;
}

.vehicle-id {
  min-width: 120px;
}

.profit {
  min-width: 140px;
}

.profit .value {
  color: #10b981;
}

/* Confidence Section */
.confidence-section {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  min-width: 250px;
}

.confidence-label {
  font-size: 0.75rem;
  font-weight: 700;
  color: #94a3b8;
  white-space: nowrap;
  letter-spacing: 0.5px;
}

.confidence-track {
  flex: 1;
  height: 6px;
  background: rgba(255, 255, 255, 0.1);
  border-radius: 3px;
  overflow: hidden;
  position: relative;
}

.confidence-fill {
  height: 100%;
  transition: width 0.5s ease, background 0.3s ease;
  border-radius: 3px;
  position: relative;
}

.confidence-fill::after {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.3), transparent);
  animation: shimmer 2s infinite;
}

@keyframes shimmer {
  0% { transform: translateX(-100%); }
  100% { transform: translateX(100%); }
}

.confidence-high {
  background: linear-gradient(90deg, #10b981 0%, #059669 100%);
}

.confidence-medium {
  background: linear-gradient(90deg, #f59e0b 0%, #d97706 100%);
}

.confidence-low {
  background: linear-gradient(90deg, #ef4444 0%, #dc2626 100%);
}

.confidence-value {
  font-size: 0.875rem;
  font-weight: 700;
  color: #e0e6ed;
  min-width: 45px;
  text-align: right;
}

/* Suggestions Container */
.suggestions-container {
  flex: 1;
  display: flex;
  align-items: center;
  gap: 1rem;
  background: rgba(255, 87, 34, 0.05);
  border: 1px solid rgba(255, 87, 34, 0.2);
  border-radius: 8px;
  padding: 0.6rem 1.2rem;
  min-height: 48px;
  transition: all 0.3s ease;
}

.suggestions-container:hover {
  background: rgba(255, 87, 34, 0.08);
  border-color: rgba(255, 87, 34, 0.3);
}

.suggestion-icon {
  width: 32px;
  height: 32px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 1.2rem;
  font-weight: 700;
  flex-shrink: 0;
  transition: all 0.3s ease;
}

.suggestion-icon.normal {
  background: rgba(16, 185, 129, 0.2);
  color: #10b981;
  border: 2px solid #10b981;
}

.suggestion-icon.warning {
  background: rgba(245, 158, 11, 0.2);
  color: #f59e0b;
  border: 2px solid #f59e0b;
  animation: pulse-warning 2s infinite;
}

.suggestion-icon.info {
  background: rgba(59, 130, 246, 0.2);
  color: #3b82f6;
  border: 2px solid #3b82f6;
}

@keyframes pulse-warning {
  0%, 100% {
    transform: scale(1);
    opacity: 1;
  }
  50% {
    transform: scale(1.05);
    opacity: 0.9;
  }
}

.suggestion-text {
  flex: 1;
  display: flex;
  align-items: center;
}

.suggestion-message {
  font-size: 0.95rem;
  font-weight: 600;
  color: #e0e6ed;
  line-height: 1.4;
  letter-spacing: 0.3px;
}

/* Animations */
.slide-down-enter-active,
.slide-down-leave-active {
  transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
}

.slide-down-enter-from {
  transform: translateY(-100%);
  opacity: 0;
}

.slide-down-leave-to {
  transform: translateY(-100%);
  opacity: 0;
}

/* Responsive */
@media (max-width: 1400px) {
  .ai-suggestions {
    gap: 1rem;
    padding: 0 1rem;
  }

  .confidence-section {
    min-width: 200px;
  }

  .tag {
    padding: 0.3rem 0.8rem;
    font-size: 0.8rem;
  }

  .suggestion-message {
    font-size: 0.875rem;
  }
}

@media (max-width: 1200px) {
  .confidence-label {
    display: none;
  }

  .profit {
    display: none;
  }
}
</style>
