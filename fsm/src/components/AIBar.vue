<template>
  <Transition name="slide-down">
    <div v-if="systemStore.ui.showAIBar" class="ai-bar">
      <div class="tag">
        <span style="color: var(--primary)">ID:</span>
        <span>{{ fleetStore.currentVehicle.id }}</span>
      </div>

      <div class="tag">
        <span style="color: var(--warn)">PROFIT:</span>
        <span>${{ fleetStore.currentVehicle.money.toFixed(2) }}</span>
      </div>

      <div class="confidence-bar">
        <span class="label">CONFIDENCE</span>
        <div class="bar-track">
          <div class="bar-fill" :style="{ width: systemStore.aiConfidence + '%' }"></div>
        </div>
      </div>
    </div>
  </Transition>
</template>

<script setup lang="ts">
import { useFleetStore } from '@/stores/fleet'
import { useSystemStore } from '@/stores/system'

const fleetStore = useFleetStore()
const systemStore = useSystemStore()
</script>

<style scoped>
.ai-bar {
  height: var(--ai-h);
  background: #080c12;
  border-bottom: 1px solid var(--border);
  display: flex;
  align-items: center;
  padding: 0 10px;
  gap: 15px;
  flex-shrink: 0;
}

.tag {
  background: #111;
  border: 1px solid #333;
  padding: 2px 8px;
  border-radius: 3px;
  display: flex;
  gap: 6px;
  align-items: center;
  font-size: 11px;
}

.confidence-bar {
  flex: 1;
  display: flex;
  align-items: center;
  gap: 8px;
  margin-left: 10px;
}

.label {
  font-size: 9px;
  color: #666;
  white-space: nowrap;
}

.bar-track {
  flex: 1;
  height: 4px;
  background: #222;
  border-radius: 2px;
  overflow: hidden;
}

.bar-fill {
  height: 100%;
  background: var(--primary);
  transition: width 0.3s ease;
}

.slide-down-enter-active,
.slide-down-leave-active {
  transition: all 0.3s ease;
}

.slide-down-enter-from {
  transform: translateY(-100%);
  opacity: 0;
}

.slide-down-leave-to {
  transform: translateY(-100%);
  opacity: 0;
}
</style>
