<template>
  <header class="app-header">
    <div class="brand">
      FSM-<span>PILOT</span> V1.1
    </div>

    <div class="controls">
      <button
        :class="['btn-ui', { active: systemStore.ui.showLeftSidebar }]"
        @click="systemStore.toggleUI('showLeftSidebar')"
      >
        MAP
      </button>
      <button
        :class="['btn-ui', { active: systemStore.ui.showAIBar }]"
        @click="systemStore.toggleUI('showAIBar')"
      >
        AI-BAR
      </button>
      <button
        :class="['btn-ui', { active: systemStore.ui.showLidar }]"
        @click="systemStore.toggleUI('showLidar')"
      >
        LIDAR
      </button>
      <button
        :class="['btn-ui', { active: systemStore.ui.showPIP }]"
        @click="systemStore.toggleUI('showPIP')"
      >
        PIP (4)
      </button>
      <button
        :class="['btn-ui', { active: systemStore.ui.showRightSidebar }]"
        @click="systemStore.toggleUI('showRightSidebar')"
      >
        CTRL
      </button>
    </div>

    <div class="status-bar">
      <span class="sys-time">{{ currentTime }}</span>
      <span :class="['rec-status', { recording: systemStore.recording.isRecording }]">
        {{ systemStore.recording.isRecording ? '● RECORDING' : 'STANDBY' }}
      </span>
    </div>
  </header>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue'
import { useSystemStore } from '@/stores/system'

const systemStore = useSystemStore()
const currentTime = ref('00:00:00')
let timeInterval: number | null = null

const updateTime = () => {
  currentTime.value = new Date().toLocaleTimeString()
}

onMounted(() => {
  updateTime()
  timeInterval = window.setInterval(updateTime, 1000)
})

onUnmounted(() => {
  if (timeInterval) clearInterval(timeInterval)
})
</script>

<style scoped>
.app-header {
  height: var(--header-h);
  background: #05080e;
  border-bottom: 1px solid var(--border);
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 15px;
  flex-shrink: 0;
  z-index: 100;
}

.brand {
  font-weight: 800;
  letter-spacing: 1px;
  color: #fff;
  font-size: 13px;
}

.brand span {
  color: var(--primary);
}

.controls {
  display: flex;
  gap: 5px;
}

.status-bar {
  display: flex;
  align-items: center;
  gap: 10px;
}

.sys-time {
  color: #fff;
  font-weight: bold;
  font-size: 12px;
}

.rec-status {
  color: #555;
  font-size: 10px;
  transition: color 0.3s;
}

.rec-status.recording {
  color: var(--danger);
  font-weight: bold;
}
</style>
