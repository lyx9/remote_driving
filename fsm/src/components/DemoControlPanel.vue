<template>
  <div class="demo-control-panel">
    <!-- 控制模式切换 -->
    <div class="control-mode-section">
      <div class="section-header">CONTROL MODE</div>
      <div class="mode-buttons">
        <button
          :class="['mode-btn', { active: controlMode === 'keyboard' }]"
          @click="setControlMode('keyboard')"
        >
          <span class="icon">⌨</span>
          <span>Keyboard</span>
        </button>
        <button
          :class="['mode-btn', { active: controlMode === 'wheel' }]"
          @click="setControlMode('wheel')"
        >
          <span class="icon">🎮</span>
          <span>Wheel</span>
        </button>
        <button
          :class="['mode-btn', { active: controlMode === 'auto' }]"
          @click="setControlMode('auto')"
        >
          <span class="icon">🤖</span>
          <span>Auto</span>
        </button>
      </div>
    </div>

    <!-- 键盘状态 -->
    <div v-if="controlMode === 'keyboard'" class="keyboard-section">
      <div class="section-header">KEYBOARD INPUT</div>
      <div class="key-status">
        <div class="key-row">
          <div :class="['key', { pressed: keyboardState.forward }]">W</div>
        </div>
        <div class="key-row">
          <div :class="['key', { pressed: keyboardState.left }]">A</div>
          <div :class="['key', { pressed: keyboardState.backward }]">S</div>
          <div :class="['key', { pressed: keyboardState.right }]">D</div>
        </div>
        <div class="key-row special">
          <div :class="['key wide', { pressed: keyboardState.brake }]">SPACE</div>
          <div :class="['key', { pressed: keyboardState.emergency }]">E</div>
        </div>
      </div>
      <div class="input-values">
        <div class="value-item">
          <span class="label">Throttle</span>
          <div class="progress-bar">
            <div class="progress" :style="{ width: `${throttle * 100}%` }"></div>
          </div>
          <span class="value">{{ (throttle * 100).toFixed(0) }}%</span>
        </div>
        <div class="value-item">
          <span class="label">Brake</span>
          <div class="progress-bar brake">
            <div class="progress" :style="{ width: `${brake * 100}%` }"></div>
          </div>
          <span class="value">{{ (brake * 100).toFixed(0) }}%</span>
        </div>
        <div class="value-item">
          <span class="label">Steering</span>
          <div class="steering-indicator">
            <div class="steering-center"></div>
            <div class="steering-pointer" :style="{ left: `${50 + steering * 50}%` }"></div>
          </div>
          <span class="value">{{ (steering * 100).toFixed(0) }}%</span>
        </div>
      </div>
    </div>

    <!-- 方向盘状态 -->
    <div v-if="controlMode === 'wheel'" class="wheel-section">
      <div class="section-header">STEERING WHEEL</div>
      <div v-if="wheelConnected" class="wheel-connected">
        <div class="wheel-name">{{ wheelName }}</div>
        <div class="wheel-status connected">Connected</div>
      </div>
      <div v-else class="wheel-disconnected">
        <div class="wheel-icon">🎮</div>
        <div class="wheel-message">No wheel detected</div>
        <div class="wheel-hint">Connect a Logitech G29/G920 or compatible wheel</div>
      </div>
    </div>

    <!-- 录制控制 -->
    <div class="recording-section">
      <div class="section-header">RECORDING</div>
      <div class="recording-controls">
        <button
          :class="['rec-btn', { recording: isRecording }]"
          @click="toggleRecording"
        >
          <span class="rec-icon">{{ isRecording ? '■' : '●' }}</span>
          <span>{{ isRecording ? 'STOP' : 'REC' }}</span>
        </button>
        <div v-if="isRecording" class="recording-info">
          <div class="rec-time">{{ formatDuration(recordingDuration) }}</div>
          <div class="rec-size">{{ formatSize(recordingSize) }}</div>
        </div>
      </div>

      <!-- 快速标签 -->
      <div v-if="isRecording" class="quick-tags">
        <div class="tags-label">Quick Tags:</div>
        <div class="tag-buttons">
          <button
            v-for="tag in quickTags"
            :key="tag.name"
            class="tag-btn"
            :style="{ borderColor: tag.color }"
            @click="addQuickTag(tag)"
          >
            {{ tag.name }}
          </button>
        </div>
      </div>

      <!-- 已添加的标签 -->
      <div v-if="sessionTags.length > 0" class="session-tags">
        <div class="tags-label">Session Tags:</div>
        <div class="tag-list">
          <span
            v-for="(tag, index) in sessionTags"
            :key="index"
            class="tag-chip"
            :style="{ backgroundColor: tag.color + '33', borderColor: tag.color }"
          >
            {{ tag.name }}
            <span class="tag-time">{{ formatTagTime(tag.timestamp) }}</span>
          </span>
        </div>
      </div>
    </div>

    <!-- 模拟场景选择 -->
    <div class="scenario-section">
      <div class="section-header">MOCK SCENARIO</div>
      <div class="scenario-select">
        <select v-model="selectedScenario" @change="loadScenario">
          <option value="">Select a scenario...</option>
          <option v-for="s in scenarios" :key="s.id" :value="s.id">
            {{ s.name }}
          </option>
        </select>
      </div>
      <div v-if="currentScenario" class="scenario-info">
        <div class="scenario-name">{{ currentScenario.name }}</div>
        <div class="scenario-desc">{{ currentScenario.description }}</div>
      </div>
      <div class="sim-controls">
        <button
          :class="['sim-btn', { active: simulationRunning }]"
          @click="toggleSimulation"
        >
          {{ simulationRunning ? '⏸ Pause' : '▶ Start' }}
        </button>
        <button class="sim-btn reset" @click="resetSimulation">
          ↺ Reset
        </button>
      </div>
    </div>

    <!-- 车辆状态 -->
    <div v-if="vehicleStatus" class="vehicle-status-section">
      <div class="section-header">VEHICLE STATUS</div>
      <div class="status-grid">
        <div class="status-item">
          <span class="status-label">Speed</span>
          <span class="status-value">{{ vehicleStatus.speed.toFixed(1) }} km/h</span>
        </div>
        <div class="status-item">
          <span class="status-label">Gear</span>
          <span class="status-value gear">{{ vehicleStatus.gear }}</span>
        </div>
        <div class="status-item">
          <span class="status-label">Heading</span>
          <span class="status-value">{{ vehicleStatus.heading.toFixed(1) }}°</span>
        </div>
        <div class="status-item">
          <span class="status-label">Battery</span>
          <span class="status-value">{{ vehicleStatus.battery }}%</span>
        </div>
      </div>
      <div v-if="vehicleStatus.emergency" class="emergency-alert">
        ⚠ EMERGENCY STOP ACTIVE
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted, watch } from 'vue'
import { useKeyboardControl, type ControlCommand } from '@/composables/useKeyboardControl'
import { useMockSimulator } from '@/composables/useMockSimulator'
import { useMockRecording, PRESET_TAGS, type TagInfo } from '@/composables/useRecording'
import { useSteeringWheel } from '@/composables/useSteeringWheel'
import { useFleetStore } from '@/stores/fleet'

const fleetStore = useFleetStore()

// 控制模式
const controlMode = ref<'keyboard' | 'wheel' | 'auto'>('keyboard')

// 键盘控制
const keyboardState = ref({
  forward: false,
  backward: false,
  left: false,
  right: false,
  brake: false,
  emergency: false
})
const throttle = ref(0)
const brake = ref(0)
const steering = ref(0)

// 方向盘
const { isConnected: wheelConnected, wheelName, input: wheelInput } = useSteeringWheel()

// Mock 模拟器
const {
  currentScenario,
  isRunning: simulationRunning,
  vehicleStatus,
  availableScenarios: scenarios,
  loadScenario: loadMockScenario,
  applyControl,
  start: startSimulation,
  pause: pauseSimulation,
  reset: resetSimulation
} = useMockSimulator()

const selectedScenario = ref('')

// 录制
const {
  session,
  isRecording,
  startRecording,
  stopRecording,
  addTag,
  formatDuration,
  formatSize
} = useMockRecording(fleetStore.currentVehicle?.id || 'mock-vehicle')

const recordingDuration = computed(() => session.value?.duration || 0)
const recordingSize = computed(() => session.value?.size || 0)
const sessionTags = computed(() => session.value?.tags || [])

// 快速标签
const quickTags = PRESET_TAGS.slice(0, 5)

// 键盘控制回调
const handleKeyboardCommand = (cmd: ControlCommand) => {
  throttle.value = cmd.throttle
  brake.value = cmd.brake
  steering.value = cmd.steering

  // 更新键盘状态
  if (cmd.keys) {
    keyboardState.value = {
      forward: cmd.keys.forward || false,
      backward: cmd.keys.backward || false,
      left: cmd.keys.left || false,
      right: cmd.keys.right || false,
      brake: cmd.keys.braking || false,
      emergency: cmd.keys.emergency || false
    }
  }

  // 应用到模拟器
  if (currentScenario.value && simulationRunning.value) {
    applyControl({
      throttle: cmd.throttle,
      brake: cmd.brake,
      steering: cmd.steering,
      gear: cmd.gear,
      turnSignal: cmd.turn_signal,
      hazard: cmd.hazard,
      emergency: cmd.emergency_stop
    })
  }

  // 更新 fleet store
  fleetStore.updateVehicleSpeed(vehicleStatus.value?.speed || 0)
  fleetStore.updateVehicleSteering(cmd.steering * 45)
  fleetStore.updateVehicleGear(cmd.gear)
}

// 初始化键盘控制
const vehicleId = computed(() => fleetStore.currentVehicle?.id || 'mock-vehicle')
const { start: startKeyboard, stop: stopKeyboard } = useKeyboardControl(handleKeyboardCommand)

// 控制模式切换
const setControlMode = (mode: 'keyboard' | 'wheel' | 'auto') => {
  controlMode.value = mode

  if (mode === 'keyboard') {
    startKeyboard(vehicleId.value)
  } else {
    stopKeyboard()
  }
}

// 加载场景
const loadScenario = () => {
  if (selectedScenario.value) {
    loadMockScenario(selectedScenario.value)
  }
}

// 切换模拟
const toggleSimulation = () => {
  if (simulationRunning.value) {
    pauseSimulation()
  } else {
    startSimulation()
  }
}

// 切换录制
const toggleRecording = () => {
  if (isRecording.value) {
    stopRecording()
  } else {
    startRecording(['rosbag', 'video'])
  }
}

// 添加快速标签
const addQuickTag = (tag: TagInfo) => {
  addTag(tag)
}

// 格式化标签时间
const formatTagTime = (timestamp?: number): string => {
  if (!timestamp) return ''
  const secs = Math.floor(timestamp / 1000)
  const mins = Math.floor(secs / 60)
  const remainSecs = secs % 60
  return `@${mins}:${remainSecs.toString().padStart(2, '0')}`
}

// 监听方向盘输入
watch(wheelInput, (input) => {
  if (controlMode.value === 'wheel' && wheelConnected.value) {
    throttle.value = input.throttle
    brake.value = input.brake
    steering.value = input.steering

    if (currentScenario.value && simulationRunning.value) {
      applyControl({
        throttle: input.throttle,
        brake: input.brake,
        steering: input.steering,
        gear: input.buttons.reverse ? 'R' : 'D',
        turnSignal: input.buttons.leftSignal ? -1 : input.buttons.rightSignal ? 1 : 0,
        hazard: input.buttons.hazard,
        emergency: input.buttons.emergency
      })
    }
  }
}, { deep: true })

onMounted(() => {
  if (controlMode.value === 'keyboard') {
    startKeyboard(vehicleId.value)
  }
  // 默认加载第一个场景
  if (scenarios.length > 0) {
    selectedScenario.value = scenarios[0].id
    loadScenario()
  }
})

onUnmounted(() => {
  stopKeyboard()
})
</script>

<style scoped>
.demo-control-panel {
  display: flex;
  flex-direction: column;
  gap: 12px;
  padding: 10px;
  height: 100%;
  overflow-y: auto;
}

.section-header {
  font-size: 10px;
  font-weight: bold;
  color: var(--primary, #ff5722);
  letter-spacing: 0.5px;
  margin-bottom: 8px;
  padding-bottom: 4px;
  border-bottom: 1px solid #333;
}

/* Control Mode */
.mode-buttons {
  display: flex;
  gap: 6px;
}

.mode-btn {
  flex: 1;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 4px;
  padding: 8px 4px;
  background: #1a1a2e;
  border: 1px solid #333;
  color: #888;
  cursor: pointer;
  transition: all 0.2s;
  font-size: 10px;
}

.mode-btn:hover {
  border-color: #555;
  color: #aaa;
}

.mode-btn.active {
  border-color: var(--primary, #ff5722);
  color: #fff;
  background: rgba(255, 87, 34, 0.1);
}

.mode-btn .icon {
  font-size: 16px;
}

/* Keyboard Section */
.key-status {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 4px;
  margin-bottom: 12px;
}

.key-row {
  display: flex;
  gap: 4px;
}

.key-row.special {
  margin-top: 4px;
}

.key {
  width: 32px;
  height: 32px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: #1a1a2e;
  border: 1px solid #444;
  color: #666;
  font-size: 11px;
  font-weight: bold;
  transition: all 0.1s;
}

.key.wide {
  width: 72px;
  font-size: 9px;
}

.key.pressed {
  background: var(--primary, #ff5722);
  border-color: var(--primary, #ff5722);
  color: #000;
  box-shadow: 0 0 10px rgba(255, 87, 34, 0.5);
}

/* Input Values */
.input-values {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.value-item {
  display: flex;
  align-items: center;
  gap: 8px;
}

.value-item .label {
  width: 60px;
  font-size: 10px;
  color: #888;
}

.progress-bar {
  flex: 1;
  height: 6px;
  background: #1a1a2e;
  border-radius: 3px;
  overflow: hidden;
}

.progress-bar .progress {
  height: 100%;
  background: var(--primary, #ff5722);
  transition: width 0.1s;
}

.progress-bar.brake .progress {
  background: #ff4444;
}

.value-item .value {
  width: 40px;
  text-align: right;
  font-size: 10px;
  color: #aaa;
  font-family: monospace;
}

/* Steering Indicator */
.steering-indicator {
  flex: 1;
  height: 6px;
  background: #1a1a2e;
  position: relative;
  border-radius: 3px;
}

.steering-center {
  position: absolute;
  left: 50%;
  top: 0;
  width: 2px;
  height: 100%;
  background: #444;
  transform: translateX(-50%);
}

.steering-pointer {
  position: absolute;
  top: -2px;
  width: 10px;
  height: 10px;
  background: var(--primary, #ff5722);
  border-radius: 50%;
  transform: translateX(-50%);
  transition: left 0.1s;
}

/* Wheel Section */
.wheel-connected {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px;
  background: rgba(0, 255, 136, 0.1);
  border: 1px solid rgba(0, 255, 136, 0.3);
}

.wheel-name {
  font-size: 11px;
  color: #fff;
}

.wheel-status.connected {
  font-size: 10px;
  color: #00ff88;
}

.wheel-disconnected {
  text-align: center;
  padding: 20px;
  color: #666;
}

.wheel-icon {
  font-size: 32px;
  margin-bottom: 8px;
  opacity: 0.5;
}

.wheel-message {
  font-size: 12px;
  color: #888;
}

.wheel-hint {
  font-size: 9px;
  color: #555;
  margin-top: 4px;
}

/* Recording Section */
.recording-controls {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 10px;
}

.rec-btn {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 8px 16px;
  background: #aa0033;
  border: none;
  color: #fff;
  cursor: pointer;
  font-size: 12px;
  font-weight: bold;
  transition: all 0.2s;
}

.rec-btn:hover {
  background: #cc0044;
}

.rec-btn.recording {
  background: #880022;
  animation: rec-pulse 1s infinite;
}

@keyframes rec-pulse {
  0%, 100% { box-shadow: 0 0 0 0 rgba(255, 0, 68, 0.5); }
  50% { box-shadow: 0 0 0 8px rgba(255, 0, 68, 0); }
}

.rec-icon {
  font-size: 14px;
}

.recording-info {
  display: flex;
  flex-direction: column;
  gap: 2px;
}

.rec-time {
  font-size: 14px;
  font-weight: bold;
  color: #ff4444;
  font-family: monospace;
}

.rec-size {
  font-size: 10px;
  color: #888;
}

/* Quick Tags */
.quick-tags {
  margin-top: 10px;
}

.tags-label {
  font-size: 9px;
  color: #666;
  margin-bottom: 6px;
}

.tag-buttons {
  display: flex;
  flex-wrap: wrap;
  gap: 4px;
}

.tag-btn {
  padding: 4px 8px;
  background: transparent;
  border: 1px solid;
  color: #aaa;
  font-size: 9px;
  cursor: pointer;
  transition: all 0.2s;
}

.tag-btn:hover {
  background: rgba(255, 255, 255, 0.1);
  color: #fff;
}

/* Session Tags */
.session-tags {
  margin-top: 10px;
}

.tag-list {
  display: flex;
  flex-wrap: wrap;
  gap: 4px;
}

.tag-chip {
  display: inline-flex;
  align-items: center;
  gap: 4px;
  padding: 2px 6px;
  border: 1px solid;
  font-size: 9px;
  color: #fff;
}

.tag-time {
  font-size: 8px;
  opacity: 0.7;
}

/* Scenario Section */
.scenario-select select {
  width: 100%;
  padding: 8px;
  background: #1a1a2e;
  border: 1px solid #444;
  color: #fff;
  font-size: 11px;
  cursor: pointer;
}

.scenario-info {
  margin-top: 8px;
  padding: 8px;
  background: rgba(255, 255, 255, 0.05);
  border-left: 2px solid var(--primary, #ff5722);
}

.scenario-name {
  font-size: 11px;
  color: #fff;
  font-weight: bold;
}

.scenario-desc {
  font-size: 9px;
  color: #888;
  margin-top: 2px;
}

.sim-controls {
  display: flex;
  gap: 6px;
  margin-top: 10px;
}

.sim-btn {
  flex: 1;
  padding: 8px;
  background: #1a1a2e;
  border: 1px solid #444;
  color: #aaa;
  font-size: 11px;
  cursor: pointer;
  transition: all 0.2s;
}

.sim-btn:hover {
  border-color: #666;
  color: #fff;
}

.sim-btn.active {
  background: rgba(255, 87, 34, 0.2);
  border-color: var(--primary, #ff5722);
  color: #fff;
}

.sim-btn.reset {
  flex: 0 0 auto;
  width: 80px;
}

/* Vehicle Status */
.status-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 8px;
}

.status-item {
  display: flex;
  justify-content: space-between;
  padding: 6px 8px;
  background: rgba(255, 255, 255, 0.05);
}

.status-label {
  font-size: 9px;
  color: #888;
}

.status-value {
  font-size: 11px;
  color: #fff;
  font-family: monospace;
}

.status-value.gear {
  color: var(--primary, #ff5722);
  font-weight: bold;
}

.emergency-alert {
  margin-top: 10px;
  padding: 10px;
  background: rgba(255, 0, 0, 0.2);
  border: 1px solid #ff4444;
  color: #ff4444;
  text-align: center;
  font-size: 11px;
  font-weight: bold;
  animation: emergency-blink 0.5s infinite;
}

@keyframes emergency-blink {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.5; }
}
</style>
