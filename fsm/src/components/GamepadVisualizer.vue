<!--
  FSM-Pilot V2.0 - Gamepad Visualizer

  @component   GamepadVisualizer
  @description 游戏手柄/键盘控制状态可视化
-->
<template>
  <div class="gamepad-visualizer">
    <!-- 控制器状态 -->
    <div class="controller-status">
      <div class="status-icon" :class="{ connected: gamepadConnected }">
        {{ gamepadConnected ? '🎮' : '⌨️' }}
      </div>
      <div class="status-text">
        <span class="status-label">{{ gamepadConnected ? 'Gamepad' : 'Keyboard' }}</span>
        <span class="status-state" :class="{ active: isActive }">
          {{ isActive ? 'ACTIVE' : 'IDLE' }}
        </span>
      </div>
    </div>

    <!-- 控制输入显示 -->
    <div class="control-display">
      <!-- 转向 -->
      <div class="control-item">
        <span class="control-label">Steering</span>
        <div class="steering-bar">
          <div class="steering-center"></div>
          <div
            class="steering-indicator"
            :style="{ left: `${(controlInput.steering + 1) * 50}%` }"
          ></div>
        </div>
        <span class="control-value">{{ (controlInput.steering * 100).toFixed(0) }}%</span>
      </div>

      <!-- 油门/刹车 -->
      <div class="control-row">
        <div class="control-item">
          <span class="control-label">Throttle</span>
          <div class="progress-bar">
            <div
              class="progress-fill throttle"
              :style="{ width: `${controlInput.throttle * 100}%` }"
            ></div>
          </div>
          <span class="control-value">{{ (controlInput.throttle * 100).toFixed(0) }}%</span>
        </div>

        <div class="control-item">
          <span class="control-label">Brake</span>
          <div class="progress-bar">
            <div
              class="progress-fill brake"
              :style="{ width: `${controlInput.brake * 100}%` }"
            ></div>
          </div>
          <span class="control-value">{{ (controlInput.brake * 100).toFixed(0) }}%</span>
        </div>
      </div>

      <!-- 档位 -->
      <div class="control-item">
        <span class="control-label">Gear</span>
        <div class="gear-selector">
          <button
            v-for="gear in ['P', 'R', 'N', 'D']"
            :key="gear"
            :class="['gear-btn', { active: controlInput.gear === gear }]"
          >
            {{ gear }}
          </button>
        </div>
      </div>

      <!-- 紧急制动 -->
      <div class="emergency-indicator" v-if="controlInput.emergency">
        <span class="emergency-icon">🚨</span>
        <span class="emergency-text">EMERGENCY BRAKE</span>
      </div>
    </div>

    <!-- 键盘提示 -->
    <div class="keyboard-help" v-if="!gamepadConnected">
      <div class="help-title">Keyboard Controls</div>
      <div class="help-items">
        <div class="help-item"><kbd>W/↑</kbd> Throttle</div>
        <div class="help-item"><kbd>S/↓</kbd> Brake</div>
        <div class="help-item"><kbd>A/←</kbd> Left</div>
        <div class="help-item"><kbd>D/→</kbd> Right</div>
        <div class="help-item"><kbd>P/R/N/D</kbd> Gear</div>
        <div class="help-item"><kbd>Space</kbd> Emergency</div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { computed } from 'vue'
import { type ControlInput } from '@/composables/useGamepadController'

interface Props {
  controlInput: ControlInput
  gamepadConnected: boolean
}

const props = defineProps<Props>()

const isActive = computed(() => {
  return (
    Math.abs(props.controlInput.steering) > 0.01 ||
    props.controlInput.throttle > 0.01 ||
    props.controlInput.brake > 0.01
  )
})
</script>

<style scoped>
.gamepad-visualizer {
  background: #14141f;
  border: 1px solid #2a2a3a;
  border-radius: 8px;
  padding: 16px;
  color: #fff;
}

.controller-status {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 16px;
  padding-bottom: 12px;
  border-bottom: 1px solid #2a2a3a;
}

.status-icon {
  font-size: 32px;
  opacity: 0.5;
  transition: opacity 0.3s;
}

.status-icon.connected {
  opacity: 1;
}

.status-text {
  display: flex;
  flex-direction: column;
  gap: 2px;
}

.status-label {
  font-size: 12px;
  color: #888;
  font-weight: 600;
}

.status-state {
  font-size: 10px;
  color: #666;
  font-family: 'JetBrains Mono', monospace;
}

.status-state.active {
  color: var(--primary, #ff5722);
  font-weight: 700;
}

.control-display {
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.control-item {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.control-row {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 12px;
}

.control-label {
  font-size: 11px;
  color: #888;
  font-weight: 600;
}

.control-value {
  font-size: 11px;
  color: var(--primary, #ff5722);
  font-family: 'JetBrains Mono', monospace;
  text-align: right;
}

.steering-bar {
  position: relative;
  height: 24px;
  background: #1a1a2e;
  border-radius: 12px;
  border: 1px solid #333;
}

.steering-center {
  position: absolute;
  left: 50%;
  top: 0;
  bottom: 0;
  width: 2px;
  background: #666;
  transform: translateX(-50%);
}

.steering-indicator {
  position: absolute;
  top: 2px;
  bottom: 2px;
  width: 8px;
  background: var(--primary, #ff5722);
  border-radius: 4px;
  transform: translateX(-50%);
  transition: left 0.1s;
  box-shadow: 0 0 8px rgba(255, 87, 34, 0.6);
}

.progress-bar {
  position: relative;
  height: 20px;
  background: #1a1a2e;
  border-radius: 4px;
  border: 1px solid #333;
  overflow: hidden;
}

.progress-fill {
  height: 100%;
  transition: width 0.1s;
}

.progress-fill.throttle {
  background: linear-gradient(90deg, #00ff88 0%, #00ff88 100%);
}

.progress-fill.brake {
  background: linear-gradient(90deg, #ff4444 0%, #ff4444 100%);
}

.gear-selector {
  display: flex;
  gap: 4px;
}

.gear-btn {
  flex: 1;
  padding: 8px;
  background: #1a1a2e;
  border: 1px solid #333;
  color: #888;
  font-weight: 600;
  font-size: 14px;
  border-radius: 4px;
  cursor: default;
  transition: all 0.2s;
}

.gear-btn.active {
  background: var(--primary, #ff5722);
  border-color: var(--primary, #ff5722);
  color: #000;
  box-shadow: 0 0 12px rgba(255, 87, 34, 0.6);
}

.emergency-indicator {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 8px 12px;
  background: rgba(255, 68, 68, 0.2);
  border: 2px solid #ff4444;
  border-radius: 4px;
  animation: pulse 1s infinite;
}

.emergency-icon {
  font-size: 20px;
}

.emergency-text {
  font-weight: 700;
  color: #ff4444;
  font-size: 12px;
}

@keyframes pulse {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.6; }
}

.keyboard-help {
  margin-top: 16px;
  padding-top: 12px;
  border-top: 1px solid #2a2a3a;
}

.help-title {
  font-size: 11px;
  color: #888;
  font-weight: 600;
  margin-bottom: 8px;
}

.help-items {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 6px;
}

.help-item {
  font-size: 10px;
  color: #666;
  display: flex;
  align-items: center;
  gap: 6px;
}

kbd {
  padding: 2px 6px;
  background: #1a1a2e;
  border: 1px solid #333;
  border-radius: 3px;
  font-size: 9px;
  font-family: 'JetBrains Mono', monospace;
  color: #fff;
  min-width: 24px;
  text-align: center;
}
</style>
