<template>
  <Transition name="slide-right">
    <aside v-if="systemStore.ui.showRightSidebar" class="sidebar right">
      <div class="sidebar-inner">

        <!-- Gamepad Control Card -->
        <div class="card gamepad-card">
          <div class="card-header">REMOTE CONTROL</div>
          <div class="card-body">
            <GamepadVisualizer
              :controlInput="controlInput"
              :gamepadConnected="gamepadConnected"
            />
          </div>
        </div>

        <!-- Transmission Card -->
        <div class="card transmission-card">
          <div class="card-header">TRANSMISSION</div>
          <div class="card-body">
            <div class="gear-box">
              <button
                v-for="gear in ['P', 'R', 'N', 'D']"
                :key="gear"
                :class="['gear-btn', { active: fleetStore.currentVehicle.gear === gear }]"
                @click="handleGearChange(gear as 'P' | 'R' | 'N' | 'D')"
              >
                {{ gear }}
              </button>
            </div>
          </div>
        </div>

        <!-- Black Box Card -->
        <div class="card blackbox-card">
          <div class="card-header">BLACK BOX</div>
          <div class="card-body">
            <div class="rec-panel">
              <div class="rec-info">
                <div :class="['rec-dot', { recording: systemStore.recording.isRecording }]"></div>
                <span class="rec-label">DRIVE LOG</span>
              </div>
              <button
                :class="['btn-ui', 'btn-rec', { active: systemStore.recording.isRecording }]"
                @click="handleToggleRecording"
              >
                {{ systemStore.recording.isRecording ? '■ STOP' : '● REC' }}
              </button>
            </div>
            <div class="rec-description">
              Syncs 5 video feeds and telemetry to local storage.
            </div>
          </div>
        </div>

        <!-- Telemetry Card -->
        <div class="card telemetry-card">
          <div class="card-header">TELEMETRY</div>
          <div class="card-body telemetry-content">
            <div class="speed-display">
              <div class="speed-value">{{ fleetStore.currentVehicle.speed || 0 }}</div>
              <div class="speed-unit">KM/H</div>
            </div>

            <div
              class="steering-wheel"
              :style="{ transform: `rotate(${fleetStore.currentVehicle.steering || 0}deg)` }"
            >
              <div class="wheel-indicator"></div>
            </div>
          </div>
        </div>

        <!-- Video Transmission Card -->
        <div class="card video-transmission-card">
          <div class="card-header">VIDEO TRANSMISSION</div>
          <div class="card-body video-transmission-content">
            <!-- Bandwidth & Latency Display -->
            <div class="transmission-stats">
              <div class="stat-item">
                <div class="stat-label">BANDWIDTH</div>
                <div class="stat-value">{{ systemStore.videoTransmission.bandwidth.toFixed(1) }} <span class="stat-unit">Mbps</span></div>
              </div>
              <div class="stat-item">
                <div class="stat-label">LATENCY</div>
                <div class="stat-value">{{ fleetStore.currentVehicle.latency_ms }} <span class="stat-unit">ms</span></div>
              </div>
            </div>

            <!-- Frame Rate Control -->
            <div class="control-group">
              <div class="control-label">
                <span>FRAME RATE</span>
                <span class="control-value">{{ systemStore.videoTransmission.frameRate }} FPS</span>
              </div>
              <input
                type="range"
                min="10"
                max="60"
                step="5"
                :value="systemStore.videoTransmission.frameRate"
                @input="handleFrameRateChange"
                class="slider"
              />
              <div class="control-hints">
                <span>10</span>
                <span>30</span>
                <span>60</span>
              </div>
            </div>

            <!-- Compression Control -->
            <div class="control-group">
              <div class="control-label">
                <span>COMPRESSION</span>
                <span class="control-value">{{ systemStore.videoTransmission.compression }}%</span>
              </div>
              <input
                type="range"
                min="20"
                max="100"
                step="5"
                :value="systemStore.videoTransmission.compression"
                @input="handleCompressionChange"
                class="slider"
              />
              <div class="control-hints">
                <span>Low</span>
                <span>Med</span>
                <span>High</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </aside>
  </Transition>
</template>

<script setup lang="ts">
import { onMounted, onUnmounted } from 'vue'
import { useFleetStore } from '@/stores/fleet'
import { useSystemStore } from '@/stores/system'
import { useGamepadController } from '@/composables/useGamepadController'
import GamepadVisualizer from './GamepadVisualizer.vue'

const fleetStore = useFleetStore()
const systemStore = useSystemStore()

// 游戏手柄控制
const { controlInput, gamepadConnected } = useGamepadController()

const handleGearChange = (gear: 'P' | 'R' | 'N' | 'D') => {
  fleetStore.updateVehicleGear(gear)
  systemStore.addLog(`Shifted to [${gear}]`)
}

const handleToggleRecording = () => {
  if (systemStore.recording.isRecording) {
    systemStore.stopRecording()
  } else {
    systemStore.startRecording(fleetStore.currentVehicle.id)
  }
}

// Video transmission controls
const handleFrameRateChange = (event: Event) => {
  const target = event.target as HTMLInputElement
  const fps = parseInt(target.value)
  systemStore.setFrameRate(fps)
}

const handleCompressionChange = (event: Event) => {
  const target = event.target as HTMLInputElement
  const quality = parseInt(target.value)
  systemStore.setCompression(quality)
}

// Simulate bandwidth updates
let bandwidthInterval: number | null = null

const updateBandwidth = () => {
  // Simulate bandwidth based on frame rate and compression
  const fps = systemStore.videoTransmission.frameRate
  const compression = systemStore.videoTransmission.compression
  const baseBandwidth = (fps / 30) * (compression / 100) * 4.5
  const variance = (Math.random() - 0.5) * 0.5
  const bandwidth = Math.max(0.5, baseBandwidth + variance)

  systemStore.updateVideoTransmission({ bandwidth })
}

onMounted(() => {
  // Update bandwidth every 500ms
  bandwidthInterval = window.setInterval(updateBandwidth, 500)
})

onUnmounted(() => {
  if (bandwidthInterval) {
    clearInterval(bandwidthInterval)
  }
})
</script>

<style scoped>
.sidebar.right {
  border-right: none;
  border-left: 1px solid var(--border);
}

.sidebar-inner {
  width: var(--side-w);
  height: 100%;
  display: flex;
  flex-direction: column;
}

.card {
  display: flex;
  flex-direction: column;
  border-bottom: 1px solid #222;
  min-height: 0;
}

.transmission-card,
.blackbox-card,
.video-transmission-card {
  flex: 0 0 auto;
}

.telemetry-card {
  flex: 1;
}

.card-header {
  padding: 6px 10px;
  background: rgba(255, 87, 34, 0.05);
  color: var(--primary);
  font-weight: bold;
  border-bottom: 1px solid #222;
  font-size: 10px;
  letter-spacing: 0.5px;
}

.card-body {
  background: var(--bg-panel);
  padding: 15px 10px;
}

/* Gears */
.gear-box {
  display: flex;
  gap: 5px;
  justify-content: center;
}

.gear-btn {
  width: 35px;
  height: 35px;
  border: 1px solid #444;
  color: #666;
  font-weight: bold;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 14px;
  cursor: pointer;
  background: #0e151e;
  transition: all 0.2s;
}

.gear-btn:hover {
  border-color: #666;
  color: #aaa;
}

.gear-btn.active {
  border-color: var(--primary);
  color: #fff;
  background: var(--primary-dim);
  box-shadow: 0 0 10px var(--primary-dim);
}

/* Black Box */
.rec-panel {
  display: flex;
  align-items: center;
  justify-content: space-between;
  background: rgba(255, 255, 255, 0.05);
  padding: 8px;
  border: 1px solid #333;
  margin-bottom: 10px;
  border-radius: 2px;
}

.rec-info {
  display: flex;
  align-items: center;
  gap: 8px;
}

.rec-dot {
  width: 10px;
  height: 10px;
  background: #555;
  border-radius: 50%;
  transition: background 0.3s;
}

.rec-dot.recording {
  background: var(--danger);
  animation: pulse 1s infinite;
}

@keyframes pulse {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.4; }
}

.rec-label {
  font-size: 10px;
  font-weight: bold;
  color: #fff;
}

.btn-rec {
  background: var(--danger);
  color: #fff;
  border: none;
  font-weight: bold;
  padding: 5px 10px;
  cursor: pointer;
  font-size: 10px;
  border-radius: 2px;
  transition: all 0.2s;
}

.btn-rec:hover {
  background: #ff1166;
}

.btn-rec.active {
  background: #aa0033;
}

.rec-description {
  font-size: 9px;
  color: #666;
  line-height: 1.4;
}

/* Telemetry */
.telemetry-content {
  text-align: center;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 20px;
}

.speed-display {
  margin-top: 10px;
}

.speed-value {
  font-size: 40px;
  color: #fff;
  font-weight: bold;
  line-height: 1;
}

.speed-unit {
  color: #666;
  font-size: 10px;
  margin-top: 5px;
}

.steering-wheel {
  width: 120px;
  height: 120px;
  border: 4px solid #222;
  border-radius: 50%;
  position: relative;
  transition: transform 0.3s ease;
}

.wheel-indicator {
  position: absolute;
  top: 0;
  left: 50%;
  width: 4px;
  height: 20px;
  background: var(--primary);
  transform: translateX(-50%);
}

.slide-right-enter-active,
.slide-right-leave-active {
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.slide-right-enter-from {
  transform: translateX(100%);
  opacity: 0;
}

.slide-right-leave-to {
  transform: translateX(100%);
  opacity: 0;
}

/* Video Transmission */
.video-transmission-content {
  padding: 10px;
}

.transmission-stats {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 10px;
  margin-bottom: 15px;
  padding: 10px;
  background: rgba(255, 255, 255, 0.03);
  border: 1px solid #333;
  border-radius: 2px;
}

.stat-item {
  text-align: center;
}

.stat-label {
  font-size: 9px;
  color: #666;
  margin-bottom: 4px;
  letter-spacing: 0.5px;
}

.stat-value {
  font-size: 16px;
  font-weight: bold;
  color: #fff;
}

.stat-unit {
  font-size: 10px;
  color: #888;
  font-weight: normal;
}

.control-group {
  margin-bottom: 15px;
}

.control-label {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 8px;
  font-size: 9px;
  color: #aaa;
  letter-spacing: 0.5px;
}

.control-value {
  color: var(--primary);
  font-weight: bold;
  font-size: 10px;
}

.slider {
  width: 100%;
  height: 4px;
  background: #222;
  outline: none;
  border-radius: 2px;
  -webkit-appearance: none;
  appearance: none;
}

.slider::-webkit-slider-thumb {
  -webkit-appearance: none;
  appearance: none;
  width: 14px;
  height: 14px;
  background: var(--primary);
  cursor: pointer;
  border-radius: 50%;
  box-shadow: 0 0 8px rgba(255, 87, 34, 0.5);
}

.slider::-moz-range-thumb {
  width: 14px;
  height: 14px;
  background: var(--primary);
  cursor: pointer;
  border-radius: 50%;
  border: none;
  box-shadow: 0 0 8px rgba(255, 87, 34, 0.5);
}

.slider:hover::-webkit-slider-thumb {
  background: #ff6b3d;
  box-shadow: 0 0 12px rgba(255, 87, 34, 0.8);
}

.slider:hover::-moz-range-thumb {
  background: #ff6b3d;
  box-shadow: 0 0 12px rgba(255, 87, 34, 0.8);
}

.control-hints {
  display: flex;
  justify-content: space-between;
  margin-top: 4px;
  font-size: 8px;
  color: #555;
}
</style>
