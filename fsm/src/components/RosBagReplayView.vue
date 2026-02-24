<!--
  FSM-Pilot V2.0 - Remote Driving System

  @project     FSM-Pilot Remote Driving Platform
  @author      Li Yixiang
  @institution City University of Hong Kong
  @copyright   2025 City University of Hong Kong. All rights reserved.
  @license     Proprietary

  @component   RosBagReplayView
  @description RosBag回放完整视图，整合播放器和可视化组件
-->
<template>
  <div class="rosbag-replay-view">
    <!-- 顶部标题栏 -->
    <div class="replay-header">
      <div class="header-left">
        <h1 class="title">
          <span class="icon">📼</span>
          RosBag Replay
        </h1>
        <span class="subtitle">Autoware.universe Data Playback</span>
      </div>
      <div class="header-right">
        <button class="btn-header" @click="showFileDialog = true">
          📁 Open Bag
        </button>
        <button class="btn-header" @click="exportData" :disabled="!currentBag">
          📤 Export
        </button>
        <button class="btn-header" @click="showSettings = !showSettings">
          ⚙️ Settings
        </button>
      </div>
    </div>

    <!-- 主内容区域 -->
    <div class="replay-content">
      <!-- 左侧：可视化 -->
      <div class="viz-section">
        <RosBagVisualization
          ref="vizRef"
          :kinematic-state="currentKinematicState"
          :detected-objects="currentDetectedObjects"
        />
      </div>

      <!-- 右侧：控制面板 -->
      <div class="control-section">
        <RosBagPlayer
          ref="playerRef"
          @update:vehicle-status="handleVehicleStatusUpdate"
          @update:kinematic-state="handleKinematicStateUpdate"
          @update:detected-objects="handleDetectedObjectsUpdate"
        />

        <!-- 数据统计面板 -->
        <div class="stats-panel">
          <div class="panel-header">
            <span>Statistics</span>
          </div>
          <div class="stats-grid">
            <div class="stat-item">
              <span class="stat-label">Total Distance</span>
              <span class="stat-value">{{ totalDistance.toFixed(1) }} m</span>
            </div>
            <div class="stat-item">
              <span class="stat-label">Avg Speed</span>
              <span class="stat-value">{{ avgSpeed.toFixed(1) }} m/s</span>
            </div>
            <div class="stat-item">
              <span class="stat-label">Max Speed</span>
              <span class="stat-value">{{ maxSpeed.toFixed(1) }} m/s</span>
            </div>
            <div class="stat-item">
              <span class="stat-label">Objects Detected</span>
              <span class="stat-value">{{ totalObjectsDetected }}</span>
            </div>
          </div>
        </div>

        <!-- 快捷操作 -->
        <div class="quick-actions">
          <button class="action-btn" @click="resetView">
            🔄 Reset View
          </button>
          <button class="action-btn" @click="clearTrajectory">
            🗑️ Clear Trail
          </button>
          <button class="action-btn" @click="screenshot">
            📷 Screenshot
          </button>
        </div>
      </div>
    </div>

    <!-- 文件选择对话框 -->
    <Transition name="fade">
      <div class="modal-overlay" v-if="showFileDialog" @click.self="showFileDialog = false">
        <div class="file-dialog">
          <div class="dialog-header">
            <h2>Open RosBag File</h2>
            <button class="close-btn" @click="showFileDialog = false">×</button>
          </div>
          <div class="dialog-content">
            <div class="file-input-area">
              <input
                type="file"
                ref="fileInputRef"
                accept=".mcap,.db3"
                @change="handleFileSelect"
                hidden
              />
              <div class="drop-zone" @click="triggerFileInput" @drop.prevent="handleDrop" @dragover.prevent>
                <div class="drop-icon">📁</div>
                <div class="drop-text">
                  Click or drag a RosBag file here
                </div>
                <div class="drop-hint">
                  Supports .mcap and .db3 formats
                </div>
              </div>
            </div>

            <div class="recent-files">
              <div class="section-title">Recent Files</div>
              <div class="file-list">
                <div
                  v-for="file in recentFiles"
                  :key="file.id"
                  class="file-item"
                  @click="loadRecentFile(file)"
                >
                  <span class="file-icon">📼</span>
                  <div class="file-info">
                    <span class="file-name">{{ file.name }}</span>
                    <span class="file-meta">{{ formatFileSize(file.size) }} | {{ formatDate(file.date) }}</span>
                  </div>
                </div>
              </div>
            </div>

            <div class="mock-option">
              <button class="btn-mock" @click="loadMockData">
                🎭 Load Mock Data (Demo)
              </button>
            </div>
          </div>
        </div>
      </div>
    </Transition>

    <!-- 设置面板 -->
    <Transition name="slide-right">
      <div class="settings-panel" v-if="showSettings">
        <div class="panel-header">
          <span>Settings</span>
          <button class="close-btn" @click="showSettings = false">×</button>
        </div>
        <div class="settings-content">
          <div class="setting-group">
            <label>Visualization</label>
            <div class="setting-item">
              <span>Grid Size</span>
              <input type="range" v-model.number="settings.gridSize" min="5" max="50" step="5" />
              <span>{{ settings.gridSize }}m</span>
            </div>
            <div class="setting-item">
              <span>Trajectory Length</span>
              <input type="range" v-model.number="settings.trajectoryLength" min="100" max="1000" step="100" />
              <span>{{ settings.trajectoryLength }} pts</span>
            </div>
          </div>

          <div class="setting-group">
            <label>Playback</label>
            <div class="setting-item">
              <span>Default Rate</span>
              <select v-model.number="settings.defaultRate">
                <option :value="0.5">0.5x</option>
                <option :value="1">1x</option>
                <option :value="2">2x</option>
              </select>
            </div>
            <div class="setting-item">
              <span>Auto Loop</span>
              <input type="checkbox" v-model="settings.autoLoop" />
            </div>
          </div>

          <div class="setting-group">
            <label>Display</label>
            <div class="setting-item">
              <span>Show FPS</span>
              <input type="checkbox" v-model="settings.showFps" />
            </div>
            <div class="setting-item">
              <span>Dark Mode</span>
              <input type="checkbox" v-model="settings.darkMode" checked disabled />
            </div>
          </div>
        </div>
      </div>
    </Transition>
  </div>
</template>

<script setup lang="ts">
import { ref, reactive, onMounted } from 'vue'
import RosBagPlayer from './RosBagPlayer.vue'
import RosBagVisualization from './RosBagVisualization.vue'
import {
  useRosBagPlayer,
  formatFileSize,
  type AutowareVehicleStatus,
  type AutowareKinematicState,
  type AutowareDetectedObjects
} from '@/services/rosbagPlayer'

// 组件引用
const playerRef = ref<InstanceType<typeof RosBagPlayer> | null>(null)
const vizRef = ref<InstanceType<typeof RosBagVisualization> | null>(null)
const fileInputRef = ref<HTMLInputElement | null>(null)

// RosBag 播放器
const {
  currentBag,
  currentKinematicState,
  currentDetectedObjects
} = useRosBagPlayer()

// UI 状态
const showFileDialog = ref(false)
const showSettings = ref(false)

// 设置
const settings = reactive({
  gridSize: 10,
  trajectoryLength: 500,
  defaultRate: 1,
  autoLoop: false,
  showFps: true,
  darkMode: true
})

// 统计数据
const totalDistance = ref(0)
const avgSpeed = ref(0)
const maxSpeed = ref(0)
const totalObjectsDetected = ref(0)
const speedHistory: number[] = []

// 最近文件
const recentFiles = ref<Array<{ id: string; name: string; size: number; date: string }>>([
  { id: '1', name: 'autoware_test_2025-01-05.mcap', size: 1024 * 1024 * 850, date: '2025-01-05T10:30:00' },
  { id: '2', name: 'parking_scenario.db3', size: 1024 * 1024 * 320, date: '2025-01-04T15:20:00' },
  { id: '3', name: 'highway_drive.mcap', size: 1024 * 1024 * 1500, date: '2025-01-03T09:00:00' }
])

// 事件处理
const handleVehicleStatusUpdate = (status: AutowareVehicleStatus) => {
  const speed = status.longitudinal_velocity
  speedHistory.push(speed)

  // 更新最大速度
  maxSpeed.value = Math.max(maxSpeed.value, speed)

  // 更新平均速度 (最近100个采样)
  const recentSpeeds = speedHistory.slice(-100)
  avgSpeed.value = recentSpeeds.reduce((a, b) => a + b, 0) / recentSpeeds.length

  // 估算距离 (假设10ms间隔)
  totalDistance.value += speed * 0.01
}

const handleKinematicStateUpdate = (_state: AutowareKinematicState) => {
  // 可以在这里添加额外处理
}

const handleDetectedObjectsUpdate = (objects: AutowareDetectedObjects) => {
  totalObjectsDetected.value += objects.objects.length
}

// 文件操作
const triggerFileInput = () => {
  fileInputRef.value?.click()
}

const handleFileSelect = (e: Event) => {
  const input = e.target as HTMLInputElement
  const file = input.files?.[0]
  if (file) {
    console.log('Selected file:', file.name)
    // TODO: 实现文件解析
    showFileDialog.value = false
  }
}

const handleDrop = (e: DragEvent) => {
  const file = e.dataTransfer?.files?.[0]
  if (file) {
    console.log('Dropped file:', file.name)
    showFileDialog.value = false
  }
}

const loadRecentFile = (file: { id: string; name: string }) => {
  console.log('Loading recent file:', file.name)
  showFileDialog.value = false
}

const loadMockData = async () => {
  showFileDialog.value = false
  await playerRef.value?.loadMockBag()
}

// 视图操作
const resetView = () => {
  vizRef.value?.resetView()
}

const clearTrajectory = () => {
  vizRef.value?.clearTrajectory()
  totalDistance.value = 0
  speedHistory.length = 0
  avgSpeed.value = 0
  maxSpeed.value = 0
  totalObjectsDetected.value = 0
}

const screenshot = () => {
  // TODO: 实现截图功能
  console.log('Screenshot')
}

const exportData = () => {
  // TODO: 实现数据导出
  console.log('Export data')
}

const formatDate = (dateStr: string): string => {
  const date = new Date(dateStr)
  return date.toLocaleDateString('zh-CN')
}

// 生命周期
onMounted(() => {
  // 可以在这里初始化
})
</script>

<style scoped>
.rosbag-replay-view {
  display: flex;
  flex-direction: column;
  height: 100%;
  background: #0a0a12;
  color: #e0e0e0;
}

/* Header */
.replay-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 16px;
  background: #14141f;
  border-bottom: 1px solid #2a2a3a;
}

.header-left {
  display: flex;
  align-items: center;
  gap: 16px;
}

.title {
  font-size: 18px;
  font-weight: bold;
  color: var(--primary, #ff5722);
  margin: 0;
  display: flex;
  align-items: center;
  gap: 8px;
}

.icon {
  font-size: 24px;
}

.subtitle {
  font-size: 12px;
  color: #666;
}

.header-right {
  display: flex;
  gap: 8px;
}

.btn-header {
  padding: 8px 16px;
  background: #1a1a2e;
  border: 1px solid #333;
  color: #aaa;
  font-size: 12px;
  cursor: pointer;
  transition: all 0.2s;
}

.btn-header:hover {
  background: #252540;
  color: #fff;
}

.btn-header:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

/* Content */
.replay-content {
  flex: 1;
  display: flex;
  overflow: hidden;
}

.viz-section {
  flex: 2;
  border-right: 1px solid #2a2a3a;
}

.control-section {
  flex: 1;
  min-width: 350px;
  max-width: 450px;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

/* Stats Panel */
.stats-panel {
  padding: 12px;
  background: #14141f;
  border-top: 1px solid #2a2a3a;
}

.panel-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  font-size: 11px;
  font-weight: bold;
  color: var(--primary, #ff5722);
  margin-bottom: 10px;
  padding-bottom: 6px;
  border-bottom: 1px solid #2a2a3a;
}

.stats-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 8px;
}

.stat-item {
  display: flex;
  flex-direction: column;
  gap: 2px;
}

.stat-label {
  font-size: 9px;
  color: #666;
  text-transform: uppercase;
}

.stat-value {
  font-size: 14px;
  color: #fff;
  font-weight: bold;
  font-family: 'JetBrains Mono', monospace;
}

/* Quick Actions */
.quick-actions {
  display: flex;
  gap: 8px;
  padding: 12px;
  background: #0d0d14;
}

.action-btn {
  flex: 1;
  padding: 8px;
  background: #1a1a2e;
  border: 1px solid #333;
  color: #888;
  font-size: 11px;
  cursor: pointer;
  transition: all 0.2s;
}

.action-btn:hover {
  background: #252540;
  color: #fff;
}

/* Modal */
.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.7);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1000;
}

.file-dialog {
  width: 500px;
  background: #14141f;
  border: 1px solid #333;
  border-radius: 8px;
  overflow: hidden;
}

.dialog-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px;
  border-bottom: 1px solid #2a2a3a;
}

.dialog-header h2 {
  margin: 0;
  font-size: 16px;
  color: #fff;
}

.close-btn {
  width: 28px;
  height: 28px;
  background: transparent;
  border: none;
  color: #888;
  font-size: 20px;
  cursor: pointer;
}

.close-btn:hover {
  color: #fff;
}

.dialog-content {
  padding: 16px;
}

.drop-zone {
  border: 2px dashed #333;
  border-radius: 8px;
  padding: 40px;
  text-align: center;
  cursor: pointer;
  transition: all 0.2s;
}

.drop-zone:hover {
  border-color: var(--primary, #ff5722);
  background: rgba(255, 87, 34, 0.05);
}

.drop-icon {
  font-size: 48px;
  margin-bottom: 12px;
}

.drop-text {
  font-size: 14px;
  color: #888;
}

.drop-hint {
  font-size: 11px;
  color: #555;
  margin-top: 8px;
}

.recent-files {
  margin-top: 20px;
}

.section-title {
  font-size: 12px;
  font-weight: bold;
  color: #666;
  margin-bottom: 10px;
}

.file-list {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.file-item {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 10px;
  background: rgba(255, 255, 255, 0.03);
  border-radius: 4px;
  cursor: pointer;
  transition: background 0.2s;
}

.file-item:hover {
  background: rgba(255, 255, 255, 0.08);
}

.file-icon {
  font-size: 24px;
}

.file-info {
  display: flex;
  flex-direction: column;
  gap: 2px;
}

.file-name {
  font-size: 12px;
  color: #ddd;
}

.file-meta {
  font-size: 10px;
  color: #666;
}

.mock-option {
  margin-top: 20px;
  text-align: center;
}

.btn-mock {
  padding: 10px 24px;
  background: rgba(255, 87, 34, 0.1);
  border: 1px solid var(--primary, #ff5722);
  color: var(--primary, #ff5722);
  font-size: 12px;
  cursor: pointer;
  transition: all 0.2s;
}

.btn-mock:hover {
  background: rgba(255, 87, 34, 0.2);
}

/* Settings Panel */
.settings-panel {
  position: fixed;
  top: 0;
  right: 0;
  width: 300px;
  height: 100%;
  background: #14141f;
  border-left: 1px solid #2a2a3a;
  z-index: 1000;
  display: flex;
  flex-direction: column;
}

.settings-content {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
}

.setting-group {
  margin-bottom: 20px;
}

.setting-group > label {
  display: block;
  font-size: 11px;
  font-weight: bold;
  color: var(--primary, #ff5722);
  margin-bottom: 10px;
  padding-bottom: 6px;
  border-bottom: 1px solid #2a2a3a;
}

.setting-item {
  display: flex;
  align-items: center;
  gap: 10px;
  margin-bottom: 8px;
  font-size: 12px;
  color: #888;
}

.setting-item span:first-child {
  flex: 1;
}

.setting-item input[type="range"] {
  width: 80px;
}

.setting-item input[type="checkbox"] {
  accent-color: var(--primary, #ff5722);
}

.setting-item select {
  padding: 4px 8px;
  background: #1a1a2e;
  border: 1px solid #333;
  color: #fff;
  font-size: 11px;
}

/* Transitions */
.fade-enter-active,
.fade-leave-active {
  transition: opacity 0.3s;
}

.fade-enter-from,
.fade-leave-to {
  opacity: 0;
}

.slide-right-enter-active,
.slide-right-leave-active {
  transition: transform 0.3s ease;
}

.slide-right-enter-from,
.slide-right-leave-to {
  transform: translateX(100%);
}
</style>
