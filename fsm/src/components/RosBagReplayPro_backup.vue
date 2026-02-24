<!--
  FSM-Pilot V2.0 - Remote Driving System

  @project     FSM-Pilot Remote Driving Platform
  @author      Li Yixiang
  @institution City University of Hong Kong
  @copyright   2025 City University of Hong Kong. All rights reserved.
  @license     Proprietary

  @component   RosBagReplayPro
  @description 专业级 RosBag 回放视图，支持大文件流式播放 (>15GB)
-->
<template>
  <div class="rosbag-replay-pro">
    <NavBar />

    <!-- 顶部工具栏 -->
    <header class="replay-header">
      <div class="header-left">
        <button class="btn-primary" @click="showBagSelector = true" :disabled="!isConnected">
          📁 Select RosBag
        </button>
        <h1 class="title">
          <span class="icon">📼</span>
          RosBag Replay Pro
        </h1>
        <span class="version">v2.0 - Streaming</span>
      </div>
      <div class="header-center">
        <div class="connection-status" :class="connectionState">
          <span class="status-dot"></span>
          <span class="status-text">{{ connectionStatusText }}</span>
        </div>
        <div class="bag-info" v-if="currentBag">
          <span class="bag-name">{{ currentBag.name }}</span>
          <span class="bag-meta">
            {{ currentBag.sizeGB }} GB |
            {{ currentBag.topicCount }} topics |
            {{ formatDuration(currentBag.duration) }}
          </span>
        </div>
      </div>
      <div class="header-right">
        <button class="btn-secondary" @click="showSettings = true">
          ⚙️ Settings
        </button>
      </div>
    </header>

    <!-- RosBag 选择对话框 -->
    <div v-if="showBagSelector" class="modal-overlay" @click="showBagSelector = false">
      <div class="modal-content" @click.stop>
        <div class="modal-header">
          <h2>Select RosBag File</h2>
          <button class="btn-close" @click="showBagSelector = false">✕</button>
        </div>
        <div class="modal-body">
          <div v-if="availableBags.length === 0" class="no-bags">
            <p>No RosBag files found in /home/lyx/fsm/rosbag/1210</p>
            <button class="btn-primary" @click="refreshBagList">🔄 Refresh</button>
          </div>
          <div v-else class="bag-list">
            <div
              v-for="bag in availableBags"
              :key="bag.path"
              class="bag-item"
              :class="{ selected: currentBag?.path === bag.path }"
              @click="selectBag(bag)"
            >
              <div class="bag-item-header">
                <span class="bag-item-name">{{ bag.name }}</span>
                <span class="bag-item-size">{{ bag.sizeGB }} GB</span>
              </div>
              <div class="bag-item-meta">
                <span>{{ bag.messageCount.toLocaleString() }} messages</span>
                <span>{{ bag.topicCount }} topics</span>
                <span>{{ formatDuration(bag.duration) }}</span>
              </div>
            </div>
          </div>
        </div>
        <div class="modal-footer">
          <button class="btn-secondary" @click="showBagSelector = false">Cancel</button>
          <button
            class="btn-primary"
            @click="openSelectedBag"
            :disabled="!selectedBag"
          >
            Open RosBag
          </button>
        </div>
      </div>
    </div>

    <!-- 主内容区域 -->
    <main class="replay-main">
      <!-- 左侧：Topic 列表 -->
      <aside class="topic-sidebar">
        <div class="sidebar-header">
          <h3>Topics ({{ topics.length }})</h3>
          <button class="btn-small" @click="selectAllTopics">Select All</button>
        </div>
        <div class="topic-list">
          <div
            v-for="topic in topics"
            :key="topic.name"
            class="topic-item"
            :class="{ selected: selectedTopics.includes(topic.name) }"
            @click="toggleTopic(topic.name)"
          >
            <div class="topic-checkbox">
              <input
                type="checkbox"
                :checked="selectedTopics.includes(topic.name)"
                @click.stop
              />
            </div>
            <div class="topic-info">
              <div class="topic-name">{{ topic.name }}</div>
              <div class="topic-type">{{ topic.type }}</div>
              <div class="topic-count">{{ topic.messageCount }} msgs</div>
            </div>
          </div>
        </div>
      </aside>

      <!-- 中间：可视化区域 -->
      <section class="viz-area">
        <div class="viz-header">
          <div class="view-tabs">
            <button
              v-for="tab in viewTabs"
              :key="tab.id"
              :class="['view-tab', { active: activeView === tab.id }]"
              @click="activeView = tab.id"
            >
              <span class="tab-icon">{{ tab.icon }}</span>
              <span class="tab-name">{{ tab.name }}</span>
            </button>
          </div>
        </div>

        <div class="view-container">
          <!-- Camera 视图 -->
          <div v-show="activeView === 'cameras'" class="view-panel camera-grid">
            <div
              v-for="camera in activeCameras"
              :key="camera.topic"
              class="camera-panel"
            >
              <div class="camera-header">
                <span class="camera-name">{{ camera.name }}</span>
                <span class="camera-topic">{{ camera.topic }}</span>
              </div>
              <div class="camera-image">
                <img
                  v-if="camera.imageData"
                  :src="camera.imageData"
                  :alt="camera.name"
                />
                <div v-else class="no-image">No image data</div>
              </div>
            </div>
          </div>

          <!-- Point Cloud 视图 -->
          <div v-show="activeView === 'pointcloud'" class="view-panel">
            <div class="pointcloud-viewer">
              <canvas ref="pointcloudCanvas"></canvas>
              <div class="pointcloud-info">
                <p>Point Count: {{ pointCloudPointCount }}</p>
              </div>
            </div>
          </div>

          <!-- Vehicle Data 视图 -->
          <div v-show="activeView === 'vehicle'" class="view-panel">
            <div class="vehicle-data">
              <div class="data-grid">
                <div class="data-card">
                  <div class="data-label">Speed</div>
                  <div class="data-value">{{ vehicleData.speed.toFixed(1) }} km/h</div>
                </div>
                <div class="data-card">
                  <div class="data-label">Steering Angle</div>
                  <div class="data-value">{{ vehicleData.steerAngle.toFixed(1) }}°</div>
                </div>
                <div class="data-card">
                  <div class="data-label">Brake Rate</div>
                  <div class="data-value">{{ vehicleData.brakeRate.toFixed(1) }}%</div>
                </div>
                <div class="data-card">
                  <div class="data-label">Battery SOC</div>
                  <div class="data-value">{{ vehicleData.soc }}%</div>
                </div>
              </div>
            </div>
          </div>

          <!-- Message Log 视图 -->
          <div v-show="activeView === 'messages'" class="view-panel">
            <div class="message-log">
              <div
                v-for="(msg, idx) in recentMessages"
                :key="idx"
                class="message-item"
              >
                <span class="msg-time">{{ formatTimestamp(msg.timestamp) }}</span>
                <span class="msg-topic">{{ msg.topic }}</span>
                <span class="msg-type">{{ msg.topicType }}</span>
              </div>
            </div>
          </div>
        </div>
      </section>

      <!-- 右侧：控制面板 -->
      <aside class="control-panel">
        <div class="panel-section">
          <h3>Playback Control</h3>
          <div class="playback-controls">
            <button
              class="btn-control btn-play"
              @click="startPlayback"
              :disabled="!currentBag || isStreaming || selectedTopics.length === 0"
            >
              ▶️ Play
            </button>
            <button
              class="btn-control btn-stop"
              @click="stopPlayback"
              :disabled="!isStreaming"
            >
              ⏹️ Stop
            </button>
          </div>
          <div class="playback-info">
            <div class="info-row">
              <span>Messages:</span>
              <span class="info-value">{{ messageCount.toLocaleString() }}</span>
            </div>
            <div class="info-row">
              <span>Status:</span>
              <span class="info-value">{{ streamState }}</span>
            </div>
          </div>
        </div>

        <div class="panel-section">
          <h3>Selected Topics</h3>
          <div class="selected-topics-list">
            <div
              v-for="topic in selectedTopics"
              :key="topic"
              class="selected-topic-item"
            >
              <span class="topic-name-short">{{ getTopicShortName(topic) }}</span>
              <button class="btn-remove" @click="toggleTopic(topic)">✕</button>
            </div>
            <div v-if="selectedTopics.length === 0" class="no-selection">
              No topics selected
            </div>
          </div>
        </div>

        <div class="panel-section">
          <h3>Statistics</h3>
          <div class="stats-grid">
            <div class="stat-item">
              <div class="stat-label">Camera Frames</div>
              <div class="stat-value">{{ stats.cameraFrames }}</div>
            </div>
            <div class="stat-item">
              <div class="stat-label">Point Clouds</div>
              <div class="stat-value">{{ stats.pointClouds }}</div>
            </div>
            <div class="stat-item">
              <div class="stat-label">CAN Messages</div>
              <div class="stat-value">{{ stats.canMessages }}</div>
            </div>
          </div>
        </div>
      </aside>
    </main>

    <!-- Settings Modal -->
    <div v-if="showSettings" class="modal-overlay" @click="showSettings = false">
      <div class="modal-content" @click.stop>
        <div class="modal-header">
          <h2>Settings</h2>
          <button class="btn-close" @click="showSettings = false">✕</button>
        </div>
        <div class="modal-body">
          <div class="setting-item">
            <label>Server URL</label>
            <input
              v-model="serverUrl"
              type="text"
              placeholder="ws://localhost:8765"
            />
          </div>
          <div class="setting-item">
            <label>Playback Rate</label>
            <input
              v-model.number="playbackRate"
              type="number"
              min="0.1"
              max="10"
              step="0.1"
            />
          </div>
        </div>
        <div class="modal-footer">
          <button class="btn-secondary" @click="showSettings = false">Cancel</button>
          <button class="btn-primary" @click="applySettings">Apply</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'
import NavBar from './NavBar.vue'
import { getRosBagStreamService, type RosBagInfo, type TopicInfo, type RosBagMessage } from '@/services/rosbagStreamService'

// ======================== Service Instance ========================

const streamService = getRosBagStreamService('ws://localhost:8765')

// ======================== Reactive State ========================

// Connection state
const connectionState = streamService.connectionState
const streamState = streamService.streamState
const isConnected = computed(() => streamService.isConnected)
const isStreaming = computed(() => streamService.isStreaming)

// Bag management
const availableBags = streamService.availableBags
const currentBag = streamService.currentBag
const topics = streamService.topics
const messageCount = streamService.messageCount

// UI state
const showBagSelector = ref(false)
const showSettings = ref(false)
const selectedBag = ref<RosBagInfo | null>(null)
const selectedTopics = ref<string[]>([])
const activeView = ref('cameras')

// Settings
const serverUrl = ref('ws://localhost:8765')
const playbackRate = ref(1.0)

// View tabs
const viewTabs = ref([
  { id: 'cameras', name: 'Cameras', icon: '📷' },
  { id: 'pointcloud', name: 'Point Cloud', icon: '🔵' },
  { id: 'vehicle', name: 'Vehicle Data', icon: '🚗' },
  { id: 'messages', name: 'Messages', icon: '📝' }
])

// Camera data
const activeCameras = ref<Array<{
  topic: string
  name: string
  imageData: string | null
}>>([])

// Point cloud data
const pointcloudCanvas = ref<HTMLCanvasElement | null>(null)
const pointCloudPointCount = ref(0)

// Vehicle data
const vehicleData = ref({
  speed: 0,
  steerAngle: 0,
  brakeRate: 0,
  soc: 0
})

// Message log
const recentMessages = ref<RosBagMessage[]>([])
const maxRecentMessages = 100

// Statistics
const stats = ref({
  cameraFrames: 0,
  pointClouds: 0,
  canMessages: 0
})

// ======================== Computed Properties ========================

const connectionStatusText = computed(() => {
  switch (connectionState.value) {
    case 'connected':
      return 'Connected'
    case 'connecting':
      return 'Connecting...'
    case 'disconnected':
      return 'Disconnected'
    case 'error':
      return 'Connection Error'
    default:
      return 'Unknown'
  }
})

// ======================== Utility Functions ========================

function formatDuration(seconds: number): string {
  const mins = Math.floor(seconds / 60)
  const secs = Math.floor(seconds % 60)
  return `${mins}:${secs.toString().padStart(2, '0')}`
}

function formatTimestamp(timestamp: number): string {
  const date = new Date(timestamp / 1000000) // Convert from nanoseconds
  return date.toLocaleTimeString()
}

function getTopicShortName(topic: string): string {
  const parts = topic.split('/')
  return parts[parts.length - 1] || topic
}

// ======================== Bag Management Functions ========================

async function refreshBagList() {
  try {
    await streamService.listBags()
  } catch (error) {
    console.error('Failed to refresh bag list:', error)
  }
}

function selectBag(bag: RosBagInfo) {
  selectedBag.value = bag
}

async function openSelectedBag() {
  if (!selectedBag.value) return

  try {
    await streamService.openBag(selectedBag.value.path)
    showBagSelector.value = false

    // Auto-select camera topics
    autoSelectCameraTopics()
  } catch (error) {
    console.error('Failed to open bag:', error)
  }
}

function autoSelectCameraTopics() {
  const cameraTopics = topics.value
    .filter(t => t.name.includes('/camera') && t.name.includes('/compressed'))
    .map(t => t.name)

  selectedTopics.value = cameraTopics.slice(0, 6) // Select first 6 cameras
}

// ======================== Topic Selection Functions ========================

function toggleTopic(topicName: string) {
  const index = selectedTopics.value.indexOf(topicName)
  if (index > -1) {
    selectedTopics.value.splice(index, 1)
  } else {
    selectedTopics.value.push(topicName)
  }
}

function selectAllTopics() {
  selectedTopics.value = topics.value.map(t => t.name)
}

// ======================== Playback Control Functions ========================

function startPlayback() {
  if (!currentBag.value || selectedTopics.value.length === 0) return

  // Initialize camera panels
  initializeCameraPanels()

  // Start streaming
  streamService.streamTopics({
    topics: selectedTopics.value,
    playbackRate: playbackRate.value
  })
}

function stopPlayback() {
  streamService.stopStream()
}

function applySettings() {
  showSettings.value = false
  // Reconnect with new URL if changed
  if (serverUrl.value !== streamService['serverUrl']) {
    streamService.disconnect()
    streamService['serverUrl'] = serverUrl.value
    streamService.connect()
  }
}

// ======================== Message Handling Functions ========================

function initializeCameraPanels() {
  const cameraTopics = selectedTopics.value.filter(t => t.includes('/camera'))
  activeCameras.value = cameraTopics.map(topic => ({
    topic,
    name: extractCameraName(topic),
    imageData: null
  }))
}

function extractCameraName(topic: string): string {
  const match = topic.match(/camera(\d+)/)
  return match ? `Camera ${match[1]}` : topic
}

function handleMessage(message: RosBagMessage) {
  // Add to recent messages
  recentMessages.value.unshift(message)
  if (recentMessages.value.length > maxRecentMessages) {
    recentMessages.value.pop()
  }

  // Route to appropriate handler
  if (message.topic.includes('/camera') && message.topic.includes('/compressed')) {
    handleCameraMessage(message)
  } else if (message.topic.includes('/pointcloud')) {
    handlePointCloudMessage(message)
  } else if (message.topic.includes('/chassis_can')) {
    handleVehicleMessage(message)
  }
}

function handleCameraMessage(message: any) {
  const camera = activeCameras.value.find(c => c.topic === message.topic)
  if (camera && message.messageType === 'image' && message.data) {
    // Message is already decoded by server, data is JPEG/PNG base64
    const format = message.format || 'jpeg'
    camera.imageData = `data:image/${format};base64,${message.data}`
    stats.value.cameraFrames++
  }
}

function handlePointCloudMessage(message: any) {
  // Decode point cloud data (simplified)
  pointCloudPointCount.value++
  stats.value.pointClouds++
}

function handleVehicleMessage(message: any) {
  try {
    // Message is already decoded by server
    if (message.messageType === 'data' && typeof message.data === 'number') {
      if (message.topic.includes('/speed')) {
        vehicleData.value.speed = message.data
      } else if (message.topic.includes('/steerAngle')) {
        vehicleData.value.steerAngle = message.data
      } else if (message.topic.includes('/brakeRate')) {
        vehicleData.value.brakeRate = message.data
      } else if (message.topic.includes('/SOC')) {
        vehicleData.value.soc = message.data
      }
      stats.value.canMessages++
    }
  } catch (error) {
    console.error('Error handling vehicle message:', error)
  }
}

// ======================== Lifecycle Hooks ========================

onMounted(async () => {
  console.log('[RosBagReplayPro] Component mounted')

  try {
    // Connect to streaming server
    await streamService.connect()

    // Load available bags
    await refreshBagList()

    // Register message handler
    streamService.onMessage(handleMessage)
  } catch (error) {
    console.error('[RosBagReplayPro] Initialization error:', error)
  }
})

onUnmounted(() => {
  console.log('[RosBagReplayPro] Component unmounted')

  // Stop streaming if active
  if (isStreaming.value) {
    streamService.stopStream()
  }

  // Remove message handler
  streamService.removeMessageHandler()

  // Disconnect from server
  streamService.disconnect()
})
</script>

<style scoped>
/* ======================== Layout ======================== */

.rosbag-replay-pro {
  display: flex;
  flex-direction: column;
  height: 100vh;
  background: #0a0e27;
  color: #e0e6ed;
  font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
}

/* ======================== Header ======================== */

.replay-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 1rem 2rem;
  background: linear-gradient(135deg, #1a1f3a 0%, #2d3561 100%);
  border-bottom: 2px solid #3d4785;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.3);
}

.header-left {
  display: flex;
  align-items: center;
  gap: 1rem;
}

.title {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  font-size: 1.5rem;
  font-weight: 700;
  margin: 0;
}

.icon {
  font-size: 1.8rem;
}

.version {
  padding: 0.25rem 0.75rem;
  background: rgba(99, 102, 241, 0.2);
  border: 1px solid #6366f1;
  border-radius: 12px;
  font-size: 0.75rem;
  font-weight: 600;
  color: #a5b4fc;
}

.header-center {
  flex: 1;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
}

.connection-status {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.5rem 1rem;
  border-radius: 8px;
  font-size: 0.875rem;
  font-weight: 500;
}

.connection-status.connected {
  background: rgba(34, 197, 94, 0.1);
  color: #4ade80;
}

.connection-status.connecting {
  background: rgba(251, 191, 36, 0.1);
  color: #fbbf24;
}

.connection-status.disconnected {
  background: rgba(239, 68, 68, 0.1);
  color: #f87171;
}

.status-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  background: currentColor;
  animation: pulse 2s infinite;
}

@keyframes pulse {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.5; }
}

.bag-info {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.25rem;
}

.bag-name {
  font-size: 1rem;
  font-weight: 600;
  color: #e0e6ed;
}

.bag-meta {
  font-size: 0.875rem;
  color: #94a3b8;
}

/* ======================== Buttons ======================== */

.btn-primary, .btn-secondary, .btn-small, .btn-control {
  padding: 0.5rem 1rem;
  border: none;
  border-radius: 8px;
  font-size: 0.875rem;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.2s;
}

.btn-primary {
  background: linear-gradient(135deg, #6366f1 0%, #8b5cf6 100%);
  color: white;
}

.btn-primary:hover:not(:disabled) {
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(99, 102, 241, 0.4);
}

.btn-primary:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.btn-secondary {
  background: rgba(148, 163, 184, 0.1);
  color: #94a3b8;
  border: 1px solid #475569;
}

.btn-secondary:hover {
  background: rgba(148, 163, 184, 0.2);
}

.btn-small {
  padding: 0.25rem 0.5rem;
  font-size: 0.75rem;
}

/* ======================== Modal ======================== */

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

.modal-content {
  background: #1a1f3a;
  border-radius: 16px;
  border: 1px solid #3d4785;
  box-shadow: 0 20px 60px rgba(0, 0, 0, 0.5);
  max-width: 600px;
  width: 90%;
  max-height: 80vh;
  display: flex;
  flex-direction: column;
}

.modal-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 1.5rem;
  border-bottom: 1px solid #3d4785;
}

.modal-header h2 {
  margin: 0;
  font-size: 1.25rem;
  font-weight: 700;
}

.btn-close {
  background: none;
  border: none;
  color: #94a3b8;
  font-size: 1.5rem;
  cursor: pointer;
  padding: 0;
  width: 32px;
  height: 32px;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: 8px;
  transition: all 0.2s;
}

.btn-close:hover {
  background: rgba(239, 68, 68, 0.1);
  color: #f87171;
}

.modal-body {
  padding: 1.5rem;
  overflow-y: auto;
  flex: 1;
}

.modal-footer {
  display: flex;
  gap: 1rem;
  justify-content: flex-end;
  padding: 1.5rem;
  border-top: 1px solid #3d4785;
}

/* ======================== Bag List ======================== */

.bag-list {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}

.bag-item {
  padding: 1rem;
  background: rgba(99, 102, 241, 0.05);
  border: 1px solid #3d4785;
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s;
}

.bag-item:hover {
  background: rgba(99, 102, 241, 0.1);
  border-color: #6366f1;
}

.bag-item.selected {
  background: rgba(99, 102, 241, 0.15);
  border-color: #6366f1;
}

.bag-item-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 0.5rem;
}

.bag-item-name {
  font-weight: 600;
  color: #e0e6ed;
}

.bag-item-size {
  color: #6366f1;
  font-weight: 600;
}

.bag-item-meta {
  display: flex;
  gap: 1rem;
  font-size: 0.875rem;
  color: #94a3b8;
}

/* ======================== Main Layout ======================== */

.replay-main {
  display: flex;
  flex: 1;
  overflow: hidden;
}

.topic-sidebar {
  width: 300px;
  background: #1a1f3a;
  border-right: 1px solid #3d4785;
  display: flex;
  flex-direction: column;
}

.sidebar-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 1rem;
  border-bottom: 1px solid #3d4785;
}

.sidebar-header h3 {
  margin: 0;
  font-size: 1rem;
  font-weight: 600;
}

.topic-list {
  flex: 1;
  overflow-y: auto;
  padding: 0.5rem;
}

.topic-item {
  display: flex;
  gap: 0.75rem;
  padding: 0.75rem;
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s;
  margin-bottom: 0.5rem;
}

.topic-item:hover {
  background: rgba(99, 102, 241, 0.1);
}

.topic-item.selected {
  background: rgba(99, 102, 241, 0.15);
  border: 1px solid #6366f1;
}

.topic-checkbox input {
  cursor: pointer;
}

.topic-info {
  flex: 1;
}

.topic-name {
  font-size: 0.875rem;
  font-weight: 600;
  color: #e0e6ed;
  margin-bottom: 0.25rem;
}

.topic-type {
  font-size: 0.75rem;
  color: #94a3b8;
  margin-bottom: 0.25rem;
}

.topic-count {
  font-size: 0.75rem;
  color: #6366f1;
}

/* ======================== Visualization Area ======================== */

.viz-area {
  flex: 1;
  display: flex;
  flex-direction: column;
  background: #0f1729;
}

.viz-header {
  padding: 1rem;
  border-bottom: 1px solid #3d4785;
}

.view-tabs {
  display: flex;
  gap: 0.5rem;
}

.view-tab {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.5rem 1rem;
  background: rgba(99, 102, 241, 0.05);
  border: 1px solid transparent;
  border-radius: 8px;
  cursor: pointer;
  transition: all 0.2s;
  color: #94a3b8;
}

.view-tab:hover {
  background: rgba(99, 102, 241, 0.1);
  color: #e0e6ed;
}

.view-tab.active {
  background: rgba(99, 102, 241, 0.15);
  border-color: #6366f1;
  color: #e0e6ed;
}

.view-container {
  flex: 1;
  overflow: hidden;
}

.view-panel {
  width: 100%;
  height: 100%;
  padding: 1rem;
}

.camera-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(400px, 1fr));
  gap: 1rem;
  overflow-y: auto;
}

.camera-panel {
  background: #1a1f3a;
  border: 1px solid #3d4785;
  border-radius: 8px;
  overflow: hidden;
}

.camera-header {
  display: flex;
  justify-content: space-between;
  padding: 0.75rem;
  background: rgba(99, 102, 241, 0.05);
  border-bottom: 1px solid #3d4785;
}

.camera-name {
  font-weight: 600;
  color: #e0e6ed;
}

.camera-topic {
  font-size: 0.75rem;
  color: #94a3b8;
}

.camera-image {
  aspect-ratio: 16/9;
  background: #0a0e27;
  display: flex;
  align-items: center;
  justify-content: center;
}

.camera-image img {
  width: 100%;
  height: 100%;
  object-fit: contain;
}

.no-image {
  color: #475569;
  font-size: 0.875rem;
}

/* ======================== Control Panel ======================== */

.control-panel {
  width: 320px;
  background: #1a1f3a;
  border-left: 1px solid #3d4785;
  display: flex;
  flex-direction: column;
  overflow-y: auto;
}

.panel-section {
  padding: 1.5rem;
  border-bottom: 1px solid #3d4785;
}

.panel-section h3 {
  margin: 0 0 1rem 0;
  font-size: 1rem;
  font-weight: 600;
}

.playback-controls {
  display: flex;
  gap: 0.5rem;
  margin-bottom: 1rem;
}

.btn-control {
  flex: 1;
  padding: 0.75rem;
  font-size: 1rem;
}

.btn-play {
  background: linear-gradient(135deg, #10b981 0%, #059669 100%);
  color: white;
}

.btn-stop {
  background: linear-gradient(135deg, #ef4444 0%, #dc2626 100%);
  color: white;
}

.playback-info, .stats-grid {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
}

.info-row, .stat-item {
  display: flex;
  justify-content: space-between;
  padding: 0.5rem;
  background: rgba(99, 102, 241, 0.05);
  border-radius: 6px;
}

.info-value, .stat-value {
  font-weight: 600;
  color: #6366f1;
}

.selected-topics-list {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
  max-height: 200px;
  overflow-y: auto;
}

.selected-topic-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 0.5rem;
  background: rgba(99, 102, 241, 0.05);
  border-radius: 6px;
}

.topic-name-short {
  font-size: 0.875rem;
  color: #e0e6ed;
}

.btn-remove {
  background: none;
  border: none;
  color: #ef4444;
  cursor: pointer;
  padding: 0.25rem;
  border-radius: 4px;
  transition: all 0.2s;
}

.btn-remove:hover {
  background: rgba(239, 68, 68, 0.1);
}

.no-selection {
  text-align: center;
  color: #475569;
  font-size: 0.875rem;
  padding: 1rem;
}

/* ======================== Vehicle Data & Message Log ======================== */

.vehicle-data, .message-log {
  height: 100%;
  overflow-y: auto;
}

.data-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 1rem;
}

.data-card {
  background: #1a1f3a;
  border: 1px solid #3d4785;
  border-radius: 8px;
  padding: 1.5rem;
  text-align: center;
}

.data-label {
  font-size: 0.875rem;
  color: #94a3b8;
  margin-bottom: 0.5rem;
}

.data-value {
  font-size: 1.5rem;
  font-weight: 700;
  color: #6366f1;
}

.message-item {
  display: flex;
  gap: 1rem;
  padding: 0.75rem;
  background: #1a1f3a;
  border: 1px solid #3d4785;
  border-radius: 6px;
  margin-bottom: 0.5rem;
  font-size: 0.875rem;
}

.msg-time {
  color: #94a3b8;
  min-width: 100px;
}

.msg-topic {
  color: #6366f1;
  flex: 1;
}

.msg-type {
  color: #94a3b8;
}

/* ======================== Settings ======================== */

.setting-item {
  margin-bottom: 1rem;
}

.setting-item label {
  display: block;
  margin-bottom: 0.5rem;
  font-size: 0.875rem;
  font-weight: 600;
  color: #e0e6ed;
}

.setting-item input {
  width: 100%;
  padding: 0.5rem;
  background: #0f1729;
  border: 1px solid #3d4785;
  border-radius: 6px;
  color: #e0e6ed;
  font-size: 0.875rem;
}

.setting-item input:focus {
  outline: none;
  border-color: #6366f1;
}
</style>
