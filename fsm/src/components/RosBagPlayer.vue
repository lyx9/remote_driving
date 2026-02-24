<!--
  FSM-Pilot V2.0 - Remote Driving System

  @project     FSM-Pilot Remote Driving Platform
  @author      Li Yixiang
  @institution City University of Hong Kong
  @copyright   2025 City University of Hong Kong. All rights reserved.
  @license     Proprietary

  @component   RosBagPlayer
  @description RosBag回放播放器组件，支持Autoware.universe实车数据回放
-->
<template>
  <div class="rosbag-player">
    <!-- 头部信息 -->
    <div class="player-header">
      <div class="bag-info" v-if="currentBag">
        <span class="bag-name">{{ currentBag.name }}</span>
        <span class="bag-meta">
          {{ formatFileSize(currentBag.size) }} |
          {{ currentBag.topics.length }} topics |
          {{ currentBag.messageCount.toLocaleString() }} msgs
        </span>
      </div>
      <div class="bag-info" v-else>
        <span class="no-bag">No RosBag loaded</span>
      </div>
      <div class="header-actions">
        <button class="btn-action" @click="showTopicList = !showTopicList" :class="{ active: showTopicList }">
          Topics
        </button>
        <button class="btn-action" @click="loadMockBag" :disabled="isLoading">
          {{ isLoading ? 'Loading...' : 'Load Mock' }}
        </button>
      </div>
    </div>

    <!-- 主内容区域 -->
    <div class="player-content">
      <!-- 车辆状态面板 -->
      <div class="status-panel">
        <div class="panel-title">Vehicle Status</div>
        <div class="status-grid">
          <div class="status-item">
            <span class="label">Speed</span>
            <span class="value">{{ currentSpeed.toFixed(1) }} m/s</span>
            <span class="unit">({{ (currentSpeed * 3.6).toFixed(1) }} km/h)</span>
          </div>
          <div class="status-item">
            <span class="label">Steering</span>
            <span class="value">{{ (currentSteering * 180 / Math.PI).toFixed(1) }}°</span>
          </div>
          <div class="status-item">
            <span class="label">Acceleration</span>
            <span class="value">{{ currentAcceleration.toFixed(2) }} m/s²</span>
          </div>
          <div class="status-item">
            <span class="label">Heading Rate</span>
            <span class="value">{{ (currentHeadingRate * 180 / Math.PI).toFixed(2) }}°/s</span>
          </div>
        </div>
      </div>

      <!-- 定位信息面板 -->
      <div class="location-panel">
        <div class="panel-title">Localization</div>
        <div class="location-grid">
          <div class="location-item">
            <span class="label">X</span>
            <span class="value">{{ currentPosition.x.toFixed(2) }} m</span>
          </div>
          <div class="location-item">
            <span class="label">Y</span>
            <span class="value">{{ currentPosition.y.toFixed(2) }} m</span>
          </div>
          <div class="location-item">
            <span class="label">Z</span>
            <span class="value">{{ currentPosition.z.toFixed(2) }} m</span>
          </div>
          <div class="location-item">
            <span class="label">Yaw</span>
            <span class="value">{{ currentYaw.toFixed(1) }}°</span>
          </div>
        </div>
      </div>

      <!-- 感知目标面板 -->
      <div class="perception-panel">
        <div class="panel-title">
          Detected Objects
          <span class="object-count">{{ detectedObjectCount }}</span>
        </div>
        <div class="objects-list" v-if="currentDetectedObjects">
          <div
            v-for="(obj, index) in currentDetectedObjects.objects.slice(0, 5)"
            :key="index"
            class="object-item"
          >
            <span class="obj-type" :class="getObjectTypeClass(obj)">
              {{ getObjectTypeName(obj) }}
            </span>
            <span class="obj-distance">
              {{ getObjectDistance(obj).toFixed(1) }}m
            </span>
            <span class="obj-confidence">
              {{ (obj.existence_probability * 100).toFixed(0) }}%
            </span>
          </div>
        </div>
        <div class="no-objects" v-else>
          No objects detected
        </div>
      </div>
    </div>

    <!-- 时间轴控制 -->
    <div class="timeline-section">
      <!-- 进度条 -->
      <div class="timeline-bar" @click="handleTimelineClick" ref="timelineRef">
        <div class="timeline-progress" :style="{ width: `${progress * 100}%` }"></div>
        <div class="timeline-thumb" :style="{ left: `${progress * 100}%` }"></div>
      </div>

      <!-- 时间显示 -->
      <div class="time-display">
        <span class="current-time">{{ currentTimeFormatted }}</span>
        <span class="separator">/</span>
        <span class="duration">{{ durationFormatted }}</span>
      </div>

      <!-- 播放控制 -->
      <div class="playback-controls">
        <button class="ctrl-btn" @click="stop" title="Stop">
          ⏹
        </button>
        <button class="ctrl-btn" @click="stepBackward" title="Step Backward">
          ⏮
        </button>
        <button class="ctrl-btn play-btn" @click="togglePlay" :disabled="!currentBag">
          {{ playbackState.isPlaying ? '⏸' : '▶' }}
        </button>
        <button class="ctrl-btn" @click="stepForward" title="Step Forward">
          ⏭
        </button>
        <button class="ctrl-btn" :class="{ active: playbackState.loopEnabled }" @click="toggleLoop" title="Loop">
          🔁
        </button>
      </div>

      <!-- 播放速率 -->
      <div class="playback-rate">
        <span class="rate-label">Speed:</span>
        <select v-model.number="selectedRate" @change="handleRateChange">
          <option :value="0.25">0.25x</option>
          <option :value="0.5">0.5x</option>
          <option :value="1">1x</option>
          <option :value="2">2x</option>
          <option :value="4">4x</option>
          <option :value="8">8x</option>
        </select>
      </div>
    </div>

    <!-- Topic 列表侧边栏 -->
    <Transition name="slide-left">
      <div class="topic-sidebar" v-if="showTopicList && currentBag">
        <div class="sidebar-header">
          <span>Topics ({{ currentBag.topics.length }})</span>
          <button class="close-btn" @click="showTopicList = false">×</button>
        </div>
        <div class="topic-list">
          <div
            v-for="topic in currentBag.topics"
            :key="topic.name"
            class="topic-item"
            :class="{ selected: selectedTopics.has(topic.name) }"
            @click="toggleTopic(topic.name)"
          >
            <div class="topic-name">{{ topic.name }}</div>
            <div class="topic-meta">
              <span class="topic-type">{{ getShortTypeName(topic.type) }}</span>
              <span class="topic-freq">{{ topic.frequency }} Hz</span>
            </div>
          </div>
        </div>
      </div>
    </Transition>

    <!-- 元数据信息 -->
    <div class="metadata-bar" v-if="currentBag?.metadata">
      <span class="meta-item" v-if="currentBag.metadata.vehicle_id">
        🚗 {{ currentBag.metadata.vehicle_id }}
      </span>
      <span class="meta-item" v-if="currentBag.metadata.location">
        📍 {{ currentBag.metadata.location }}
      </span>
      <span class="meta-item" v-if="currentBag.metadata.date">
        📅 {{ formatDate(currentBag.metadata.date) }}
      </span>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'
import {
  useRosBagPlayer,
  formatFileSize,
  quaternionToEuler,
  radToDeg,
  type AutowareDetectedObject
} from '@/services/rosbagPlayer'

// RosBag 播放器
const {
  isLoading,
  currentBag,
  playbackState,
  progress,
  currentTimeFormatted,
  durationFormatted,
  currentVehicleStatus,
  currentKinematicState,
  currentDetectedObjects,
  loadMockBag: loadMock,
  play,
  pause,
  stop,
  seekToProgress,
  setPlaybackRate,
  toggleLoop,
  stepForward,
  stepBackward,
  dispose
} = useRosBagPlayer()

// UI 状态
const showTopicList = ref(false)
const selectedTopics = ref(new Set<string>())
const selectedRate = ref(1)
const timelineRef = ref<HTMLElement | null>(null)

// 计算属性 - 车辆状态
const currentSpeed = computed(() => {
  return currentVehicleStatus.value?.longitudinal_velocity || 0
})

const currentSteering = computed(() => {
  return currentVehicleStatus.value?.steering_tire_angle || 0
})

const currentAcceleration = computed(() => {
  return currentVehicleStatus.value?.longitudinal_acceleration || 0
})

const currentHeadingRate = computed(() => {
  return currentVehicleStatus.value?.heading_rate || 0
})

// 计算属性 - 定位
const currentPosition = computed(() => {
  const pose = currentKinematicState.value?.pose?.pose?.position
  return {
    x: pose?.x || 0,
    y: pose?.y || 0,
    z: pose?.z || 0
  }
})

const currentYaw = computed(() => {
  const orientation = currentKinematicState.value?.pose?.pose?.orientation
  if (!orientation) return 0
  const euler = quaternionToEuler(orientation)
  return radToDeg(euler.yaw)
})

// 计算属性 - 感知
const detectedObjectCount = computed(() => {
  return currentDetectedObjects.value?.objects?.length || 0
})

// 方法
const loadMockBag = async () => {
  await loadMock()
}

const togglePlay = () => {
  if (playbackState.isPlaying) {
    pause()
  } else {
    play()
  }
}

const handleTimelineClick = (e: MouseEvent) => {
  if (!timelineRef.value || !currentBag.value) return

  const rect = timelineRef.value.getBoundingClientRect()
  const percent = (e.clientX - rect.left) / rect.width
  seekToProgress(percent)
}

const handleRateChange = () => {
  setPlaybackRate(selectedRate.value)
}

const toggleTopic = (topicName: string) => {
  if (selectedTopics.value.has(topicName)) {
    selectedTopics.value.delete(topicName)
  } else {
    selectedTopics.value.add(topicName)
  }
}

const getShortTypeName = (typeName: string): string => {
  const parts = typeName.split('/')
  return parts[parts.length - 1]
}

const getObjectTypeName = (obj: AutowareDetectedObject): string => {
  const label = obj.classification[0]?.label || 0
  const types = ['Car', 'Pedestrian', 'Cyclist', 'Truck', 'Bus', 'Unknown']
  return types[label] || 'Unknown'
}

const getObjectTypeClass = (obj: AutowareDetectedObject): string => {
  const label = obj.classification[0]?.label || 0
  const classes = ['car', 'pedestrian', 'cyclist', 'truck', 'bus', 'unknown']
  return classes[label] || 'unknown'
}

const getObjectDistance = (obj: AutowareDetectedObject): number => {
  const pos = obj.kinematics.pose_with_covariance.pose.position
  return Math.sqrt(pos.x * pos.x + pos.y * pos.y)
}

const formatDate = (dateStr: string): string => {
  const date = new Date(dateStr)
  return date.toLocaleDateString('zh-CN', {
    year: 'numeric',
    month: '2-digit',
    day: '2-digit',
    hour: '2-digit',
    minute: '2-digit'
  })
}

// 生命周期
onMounted(() => {
  // 可以自动加载 Mock 数据用于演示
  // loadMockBag()
})

onUnmounted(() => {
  dispose()
})

// 暴露给父组件
defineExpose({
  loadMockBag,
  play,
  pause,
  stop
})
</script>

<style scoped>
.rosbag-player {
  display: flex;
  flex-direction: column;
  height: 100%;
  background: #0d0d14;
  color: #e0e0e0;
  font-family: 'JetBrains Mono', monospace;
  position: relative;
}

/* Header */
.player-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px 12px;
  background: #14141f;
  border-bottom: 1px solid #2a2a3a;
}

.bag-info {
  display: flex;
  flex-direction: column;
  gap: 2px;
}

.bag-name {
  font-size: 13px;
  font-weight: bold;
  color: var(--primary, #ff5722);
}

.bag-meta {
  font-size: 10px;
  color: #666;
}

.no-bag {
  font-size: 12px;
  color: #666;
  font-style: italic;
}

.header-actions {
  display: flex;
  gap: 8px;
}

.btn-action {
  padding: 6px 12px;
  background: #1a1a2e;
  border: 1px solid #333;
  color: #aaa;
  font-size: 11px;
  cursor: pointer;
  transition: all 0.2s;
}

.btn-action:hover {
  background: #252540;
  color: #fff;
}

.btn-action.active {
  background: rgba(255, 87, 34, 0.15);
  border-color: var(--primary, #ff5722);
  color: var(--primary, #ff5722);
}

.btn-action:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

/* Content */
.player-content {
  flex: 1;
  display: grid;
  grid-template-columns: 1fr 1fr;
  grid-template-rows: auto auto;
  gap: 10px;
  padding: 10px;
  overflow-y: auto;
}

.panel-title {
  font-size: 11px;
  font-weight: bold;
  color: var(--primary, #ff5722);
  margin-bottom: 8px;
  padding-bottom: 4px;
  border-bottom: 1px solid #2a2a3a;
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.status-panel,
.location-panel,
.perception-panel {
  background: #14141f;
  border: 1px solid #2a2a3a;
  padding: 10px;
}

.perception-panel {
  grid-column: span 2;
}

.status-grid,
.location-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 8px;
}

.status-item,
.location-item {
  display: flex;
  flex-direction: column;
  gap: 2px;
}

.status-item .label,
.location-item .label {
  font-size: 9px;
  color: #666;
  text-transform: uppercase;
}

.status-item .value,
.location-item .value {
  font-size: 14px;
  color: #fff;
  font-weight: bold;
}

.status-item .unit {
  font-size: 10px;
  color: #888;
}

/* Objects */
.object-count {
  background: var(--primary, #ff5722);
  color: #000;
  padding: 2px 6px;
  border-radius: 10px;
  font-size: 10px;
}

.objects-list {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.object-item {
  display: flex;
  align-items: center;
  gap: 10px;
  padding: 6px 8px;
  background: rgba(255, 255, 255, 0.03);
  border-left: 3px solid #444;
}

.object-item .obj-type {
  font-size: 10px;
  padding: 2px 6px;
  border-radius: 3px;
  min-width: 70px;
  text-align: center;
}

.obj-type.car { background: rgba(0, 150, 255, 0.2); border-color: #0096ff; color: #0096ff; }
.obj-type.pedestrian { background: rgba(255, 200, 0, 0.2); border-color: #ffc800; color: #ffc800; }
.obj-type.cyclist { background: rgba(0, 255, 100, 0.2); border-color: #00ff64; color: #00ff64; }
.obj-type.truck { background: rgba(255, 100, 0, 0.2); border-color: #ff6400; color: #ff6400; }

.object-item .obj-distance {
  font-size: 11px;
  color: #aaa;
  min-width: 50px;
}

.object-item .obj-confidence {
  font-size: 10px;
  color: #666;
}

.no-objects {
  font-size: 11px;
  color: #555;
  font-style: italic;
  padding: 10px;
  text-align: center;
}

/* Timeline */
.timeline-section {
  padding: 12px;
  background: #14141f;
  border-top: 1px solid #2a2a3a;
}

.timeline-bar {
  height: 8px;
  background: #1a1a2e;
  border-radius: 4px;
  cursor: pointer;
  position: relative;
  margin-bottom: 10px;
}

.timeline-progress {
  height: 100%;
  background: linear-gradient(90deg, var(--primary, #ff5722), #00a0cc);
  border-radius: 4px;
  transition: width 0.1s;
}

.timeline-thumb {
  position: absolute;
  top: 50%;
  width: 14px;
  height: 14px;
  background: #fff;
  border-radius: 50%;
  transform: translate(-50%, -50%);
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.4);
  transition: left 0.1s;
}

.time-display {
  display: flex;
  justify-content: center;
  gap: 8px;
  margin-bottom: 10px;
  font-size: 14px;
  font-family: 'JetBrains Mono', monospace;
}

.current-time {
  color: var(--primary, #ff5722);
  font-weight: bold;
}

.separator {
  color: #444;
}

.duration {
  color: #888;
}

.playback-controls {
  display: flex;
  justify-content: center;
  gap: 8px;
  margin-bottom: 10px;
}

.ctrl-btn {
  width: 36px;
  height: 36px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: #1a1a2e;
  border: 1px solid #333;
  color: #aaa;
  font-size: 14px;
  cursor: pointer;
  transition: all 0.2s;
  border-radius: 4px;
}

.ctrl-btn:hover {
  background: #252540;
  color: #fff;
}

.ctrl-btn.play-btn {
  width: 48px;
  height: 48px;
  font-size: 18px;
  background: var(--primary, #ff5722);
  color: #000;
  border: none;
}

.ctrl-btn.play-btn:hover {
  background: #00d4e0;
}

.ctrl-btn.play-btn:disabled {
  background: #333;
  color: #666;
  cursor: not-allowed;
}

.ctrl-btn.active {
  background: rgba(255, 87, 34, 0.2);
  border-color: var(--primary, #ff5722);
  color: var(--primary, #ff5722);
}

.playback-rate {
  display: flex;
  justify-content: center;
  align-items: center;
  gap: 8px;
}

.rate-label {
  font-size: 11px;
  color: #666;
}

.playback-rate select {
  padding: 4px 8px;
  background: #1a1a2e;
  border: 1px solid #333;
  color: #fff;
  font-size: 11px;
  cursor: pointer;
}

/* Topic Sidebar */
.topic-sidebar {
  position: absolute;
  top: 0;
  right: 0;
  width: 300px;
  height: 100%;
  background: #14141f;
  border-left: 1px solid #2a2a3a;
  z-index: 100;
  display: flex;
  flex-direction: column;
}

.sidebar-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 10px 12px;
  border-bottom: 1px solid #2a2a3a;
  font-size: 12px;
  font-weight: bold;
  color: var(--primary, #ff5722);
}

.close-btn {
  width: 24px;
  height: 24px;
  background: transparent;
  border: none;
  color: #888;
  font-size: 18px;
  cursor: pointer;
}

.close-btn:hover {
  color: #fff;
}

.topic-list {
  flex: 1;
  overflow-y: auto;
}

.topic-item {
  padding: 8px 12px;
  border-bottom: 1px solid #1a1a2e;
  cursor: pointer;
  transition: background 0.2s;
}

.topic-item:hover {
  background: rgba(255, 255, 255, 0.03);
}

.topic-item.selected {
  background: rgba(255, 87, 34, 0.1);
  border-left: 3px solid var(--primary, #ff5722);
}

.topic-name {
  font-size: 11px;
  color: #ddd;
  word-break: break-all;
}

.topic-meta {
  display: flex;
  gap: 10px;
  margin-top: 4px;
}

.topic-type {
  font-size: 9px;
  color: #666;
}

.topic-freq {
  font-size: 9px;
  color: var(--primary, #ff5722);
}

/* Metadata Bar */
.metadata-bar {
  display: flex;
  gap: 16px;
  padding: 6px 12px;
  background: #0a0a12;
  border-top: 1px solid #2a2a3a;
}

.meta-item {
  font-size: 10px;
  color: #888;
}

/* Transitions */
.slide-left-enter-active,
.slide-left-leave-active {
  transition: transform 0.3s ease;
}

.slide-left-enter-from,
.slide-left-leave-to {
  transform: translateX(100%);
}
</style>
