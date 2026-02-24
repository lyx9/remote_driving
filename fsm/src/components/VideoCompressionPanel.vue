<!--
  FSM-Pilot V2.0 - Remote Driving System

  @project     FSM-Pilot Remote Driving Platform
  @author      Li Yixiang
  @institution City University of Hong Kong
  @copyright   2025 City University of Hong Kong. All rights reserved.
  @license     Proprietary

  @component   VideoCompressionPanel
  @description 视频压缩控制面板
-->
<template>
  <div class="compression-panel">
    <div class="panel-header">
      <div class="header-title">
        <span class="icon">🎥</span>
        <span class="title-text">视频压缩</span>
      </div>
      <button
        class="toggle-btn"
        :class="{ active: enabled }"
        @click="toggleCompression"
      >
        <span v-if="enabled">✓ 已启用</span>
        <span v-else>启用</span>
      </button>
    </div>

    <div class="panel-content" v-if="enabled">
      <!-- 压缩模式选择 -->
      <div class="mode-section">
        <div class="section-label">压缩模式</div>
        <div class="mode-buttons">
          <button
            v-for="(preset, modeKey) in COMPRESSION_MODES"
            :key="modeKey"
            class="mode-btn"
            :class="{ active: mode === modeKey }"
            @click="setMode(modeKey as CompressionMode)"
          >
            <div class="mode-name">{{ getModeDisplayName(modeKey) }}</div>
            <div class="mode-desc">{{ preset.description }}</div>
            <div class="mode-savings" v-if="preset.savingsPercent > 0">
              省流 {{ preset.savingsPercent }}%
            </div>
          </button>
        </div>

        <!-- 自动模式 -->
        <div class="auto-mode">
          <label class="checkbox-label">
            <input
              type="checkbox"
              :checked="mode === 'auto'"
              @change="toggleAuto"
            />
            <span>自动调整 (根据网络状况)</span>
          </label>
        </div>
      </div>

      <!-- 当前配置 -->
      <div class="config-section">
        <div class="section-label">当前配置</div>
        <div class="config-grid">
          <div class="config-item">
            <span class="config-label">帧率</span>
            <span class="config-value">{{ currentPreset.fps }} FPS</span>
          </div>
          <div class="config-item">
            <span class="config-label">码率</span>
            <span class="config-value">{{ currentPreset.bitrate }} kbps</span>
          </div>
          <div class="config-item">
            <span class="config-label">带宽节省</span>
            <span class="config-value savings">{{ currentPreset.savingsPercent.toFixed(1) }}%</span>
          </div>
        </div>
      </div>

      <!-- 带宽估算 -->
      <div class="bandwidth-section">
        <div class="section-label">带宽估算 (4 摄像头)</div>
        <div class="bandwidth-comparison">
          <div class="bandwidth-bar">
            <div class="bar-label">未压缩</div>
            <div class="bar-container">
              <div class="bar-fill uncompressed" :style="{ width: '100%' }">
                {{ bandwidthEstimate.uncompressed.toFixed(1) }} Mbps
              </div>
            </div>
          </div>
          <div class="bandwidth-bar">
            <div class="bar-label">已压缩</div>
            <div class="bar-container">
              <div
                class="bar-fill compressed"
                :style="{ width: `${(bandwidthEstimate.compressed / bandwidthEstimate.uncompressed) * 100}%` }"
              >
                {{ bandwidthEstimate.compressed.toFixed(1) }} Mbps
              </div>
            </div>
          </div>
        </div>
        <div class="bandwidth-savings">
          节省 <strong>{{ bandwidthEstimate.savings.toFixed(1) }}%</strong> 带宽
        </div>
      </div>

      <!-- 网络质量 -->
      <div class="network-section" v-if="mode === 'auto'">
        <div class="section-label">网络状况</div>
        <div class="network-metrics">
          <div class="metric-item">
            <span class="metric-icon">📶</span>
            <span class="metric-label">带宽</span>
            <span class="metric-value">{{ networkQuality.bandwidth.toFixed(1) }} Mbps</span>
          </div>
          <div class="metric-item">
            <span class="metric-icon">⏱️</span>
            <span class="metric-label">延迟</span>
            <span class="metric-value">{{ networkQuality.latency }} ms</span>
          </div>
          <div class="metric-item">
            <span class="metric-icon">📉</span>
            <span class="metric-label">丢包</span>
            <span class="metric-value">{{ networkQuality.packetLoss.toFixed(1) }}%</span>
          </div>
        </div>
      </div>

      <!-- 统计信息 -->
      <div class="stats-section" v-if="showStats">
        <div class="section-label">实时统计</div>
        <div class="stats-grid">
          <div class="stat-item">
            <span class="stat-label">实际帧率</span>
            <span class="stat-value">{{ stats.currentFps.toFixed(1) }} FPS</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">实际码率</span>
            <span class="stat-value">{{ stats.currentBitrate }} kbps</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">丢帧数</span>
            <span class="stat-value">{{ stats.droppedFrames }}</span>
          </div>
          <div class="stat-item">
            <span class="stat-label">总帧数</span>
            <span class="stat-value">{{ stats.totalFrames }}</span>
          </div>
        </div>
      </div>

      <!-- 推荐模式 -->
      <div class="recommendation-section" v-if="recommendation">
        <div class="recommendation-card">
          <span class="recommendation-icon">💡</span>
          <div class="recommendation-text">
            <div class="recommendation-title">建议使用 <strong>{{ getModeDisplayName(recommendation.mode) }}</strong></div>
            <div class="recommendation-reason">{{ recommendation.reason }}</div>
          </div>
          <button
            class="apply-btn"
            v-if="recommendation.mode !== mode"
            @click="setMode(recommendation.mode)"
          >
            应用
          </button>
        </div>
      </div>
    </div>

    <div class="panel-footer" v-if="enabled">
      <button class="action-btn" @click="showStats = !showStats">
        {{ showStats ? '隐藏' : '显示' }}统计
      </button>
      <button class="action-btn" @click="resetStats">
        重置统计
      </button>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue'
import {
  useVideoCompressionService,
  type CompressionMode,
  COMPRESSION_MODES
} from '@/services/videoCompressionService'

const compressionService = useVideoCompressionService()

// 解构服务
const {
  enabled,
  mode,
  currentPreset,
  stats,
  networkQuality,
  enableCompression,
  disableCompression,
  setCompressionMode,
  toggleAutoAdjust,
  estimateBandwidth,
  getRecommendedMode,
  resetStats
} = compressionService

// 本地状态
const showStats = ref(false)

// 计算属性
const bandwidthEstimate = computed(() => estimateBandwidth(4))

const recommendation = computed(() => {
  if (mode.value === 'auto') return null
  return getRecommendedMode(networkQuality.value.bandwidth, 4)
})

// 方法
function toggleCompression() {
  if (enabled.value) {
    disableCompression()
  } else {
    enableCompression('light')
  }
}

function setMode(newMode: CompressionMode) {
  setCompressionMode(newMode)
}

function toggleAuto() {
  toggleAutoAdjust()
}

function getModeDisplayName(modeKey: string): string {
  const names: Record<string, string> = {
    none: '无压缩',
    light: '轻度',
    medium: '中度',
    heavy: '重度',
    auto: '自动'
  }
  return names[modeKey] || modeKey
}
</script>

<style scoped>
.compression-panel {
  background: #14141f;
  border-radius: 8px;
  border: 1px solid #2a2a3a;
  overflow: hidden;
}

/* Header */
.panel-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 16px;
  background: #1a1a2e;
  border-bottom: 1px solid #2a2a3a;
}

.header-title {
  display: flex;
  align-items: center;
  gap: 8px;
}

.icon {
  font-size: 18px;
}

.title-text {
  font-size: 14px;
  font-weight: 600;
  color: #fff;
}

.toggle-btn {
  padding: 6px 16px;
  background: #2a2a3a;
  border: 1px solid #3a3a4a;
  color: #aaa;
  font-size: 12px;
  border-radius: 4px;
  cursor: pointer;
  transition: all 0.2s;
}

.toggle-btn:hover {
  background: #3a3a4a;
  color: #fff;
}

.toggle-btn.active {
  background: var(--primary, #ff5722);
  border-color: var(--primary, #ff5722);
  color: #000;
}

/* Content */
.panel-content {
  padding: 16px;
}

.section-label {
  font-size: 11px;
  font-weight: 600;
  color: var(--primary, #ff5722);
  text-transform: uppercase;
  letter-spacing: 0.5px;
  margin-bottom: 10px;
}

/* Mode Section */
.mode-section {
  margin-bottom: 20px;
}

.mode-buttons {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 8px;
  margin-bottom: 12px;
}

.mode-btn {
  padding: 12px;
  background: rgba(255, 255, 255, 0.02);
  border: 1px solid #2a2a3a;
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.2s;
  text-align: left;
}

.mode-btn:hover {
  background: rgba(255, 255, 255, 0.05);
  border-color: #3a3a4a;
}

.mode-btn.active {
  background: rgba(255, 87, 34, 0.1);
  border-color: var(--primary, #ff5722);
}

.mode-name {
  font-size: 12px;
  font-weight: 600;
  color: #fff;
  margin-bottom: 4px;
}

.mode-desc {
  font-size: 10px;
  color: #888;
  margin-bottom: 4px;
}

.mode-savings {
  font-size: 10px;
  color: var(--primary, #ff5722);
  font-weight: 600;
}

.auto-mode {
  padding: 10px;
  background: rgba(255, 87, 34, 0.05);
  border-radius: 4px;
}

.checkbox-label {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 12px;
  color: #ddd;
  cursor: pointer;
}

.checkbox-label input[type="checkbox"] {
  accent-color: var(--primary, #ff5722);
}

/* Config Section */
.config-section {
  margin-bottom: 20px;
}

.config-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 8px;
}

.config-item {
  display: flex;
  flex-direction: column;
  gap: 4px;
  padding: 10px;
  background: rgba(255, 255, 255, 0.02);
  border-radius: 4px;
}

.config-label {
  font-size: 10px;
  color: #888;
}

.config-value {
  font-size: 13px;
  font-weight: 600;
  color: #fff;
  font-family: 'JetBrains Mono', monospace;
}

.config-value.savings {
  color: var(--primary, #ff5722);
}

/* Bandwidth Section */
.bandwidth-section {
  margin-bottom: 20px;
}

.bandwidth-comparison {
  margin-bottom: 10px;
}

.bandwidth-bar {
  display: flex;
  align-items: center;
  gap: 12px;
  margin-bottom: 8px;
}

.bar-label {
  font-size: 11px;
  color: #888;
  width: 60px;
}

.bar-container {
  flex: 1;
  height: 28px;
  background: rgba(255, 255, 255, 0.05);
  border-radius: 4px;
  overflow: hidden;
}

.bar-fill {
  height: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 11px;
  font-weight: 600;
  transition: width 0.3s;
}

.bar-fill.uncompressed {
  background: linear-gradient(90deg, #ff6b6b, #ff8787);
  color: #fff;
}

.bar-fill.compressed {
  background: linear-gradient(90deg, var(--primary, #ff5722), #00d4e0);
  color: #000;
}

.bandwidth-savings {
  font-size: 12px;
  color: #ddd;
  text-align: center;
}

.bandwidth-savings strong {
  color: var(--primary, #ff5722);
  font-size: 14px;
}

/* Network Section */
.network-section {
  margin-bottom: 20px;
}

.network-metrics {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 8px;
}

.metric-item {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 4px;
  padding: 10px;
  background: rgba(255, 255, 255, 0.02);
  border-radius: 4px;
}

.metric-icon {
  font-size: 16px;
}

.metric-label {
  font-size: 10px;
  color: #888;
}

.metric-value {
  font-size: 12px;
  font-weight: 600;
  color: #fff;
  font-family: 'JetBrains Mono', monospace;
}

/* Stats Section */
.stats-section {
  margin-bottom: 20px;
}

.stats-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 8px;
}

.stat-item {
  display: flex;
  flex-direction: column;
  gap: 4px;
  padding: 10px;
  background: rgba(255, 255, 255, 0.02);
  border-radius: 4px;
}

.stat-label {
  font-size: 10px;
  color: #888;
}

.stat-value {
  font-size: 13px;
  font-weight: 600;
  color: #fff;
  font-family: 'JetBrains Mono', monospace;
}

/* Recommendation Section */
.recommendation-section {
  margin-bottom: 20px;
}

.recommendation-card {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 12px;
  background: rgba(255, 193, 7, 0.1);
  border: 1px solid rgba(255, 193, 7, 0.3);
  border-radius: 6px;
}

.recommendation-icon {
  font-size: 20px;
}

.recommendation-text {
  flex: 1;
}

.recommendation-title {
  font-size: 12px;
  color: #fff;
  margin-bottom: 4px;
}

.recommendation-title strong {
  color: #ffc107;
}

.recommendation-reason {
  font-size: 10px;
  color: #bbb;
}

.apply-btn {
  padding: 6px 12px;
  background: #ffc107;
  border: none;
  color: #000;
  font-size: 11px;
  font-weight: 600;
  border-radius: 4px;
  cursor: pointer;
  transition: all 0.2s;
}

.apply-btn:hover {
  background: #ffca28;
}

/* Footer */
.panel-footer {
  display: flex;
  gap: 8px;
  padding: 12px 16px;
  background: #1a1a2e;
  border-top: 1px solid #2a2a3a;
}

.action-btn {
  flex: 1;
  padding: 8px;
  background: #2a2a3a;
  border: 1px solid #3a3a4a;
  color: #aaa;
  font-size: 11px;
  border-radius: 4px;
  cursor: pointer;
  transition: all 0.2s;
}

.action-btn:hover {
  background: #3a3a4a;
  color: #fff;
}
</style>
