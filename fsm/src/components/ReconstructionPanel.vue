<!--
  Guardian Mobility v0.0 - 3DGS Reconstruction Panel

  ROS Bag到3D Gaussian Splatting重建面板

  功能:
  - ROS bag文件上传
  - 重建参数配置
  - 实时进度监控
  - 结果可视化

  @author Li Yixiang
  @institution City University of Hong Kong
-->
<template>
  <div class="reconstruction-panel">
    <div class="panel-header">
      <h2>🎨 3D Gaussian Splatting Reconstruction</h2>
      <p class="subtitle">从ROS Bag重建3D场景</p>
    </div>

    <!-- 步骤指示器 -->
    <div class="steps">
      <div
        v-for="(step, index) in steps"
        :key="index"
        :class="['step', { active: currentStep === index, completed: currentStep > index }]"
      >
        <div class="step-number">{{ index + 1 }}</div>
        <div class="step-label">{{ step }}</div>
      </div>
    </div>

    <!-- 步骤1: 文件上传 -->
    <div v-if="currentStep === 0" class="step-content">
      <div class="upload-section">
        <div class="upload-area" @drop.prevent="handleDrop" @dragover.prevent>
          <input
            ref="fileInput"
            type="file"
            accept=".bag"
            @change="handleFileSelect"
            style="display: none"
          />

          <div v-if="!selectedFile" class="upload-placeholder" @click="$refs.fileInput.click()">
            <div class="upload-icon">📦</div>
            <h3>选择或拖拽ROS Bag文件</h3>
            <p>支持 .bag 格式</p>
            <button class="btn-primary">选择文件</button>
          </div>

          <div v-else class="file-info">
            <div class="file-icon">✅</div>
            <div class="file-details">
              <h4>{{ selectedFile.name }}</h4>
              <p>大小: {{ formatFileSize(selectedFile.size) }}</p>
            </div>
            <button class="btn-secondary" @click="clearFile">更换文件</button>
          </div>
        </div>

        <div class="recent-files" v-if="recentFiles.length > 0">
          <h4>最近使用的文件</h4>
          <div
            v-for="file in recentFiles"
            :key="file.path"
            class="recent-file-item"
            @click="selectRecentFile(file)"
          >
            <span class="file-name">{{ file.name }}</span>
            <span class="file-date">{{ formatDate(file.date) }}</span>
          </div>
        </div>
      </div>

      <div class="actions">
        <button
          class="btn-primary btn-large"
          :disabled="!selectedFile"
          @click="nextStep"
        >
          下一步 →
        </button>
      </div>
    </div>

    <!-- 步骤2: 参数配置 -->
    <div v-if="currentStep === 1" class="step-content">
      <div class="config-section">
        <div class="config-group">
          <h3>📷 ROS配置</h3>
          <div class="form-group">
            <label>图像话题</label>
            <input v-model="config.imageTopic" type="text" placeholder="/camera/image_raw" />
          </div>
          <div class="form-group">
            <label>相机信息话题</label>
            <input v-model="config.cameraInfoTopic" type="text" placeholder="/camera/camera_info" />
          </div>
          <div class="form-group">
            <label>最大图像数量</label>
            <input v-model.number="config.maxImages" type="number" placeholder="不限制" />
            <small>留空表示使用所有图像</small>
          </div>
        </div>

        <div class="config-group">
          <h3>🔧 COLMAP配置</h3>
          <div class="form-group">
            <label>重建质量</label>
            <select v-model="config.colmapQuality">
              <option value="low">低 (快速)</option>
              <option value="medium">中等</option>
              <option value="high">高 (推荐)</option>
              <option value="extreme">极高 (慢)</option>
            </select>
          </div>
          <div class="form-group">
            <label>匹配器类型</label>
            <select v-model="config.colmapMatcher">
              <option value="exhaustive">穷举匹配 (精确)</option>
              <option value="sequential">顺序匹配 (快速)</option>
            </select>
          </div>
        </div>

        <div class="config-group">
          <h3>✨ 3DGS配置</h3>
          <div class="form-group">
            <label>训练迭代次数</label>
            <input v-model.number="config.iterations" type="number" min="1000" max="50000" />
            <small>推荐: 30000</small>
          </div>
          <div class="form-group">
            <label>分辨率</label>
            <select v-model.number="config.resolution">
              <option :value="1">全分辨率</option>
              <option :value="2">1/2分辨率</option>
              <option :value="4">1/4分辨率</option>
            </select>
          </div>
          <div class="form-group">
            <label>球谐函数阶数</label>
            <input v-model.number="config.shDegree" type="number" min="0" max="4" />
            <small>推荐: 3</small>
          </div>
        </div>

        <div class="config-group">
          <h3>⚙️ 性能配置</h3>
          <div class="form-group">
            <label>
              <input v-model="config.useGpu" type="checkbox" />
              使用GPU加速
            </label>
          </div>
          <div class="form-group" v-if="config.useGpu">
            <label>GPU ID</label>
            <input v-model.number="config.gpuId" type="number" min="0" />
          </div>
          <div class="form-group">
            <label>图像下采样因子</label>
            <input v-model.number="config.downsampleFactor" type="number" min="0.1" max="1" step="0.1" />
            <small>1.0 = 原始大小</small>
          </div>
        </div>
      </div>

      <div class="actions">
        <button class="btn-secondary" @click="prevStep">← 上一步</button>
        <button class="btn-primary btn-large" @click="startReconstruction">
          开始重建 🚀
        </button>
      </div>
    </div>

    <!-- 步骤3: 重建进行中 -->
    <div v-if="currentStep === 2" class="step-content">
      <div class="progress-section">
        <div class="progress-header">
          <h3>{{ currentPhase }}</h3>
          <span class="progress-percentage">{{ progressPercentage }}%</span>
        </div>

        <div class="progress-bar">
          <div class="progress-fill" :style="{ width: progressPercentage + '%' }"></div>
        </div>

        <div class="progress-details">
          <div class="detail-item">
            <span class="label">当前阶段:</span>
            <span class="value">{{ currentPhase }}</span>
          </div>
          <div class="detail-item">
            <span class="label">已用时间:</span>
            <span class="value">{{ formatDuration(elapsedTime) }}</span>
          </div>
          <div class="detail-item">
            <span class="label">预计剩余:</span>
            <span class="value">{{ formatDuration(estimatedRemaining) }}</span>
          </div>
        </div>

        <div class="log-viewer">
          <h4>实时日志</h4>
          <div class="log-content" ref="logContent">
            <div
              v-for="(log, index) in logs"
              :key="index"
              :class="['log-entry', log.level]"
            >
              <span class="log-time">{{ log.time }}</span>
              <span class="log-message">{{ log.message }}</span>
            </div>
          </div>
        </div>
      </div>

      <div class="actions">
        <button class="btn-danger" @click="cancelReconstruction">
          取消重建
        </button>
      </div>
    </div>

    <!-- 步骤4: 完成 -->
    <div v-if="currentStep === 3" class="step-content">
      <div v-if="reconstructionResult.success" class="result-success">
        <div class="success-icon">✅</div>
        <h2>重建完成！</h2>
        <p>{{ reconstructionResult.message }}</p>

        <div class="result-stats">
          <div class="stat-card">
            <div class="stat-value">{{ reconstructionResult.stats?.image_count || 0 }}</div>
            <div class="stat-label">图像数量</div>
          </div>
          <div class="stat-card">
            <div class="stat-value">{{ formatDuration(reconstructionResult.stats?.duration_seconds || 0) }}</div>
            <div class="stat-label">总耗时</div>
          </div>
          <div class="stat-card">
            <div class="stat-value">{{ formatFileSize(getOutputSize()) }}</div>
            <div class="stat-label">输出大小</div>
          </div>
        </div>

        <div class="result-actions">
          <button class="btn-primary" @click="openViewer">
            🎨 打开3D查看器
          </button>
          <button class="btn-secondary" @click="downloadResult">
            📥 下载结果
          </button>
          <button class="btn-secondary" @click="openFolder">
            📁 打开文件夹
          </button>
        </div>

        <!-- 3D预览 -->
        <div class="preview-container">
          <h3>3D预览</h3>
          <div class="preview-canvas" ref="previewCanvas">
            <iframe
              v-if="reconstructionResult.viewer_url"
              :src="reconstructionResult.viewer_url"
              frameborder="0"
            ></iframe>
          </div>
        </div>
      </div>

      <div v-else class="result-error">
        <div class="error-icon">❌</div>
        <h2>重建失败</h2>
        <p>{{ reconstructionResult.message }}</p>
        <div class="error-details">
          <pre>{{ reconstructionResult.error }}</pre>
        </div>
      </div>

      <div class="actions">
        <button class="btn-secondary" @click="reset">
          重新开始
        </button>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted, nextTick } from 'vue'

// 步骤
const steps = ['选择文件', '配置参数', '重建中', '完成']
const currentStep = ref(0)

// 文件选择
const fileInput = ref<HTMLInputElement | null>(null)
const selectedFile = ref<File | null>(null)
const recentFiles = ref<Array<any>>([])

// 配置
const config = ref({
  imageTopic: '/camera/image_raw',
  cameraInfoTopic: '/camera/camera_info',
  maxImages: null as number | null,
  colmapQuality: 'high',
  colmapMatcher: 'exhaustive',
  iterations: 30000,
  resolution: 1,
  shDegree: 3,
  useGpu: true,
  gpuId: 0,
  downsampleFactor: 1.0
})

// 进度
const currentPhase = ref('准备中...')
const progressPercentage = ref(0)
const elapsedTime = ref(0)
const estimatedRemaining = ref(0)
const logs = ref<Array<any>>([])
const logContent = ref<HTMLElement | null>(null)

// 结果
const reconstructionResult = ref<any>({
  success: false,
  message: '',
  stats: null
})

// 定时器
let progressTimer: number | null = null
let logTimer: number | null = null

// 文件处理
function handleFileSelect(event: Event) {
  const target = event.target as HTMLInputElement
  if (target.files && target.files[0]) {
    selectedFile.value = target.files[0]
  }
}

function handleDrop(event: DragEvent) {
  if (event.dataTransfer?.files && event.dataTransfer.files[0]) {
    selectedFile.value = event.dataTransfer.files[0]
  }
}

function clearFile() {
  selectedFile.value = null
  if (fileInput.value) {
    fileInput.value.value = ''
  }
}

function selectRecentFile(file: any) {
  // 实现选择最近文件的逻辑
  console.log('选择最近文件:', file)
}

// 步骤控制
function nextStep() {
  if (currentStep.value < steps.length - 1) {
    currentStep.value++
  }
}

function prevStep() {
  if (currentStep.value > 0) {
    currentStep.value--
  }
}

// 重建控制
async function startReconstruction() {
  currentStep.value = 2
  progressPercentage.value = 0
  elapsedTime.value = 0
  logs.value = []

  // 模拟重建过程
  simulateReconstruction()
}

function simulateReconstruction() {
  const phases = [
    { name: '提取图像...', duration: 10 },
    { name: 'COLMAP特征提取...', duration: 20 },
    { name: 'COLMAP特征匹配...', duration: 15 },
    { name: 'COLMAP稀疏重建...', duration: 25 },
    { name: '3DGS训练...', duration: 30 }
  ]

  let currentPhaseIndex = 0
  let phaseProgress = 0

  progressTimer = window.setInterval(() => {
    elapsedTime.value++

    if (currentPhaseIndex < phases.length) {
      const phase = phases[currentPhaseIndex]
      currentPhase.value = phase.name

      phaseProgress++
      if (phaseProgress >= phase.duration) {
        currentPhaseIndex++
        phaseProgress = 0

        addLog('info', `${phase.name} 完成`)
      }

      // 计算总进度
      const totalDuration = phases.reduce((sum, p) => sum + p.duration, 0)
      const completedDuration = phases.slice(0, currentPhaseIndex).reduce((sum, p) => sum + p.duration, 0) + phaseProgress
      progressPercentage.value = Math.min(100, Math.round((completedDuration / totalDuration) * 100))

      // 估计剩余时间
      estimatedRemaining.value = totalDuration - completedDuration
    } else {
      // 完成
      if (progressTimer) clearInterval(progressTimer)
      finishReconstruction(true)
    }
  }, 1000)
}

function addLog(level: string, message: string) {
  const now = new Date()
  logs.value.push({
    time: now.toLocaleTimeString(),
    level,
    message
  })

  // 自动滚动到底部
  nextTick(() => {
    if (logContent.value) {
      logContent.value.scrollTop = logContent.value.scrollHeight
    }
  })
}

function cancelReconstruction() {
  if (progressTimer) clearInterval(progressTimer)
  addLog('error', '重建已取消')
  finishReconstruction(false, '用户取消')
}

function finishReconstruction(success: boolean, error?: string) {
  currentStep.value = 3
  reconstructionResult.value = {
    success,
    message: success ? '3D场景重建成功！' : '重建失败',
    error,
    stats: success ? {
      image_count: 150,
      duration_seconds: elapsedTime.value
    } : null,
    viewer_url: success ? 'http://localhost:8080/viewer' : null
  }
}

// 结果操作
function openViewer() {
  if (reconstructionResult.value.viewer_url) {
    window.open(reconstructionResult.value.viewer_url, '_blank')
  }
}

function downloadResult() {
  console.log('下载结果')
}

function openFolder() {
  console.log('打开文件夹')
}

function getOutputSize() {
  return 1024 * 1024 * 500 // 500MB示例
}

function reset() {
  currentStep.value = 0
  selectedFile.value = null
  progressPercentage.value = 0
  logs.value = []
  reconstructionResult.value = { success: false, message: '', stats: null }
}

// 工具函数
function formatFileSize(bytes: number): string {
  if (bytes < 1024) return bytes + ' B'
  if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(2) + ' KB'
  if (bytes < 1024 * 1024 * 1024) return (bytes / (1024 * 1024)).toFixed(2) + ' MB'
  return (bytes / (1024 * 1024 * 1024)).toFixed(2) + ' GB'
}

function formatDuration(seconds: number): string {
  const h = Math.floor(seconds / 3600)
  const m = Math.floor((seconds % 3600) / 60)
  const s = seconds % 60
  if (h > 0) return `${h}h ${m}m ${s}s`
  if (m > 0) return `${m}m ${s}s`
  return `${s}s`
}

function formatDate(date: string): string {
  return new Date(date).toLocaleString()
}

// 生命周期
onMounted(() => {
  // 加载最近文件
  loadRecentFiles()
})

onUnmounted(() => {
  if (progressTimer) clearInterval(progressTimer)
  if (logTimer) clearInterval(logTimer)
})

function loadRecentFiles() {
  // 从localStorage加载
  const stored = localStorage.getItem('recent_rosbags')
  if (stored) {
    recentFiles.value = JSON.parse(stored)
  }
}
</script>

<style scoped>
.reconstruction-panel {
  background: #0a0a12;
  color: #fff;
  padding: 30px;
  border-radius: 12px;
  max-width: 1400px;
  margin: 0 auto;
}

.panel-header {
  text-align: center;
  margin-bottom: 40px;
}

.panel-header h2 {
  font-size: 2em;
  margin-bottom: 10px;
}

.subtitle {
  color: #888;
  font-size: 1.1em;
}

/* 步骤指示器 */
.steps {
  display: flex;
  justify-content: space-between;
  margin-bottom: 40px;
  position: relative;
}

.steps::before {
  content: '';
  position: absolute;
  top: 20px;
  left: 0;
  right: 0;
  height: 2px;
  background: #333;
  z-index: 0;
}

.step {
  flex: 1;
  display: flex;
  flex-direction: column;
  align-items: center;
  position: relative;
  z-index: 1;
}

.step-number {
  width: 40px;
  height: 40px;
  border-radius: 50%;
  background: #333;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: bold;
  margin-bottom: 10px;
  transition: all 0.3s;
}

.step.active .step-number {
  background: #667eea;
  box-shadow: 0 0 20px rgba(102, 126, 234, 0.5);
}

.step.completed .step-number {
  background: #10b981;
}

.step-label {
  font-size: 0.9em;
  color: #888;
}

.step.active .step-label {
  color: #fff;
  font-weight: 600;
}

/* 步骤内容 */
.step-content {
  min-height: 400px;
}

/* 上传区域 */
.upload-area {
  border: 2px dashed #444;
  border-radius: 12px;
  padding: 60px;
  text-align: center;
  transition: all 0.3s;
  cursor: pointer;
}

.upload-area:hover {
  border-color: #667eea;
  background: rgba(102, 126, 234, 0.05);
}

.upload-icon {
  font-size: 4em;
  margin-bottom: 20px;
}

.upload-placeholder h3 {
  margin-bottom: 10px;
}

.file-info {
  display: flex;
  align-items: center;
  gap: 20px;
}

.file-icon {
  font-size: 3em;
}

.file-details {
  flex: 1;
  text-align: left;
}

/* 配置表单 */
.config-section {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 30px;
}

.config-group {
  background: #1a1a24;
  padding: 20px;
  border-radius: 8px;
}

.config-group h3 {
  margin-bottom: 20px;
  color: #667eea;
}

.form-group {
  margin-bottom: 20px;
}

.form-group label {
  display: block;
  margin-bottom: 8px;
  color: #ccc;
}

.form-group input[type="text"],
.form-group input[type="number"],
.form-group select {
  width: 100%;
  padding: 10px;
  background: #0a0a12;
  border: 1px solid #333;
  border-radius: 6px;
  color: #fff;
}

.form-group small {
  display: block;
  margin-top: 5px;
  color: #888;
  font-size: 0.85em;
}

/* 进度显示 */
.progress-section {
  background: #1a1a24;
  padding: 30px;
  border-radius: 12px;
}

.progress-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 20px;
}

.progress-percentage {
  font-size: 2em;
  font-weight: bold;
  color: #667eea;
}

.progress-bar {
  height: 20px;
  background: #0a0a12;
  border-radius: 10px;
  overflow: hidden;
  margin-bottom: 30px;
}

.progress-fill {
  height: 100%;
  background: linear-gradient(90deg, #667eea, #764ba2);
  transition: width 0.5s;
}

.progress-details {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 20px;
  margin-bottom: 30px;
}

.detail-item {
  display: flex;
  flex-direction: column;
  gap: 5px;
}

.detail-item .label {
  color: #888;
  font-size: 0.9em;
}

.detail-item .value {
  font-size: 1.2em;
  font-weight: 600;
}

/* 日志查看器 */
.log-viewer {
  margin-top: 30px;
}

.log-content {
  background: #0a0a12;
  border-radius: 8px;
  padding: 15px;
  max-height: 300px;
  overflow-y: auto;
  font-family: 'Courier New', monospace;
  font-size: 0.9em;
}

.log-entry {
  margin-bottom: 5px;
  display: flex;
  gap: 10px;
}

.log-time {
  color: #888;
}

.log-entry.info .log-message {
  color: #4ec9b0;
}

.log-entry.error .log-message {
  color: #f48771;
}

/* 结果显示 */
.result-success,
.result-error {
  text-align: center;
  padding: 40px;
}

.success-icon,
.error-icon {
  font-size: 5em;
  margin-bottom: 20px;
}

.result-stats {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 20px;
  margin: 40px 0;
}

.stat-card {
  background: #1a1a24;
  padding: 30px;
  border-radius: 12px;
}

.stat-value {
  font-size: 2.5em;
  font-weight: bold;
  color: #667eea;
  margin-bottom: 10px;
}

.stat-label {
  color: #888;
}

.result-actions {
  display: flex;
  gap: 15px;
  justify-content: center;
  margin: 30px 0;
}

.preview-container {
  margin-top: 40px;
}

.preview-canvas {
  width: 100%;
  height: 600px;
  background: #000;
  border-radius: 12px;
  overflow: hidden;
}

.preview-canvas iframe {
  width: 100%;
  height: 100%;
}

/* 按钮 */
.actions {
  display: flex;
  gap: 15px;
  justify-content: center;
  margin-top: 30px;
}

.btn-primary,
.btn-secondary,
.btn-danger {
  padding: 12px 30px;
  border: none;
  border-radius: 8px;
  font-size: 1em;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s;
}

.btn-primary {
  background: linear-gradient(135deg, #667eea, #764ba2);
  color: white;
}

.btn-primary:hover {
  transform: translateY(-2px);
  box-shadow: 0 5px 15px rgba(102, 126, 234, 0.4);
}

.btn-primary:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.btn-large {
  padding: 15px 40px;
  font-size: 1.1em;
}

.btn-secondary {
  background: #333;
  color: white;
}

.btn-secondary:hover {
  background: #444;
}

.btn-danger {
  background: #ef4444;
  color: white;
}

.btn-danger:hover {
  background: #dc2626;
}
</style>
