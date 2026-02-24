<template>
  <div class="database-visualization">
    <!-- 页面头部 -->
    <div class="page-header">
      <h1 class="page-title">
        <span class="icon">🗄️</span>
        Database Visualization
        <span class="subtitle">Database Visualization</span>
      </h1>

      <div class="header-actions">
        <button @click="refreshData" class="btn btn-refresh" :disabled="loading">
          <span class="icon">🔄</span>
          Refresh Data
        </button>
        <button @click="showImportDialog = true" class="btn btn-import">
          <span class="icon">📥</span>
          Import Data
        </button>
        <button @click="exportData" class="btn btn-export">
          <span class="icon">📤</span>
          Export Data
        </button>
        <button @click="clearAllData" class="btn btn-danger">
          <span class="icon">🗑️</span>
          Clear Data
        </button>
      </div>
    </div>

    <!-- 统计卡片 -->
    <div class="stats-grid" v-if="stats">
      <div class="stat-card">
        <div class="stat-icon">🚗</div>
        <div class="stat-content">
          <div class="stat-value">{{ stats.totalVehicles }}</div>
          <div class="stat-label">Vehicle Records</div>
        </div>
      </div>

      <div class="stat-card">
        <div class="stat-icon">🎮</div>
        <div class="stat-content">
          <div class="stat-value">{{ stats.totalTakeovers }}</div>
          <div class="stat-label">Takeover Events</div>
        </div>
      </div>

      <div class="stat-card">
        <div class="stat-icon">🤖</div>
        <div class="stat-content">
          <div class="stat-value">{{ stats.totalAIAnalyses }}</div>
          <div class="stat-label">AI Analysis</div>
        </div>
      </div>

      <div class="stat-card">
        <div class="stat-icon">👥</div>
        <div class="stat-content">
          <div class="stat-value">{{ stats.totalOperators }}</div>
          <div class="stat-label">Operators</div>
        </div>
      </div>
    </div>

    <!-- 数据表格标签页 -->
    <div class="data-tabs">
      <button
        v-for="tab in tabs"
        :key="tab.id"
        @click="activeTab = tab.id"
        :class="['tab-button', { active: activeTab === tab.id }]"
      >
        <span class="tab-icon">{{ tab.icon }}</span>
        {{ tab.label }}
        <span class="tab-count">({{ getTabCount(tab.id) }})</span>
      </button>
    </div>

    <!-- 数据表格内容 -->
    <div class="data-content">
      <div v-if="loading" class="loading-state">
        <div class="spinner"></div>
        <p>Loading data...</p>
      </div>

      <div v-else-if="getCurrentData().length === 0" class="empty-state">
        <div class="empty-icon">📭</div>
        <p>No data available</p>
        <button @click="generateMockData" class="btn btn-primary">
          Generate Mock Data
        </button>
      </div>

      <div v-else class="data-table-container">
        <!-- Vehicle Records表 -->
        <table v-if="activeTab === 'vehicles'" class="data-table">
          <thead>
            <tr>
              <th>Vehicle ID</th>
              <th>Time</th>
              <th>Location</th>
              <th>Risk Score</th>
              <th>Urgency</th>
              <th>Scenario</th>
              <th>Weather</th>
              <th>Speed</th>
              <th>Control Mode</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="vehicle in vehicles" :key="vehicle.id">
              <td><code>{{ vehicle.id }}</code></td>
              <td>{{ formatTime(vehicle.timestamp) }}</td>
              <td>{{ formatLocation(vehicle.location) }}</td>
              <td>
                <span class="risk-badge" :class="getRiskClass(vehicle.riskScore)">
                  {{ vehicle.riskScore.toFixed(1) }}
                </span>
              </td>
              <td>
                <span class="urgency-badge" :class="vehicle.urgencyLevel">
                  {{ getUrgencyText(vehicle.urgencyLevel) }}
                </span>
              </td>
              <td>{{ vehicle.scenario }}</td>
              <td>{{ vehicle.weather }}</td>
              <td>{{ vehicle.speed.toFixed(1) }} km/h</td>
              <td>{{ vehicle.controlMode }}</td>
            </tr>
          </tbody>
        </table>

        <!-- Takeover Events表 -->
        <table v-if="activeTab === 'takeovers'" class="data-table">
          <thead>
            <tr>
              <th>Event ID</th>
              <th>Vehicle ID</th>
              <th>OperatorsID</th>
              <th>Time</th>
              <th>Reason</th>
              <th>Risk Score</th>
              <th>持续Time</th>
              <th>Result</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="takeover in takeovers" :key="takeover.id">
              <td><code>{{ takeover.id }}</code></td>
              <td><code>{{ takeover.vehicleId }}</code></td>
              <td><code>{{ takeover.operatorId }}</code></td>
              <td>{{ formatTime(takeover.timestamp) }}</td>
              <td>{{ takeover.reason }}</td>
              <td>
                <span class="risk-badge" :class="getRiskClass(takeover.riskScore)">
                  {{ takeover.riskScore.toFixed(1) }}
                </span>
              </td>
              <td>{{ takeover.duration }}s</td>
              <td>
                <span class="outcome-badge" :class="takeover.outcome">
                  {{ getOutcomeText(takeover.outcome) }}
                </span>
              </td>
            </tr>
          </tbody>
        </table>

        <!-- AI Analysis表 -->
        <table v-if="activeTab === 'ai-analyses'" class="data-table">
          <thead>
            <tr>
              <th>Analysis ID</th>
              <th>Vehicle ID</th>
              <th>Time</th>
              <th>Incident Description</th>
              <th>Risk Factors</th>
              <th>Recommendations</th>
              <th>Confidence</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="analysis in aiAnalyses" :key="analysis.id">
              <td><code>{{ analysis.id }}</code></td>
              <td><code>{{ analysis.vehicleId }}</code></td>
              <td>{{ formatTime(analysis.timestamp) }}</td>
              <td class="text-cell">{{ analysis.incidentDescription }}</td>
              <td>{{ analysis.riskFactors.length }} items</td>
              <td>{{ analysis.recommendations.length }} items</td>
              <td>
                <span class="confidence-badge" :class="getConfidenceClass(analysis.confidence)">
                  {{ (analysis.confidence * 100).toFixed(0) }}%
                </span>
              </td>
            </tr>
          </tbody>
        </table>

        <!-- Operators表 -->
        <table v-if="activeTab === 'operators'" class="data-table">
          <thead>
            <tr>
              <th>OperatorsID</th>
              <th>Name</th>
              <th>Status</th>
              <th>Assigned Vehicles</th>
              <th>Total Takeovers</th>
              <th>Success Rate</th>
              <th>平均响应Time</th>
            </tr>
          </thead>
          <tbody>
            <tr v-for="operator in operators" :key="operator.id">
              <td><code>{{ operator.id }}</code></td>
              <td>{{ operator.name }}</td>
              <td>
                <span class="status-badge" :class="operator.status">
                  {{ getStatusText(operator.status) }}
                </span>
              </td>
              <td>{{ operator.assignedVehicles.length }}</td>
              <td>{{ operator.totalTakeovers }}</td>
              <td>{{ (operator.successRate * 100).toFixed(1) }}%</td>
              <td>{{ operator.averageResponseTime.toFixed(1) }}s</td>
            </tr>
          </tbody>
        </table>

        <!-- 3DGS Reconstruction面板 -->
        <div v-if="activeTab === '3dgs'" class="reconstruction-container">
          <ReconstructionPanel />
        </div>
      </div>
    </div>

    <!-- 导入对话框 -->
    <Teleport to="body">
      <div v-if="showImportDialog" class="dialog-overlay" @click="showImportDialog = false">
        <div class="dialog-container" @click.stop>
          <div class="dialog-header">
            <h2>Import Data</h2>
            <button @click="showImportDialog = false" class="close-btn">✕</button>
          </div>

          <div class="dialog-body">
            <div class="file-upload-area">
              <input
                type="file"
                ref="fileInput"
                accept=".json"
                @change="handleFileSelect"
                style="display: none"
              />
              <button @click="fileInput?.click()" class="btn btn-primary btn-large">
                <span class="icon">📁</span>
                Select JSON File
              </button>
              <p class="hint">Supports .json format data files</p>
            </div>

            <div v-if="importFile" class="file-info">
              <p><strong>Selected file:</strong> {{ importFile.name }}</p>
              <p><strong>File size:</strong> {{ formatFileSize(importFile.size) }}</p>
            </div>
          </div>

          <div class="dialog-footer">
            <button @click="showImportDialog = false" class="btn btn-secondary">
              Cancel
            </button>
            <button
              @click="confirmImport"
              :disabled="!importFile || importing"
              class="btn btn-primary"
            >
              {{ importing ? 'Importing...' : 'Confirm Import' }}
            </button>
          </div>
        </div>
      </div>
    </Teleport>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted } from 'vue'
import ReconstructionPanel from './ReconstructionPanel.vue'
import databaseService, {
  type VehicleRecord,
  type TakeoverEvent,
  type AIAnalysisRecord,
  type OperatorRecord,
  type DatabaseStats
} from '@/services/databaseService'

// ======================== Status管理 ========================

const loading = ref(false)
const activeTab = ref('vehicles')
const showImportDialog = ref(false)
const importFile = ref<File | null>(null)
const importing = ref(false)
const fileInput = ref<HTMLInputElement | null>(null)

const vehicles = ref<VehicleRecord[]>([])
const takeovers = ref<TakeoverEvent[]>([])
const aiAnalyses = ref<AIAnalysisRecord[]>([])
const operators = ref<OperatorRecord[]>([])
const stats = ref<DatabaseStats | null>(null)

// ======================== 标签页配置 ========================

const tabs = [
  { id: 'vehicles', label: 'Vehicle Records', icon: '🚗' },
  { id: 'takeovers', label: 'Takeover Events', icon: '🎮' },
  { id: 'ai-analyses', label: 'AI Analysis', icon: '🤖' },
  { id: 'operators', label: 'Operators', icon: '👥' },
  { id: '3dgs', label: '3D Reconstruction', icon: '🎨' }
]

// ======================== 计算属性 ========================

const getCurrentData = () => {
  switch (activeTab.value) {
    case 'vehicles': return vehicles.value
    case 'takeovers': return takeovers.value
    case 'ai-analyses': return aiAnalyses.value
    case 'operators': return operators.value
    default: return []
  }
}

const getTabCount = (tabId: string) => {
  switch (tabId) {
    case 'vehicles': return vehicles.value.length
    case 'takeovers': return takeovers.value.length
    case 'ai-analyses': return aiAnalyses.value.length
    case 'operators': return operators.value.length
    case '3dgs': return 0  // 3DGS不显示计数
    default: return 0
  }
}

// 继续在下一个文件Medium...
// 接上面的内容...

// ======================== 方法 ========================

/**
 * Refresh Data
 */
async function refreshData() {
  loading.value = true
  try {
    await databaseService.initialize()

    const [vehiclesData, takeoversData, analysesData, operatorsData, statsData] =
      await Promise.all([
        databaseService.getAllVehicles(),
        databaseService.getAllTakeovers(),
        databaseService.getAllAIAnalyses(),
        databaseService.getAllOperators(),
        databaseService.getStats()
      ])

    vehicles.value = vehiclesData.sort((a, b) => b.timestamp - a.timestamp)
    takeovers.value = takeoversData.sort((a, b) => b.timestamp - a.timestamp)
    aiAnalyses.value = analysesData.sort((a, b) => b.timestamp - a.timestamp)
    operators.value = operatorsData
    stats.value = statsData

    console.log('[DB Viz] Data refreshed')
  } catch (error) {
    console.error('[DB Viz] Failed to refresh data:', error)
    alert('Refresh DataFailed: ' + error)
  } finally {
    loading.value = false
  }
}

/**
 * Export Data
 */
async function exportData() {
  try {
    const jsonData = await databaseService.exportToJSON()
    const blob = new Blob([jsonData], { type: 'application/json' })
    const url = URL.createObjectURL(blob)

    const link = document.createElement('a')
    link.href = url
    link.download = `fsm-pilot-data-${Date.now()}.json`
    link.click()

    URL.revokeObjectURL(url)
    console.log('[DB Viz] Data exported')
  } catch (error) {
    console.error('[DB Viz] Export failed:', error)
    alert('导出Failed: ' + error)
  }
}

/**
 * 处理文件选择
 */
function handleFileSelect(event: Event) {
  const target = event.target as HTMLInputElement
  if (target.files && target.files.length > 0) {
    importFile.value = target.files[0]
  }
}

/**
 * Confirm Import
 */
async function confirmImport() {
  if (!importFile.value) return

  importing.value = true
  try {
    const reader = new FileReader()

    reader.onload = async (e) => {
      try {
        const jsonString = e.target?.result as string
        await databaseService.importFromJSON(jsonString)

        showImportDialog.value = false
        importFile.value = null

        await refreshData()
        alert('数据导入Success！')
      } catch (error) {
        console.error('[DB Viz] Import failed:', error)
        alert('导入Failed: ' + error)
      } finally {
        importing.value = false
      }
    }

    reader.readAsText(importFile.value)
  } catch (error) {
    console.error('[DB Viz] Import failed:', error)
    alert('导入Failed: ' + error)
    importing.value = false
  }
}

/**
 * 清空所有数据
 */
async function clearAllData() {
  const confirmed = confirm('确定要清空所有数据吗？此操作不可恢复！')
  if (!confirmed) return

  try {
    await databaseService.clearAll()
    await refreshData()
    alert('数据已清空')
  } catch (error) {
    console.error('[DB Viz] Clear failed:', error)
    alert('清空Failed: ' + error)
  }
}

/**
 * Generate Mock Data
 */
async function generateMockData() {
  loading.value = true
  try {
    // 生成10辆模拟车辆
    for (let i = 0; i < 10; i++) {
      const vehicle: VehicleRecord = {
        id: `V-${String(i + 1).padStart(3, '0')}`,
        timestamp: Date.now() - Math.random() * 3600000,
        location: {
          longitude: 114.1 + Math.random() * 0.1,
          latitude: 22.2 + Math.random() * 0.1,
          street: `Test Street ${i + 1}`,
          district: 'Demo District'
        },
        riskScore: Math.random() * 100,
        urgencyLevel: ['low', 'medium', 'high', 'critical'][Math.floor(Math.random() * 4)] as any,
        scenario: ['highway', 'urban', 'residential'][Math.floor(Math.random() * 3)],
        weather: ['clear', 'rainy', 'foggy'][Math.floor(Math.random() * 3)],
        speed: 30 + Math.random() * 70,
        controlMode: ['autonomous', 'assisted', 'manual'][Math.floor(Math.random() * 3)]
      }
      await databaseService.saveVehicle(vehicle)
    }

    // 生成5个Takeover Events
    for (let i = 0; i < 5; i++) {
      const takeover: TakeoverEvent = {
        id: `TO-${String(i + 1).padStart(3, '0')}`,
        vehicleId: `V-${String(Math.floor(Math.random() * 10) + 1).padStart(3, '0')}`,
        operatorId: `OP-${String(Math.floor(Math.random() * 3) + 1).padStart(3, '0')}`,
        timestamp: Date.now() - Math.random() * 1800000,
        reason: '检测到High风险情况',
        riskScore: 60 + Math.random() * 40,
        duration: Math.floor(30 + Math.random() * 120),
        outcome: ['success', 'failed', 'cancelled'][Math.floor(Math.random() * 3)] as any
      }
      await databaseService.saveTakeoverEvent(takeover)
    }

    // 生成3个AI Analysis
    for (let i = 0; i < 3; i++) {
      const analysis: AIAnalysisRecord = {
        id: `AI-${String(i + 1).padStart(3, '0')}`,
        vehicleId: `V-${String(Math.floor(Math.random() * 10) + 1).padStart(3, '0')}`,
        timestamp: Date.now() - Math.random() * 1800000,
        incidentDescription: '车辆检测到前方障碍物',
        riskFactors: [
          { factor: 'speed', severity: 'high', value: 85 },
          { factor: 'distance', severity: 'critical', value: 45 }
        ],
        recommendations: ['降Low车速', '准备接管'],
        confidence: 0.8 + Math.random() * 0.2
      }
      await databaseService.saveAIAnalysis(analysis)
    }

    // 生成3个Operators
    for (let i = 0; i < 3; i++) {
      const operator: OperatorRecord = {
        id: `OP-${String(i + 1).padStart(3, '0')}`,
        name: `Operators ${i + 1}`,
        status: ['idle', 'busy', 'offline'][Math.floor(Math.random() * 3)] as any,
        assignedVehicles: [],
        totalTakeovers: Math.floor(Math.random() * 50),
        successRate: 0.85 + Math.random() * 0.15,
        averageResponseTime: 2 + Math.random() * 3
      }
      await databaseService.saveOperator(operator)
    }

    await refreshData()
    alert('模拟数据生成Success！')
  } catch (error) {
    console.error('[DB Viz] Mock data generation failed:', error)
    alert('生成Failed: ' + error)
  } finally {
    loading.value = false
  }
}

// ======================== 格式化函数 ========================

function formatTime(timestamp: number): string {
  return new Date(timestamp).toLocaleString('zh-CN')
}

function formatLocation(location: any): string {
  return `${location.latitude.toFixed(4)}, ${location.longitude.toFixed(4)}`
}

function formatFileSize(bytes: number): string {
  if (bytes < 1024) return bytes + ' B'
  if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(2) + ' KB'
  return (bytes / (1024 * 1024)).toFixed(2) + ' MB'
}

function getRiskClass(score: number): string {
  if (score >= 80) return 'critical'
  if (score >= 60) return 'high'
  if (score >= 40) return 'medium'
  return 'low'
}

function getConfidenceClass(confidence: number): string {
  if (confidence >= 0.9) return 'high'
  if (confidence >= 0.7) return 'medium'
  return 'low'
}

function getUrgencyText(level: string): string {
  const map: Record<string, string> = {
    critical: 'Critical',
    high: 'High',
    medium: 'Medium',
    low: 'Low'
  }
  return map[level] || level
}

function getOutcomeText(outcome: string): string {
  const map: Record<string, string> = {
    success: 'Success',
    failed: 'Failed',
    cancelled: 'Cancel'
  }
  return map[outcome] || outcome
}

function getStatusText(status: string): string {
  const map: Record<string, string> = {
    idle: 'Idle',
    busy: 'Busy',
    offline: 'Offline'
  }
  return map[status] || status
}

// ======================== 生命周期 ========================

onMounted(() => {
  refreshData()
})

</script>

<style scoped>
.database-visualization {
  padding: 20px;
  background: linear-gradient(135deg, #1a1d29 0%, #2d1b3d 100%);
  min-height: 100vh;
  color: #e0e0e0;
}

/* 页面头部 */
.page-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 30px;
  padding: 20px;
  background: rgba(255, 255, 255, 0.05);
  border-radius: 12px;
  backdrop-filter: blur(10px);
}

.page-title {
  display: flex;
  align-items: center;
  gap: 15px;
  font-size: 28px;
  font-weight: 700;
  color: #00d9ff;
  margin: 0;
}

.page-title .icon {
  font-size: 36px;
}

.page-title .subtitle {
  font-size: 14px;
  color: #888;
  font-weight: 400;
  margin-left: 10px;
}

.header-actions {
  display: flex;
  gap: 10px;
}

/* 按钮样式 */
.btn {
  padding: 10px 20px;
  border: none;
  border-radius: 8px;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
  display: flex;
  align-items: center;
  gap: 8px;
}

.btn .icon {
  font-size: 18px;
}

.btn-refresh {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
}

.btn-import {
  background: linear-gradient(135deg, #f093fb 0%, #f5576c 100%);
  color: white;
}

.btn-export {
  background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%);
  color: white;
}

.btn-danger {
  background: linear-gradient(135deg, #fa709a 0%, #fee140 100%);
  color: white;
}

.btn-primary {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
}

.btn-secondary {
  background: #555;
  color: white;
}

.btn:hover:not(:disabled) {
  transform: translateY(-2px);
  box-shadow: 0 5px 15px rgba(0, 217, 255, 0.3);
}

.btn:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

/* 统计卡片 */
.stats-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
  gap: 20px;
  margin-bottom: 30px;
}

.stat-card {
  background: rgba(255, 255, 255, 0.05);
  border-radius: 12px;
  padding: 20px;
  display: flex;
  align-items: center;
  gap: 15px;
  backdrop-filter: blur(10px);
  border: 1px solid rgba(255, 255, 255, 0.1);
  transition: all 0.3s ease;
}

.stat-card:hover {
  transform: translateY(-5px);
  box-shadow: 0 8px 20px rgba(0, 217, 255, 0.2);
}

.stat-icon {
  font-size: 40px;
  opacity: 0.8;
}

.stat-content {
  flex: 1;
}

.stat-value {
  font-size: 32px;
  font-weight: 700;
  color: #00d9ff;
  line-height: 1;
  margin-bottom: 5px;
}

.stat-label {
  font-size: 14px;
  color: #888;
}

/* 标签页 */
.data-tabs {
  display: flex;
  gap: 10px;
  margin-bottom: 20px;
  background: rgba(255, 255, 255, 0.05);
  padding: 10px;
  border-radius: 12px;
}

.tab-button {
  flex: 1;
  padding: 12px 20px;
  background: transparent;
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  color: #888;
  font-size: 14px;
  font-weight: 600;
  cursor: pointer;
  transition: all 0.3s ease;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
}

.tab-button .tab-icon {
  font-size: 18px;
}

.tab-button.active {
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white;
  border-color: transparent;
}

.tab-button:hover:not(.active) {
  background: rgba(255, 255, 255, 0.1);
  color: #00d9ff;
}

.tab-count {
  font-size: 12px;
  opacity: 0.7;
}

/* 数据内容 */
.data-content {
  background: rgba(255, 255, 255, 0.05);
  border-radius: 12px;
  padding: 20px;
  min-height: 400px;
}

/* 加载和空Status */
.loading-state,
.empty-state {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  min-height: 400px;
  gap: 20px;
}

.spinner {
  width: 50px;
  height: 50px;
  border: 4px solid rgba(255, 255, 255, 0.1);
  border-top-color: #00d9ff;
  border-radius: 50%;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

.empty-icon {
  font-size: 64px;
  opacity: 0.5;
}

/* 数据表格 */
.data-table-container {
  overflow-x: auto;
}

.data-table {
  width: 100%;
  border-collapse: collapse;
  font-size: 14px;
}

.data-table thead {
  background: rgba(0, 217, 255, 0.1);
}

.data-table th {
  padding: 12px;
  text-align: left;
  font-weight: 600;
  color: #00d9ff;
  border-bottom: 2px solid rgba(0, 217, 255, 0.3);
}

.data-table td {
  padding: 12px;
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.data-table tbody tr {
  transition: background 0.2s ease;
}

.data-table tbody tr:hover {
  background: rgba(255, 255, 255, 0.05);
}

.data-table code {
  background: rgba(0, 217, 255, 0.1);
  padding: 2px 6px;
  border-radius: 4px;
  font-family: 'Courier New', monospace;
  font-size: 12px;
  color: #00d9ff;
}

.text-cell {
  max-width: 300px;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

/* 徽章样式 */
.risk-badge,
.urgency-badge,
.outcome-badge,
.status-badge,
.confidence-badge {
  padding: 4px 12px;
  border-radius: 12px;
  font-size: 12px;
  font-weight: 600;
  display: inline-block;
}

.risk-badge.critical,
.urgency-badge.critical {
  background: rgba(255, 59, 48, 0.2);
  color: #ff3b30;
  border: 1px solid rgba(255, 59, 48, 0.5);
}

.risk-badge.high,
.urgency-badge.high,
.confidence-badge.high {
  background: rgba(255, 149, 0, 0.2);
  color: #ff9500;
  border: 1px solid rgba(255, 149, 0, 0.5);
}

.risk-badge.medium,
.urgency-badge.medium,
.confidence-badge.medium {
  background: rgba(255, 204, 0, 0.2);
  color: #ffcc00;
  border: 1px solid rgba(255, 204, 0, 0.5);
}

.risk-badge.low,
.urgency-badge.low {
  background: rgba(52, 199, 89, 0.2);
  color: #34c759;
  border: 1px solid rgba(52, 199, 89, 0.5);
}

.outcome-badge.success {
  background: rgba(52, 199, 89, 0.2);
  color: #34c759;
  border: 1px solid rgba(52, 199, 89, 0.5);
}

.outcome-badge.failed {
  background: rgba(255, 59, 48, 0.2);
  color: #ff3b30;
  border: 1px solid rgba(255, 59, 48, 0.5);
}

.outcome-badge.cancelled {
  background: rgba(142, 142, 147, 0.2);
  color: #8e8e93;
  border: 1px solid rgba(142, 142, 147, 0.5);
}

.status-badge.idle {
  background: rgba(52, 199, 89, 0.2);
  color: #34c759;
  border: 1px solid rgba(52, 199, 89, 0.5);
}

.status-badge.busy {
  background: rgba(255, 149, 0, 0.2);
  color: #ff9500;
  border: 1px solid rgba(255, 149, 0, 0.5);
}

.status-badge.offline {
  background: rgba(142, 142, 147, 0.2);
  color: #8e8e93;
  border: 1px solid rgba(142, 142, 147, 0.5);
}

/* 对话框 */
.dialog-overlay {
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
  backdrop-filter: blur(5px);
}

.dialog-container {
  background: #1a1d29;
  border-radius: 16px;
  max-width: 500px;
  width: 90%;
  max-height: 80vh;
  overflow: auto;
  box-shadow: 0 20px 60px rgba(0, 0, 0, 0.5);
  border: 1px solid rgba(255, 255, 255, 0.1);
}

.dialog-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 20px;
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.dialog-header h2 {
  margin: 0;
  font-size: 20px;
  color: #00d9ff;
}

.close-btn {
  background: none;
  border: none;
  font-size: 24px;
  color: #888;
  cursor: pointer;
  transition: color 0.3s ease;
}

.close-btn:hover {
  color: #fff;
}

.dialog-body {
  padding: 20px;
}

.dialog-footer {
  padding: 20px;
  border-top: 1px solid rgba(255, 255, 255, 0.1);
  display: flex;
  justify-content: flex-end;
  gap: 10px;
}

.file-upload-area {
  text-align: center;
  padding: 40px 20px;
  border: 2px dashed rgba(0, 217, 255, 0.3);
  border-radius: 12px;
  background: rgba(0, 217, 255, 0.05);
}

.btn-large {
  padding: 15px 30px;
  font-size: 16px;
}

.hint {
  margin-top: 10px;
  color: #888;
  font-size: 14px;
}

.file-info {
  margin-top: 20px;
  padding: 15px;
  background: rgba(0, 217, 255, 0.05);
  border-radius: 8px;
  font-size: 14px;
}

.file-info p {
  margin: 5px 0;
}
</style>
