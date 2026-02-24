<!--
  FSM-Pilot V2.0 - Dispatch Dashboard

  @description 车队调度管理面板 - 100+ 车辆调度监控
-->
<template>
  <div class="dispatch-dashboard">
    <NavBar />

    <!-- 顶部统计卡片 -->
    <div class="stats-cards">
      <div class="stat-card">
        <div class="stat-icon">🚗</div>
        <div class="stat-content">
          <div class="stat-label">Total Vehicles</div>
          <div class="stat-value">{{ stats.totalVehicles }}</div>
          <div class="stat-sub">{{ stats.onlineVehicles }} online</div>
        </div>
      </div>

      <div class="stat-card">
        <div class="stat-icon">🔗</div>
        <div class="stat-content">
          <div class="stat-label">Connected</div>
          <div class="stat-value">{{ stats.connectedVehicles }}</div>
          <div class="stat-sub">{{ stats.operatorUtilization.toFixed(1) }}% utilization</div>
        </div>
      </div>

      <div class="stat-card">
        <div class="stat-icon">⏳</div>
        <div class="stat-content">
          <div class="stat-label">Queue</div>
          <div class="stat-value">{{ stats.queueLength }}</div>
          <div class="stat-sub">{{ formatWaitTime(stats.averageWaitTime) }} avg wait</div>
        </div>
      </div>

      <div class="stat-card">
        <div class="stat-icon">✅</div>
        <div class="stat-content">
          <div class="stat-label">Success Rate</div>
          <div class="stat-value">{{ stats.successRate.toFixed(1) }}%</div>
          <div class="stat-sub">{{ stats.totalHandovers }} handovers</div>
        </div>
      </div>
    </div>

    <!-- 主内容区域 -->
    <div class="main-content">
      <!-- 左侧：请求队列 -->
      <div class="queue-panel">
        <div class="panel-header">
          <h3>Request Queue</h3>
          <span class="badge">{{ sortedQueue.length }}</span>
        </div>

        <div class="queue-list">
          <div
            v-for="vehicle in sortedQueue"
            :key="vehicle.id"
            class="queue-item"
            :class="`priority-${vehicle.priority}`"
          >
            <div class="item-header">
              <span class="vehicle-name">{{ vehicle.name }}</span>
              <span class="priority-badge">{{ getPriorityName(vehicle.priority) }}</span>
            </div>

            <div class="item-details">
              <div class="detail-row">
                <span class="label">Event:</span>
                <span class="value">{{ vehicle.event || 'None' }}</span>
              </div>
              <div class="detail-row">
                <span class="label">Location:</span>
                <span class="value">{{ vehicle.location.city }}</span>
              </div>
              <div class="detail-row">
                <span class="label">Latency:</span>
                <span class="value" :class="getLatencyClass(vehicle.network.latency)">
                  {{ vehicle.network.latency.toFixed(0) }} ms
                </span>
              </div>
              <div class="detail-row">
                <span class="label">Wait:</span>
                <span class="value">{{ formatWaitTime(Date.now() - vehicle.requestTime) }}</span>
              </div>
            </div>

            <button
              class="assign-btn"
              @click="manualAssign(vehicle.id)"
              :disabled="availableOperators.length === 0"
            >
              Assign Now
            </button>
          </div>

          <div v-if="sortedQueue.length === 0" class="empty-state">
            <div class="empty-icon">✨</div>
            <div class="empty-text">No vehicles in queue</div>
          </div>
        </div>
      </div>

      <!-- 右侧：运营商状态 -->
      <div class="operators-panel">
        <div class="panel-header">
          <h3>Operators</h3>
          <span class="badge">{{ Array.from(operators.values()).length }}</span>
        </div>

        <div class="operators-list">
          <div
            v-for="operator in Array.from(operators.values())"
            :key="operator.id"
            class="operator-card"
            :class="{ full: operator.currentLoad >= operator.capacity }"
          >
            <div class="operator-header">
              <span class="operator-name">{{ operator.name }}</span>
              <span class="load-badge">
                {{ operator.currentLoad }} / {{ operator.capacity }}
              </span>
            </div>

            <div class="operator-info">
              <div class="info-row">
                <span class="label">Location:</span>
                <span class="value">{{ operator.location.city }}</span>
              </div>
              <div class="info-row">
                <span class="label">Avg Latency:</span>
                <span class="value">{{ operator.averageLatency.toFixed(0) }} ms</span>
              </div>
            </div>

            <div class="load-bar">
              <div
                class="load-fill"
                :style="{ width: `${(operator.currentLoad / operator.capacity) * 100}%` }"
              ></div>
            </div>

            <div class="active-vehicles" v-if="operator.activeVehicles.length > 0">
              <div class="vehicles-label">Active Vehicles:</div>
              <div class="vehicles-chips">
                <span
                  v-for="vehicleId in operator.activeVehicles"
                  :key="vehicleId"
                  class="vehicle-chip"
                  @click="releaseVehicle(vehicleId)"
                >
                  {{ vehicles.get(vehicleId)?.name || vehicleId }}
                  <span class="release-icon">×</span>
                </span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>

    <!-- 底部：模拟控制 -->
    <div class="simulation-panel">
      <div class="panel-header">
        <h3>Simulation Controls</h3>
      </div>

      <div class="controls-grid">
        <button class="control-btn" @click="simulateEmergency">
          🚨 Simulate Emergency
        </button>
        <button class="control-btn" @click="simulateHighPriority">
          ⚠️ Simulate High Priority
        </button>
        <button class="control-btn" @click="simulateRoutine">
          📋 Simulate Routine
        </button>
        <button class="control-btn reset" @click="resetSimulation">
          🔄 Reset All
        </button>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { onMounted } from 'vue'
import NavBar from './NavBar.vue'
import {
  useVehicleDispatchService,
  Priority,
  EventType,
  type Vehicle
} from '@/services/dispatchService'

const dispatch = useVehicleDispatchService()

const {
  vehicles,
  operators,
  stats,
  sortedQueue,
  availableOperators,
  requestHandover,
  releaseHandover,
  generateMockFleet,
  generateMockOperators
} = dispatch

onMounted(() => {
  // 生成模拟数据
  generateMockFleet(100)
  generateMockOperators(5)
})

function getPriorityName(priority: Priority): string {
  const names: Record<Priority, string> = {
    [Priority.EMERGENCY]: 'Emergency',
    [Priority.HIGH]: 'High',
    [Priority.MEDIUM]: 'Medium',
    [Priority.LOW]: 'Low',
    [Priority.ROUTINE]: 'Routine'
  }
  return names[priority]
}

function getLatencyClass(latency: number): string {
  if (latency < 50) return 'excellent'
  if (latency < 100) return 'good'
  if (latency < 200) return 'acceptable'
  return 'poor'
}

function formatWaitTime(ms: number): string {
  const seconds = Math.floor(ms / 1000)
  if (seconds < 60) return `${seconds}s`
  const minutes = Math.floor(seconds / 60)
  return `${minutes}m ${seconds % 60}s`
}

function manualAssign(vehicleId: string) {
  // 手动处理已在队列中的车辆
  console.log(`[Dashboard] Manually assigning ${vehicleId}`)
}

function releaseVehicle(vehicleId: string) {
  releaseHandover(vehicleId)
}

function simulateEmergency() {
  const vehicle = getRandomOnlineVehicle()
  if (vehicle) {
    requestHandover(vehicle.id, Priority.EMERGENCY, EventType.COLLISION_WARNING)
  }
}

function simulateHighPriority() {
  const vehicle = getRandomOnlineVehicle()
  if (vehicle) {
    requestHandover(vehicle.id, Priority.HIGH, EventType.TRAFFIC_VIOLATION)
  }
}

function simulateRoutine() {
  const vehicle = getRandomOnlineVehicle()
  if (vehicle) {
    requestHandover(vehicle.id, Priority.ROUTINE, EventType.SCHEDULED_HANDOVER)
  }
}

function getRandomOnlineVehicle(): Vehicle | undefined {
  const online = Array.from(vehicles.value.values()).filter(
    v => v.status === 'online'
  )
  return online[Math.floor(Math.random() * online.length)]
}

function resetSimulation() {
  location.reload()
}
</script>

<style scoped>
.dispatch-dashboard {
  min-height: 100vh;
  background: #0a0a12;
  padding: 24px;
}

/* Stats Cards */
.stats-cards {
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 16px;
  margin-bottom: 24px;
}

.stat-card {
  background: #14141f;
  border: 1px solid #2a2a3a;
  border-radius: 8px;
  padding: 20px;
  display: flex;
  gap: 16px;
  align-items: center;
}

.stat-icon {
  font-size: 32px;
}

.stat-content {
  flex: 1;
}

.stat-label {
  font-size: 11px;
  color: #888;
  text-transform: uppercase;
  margin-bottom: 4px;
}

.stat-value {
  font-size: 28px;
  font-weight: 700;
  color: var(--primary, #ff5722);
  font-family: 'JetBrains Mono', monospace;
}

.stat-sub {
  font-size: 11px;
  color: #aaa;
  margin-top: 4px;
}

/* Main Content */
.main-content {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 24px;
  margin-bottom: 24px;
}

.panel-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px;
  background: #1a1a2e;
  border-bottom: 1px solid #2a2a3a;
}

.panel-header h3 {
  font-size: 14px;
  font-weight: 600;
  color: #fff;
  margin: 0;
}

.badge {
  background: var(--primary, #ff5722);
  color: #000;
  padding: 4px 10px;
  border-radius: 12px;
  font-size: 11px;
  font-weight: 700;
}

/* Queue Panel */
.queue-panel {
  background: #14141f;
  border: 1px solid #2a2a3a;
  border-radius: 8px;
  overflow: hidden;
}

.queue-list {
  max-height: 600px;
  overflow-y: auto;
  padding: 12px;
}

.queue-item {
  background: rgba(255, 255, 255, 0.02);
  border: 1px solid #2a2a3a;
  border-left: 4px solid #888;
  border-radius: 6px;
  padding: 16px;
  margin-bottom: 12px;
}

.queue-item.priority-0 {
  border-left-color: #ff4444;
  background: rgba(255, 68, 68, 0.05);
}

.queue-item.priority-1 {
  border-left-color: #ff9944;
}

.queue-item.priority-2 {
  border-left-color: #ffcc44;
}

.item-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 12px;
}

.vehicle-name {
  font-size: 14px;
  font-weight: 600;
  color: #fff;
}

.priority-badge {
  font-size: 10px;
  padding: 4px 8px;
  border-radius: 4px;
  background: #2a2a3a;
  color: #aaa;
  text-transform: uppercase;
}

.item-details {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 8px;
  margin-bottom: 12px;
}

.detail-row {
  display: flex;
  justify-content: space-between;
  font-size: 11px;
}

.detail-row .label {
  color: #888;
}

.detail-row .value {
  color: #fff;
  font-family: 'JetBrains Mono', monospace;
}

.detail-row .value.excellent {
  color: #44ff44;
}

.detail-row .value.good {
  color: #88ff88;
}

.detail-row .value.acceptable {
  color: #ffaa44;
}

.detail-row .value.poor {
  color: #ff4444;
}

.assign-btn {
  width: 100%;
  padding: 8px;
  background: var(--primary, #ff5722);
  border: none;
  color: #000;
  font-size: 11px;
  font-weight: 600;
  border-radius: 4px;
  cursor: pointer;
  transition: all 0.2s;
}

.assign-btn:hover:not(:disabled) {
  background: #00d4e0;
}

.assign-btn:disabled {
  opacity: 0.3;
  cursor: not-allowed;
}

.empty-state {
  text-align: center;
  padding: 60px 20px;
  color: #666;
}

.empty-icon {
  font-size: 48px;
  margin-bottom: 12px;
}

.empty-text {
  font-size: 14px;
}

/* Operators Panel */
.operators-panel {
  background: #14141f;
  border: 1px solid #2a2a3a;
  border-radius: 8px;
  overflow: hidden;
}

.operators-list {
  max-height: 600px;
  overflow-y: auto;
  padding: 12px;
}

.operator-card {
  background: rgba(255, 255, 255, 0.02);
  border: 1px solid #2a2a3a;
  border-radius: 6px;
  padding: 16px;
  margin-bottom: 12px;
}

.operator-card.full {
  opacity: 0.6;
}

.operator-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 12px;
}

.operator-name {
  font-size: 14px;
  font-weight: 600;
  color: #fff;
}

.load-badge {
  font-size: 11px;
  padding: 4px 8px;
  border-radius: 4px;
  background: #2a2a3a;
  color: #aaa;
  font-family: 'JetBrains Mono', monospace;
}

.operator-info {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 8px;
  margin-bottom: 12px;
}

.info-row {
  display: flex;
  justify-content: space-between;
  font-size: 11px;
}

.info-row .label {
  color: #888;
}

.info-row .value {
  color: #fff;
  font-family: 'JetBrains Mono', monospace;
}

.load-bar {
  height: 8px;
  background: rgba(255, 255, 255, 0.05);
  border-radius: 4px;
  overflow: hidden;
  margin-bottom: 12px;
}

.load-fill {
  height: 100%;
  background: linear-gradient(90deg, var(--primary, #ff5722), #00d4e0);
  transition: width 0.3s;
}

.active-vehicles {
  margin-top: 12px;
  padding-top: 12px;
  border-top: 1px solid #2a2a3a;
}

.vehicles-label {
  font-size: 10px;
  color: #888;
  margin-bottom: 8px;
  text-transform: uppercase;
}

.vehicles-chips {
  display: flex;
  flex-wrap: wrap;
  gap: 6px;
}

.vehicle-chip {
  font-size: 10px;
  padding: 4px 8px;
  background: #2a2a3a;
  border-radius: 12px;
  color: #aaa;
  cursor: pointer;
  transition: all 0.2s;
  display: flex;
  align-items: center;
  gap: 4px;
}

.vehicle-chip:hover {
  background: #ff4444;
  color: #fff;
}

.release-icon {
  font-size: 14px;
  font-weight: bold;
}

/* Simulation Panel */
.simulation-panel {
  background: #14141f;
  border: 1px solid #2a2a3a;
  border-radius: 8px;
  overflow: hidden;
}

.controls-grid {
  display: grid;
  grid-template-columns: repeat(4, 1fr);
  gap: 12px;
  padding: 16px;
}

.control-btn {
  padding: 16px;
  background: #2a2a3a;
  border: 1px solid #3a3a4a;
  color: #fff;
  font-size: 13px;
  font-weight: 600;
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.2s;
}

.control-btn:hover {
  background: #3a3a4a;
  transform: translateY(-2px);
}

.control-btn.reset {
  background: #ff4444;
  border-color: #ff4444;
}

.control-btn.reset:hover {
  background: #ff6666;
}
</style>
