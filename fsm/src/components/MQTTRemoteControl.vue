<template>
  <div class="mqtt-remote-control">
    <!-- 顶部状态栏 -->
    <div class="status-bar">
      <div class="status-left">
        <span class="brand">FSM-Pilot</span>
        <span class="vehicle-id">{{ config.vehicleId }}</span>
        <span :class="['conn-badge', status]">
          {{ statusLabel }}
        </span>
        <span v-if="status === 'connected'" class="latency">
          {{ latency }}ms
        </span>
      </div>
      <div class="status-right">
        <span v-if="telemetry" class="telem-item">
          {{ telemetry.speed.toFixed(1) }} km/h
        </span>
        <span v-if="telemetry" class="telem-item">
          {{ telemetry.gear }}
        </span>
        <button class="btn-emergency" @click="onEmergencyStop">
          ⚠ 紧急停车
        </button>
        <button :class="['btn-connect', status]" @click="toggleConnect">
          {{ status === 'connected' ? '断开' : '连接' }}
        </button>
      </div>
    </div>

    <!-- 主体 -->
    <div class="main-layout">
      <!-- 左侧: 地图 + 遥测 -->
      <aside class="left-panel">
        <div class="telem-panel" v-if="telemetry">
          <div class="telem-row">
            <span class="label">速度</span>
            <span class="value">{{ telemetry.speed.toFixed(1) }} km/h</span>
          </div>
          <div class="telem-row">
            <span class="label">转向</span>
            <span class="value">{{ (telemetry.steering * 57.3).toFixed(1) }}°</span>
          </div>
          <div class="telem-row">
            <span class="label">档位</span>
            <span class="value gear">{{ telemetry.gear }}</span>
          </div>
          <div class="telem-row">
            <span class="label">电量</span>
            <span class="value">{{ telemetry.battery }}%</span>
          </div>
          <div class="telem-row">
            <span class="label">朝向</span>
            <span class="value">{{ telemetry.heading.toFixed(1) }}°</span>
          </div>
        </div>
        <div class="telem-panel offline" v-else>
          <p>等待遥测数据...</p>
        </div>

        <!-- 控制模式 -->
        <div class="control-panel">
          <div class="control-title">控制输入</div>
          <div class="control-row">
            <span class="label">转向</span>
            <div class="bar-wrap">
              <div class="bar-fill steering"
                :style="{ width: Math.abs(currentControl.steering) * 50 + '%',
                          left: currentControl.steering < 0 ? (50 - Math.abs(currentControl.steering)*50) + '%' : '50%' }"/>
            </div>
            <span class="val">{{ (currentControl.steering * 100).toFixed(0) }}</span>
          </div>
          <div class="control-row">
            <span class="label">油门</span>
            <div class="bar-wrap">
              <div class="bar-fill throttle" :style="{ width: currentControl.throttle * 100 + '%' }"/>
            </div>
            <span class="val">{{ (currentControl.throttle * 100).toFixed(0) }}</span>
          </div>
          <div class="control-row">
            <span class="label">刹车</span>
            <div class="bar-wrap">
              <div class="bar-fill brake" :style="{ width: currentControl.brake * 100 + '%' }"/>
            </div>
            <span class="val">{{ (currentControl.brake * 100).toFixed(0) }}</span>
          </div>
          <div class="hint">WASD / 方向键 / 手柄控制</div>
        </div>
      </aside>

      <!-- 中央: 6路摄像头 -->
      <div class="camera-grid">
        <!-- 主摄像头 (大) -->
        <div class="cam-main">
          <img
            v-if="cameraUrls[mainCamIdx]"
            :src="cameraUrls[mainCamIdx]"
            class="cam-img"
            alt="主摄像头"
          />
          <div v-else class="cam-placeholder">
            <span>CAM {{ mainCamIdx }}</span>
            <span class="cam-name">{{ camNames[mainCamIdx] }}</span>
          </div>
          <div class="cam-overlay">
            <span>{{ camNames[mainCamIdx] }}</span>
            <span class="fps">{{ cameraFps[mainCamIdx] }} fps</span>
          </div>
        </div>

        <!-- 5路副摄像头 -->
        <div class="cam-pip-grid">
          <div
            v-for="i in pipCams"
            :key="i"
            :class="['cam-pip', { active: i === mainCamIdx }]"
            @click="mainCamIdx = i"
          >
            <img
              v-if="cameraUrls[i]"
              :src="cameraUrls[i]"
              class="cam-img"
              alt=""
            />
            <div v-else class="cam-placeholder small">
              <span>CAM {{ i }}</span>
            </div>
            <div class="cam-overlay small">
              <span>{{ camNames[i] }}</span>
              <span class="fps">{{ cameraFps[i] }}fps</span>
            </div>
          </div>
        </div>
      </div>

      <!-- 右侧: 档位 + 设置 -->
      <aside class="right-panel">
        <div class="gear-panel">
          <div class="gear-title">档位</div>
          <div class="gear-buttons">
            <button
              v-for="g in ['P','R','N','D']"
              :key="g"
              :class="['gear-btn', { active: telemetry?.gear === g }]"
              @click="sendGear(g)"
            >{{ g }}</button>
          </div>
        </div>

        <div class="cam-quality-panel">
          <div class="panel-title">摄像头质量</div>
          <div v-for="(fps, i) in cameraFps" :key="i" class="cam-stat">
            <span>CAM{{ i }}</span>
            <span :class="['fps-badge', fps > 8 ? 'good' : fps > 3 ? 'ok' : 'bad']">
              {{ fps }}fps
            </span>
          </div>
        </div>

        <div class="network-panel">
          <div class="panel-title">网络</div>
          <div class="net-row">
            <span>延迟</span>
            <span :class="['net-val', latency < 100 ? 'good' : latency < 200 ? 'ok' : 'bad']">
              {{ latency }}ms
            </span>
          </div>
          <div class="net-row">
            <span>MQTT</span>
            <span :class="['net-val', status === 'connected' ? 'good' : 'bad']">
              {{ status }}
            </span>
          </div>
          <div class="net-row">
            <span>车辆</span>
            <span :class="['net-val', vehicleOnline ? 'good' : 'bad']">
              {{ vehicleOnline ? '在线' : '离线' }}
            </span>
          </div>
        </div>
      </aside>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, reactive, computed, onMounted, onUnmounted } from 'vue'
import { initMQTTService, type MQTTConfig, type ControlCommand } from '@/services/mqttService'
import { ALIYUN_MQTT_CONFIG } from '@/config/mqttConfig'
import { useGamepadController } from '@/composables/useGamepadController'
import { useKeyboardControl } from '@/composables/useKeyboardControl'

// ======================== 配置 ========================

const config = ALIYUN_MQTT_CONFIG

const camNames = ['前视主摄', '前视广角', '后视', '左后视', '右后视', '360环视']

// ======================== MQTT 服务 ========================

const mqttService = initMQTTService(config)

const status      = mqttService.status
const telemetry   = mqttService.telemetry
const vehicleOnline = mqttService.vehicleOnline
const latency     = mqttService.latency
const cameraUrls  = mqttService.cameraUrls
const cameraFps   = mqttService.cameraFps

const statusLabel = computed(() => ({
  disconnected: '未连接',
  connecting:   '连接中...',
  connected:    '已连接',
  error:        '连接错误',
}[status.value]))

// ======================== 摄像头布局 ========================

const mainCamIdx = ref(0)
const pipCams = computed(() => [0, 1, 2, 3, 4, 5].filter(i => i !== mainCamIdx.value))

// ======================== 控制状态 ========================

const currentControl = reactive({ steering: 0, throttle: 0, brake: 0 })

// 控制发送循环 (20Hz)
let controlInterval: ReturnType<typeof setInterval> | null = null

function startControlLoop() {
  if (controlInterval) return
  controlInterval = setInterval(() => {
    if (status.value !== 'connected') return
    mqttService.sendControl({ ...currentControl })
  }, 50)
}

function stopControlLoop() {
  if (controlInterval) { clearInterval(controlInterval); controlInterval = null }
}

// ======================== 键盘控制 ========================

const keys = new Set<string>()

function onKeyDown(e: KeyboardEvent) {
  keys.add(e.key.toLowerCase())
  updateControlFromKeys()
}

function onKeyUp(e: KeyboardEvent) {
  keys.delete(e.key.toLowerCase())
  updateControlFromKeys()
}

function updateControlFromKeys() {
  let steering = 0, throttle = 0, brake = 0

  if (keys.has('a') || keys.has('arrowleft'))  steering = -1.0
  if (keys.has('d') || keys.has('arrowright')) steering =  1.0
  if (keys.has('w') || keys.has('arrowup'))    throttle =  1.0
  if (keys.has('s') || keys.has('arrowdown'))  brake    =  1.0

  currentControl.steering = steering
  currentControl.throttle = throttle
  currentControl.brake    = brake
}

// ======================== 操作 ========================

async function toggleConnect() {
  if (status.value === 'connected') {
    stopControlLoop()
    mqttService.disconnect()
  } else {
    await mqttService.connect()
    startControlLoop()
  }
}

function onEmergencyStop() {
  mqttService.emergencyStop()
  currentControl.steering = 0
  currentControl.throttle = 0
  currentControl.brake    = 1.0
}

const gearMap: Record<string, number> = { P: 0, R: 1, N: 2, D: 3 }
function sendGear(g: string) {
  mqttService.sendControl({ ...currentControl, gear: gearMap[g] })
}

// ======================== 生命周期 ========================

onMounted(async () => {
  window.addEventListener('keydown', onKeyDown)
  window.addEventListener('keyup', onKeyUp)
  // 自动连接
  await mqttService.connect()
  startControlLoop()
})

onUnmounted(() => {
  window.removeEventListener('keydown', onKeyDown)
  window.removeEventListener('keyup', onKeyUp)
  stopControlLoop()
  mqttService.disconnect()
})
</script>

<style scoped>
.mqtt-remote-control {
  display: flex;
  flex-direction: column;
  height: 100vh;
  background: #0a0e1a;
  color: #e0e6f0;
  font-family: 'JetBrains Mono', monospace;
}

/* 状态栏 */
.status-bar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 8px 16px;
  background: #0d1220;
  border-bottom: 1px solid #1e2d4a;
  height: 48px;
}
.status-left, .status-right { display: flex; align-items: center; gap: 12px; }
.brand { font-weight: 700; color: #4fc3f7; font-size: 14px; }
.vehicle-id { color: #90a4ae; font-size: 12px; }
.conn-badge {
  padding: 2px 8px; border-radius: 4px; font-size: 11px; font-weight: 600;
}
.conn-badge.connected    { background: #1b5e20; color: #69f0ae; }
.conn-badge.connecting   { background: #e65100; color: #ffcc02; }
.conn-badge.disconnected { background: #1a1a2e; color: #546e7a; }
.conn-badge.error        { background: #b71c1c; color: #ff5252; }
.latency { color: #69f0ae; font-size: 12px; }
.telem-item { color: #b0bec5; font-size: 13px; }
.btn-emergency {
  padding: 4px 12px; background: #b71c1c; color: #fff;
  border: none; border-radius: 4px; cursor: pointer; font-size: 12px;
}
.btn-emergency:hover { background: #d32f2f; }
.btn-connect {
  padding: 4px 14px; border: 1px solid #4fc3f7; background: transparent;
  color: #4fc3f7; border-radius: 4px; cursor: pointer; font-size: 12px;
}
.btn-connect.connected { border-color: #ef5350; color: #ef5350; }

/* 主体布局 */
.main-layout {
  display: grid;
  grid-template-columns: 200px 1fr 180px;
  flex: 1;
  overflow: hidden;
  gap: 4px;
  padding: 4px;
}

/* 左侧面板 */
.left-panel { display: flex; flex-direction: column; gap: 8px; overflow-y: auto; }
.telem-panel {
  background: #0d1220; border: 1px solid #1e2d4a; border-radius: 6px; padding: 10px;
}
.telem-panel.offline { color: #546e7a; font-size: 12px; text-align: center; padding: 20px; }
.telem-row { display: flex; justify-content: space-between; padding: 3px 0; font-size: 12px; }
.telem-row .label { color: #546e7a; }
.telem-row .value { color: #e0e6f0; font-weight: 600; }
.telem-row .value.gear { color: #4fc3f7; font-size: 16px; }

.control-panel {
  background: #0d1220; border: 1px solid #1e2d4a; border-radius: 6px; padding: 10px;
}
.control-title { font-size: 11px; color: #546e7a; margin-bottom: 8px; text-transform: uppercase; }
.control-row { display: flex; align-items: center; gap: 6px; margin-bottom: 6px; font-size: 11px; }
.control-row .label { width: 28px; color: #546e7a; }
.control-row .val { width: 24px; text-align: right; color: #90a4ae; }
.bar-wrap { flex: 1; height: 6px; background: #1e2d4a; border-radius: 3px; position: relative; overflow: hidden; }
.bar-fill { position: absolute; height: 100%; border-radius: 3px; transition: width 0.05s; }
.bar-fill.steering { background: #4fc3f7; top: 0; }
.bar-fill.throttle { background: #69f0ae; left: 0; top: 0; }
.bar-fill.brake    { background: #ef5350; left: 0; top: 0; }
.hint { font-size: 10px; color: #37474f; margin-top: 6px; }

/* 摄像头网格 */
.camera-grid {
  display: grid;
  grid-template-rows: 1fr 120px;
  gap: 4px;
  overflow: hidden;
}
.cam-main {
  position: relative; background: #050810; border-radius: 6px; overflow: hidden;
}
.cam-pip-grid {
  display: grid;
  grid-template-columns: repeat(5, 1fr);
  gap: 4px;
}
.cam-pip {
  position: relative; background: #050810; border-radius: 4px;
  overflow: hidden; cursor: pointer; border: 1px solid transparent;
}
.cam-pip.active { border-color: #4fc3f7; }
.cam-pip:hover  { border-color: #546e7a; }
.cam-img { width: 100%; height: 100%; object-fit: cover; display: block; }
.cam-placeholder {
  width: 100%; height: 100%; display: flex; flex-direction: column;
  align-items: center; justify-content: center; color: #1e2d4a; font-size: 13px; gap: 4px;
}
.cam-placeholder.small { font-size: 10px; }
.cam-name { font-size: 10px; color: #37474f; }
.cam-overlay {
  position: absolute; bottom: 0; left: 0; right: 0;
  background: linear-gradient(transparent, rgba(0,0,0,0.7));
  padding: 4px 8px; display: flex; justify-content: space-between;
  font-size: 11px; color: #b0bec5;
}
.cam-overlay.small { font-size: 9px; padding: 2px 4px; }
.fps { color: #69f0ae; }

/* 右侧面板 */
.right-panel { display: flex; flex-direction: column; gap: 8px; overflow-y: auto; }
.gear-panel, .cam-quality-panel, .network-panel {
  background: #0d1220; border: 1px solid #1e2d4a; border-radius: 6px; padding: 10px;
}
.gear-title, .panel-title {
  font-size: 11px; color: #546e7a; margin-bottom: 8px; text-transform: uppercase;
}
.gear-buttons { display: grid; grid-template-columns: repeat(4, 1fr); gap: 4px; }
.gear-btn {
  padding: 8px 4px; background: #1e2d4a; border: 1px solid #263547;
  color: #90a4ae; border-radius: 4px; cursor: pointer; font-size: 14px; font-weight: 700;
}
.gear-btn.active { background: #1565c0; border-color: #4fc3f7; color: #fff; }
.cam-stat { display: flex; justify-content: space-between; font-size: 11px; padding: 2px 0; }
.fps-badge { padding: 1px 5px; border-radius: 3px; font-size: 10px; }
.fps-badge.good { background: #1b5e20; color: #69f0ae; }
.fps-badge.ok   { background: #e65100; color: #ffcc02; }
.fps-badge.bad  { background: #1a1a2e; color: #546e7a; }
.net-row { display: flex; justify-content: space-between; font-size: 11px; padding: 3px 0; }
.net-val { font-weight: 600; }
.net-val.good { color: #69f0ae; }
.net-val.ok   { color: #ffcc02; }
.net-val.bad  { color: #ef5350; }
</style>
