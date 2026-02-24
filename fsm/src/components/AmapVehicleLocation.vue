<!--
  FSM-Pilot V2.0 - Amap Vehicle Location Component

  Real-time vehicle location display using Amap (高德地图)

  @author Li Yixiang
  @institution City University of Hong Kong
-->
<template>
  <div class="amap-container">
    <div ref="mapContainer" class="map-canvas"></div>

    <!-- Control Panel -->
    <div class="map-controls">
      <button @click="fitView" class="control-btn" title="适应视图">
        🎯
      </button>
      <button @click="toggleMapStyle" class="control-btn" title="切换样式">
        🎨
      </button>
      <button @click="clearMap" class="control-btn" title="清除标记">
        🗑️
      </button>
    </div>

    <!-- Legend -->
    <div class="map-legend">
      <div class="legend-title">图例</div>
      <div class="legend-item">
        <span class="legend-marker critical"></span>
        <span>紧急车辆</span>
      </div>
      <div class="legend-item">
        <span class="legend-marker high"></span>
        <span>高风险</span>
      </div>
      <div class="legend-item">
        <span class="legend-marker medium"></span>
        <span>中风险</span>
      </div>
      <div class="legend-item">
        <span class="legend-marker low"></span>
        <span>低风险</span>
      </div>
      <div class="legend-item">
        <span class="legend-marker operator"></span>
        <span>安全员</span>
      </div>
    </div>

    <!-- Status Bar -->
    <div class="status-bar">
      <div class="status-item">
        <span class="status-label">车辆数:</span>
        <span class="status-value">{{ vehicleMarkers.size }}</span>
      </div>
      <div class="status-item">
        <span class="status-label">安全员:</span>
        <span class="status-value">{{ operatorMarkers.size }}</span>
      </div>
      <div class="status-item">
        <span :class="['status-indicator', amapReady ? 'online' : 'offline']">
          <span class="status-dot"></span>
          {{ amapReady ? '地图就绪' : '未配置' }}
        </span>
      </div>
    </div>

    <!-- Configuration Notice -->
    <div v-if="!amapReady" class="config-notice">
      <div class="notice-icon">🗺️</div>
      <h4>高德地图未配置</h4>
      <p>请在环境变量中配置以下信息：</p>
      <div class="notice-code">
        <code>VITE_AMAP_API_KEY=your_amap_key</code>
        <code>VITE_AMAP_JS_CODE=your_js_code</code>
      </div>
      <p class="notice-link">
        获取密钥: <a href="https://console.amap.com" target="_blank">https://console.amap.com</a>
      </p>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted, watch } from 'vue'
import { getAmapService } from '@/services/amapService'
import type { AmapMarker } from '@/services/amapService'

// Props
interface VehicleLocation {
  id: string
  longitude: number
  latitude: number
  urgency: 'critical' | 'high' | 'medium' | 'low'
  label?: string
}

interface OperatorLocation {
  id: string
  longitude: number
  latitude: number
  name: string
  status: 'idle' | 'busy'
}

interface Props {
  vehicles?: VehicleLocation[]
  operators?: OperatorLocation[]
  center?: { longitude: number; latitude: number }
  zoom?: number
}

const props = withDefaults(defineProps<Props>(), {
  vehicles: () => [],
  operators: () => [],
  zoom: 12
})

// Services
const amapService = getAmapService()

// State
const mapContainer = ref<HTMLElement | null>(null)
const amapReady = ref(false)
const mapStyle = ref<'dark' | 'light'>('dark')
const vehicleMarkers = ref(new Map<string, any>())
const operatorMarkers = ref(new Map<string, any>())

// Methods
async function initializeMap() {
  if (!mapContainer.value) return

  const initialized = await amapService.initialize()
  if (!initialized) {
    console.warn('[Amap Component] Map not configured')
    return
  }

  const center = props.center || { longitude: 114.17, latitude: 22.32 }

  amapService.createMap(mapContainer.value, {
    zoom: props.zoom,
    center: [center.longitude, center.latitude],
    mapStyle: 'amap://styles/darkblue',
    viewMode: '3D',
    pitch: 40
  })

  amapReady.value = true
  console.log('[Amap Component] Map initialized')

  // Render initial data
  updateVehicleMarkers()
  updateOperatorMarkers()
}

function updateVehicleMarkers() {
  if (!amapReady.value) return

  // Remove old markers
  for (const id of vehicleMarkers.value.keys()) {
    if (!props.vehicles.find(v => v.id === id)) {
      amapService.removeMarker(id)
      vehicleMarkers.value.delete(id)
    }
  }

  // Add/update markers
  for (const vehicle of props.vehicles) {
    const colorMap = {
      critical: '#ff3333',
      high: '#ff9933',
      medium: '#ffdd33',
      low: '#33ff99'
    }

    const marker: AmapMarker = {
      id: vehicle.id,
      position: [vehicle.longitude, vehicle.latitude],
      title: vehicle.id,
      label: vehicle.label,
      extData: {
        type: 'vehicle',
        urgency: vehicle.urgency
      }
    }

    if (vehicleMarkers.value.has(vehicle.id)) {
      // Update existing marker
      amapService.updateMarkerPosition(vehicle.id, marker.position)
    } else {
      // Add new marker
      const amapMarker = amapService.addMarker(marker)
      vehicleMarkers.value.set(vehicle.id, amapMarker)

      // Customize marker appearance
      if (amapMarker && window.AMap) {
        const content = `
          <div style="
            width: 24px;
            height: 24px;
            background: ${colorMap[vehicle.urgency]};
            border: 2px solid #fff;
            border-radius: 50%;
            box-shadow: 0 2px 8px rgba(0,0,0,0.3);
          "></div>
        `
        amapMarker.setContent(content)
      }
    }
  }
}

function updateOperatorMarkers() {
  if (!amapReady.value) return

  // Remove old markers
  for (const id of operatorMarkers.value.keys()) {
    if (!props.operators.find(o => o.id === id)) {
      amapService.removeMarker(`op-${id}`)
      operatorMarkers.value.delete(id)
    }
  }

  // Add/update markers
  for (const operator of props.operators) {
    const markerId = `op-${operator.id}`

    const marker: AmapMarker = {
      id: markerId,
      position: [operator.longitude, operator.latitude],
      title: operator.name,
      label: operator.name,
      extData: {
        type: 'operator',
        status: operator.status
      }
    }

    if (operatorMarkers.value.has(operator.id)) {
      // Update existing marker
      amapService.updateMarkerPosition(markerId, marker.position)
    } else {
      // Add new marker
      const amapMarker = amapService.addMarker(marker)
      operatorMarkers.value.set(operator.id, amapMarker)

      // Customize marker appearance
      if (amapMarker && window.AMap) {
        const color = operator.status === 'idle' ? '#ff5722' : '#888'
        const content = `
          <div style="
            width: 32px;
            height: 32px;
            background: ${color};
            border: 3px solid #fff;
            border-radius: 50%;
            box-shadow: 0 2px 8px rgba(0,0,0,0.3);
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 16px;
          ">👤</div>
        `
        amapMarker.setContent(content)
      }
    }
  }
}

function fitView() {
  if (amapReady.value) {
    amapService.fitView()
  }
}

function toggleMapStyle() {
  if (!amapReady.value || !window.AMap) return

  mapStyle.value = mapStyle.value === 'dark' ? 'light' : 'dark'

  const styles = {
    dark: 'amap://styles/darkblue',
    light: 'amap://styles/light'
  }

  const map = amapService['mapInstance']
  if (map) {
    map.setMapStyle(styles[mapStyle.value])
  }
}

function clearMap() {
  if (amapReady.value) {
    amapService.clear()
    vehicleMarkers.value.clear()
    operatorMarkers.value.clear()
  }
}

// Watchers
watch(() => props.vehicles, () => {
  updateVehicleMarkers()
}, { deep: true })

watch(() => props.operators, () => {
  updateOperatorMarkers()
}, { deep: true })

watch(() => props.center, (newCenter) => {
  if (amapReady.value && newCenter) {
    amapService.setCenter(newCenter.longitude, newCenter.latitude)
  }
})

// Lifecycle
onMounted(() => {
  initializeMap()
})

onUnmounted(() => {
  amapService.destroy()
})
</script>

<style scoped>
.amap-container {
  position: relative;
  width: 100%;
  height: 100%;
  min-height: 400px;
  background: #0a0a12;
  border-radius: 8px;
  overflow: hidden;
}

.map-canvas {
  width: 100%;
  height: 100%;
}

/* Map Controls */
.map-controls {
  position: absolute;
  top: 16px;
  right: 16px;
  display: flex;
  flex-direction: column;
  gap: 8px;
  z-index: 100;
}

.control-btn {
  width: 40px;
  height: 40px;
  background: rgba(26, 26, 46, 0.9);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  font-size: 18px;
  cursor: pointer;
  transition: all 0.2s;
  backdrop-filter: blur(10px);
}

.control-btn:hover {
  background: rgba(255, 87, 34, 0.2);
  border-color: var(--primary, #ff5722);
  transform: scale(1.1);
}

/* Legend */
.map-legend {
  position: absolute;
  top: 16px;
  left: 16px;
  background: rgba(26, 26, 46, 0.95);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  padding: 12px;
  z-index: 100;
  backdrop-filter: blur(10px);
}

.legend-title {
  font-size: 14px;
  font-weight: 700;
  color: #fff;
  margin-bottom: 8px;
}

.legend-item {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-bottom: 6px;
  font-size: 12px;
  color: #ccc;
}

.legend-item:last-child {
  margin-bottom: 0;
}

.legend-marker {
  width: 16px;
  height: 16px;
  border-radius: 50%;
  border: 2px solid #fff;
}

.legend-marker.critical {
  background: #ff3333;
}

.legend-marker.high {
  background: #ff9933;
}

.legend-marker.medium {
  background: #ffdd33;
}

.legend-marker.low {
  background: #33ff99;
}

.legend-marker.operator {
  background: #ff5722;
}

/* Status Bar */
.status-bar {
  position: absolute;
  bottom: 16px;
  left: 16px;
  right: 16px;
  display: flex;
  gap: 16px;
  padding: 12px;
  background: rgba(26, 26, 46, 0.95);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  z-index: 100;
  backdrop-filter: blur(10px);
}

.status-item {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 12px;
}

.status-label {
  color: #888;
}

.status-value {
  color: var(--primary, #ff5722);
  font-weight: 700;
}

.status-indicator {
  display: flex;
  align-items: center;
  gap: 6px;
  padding: 4px 12px;
  border-radius: 12px;
  font-size: 11px;
  font-weight: 600;
  margin-left: auto;
}

.status-indicator.online {
  background: rgba(51, 255, 153, 0.2);
  color: #33ff99;
}

.status-indicator.offline {
  background: rgba(136, 136, 136, 0.2);
  color: #888;
}

.status-dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  background: currentColor;
  animation: blink 2s ease-in-out infinite;
}

@keyframes blink {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.3; }
}

/* Configuration Notice */
.config-notice {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  max-width: 500px;
  padding: 32px;
  background: rgba(26, 26, 46, 0.98);
  border: 2px solid rgba(255, 255, 255, 0.1);
  border-radius: 12px;
  text-align: center;
  z-index: 200;
  backdrop-filter: blur(20px);
}

.notice-icon {
  font-size: 64px;
  margin-bottom: 16px;
  opacity: 0.5;
}

.config-notice h4 {
  margin: 0 0 16px 0;
  font-size: 20px;
  color: #fff;
}

.config-notice p {
  margin: 12px 0;
  font-size: 14px;
  color: #888;
  line-height: 1.6;
}

.notice-code {
  display: flex;
  flex-direction: column;
  gap: 8px;
  margin: 16px 0;
  padding: 16px;
  background: rgba(0, 0, 0, 0.3);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 6px;
  text-align: left;
}

.notice-code code {
  display: block;
  font-family: 'Monaco', 'Courier New', monospace;
  font-size: 12px;
  color: var(--primary, #ff5722);
}

.notice-link {
  margin-top: 16px;
}

.notice-link a {
  color: var(--primary, #ff5722);
  text-decoration: none;
  font-weight: 600;
}

.notice-link a:hover {
  text-decoration: underline;
}
</style>
