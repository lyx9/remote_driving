<!--
  Guardian Mobility - Left Sidebar with Amap

  @description Left sidebar with Amap and system logs
-->
<template>
  <Transition name="slide-left">
    <aside v-if="systemStore.ui.showLeftSidebar" class="sidebar left">
      <div class="sidebar-inner">
        <!-- Amap Card -->
        <div class="card map-card">
          <div class="card-header">
            <span>🗺️ LIVE FLEET MAP - HONG KONG</span>
            <span class="vehicle-count">{{ fleetStore.vehicles.length }} Vehicles</span>
          </div>
          <div class="card-body">
            <div ref="mapContainer" class="map-container"></div>
            <div class="map-legend">
              <div class="legend-item">
                <span class="legend-dot active"></span>
                <span>Active ({{ activeCount }})</span>
              </div>
              <div class="legend-item">
                <span class="legend-dot idle"></span>
                <span>Idle ({{ idleCount }})</span>
              </div>
              <div class="legend-item">
                <span class="legend-dot patrol"></span>
                <span>Patrol ({{ patrolCount }})</span>
              </div>
            </div>
          </div>
        </div>

        <!-- Log Card -->
        <div class="card log-card">
          <div class="card-header">
            <span>📋 SYSTEM LOG</span>
            <div class="log-header-right">
              <span class="log-count">{{ systemStore.filteredLogs.length }}/{{ systemStore.logs.length }}</span>
              <button class="btn-clear" @click="systemStore.clearLogs()" title="Clear logs">✕</button>
            </div>
          </div>

          <!-- 分类过滤器 -->
          <div class="log-filters">
            <button
              v-for="cat in LOG_CATEGORIES"
              :key="cat.key"
              :class="['filter-btn', { active: activeCategory === cat.key }]"
              :title="cat.label"
              @click="toggleCategory(cat.key)"
            >
              {{ cat.icon }}
              <span class="filter-count">{{ systemStore.logStats[cat.key] || 0 }}</span>
            </button>
          </div>

          <!-- 级别过滤器 -->
          <div class="level-filters">
            <button
              v-for="lvl in LOG_LEVELS"
              :key="lvl.key"
              :class="['level-btn', lvl.key, { active: activeLevel === lvl.key }]"
              @click="toggleLevel(lvl.key)"
            >{{ lvl.label }}</button>
            <button class="level-btn reset" @click="resetFilters">ALL</button>
          </div>

          <div class="card-body">
            <div class="log-box" ref="logBoxRef">
              <div
                v-for="log in systemStore.filteredLogs"
                :key="log.id"
                :class="['log-entry', log.level]"
                @click="toggleDetail(log.id)"
              >
                <span class="log-time">[{{ formatTime(log.timestamp) }}]</span>
                <span class="log-cat" :class="log.category">{{ catIcon(log.category) }}</span>
                <span class="log-msg">{{ log.message }}</span>
                <div v-if="log.detail && expandedId === log.id" class="log-detail">{{ log.detail }}</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </aside>
  </Transition>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, watch, nextTick, onUnmounted } from 'vue'
import { useSystemStore } from '@/stores/system'
import { useFleetStore } from '@/stores/fleet'
import type { LogLevel, LogCategory } from '@/types'
import AMapLoader from '@amap/amap-jsapi-loader'

const systemStore = useSystemStore()
const fleetStore = useFleetStore()

const mapContainer = ref<HTMLDivElement>()
const logBoxRef = ref<HTMLDivElement>()
let map: any = null
let markers: Map<string, any> = new Map()
let polylines: Map<string, any> = new Map()

// ======================== Log 过滤 ========================

const LOG_CATEGORIES = [
  { key: 'mqtt',      icon: '📡', label: 'MQTT' },
  { key: 'webrtc',   icon: '🔗', label: 'WebRTC' },
  { key: 'websocket',icon: '🌐', label: 'WebSocket' },
  { key: 'control',  icon: '🕹', label: 'Control' },
  { key: 'camera',   icon: '📷', label: 'Camera' },
  { key: 'telemetry',icon: '📊', label: 'Telemetry' },
  { key: 'vehicle',  icon: '🚗', label: 'Vehicle' },
  { key: 'ros',      icon: '🤖', label: 'ROS' },
  { key: 'system',   icon: '⚙', label: 'System' },
  { key: 'auth',     icon: '🔐', label: 'Auth' },
] as const

const LOG_LEVELS = [
  { key: 'error',   label: 'ERR' },
  { key: 'warning', label: 'WRN' },
  { key: 'success', label: 'OK' },
  { key: 'info',    label: 'INF' },
  { key: 'debug',   label: 'DBG' },
] as const

const activeCategory = ref<LogCategory | null>(null)
const activeLevel = ref<LogLevel | null>(null)
const expandedId = ref<number | null>(null)

function toggleCategory(cat: LogCategory) {
  activeCategory.value = activeCategory.value === cat ? null : cat
  systemStore.setLogFilter(activeLevel.value, activeCategory.value)
}

function toggleLevel(lvl: LogLevel) {
  activeLevel.value = activeLevel.value === lvl ? null : lvl
  systemStore.setLogFilter(activeLevel.value, activeCategory.value)
}

function resetFilters() {
  activeCategory.value = null
  activeLevel.value = null
  systemStore.setLogFilter(null, null)
}

function toggleDetail(id: number) {
  expandedId.value = expandedId.value === id ? null : id
}

const CATEGORY_ICONS: Record<string, string> = {
  mqtt: '📡', webrtc: '🔗', websocket: '🌐', control: '🕹',
  camera: '📷', telemetry: '📊', vehicle: '🚗', ros: '🤖',
  system: '⚙', auth: '🔐',
}
function catIcon(cat: string) { return CATEGORY_ICONS[cat] || '•' }

// Vehicle counts
const activeCount = computed(() => fleetStore.vehicles.filter(v => v.status === 'ACTIVE').length)
const idleCount = computed(() => fleetStore.vehicles.filter(v => v.status === 'IDLE').length)
const patrolCount = computed(() => fleetStore.vehicles.filter(v => v.status === 'PATROL').length)

const formatTime = (date: Date) => {
  return date.toLocaleTimeString()
}

// Initialize Amap
const initMap = async () => {
  if (!mapContainer.value) return

  try {
    // Set security configuration
    ;(window as any)._AMapSecurityConfig = {
      securityJsCode: import.meta.env.VITE_AMAP_JS_CODE || '215104d11967ff4b9b17366e0bd56f0f'
    }

    const AMap = await AMapLoader.load({
      key: import.meta.env.VITE_AMAP_API_KEY || 'fa4c4bc1d796891d00472871682f6628',
      version: '2.0',
      plugins: ['AMap.Scale', 'AMap.ToolBar']
    })

    // Create map centered on Hong Kong
    map = new AMap.Map(mapContainer.value, {
      zoom: 11,
      center: [114.1694, 22.3193], // Hong Kong (Tsim Sha Tsui)
      mapStyle: 'amap://styles/dark',
      viewMode: '2D',
      showLabel: true,
      features: ['bg', 'road', 'building']
    })

    // Add scale and toolbar
    map.addControl(new AMap.Scale())
    map.addControl(new AMap.ToolBar({
      position: {
        top: '10px',
        right: '10px'
      }
    }))

    // Initialize all vehicle markers
    updateAllVehicles()

    console.log('[LeftSidebar] Amap initialized successfully')
  } catch (error) {
    console.error('[LeftSidebar] Failed to initialize Amap:', error)
  }
}

// Update all vehicles on map
const updateAllVehicles = () => {
  if (!map) return

  fleetStore.vehicles.forEach(vehicle => {
    updateVehicleOnMap(vehicle)
  })
}

// Update single vehicle on map
const updateVehicleOnMap = (vehicle: any) => {
  if (!map) return

  const AMap = (window as any).AMap
  if (!AMap) return

  // Remove existing marker and polyline
  if (markers.has(vehicle.id)) {
    map.remove(markers.get(vehicle.id))
  }
  if (polylines.has(vehicle.id)) {
    map.remove(polylines.get(vehicle.id))
  }

  // Determine marker color based on status
  let markerColor = '#10b981' // Active - green
  if (vehicle.status === 'IDLE') {
    markerColor = '#6b7280' // Idle - gray
  } else if (vehicle.status === 'PATROL') {
    markerColor = '#f59e0b' // Patrol - orange
  }

  // Add path polyline
  if (vehicle.path && vehicle.path.length > 0) {
    const pathCoords = vehicle.path.map((p: [number, number]) => [p[1], p[0]]) // Swap lat/lng for Amap
    const polyline = new AMap.Polyline({
      path: pathCoords,
      strokeColor: '#ff5722',
      strokeWeight: 3,
      strokeOpacity: 0.6,
      strokeStyle: 'solid'
    })
    map.add(polyline)
    polylines.set(vehicle.id, polyline)
  }

  // Add vehicle marker
  const marker = new AMap.CircleMarker({
    center: [vehicle.location[1], vehicle.location[0]], // Swap lat/lng for Amap
    radius: 8,
    strokeColor: '#ffffff',
    strokeWeight: 2,
    fillColor: markerColor,
    fillOpacity: 1,
    zIndex: 100,
    bubble: true
  })

  // Add info window
  const infoContent = `
    <div style="padding: 8px; font-family: monospace; font-size: 11px; background: #1a1a1a; color: #fff; border-radius: 4px;">
      <div style="font-weight: bold; color: ${markerColor}; margin-bottom: 4px;">${vehicle.id}</div>
      <div>Type: ${vehicle.type}</div>
      <div>Status: <span style="color: ${markerColor}">${vehicle.status}</span></div>
      <div>Speed: ${vehicle.speed} km/h</div>
      <div>Battery: ${vehicle.battery_level}%</div>
      <div>Latency: ${vehicle.latency_ms}ms</div>
      <div>Profit: $${vehicle.money.toFixed(2)}</div>
    </div>
  `

  const infoWindow = new AMap.InfoWindow({
    content: infoContent,
    offset: new AMap.Pixel(0, -10)
  })

  marker.on('click', () => {
    infoWindow.open(map, marker.getCenter())
  })

  map.add(marker)
  markers.set(vehicle.id, marker)

  // If this is the current vehicle, center map on it
  if (vehicle.id === fleetStore.currentVehicle.id) {
    map.setCenter([vehicle.location[1], vehicle.location[0]], false, 300)
  }
}

// Watch for vehicle changes
watch(
  () => fleetStore.currentVehicleIndex,
  () => {
    const vehicle = fleetStore.currentVehicle
    if (map && vehicle) {
      map.setCenter([vehicle.location[1], vehicle.location[0]], false, 300)
    }
  }
)

// Watch for sidebar visibility
watch(
  () => systemStore.ui.showLeftSidebar,
  async (show) => {
    if (show) {
      await nextTick()
      setTimeout(() => {
        if (map) {
          map.resize()
        }
      }, 350)
    }
  }
)

// Update vehicle positions periodically
let updateInterval: number | null = null

onMounted(async () => {
  await nextTick()
  await initMap()

  // Update vehicle positions every 2 seconds
  updateInterval = window.setInterval(() => {
    updateAllVehicles()
  }, 2000)
})

onUnmounted(() => {
  if (updateInterval) {
    clearInterval(updateInterval)
  }
  if (map) {
    map.destroy()
  }
})
</script>

<style scoped>
.sidebar {
  width: var(--side-w);
  background: #000;
  border-right: 1px solid var(--border);
  display: flex;
  flex-direction: column;
  flex-shrink: 0;
  z-index: 50;
}

.sidebar-inner {
  width: 100%;
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

.map-card {
  flex: 2.5;
  min-height: 400px;
}

.log-card {
  flex: 1;
  min-height: 200px;
}

.card-header {
  padding: 8px 12px;
  background: linear-gradient(135deg, rgba(255, 87, 34, 0.1) 0%, rgba(255, 87, 34, 0.05) 100%);
  color: var(--primary);
  font-weight: bold;
  border-bottom: 1px solid #222;
  font-size: 10px;
  letter-spacing: 0.5px;
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.vehicle-count,
.log-count {
  font-size: 9px;
  color: #666;
  font-weight: normal;
}

.card-body {
  flex: 1;
  overflow: hidden;
  position: relative;
  background: var(--bg-panel);
  padding: 0;
}

.map-container {
  width: 100%;
  height: 100%;
  position: relative;
}

.map-legend {
  position: absolute;
  bottom: 10px;
  left: 10px;
  background: rgba(0, 0, 0, 0.8);
  border: 1px solid #333;
  border-radius: 4px;
  padding: 8px;
  display: flex;
  flex-direction: column;
  gap: 6px;
  z-index: 10;
}

.legend-item {
  display: flex;
  align-items: center;
  gap: 6px;
  font-size: 10px;
  color: #ccc;
}

.legend-dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  border: 1px solid #fff;
}

.legend-dot.active {
  background: #10b981;
}

.legend-dot.idle {
  background: #6b7280;
}

.legend-dot.patrol {
  background: #f59e0b;
}

.log-box {
  height: 100%;
  overflow-y: auto;
  font-family: 'Consolas', monospace;
  color: #777;
  font-size: 10px;
  padding: 8px;
}

.log-entry {
  border-bottom: 1px solid #1a1a1a;
  padding: 4px 2px;
  line-height: 1.5;
  word-wrap: break-word;
  cursor: pointer;
  display: flex;
  flex-wrap: wrap;
  gap: 3px;
  align-items: baseline;
}
.log-entry:hover { background: #111; }
.log-time { color: #444; flex-shrink: 0; }
.log-cat  { flex-shrink: 0; font-size: 10px; }
.log-msg  { color: #777; flex: 1; }
.log-detail {
  width: 100%; margin-top: 2px; padding: 3px 6px;
  background: #0d0d0d; border-left: 2px solid #333;
  color: #555; font-size: 9px; word-break: break-all;
}

.log-entry.error   .log-msg { color: var(--danger); }
.log-entry.warning .log-msg { color: var(--warn); }
.log-entry.success .log-msg { color: #69f0ae; }
.log-entry.debug   .log-msg { color: #546e7a; }

/* 分类过滤器 */
.log-header-right { display: flex; align-items: center; gap: 6px; }
.btn-clear {
  background: none; border: none; color: #444; cursor: pointer;
  font-size: 10px; padding: 0 2px;
}
.btn-clear:hover { color: #ef5350; }

.log-filters {
  display: flex; flex-wrap: wrap; gap: 3px;
  padding: 4px 8px; border-bottom: 1px solid #1a1a1a;
}
.filter-btn {
  background: #111; border: 1px solid #222; border-radius: 3px;
  color: #444; cursor: pointer; font-size: 11px; padding: 1px 4px;
  display: flex; align-items: center; gap: 2px;
}
.filter-btn:hover { border-color: #444; color: #888; }
.filter-btn.active { border-color: #4fc3f7; color: #4fc3f7; background: #0d1a26; }
.filter-count { font-size: 9px; color: #555; }
.filter-btn.active .filter-count { color: #4fc3f7; }

.level-filters {
  display: flex; gap: 3px; padding: 3px 8px;
  border-bottom: 1px solid #1a1a1a;
}
.level-btn {
  background: #111; border: 1px solid #222; border-radius: 3px;
  color: #444; cursor: pointer; font-size: 9px; padding: 1px 5px;
}
.level-btn:hover { border-color: #444; }
.level-btn.active.error   { border-color: #ef5350; color: #ef5350; }
.level-btn.active.warning { border-color: #ffcc02; color: #ffcc02; }
.level-btn.active.success { border-color: #69f0ae; color: #69f0ae; }
.level-btn.active.info    { border-color: #4fc3f7; color: #4fc3f7; }
.level-btn.active.debug   { border-color: #546e7a; color: #546e7a; }
.level-btn.reset { margin-left: auto; color: #555; }

.slide-left-enter-active,
.slide-left-leave-active {
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.slide-left-enter-from {
  transform: translateX(-100%);
  opacity: 0;
}

.slide-left-leave-to {
  transform: translateX(-100%);
  opacity: 0;
}

/* Scrollbar styling */
.log-box::-webkit-scrollbar {
  width: 4px;
}

.log-box::-webkit-scrollbar-track {
  background: #0a0a0a;
}

.log-box::-webkit-scrollbar-thumb {
  background: #333;
  border-radius: 2px;
}

.log-box::-webkit-scrollbar-thumb:hover {
  background: #555;
}
</style>
