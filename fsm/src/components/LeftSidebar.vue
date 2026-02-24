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
            <span class="log-count">{{ systemStore.logs.length }} entries</span>
          </div>
          <div class="card-body">
            <div class="log-box">
              <div
                v-for="(log, index) in systemStore.logs"
                :key="index"
                :class="['log-entry', log.level]"
              >
                [{{ formatTime(log.timestamp) }}] {{ log.message }}
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
import AMapLoader from '@amap/amap-jsapi-loader'

const systemStore = useSystemStore()
const fleetStore = useFleetStore()

const mapContainer = ref<HTMLDivElement>()
let map: any = null
let markers: Map<string, any> = new Map()
let polylines: Map<string, any> = new Map()

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
}

.log-entry.error {
  color: var(--danger);
}

.log-entry.warning {
  color: var(--warn);
}

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
