<template>
  <Transition name="slide-left">
    <aside v-if="systemStore.ui.showLeftSidebar" class="sidebar left">
      <div class="sidebar-inner">
        <!-- Map Card -->
        <div class="card map-card">
          <div class="card-header">LIVE NAVIGATION</div>
          <div class="card-body">
            <div ref="mapContainer" class="map-container"></div>
          </div>
        </div>

        <!-- Log Card -->
        <div class="card log-card">
          <div class="card-header">SYSTEM LOG</div>
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
import { ref, onMounted, watch, nextTick } from 'vue'
import { useSystemStore } from '@/stores/system'
import { useFleetStore } from '@/stores/fleet'
import L from 'leaflet'
import type { Map, Marker, Polyline } from 'leaflet'

const systemStore = useSystemStore()
const fleetStore = useFleetStore()

const mapContainer = ref<HTMLDivElement>()
let map: Map | null = null
let mapMarker: Marker | null = null
let mapPolyline: Polyline | null = null

const formatTime = (date: Date) => {
  return date.toLocaleTimeString()
}

const initMap = () => {
  if (!mapContainer.value) return

  map = L.map(mapContainer.value, {
    zoomControl: false,
    attributionControl: false
  }).setView([31.23, 121.47], 14)

  L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png', {
    maxZoom: 19
  }).addTo(map)

  updateMapForCurrentVehicle()
}

const updateMapForCurrentVehicle = () => {
  if (!map) return

  const vehicle = fleetStore.currentVehicle

  // Remove existing layers
  if (mapMarker) map.removeLayer(mapMarker)
  if (mapPolyline) map.removeLayer(mapPolyline)

  // Add path
  mapPolyline = L.polyline(vehicle.path, {
    color: '#00f2ff',
    weight: 3,
    opacity: 0.7
  }).addTo(map)

  // Add marker
  mapMarker = L.circleMarker(vehicle.location, {
    radius: 6,
    color: '#fff',
    fillColor: '#00f2ff',
    fillOpacity: 1
  }).addTo(map)

  map.flyTo(vehicle.location, 14)
}

onMounted(async () => {
  await nextTick()
  initMap()
})

watch(
  () => fleetStore.currentVehicleIndex,
  () => {
    updateMapForCurrentVehicle()
  }
)

watch(
  () => systemStore.ui.showLeftSidebar,
  async (show) => {
    if (show) {
      await nextTick()
      setTimeout(() => {
        map?.invalidateSize()
      }, 350)
    }
  }
)
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
  flex: 2;
}

.log-card {
  flex: 1;
}

.card-header {
  padding: 6px 10px;
  background: rgba(0, 242, 255, 0.05);
  color: var(--primary);
  font-weight: bold;
  border-bottom: 1px solid #222;
  font-size: 10px;
  letter-spacing: 0.5px;
}

.card-body {
  flex: 1;
  overflow: hidden;
  position: relative;
  background: var(--bg-panel);
  padding: 8px;
}

.map-container {
  width: 100%;
  height: 100%;
}

.log-box {
  height: 100%;
  overflow-y: auto;
  font-family: 'Consolas', monospace;
  color: #777;
  font-size: 10px;
}

.log-entry {
  border-bottom: 1px solid #1a1a1a;
  padding: 3px 2px;
  line-height: 1.4;
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
</style>
