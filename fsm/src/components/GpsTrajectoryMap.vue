<!--
  FSM-Pilot V2.0 - Remote Driving System

  @project     FSM-Pilot Remote Driving Platform
  @author      Li Yixiang
  @institution City University of Hong Kong
  @copyright   2025 City University of Hong Kong. All rights reserved.
  @license     Proprietary

  @component   GpsTrajectoryMap
  @description GPS 轨迹地图可视化组件，使用 Leaflet 显示车辆行驶轨迹
-->
<template>
  <div class="gps-trajectory-map" ref="containerRef">
    <!-- 工具栏 -->
    <div class="map-toolbar">
      <div class="toolbar-left">
        <span class="map-title">GPS Trajectory</span>
        <span class="point-count">{{ trajectory.length }} points</span>
      </div>
      <div class="toolbar-right">
        <button class="tool-btn" @click="fitBounds" title="Fit to Track">
          🎯
        </button>
        <button class="tool-btn" @click="toggleFollow" :class="{ active: isFollowing }" title="Follow Vehicle">
          🚗
        </button>
        <select v-model="mapStyle" class="style-select" @change="updateMapStyle">
          <option value="dark">Dark</option>
          <option value="satellite">Satellite</option>
          <option value="street">Street</option>
        </select>
      </div>
    </div>

    <!-- 地图容器 -->
    <div class="map-container" ref="mapContainerRef"></div>

    <!-- 信息面板 -->
    <div class="info-panel" v-if="currentPosition">
      <div class="info-row">
        <span class="info-label">Latitude</span>
        <span class="info-value">{{ currentPosition.lat.toFixed(6) }}°</span>
      </div>
      <div class="info-row">
        <span class="info-label">Longitude</span>
        <span class="info-value">{{ currentPosition.lng.toFixed(6) }}°</span>
      </div>
      <div class="info-row">
        <span class="info-label">Altitude</span>
        <span class="info-value">{{ currentPosition.alt.toFixed(1) }}m</span>
      </div>
      <div class="info-row" v-if="totalDistance > 0">
        <span class="info-label">Distance</span>
        <span class="info-value">{{ (totalDistance / 1000).toFixed(2) }}km</span>
      </div>
    </div>

    <!-- 时间轴指示器 -->
    <div class="timeline-indicator" v-if="trajectory.length > 0">
      <div class="timeline-bar">
        <div class="timeline-progress" :style="{ width: `${progress * 100}%` }"></div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, shallowRef, onMounted, onUnmounted, watch, computed } from 'vue'
import L from 'leaflet'
import 'leaflet/dist/leaflet.css'

// Props
interface TrajectoryPoint {
  lat: number
  lng: number
  alt: number
  timestamp: number
}

interface Props {
  trajectory?: TrajectoryPoint[]
  currentTime?: number
}

const props = withDefaults(defineProps<Props>(), {
  trajectory: () => [],
  currentTime: 0
})

// Refs
const containerRef = ref<HTMLElement | null>(null)
const mapContainerRef = ref<HTMLElement | null>(null)
const map = shallowRef<L.Map | null>(null)
const trajectoryLine = shallowRef<L.Polyline | null>(null)
const vehicleMarker = shallowRef<L.Marker | null>(null)
const startMarker = shallowRef<L.Marker | null>(null)
const endMarker = shallowRef<L.Marker | null>(null)

// 状态
const mapStyle = ref<'dark' | 'satellite' | 'street'>('dark')
const isFollowing = ref(true)
const currentPosition = ref<TrajectoryPoint | null>(null)

// 地图图层
let tileLayer: L.TileLayer | null = null

// 计算属性
const progress = computed(() => {
  if (props.trajectory.length < 2) return 0
  const startTime = props.trajectory[0].timestamp
  const endTime = props.trajectory[props.trajectory.length - 1].timestamp
  if (endTime === startTime) return 0
  return (props.currentTime - startTime) / (endTime - startTime)
})

const totalDistance = computed(() => {
  let distance = 0
  for (let i = 1; i < props.trajectory.length; i++) {
    const p1 = props.trajectory[i - 1]
    const p2 = props.trajectory[i]
    distance += haversineDistance(p1.lat, p1.lng, p2.lat, p2.lng)
  }
  return distance
})

// 计算两点之间的距离 (米)
function haversineDistance(lat1: number, lon1: number, lat2: number, lon2: number): number {
  const R = 6371000 // 地球半径 (米)
  const dLat = (lat2 - lat1) * Math.PI / 180
  const dLon = (lon2 - lon1) * Math.PI / 180
  const a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
    Math.sin(dLon / 2) * Math.sin(dLon / 2)
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a))
  return R * c
}

// 地图样式 URL
const tileUrls = {
  dark: 'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',
  satellite: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  street: 'https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png'
}

// 初始化地图
const initMap = () => {
  if (!mapContainerRef.value) return

  // 创建地图
  map.value = L.map(mapContainerRef.value, {
    center: [31.2304, 121.4737], // 上海默认中心
    zoom: 16,
    zoomControl: false
  })

  // 添加缩放控件到右下角
  L.control.zoom({ position: 'bottomright' }).addTo(map.value)

  // 添加图层
  updateMapStyle()

  // 创建车辆图标
  const vehicleIcon = L.divIcon({
    className: 'vehicle-icon',
    html: `<div class="vehicle-marker">
      <div class="vehicle-arrow"></div>
    </div>`,
    iconSize: [30, 30],
    iconAnchor: [15, 15]
  })

  vehicleMarker.value = L.marker([0, 0], { icon: vehicleIcon })
}

// 更新地图样式
const updateMapStyle = () => {
  if (!map.value) return

  if (tileLayer) {
    map.value.removeLayer(tileLayer)
  }

  tileLayer = L.tileLayer(tileUrls[mapStyle.value], {
    attribution: '&copy; OpenStreetMap contributors',
    maxZoom: 19
  })

  tileLayer.addTo(map.value)
}

// 更新轨迹
const updateTrajectory = () => {
  if (!map.value) return

  // 移除旧的轨迹线
  if (trajectoryLine.value) {
    map.value.removeLayer(trajectoryLine.value)
  }
  if (startMarker.value) {
    map.value.removeLayer(startMarker.value)
  }
  if (endMarker.value) {
    map.value.removeLayer(endMarker.value)
  }

  if (props.trajectory.length < 2) return

  // 创建轨迹坐标
  const latlngs = props.trajectory.map(p => [p.lat, p.lng] as [number, number])

  // 绘制轨迹线
  trajectoryLine.value = L.polyline(latlngs, {
    color: '#ff5722',
    weight: 3,
    opacity: 0.8,
    smoothFactor: 1
  }).addTo(map.value)

  // 添加起点标记
  const startIcon = L.divIcon({
    className: 'start-icon',
    html: '<div class="start-marker">S</div>',
    iconSize: [24, 24],
    iconAnchor: [12, 12]
  })
  startMarker.value = L.marker(latlngs[0], { icon: startIcon }).addTo(map.value)

  // 添加终点标记
  const endIcon = L.divIcon({
    className: 'end-icon',
    html: '<div class="end-marker">E</div>',
    iconSize: [24, 24],
    iconAnchor: [12, 12]
  })
  endMarker.value = L.marker(latlngs[latlngs.length - 1], { icon: endIcon }).addTo(map.value)

  // 调整视图
  fitBounds()
}

// 更新当前位置
const updateCurrentPosition = () => {
  if (!map.value || !vehicleMarker.value || props.trajectory.length === 0) return

  // 找到当前时间对应的位置
  let targetPoint = props.trajectory[0]

  for (let i = 0; i < props.trajectory.length; i++) {
    if (props.trajectory[i].timestamp <= props.currentTime) {
      targetPoint = props.trajectory[i]
    } else {
      break
    }
  }

  currentPosition.value = targetPoint

  // 更新车辆位置
  if (!map.value.hasLayer(vehicleMarker.value)) {
    vehicleMarker.value.addTo(map.value)
  }
  vehicleMarker.value.setLatLng([targetPoint.lat, targetPoint.lng])

  // 如果跟随模式，移动地图中心
  if (isFollowing.value) {
    map.value.panTo([targetPoint.lat, targetPoint.lng], { animate: true, duration: 0.3 })
  }
}

// 调整视图以显示完整轨迹
const fitBounds = () => {
  if (!map.value || !trajectoryLine.value) return
  map.value.fitBounds(trajectoryLine.value.getBounds(), { padding: [50, 50] })
}

// 切换跟随模式
const toggleFollow = () => {
  isFollowing.value = !isFollowing.value
}

// 监听轨迹变化
watch(() => props.trajectory, () => {
  updateTrajectory()
}, { deep: true })

// 监听当前时间变化
watch(() => props.currentTime, () => {
  updateCurrentPosition()
})

// 生命周期
onMounted(() => {
  initMap()
  if (props.trajectory.length > 0) {
    updateTrajectory()
    updateCurrentPosition()
  }
})

onUnmounted(() => {
  if (map.value) {
    map.value.remove()
    map.value = null
  }
})

// 暴露方法
defineExpose({
  fitBounds,
  updateTrajectory
})
</script>

<style scoped>
.gps-trajectory-map {
  position: relative;
  width: 100%;
  height: 100%;
  display: flex;
  flex-direction: column;
  background: #0a0a12;
}

/* 工具栏 */
.map-toolbar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 8px 12px;
  background: #14141f;
  border-bottom: 1px solid #2a2a3a;
  z-index: 1000;
}

.toolbar-left {
  display: flex;
  align-items: center;
  gap: 12px;
}

.map-title {
  font-size: 13px;
  font-weight: bold;
  color: var(--primary, #ff5722);
}

.point-count {
  font-size: 11px;
  color: #888;
  padding: 2px 8px;
  background: rgba(255, 87, 34, 0.1);
  border-radius: 10px;
}

.toolbar-right {
  display: flex;
  align-items: center;
  gap: 8px;
}

.tool-btn {
  width: 32px;
  height: 32px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: #1a1a2e;
  border: 1px solid #333;
  color: #888;
  font-size: 14px;
  cursor: pointer;
  transition: all 0.2s;
  border-radius: 4px;
}

.tool-btn:hover {
  background: #252540;
  color: #fff;
}

.tool-btn.active {
  background: rgba(255, 87, 34, 0.2);
  border-color: var(--primary, #ff5722);
  color: var(--primary, #ff5722);
}

.style-select {
  padding: 4px 8px;
  background: #1a1a2e;
  border: 1px solid #333;
  color: #fff;
  font-size: 11px;
  cursor: pointer;
  border-radius: 4px;
}

/* 地图容器 */
.map-container {
  flex: 1;
  position: relative;
  z-index: 1;
}

/* 信息面板 */
.info-panel {
  position: absolute;
  top: 60px;
  left: 12px;
  background: rgba(20, 20, 31, 0.95);
  border: 1px solid #2a2a3a;
  padding: 10px 14px;
  border-radius: 4px;
  z-index: 1000;
  min-width: 150px;
}

.info-row {
  display: flex;
  justify-content: space-between;
  gap: 16px;
  font-size: 11px;
  margin-bottom: 4px;
}

.info-row:last-child {
  margin-bottom: 0;
}

.info-label {
  color: #666;
}

.info-value {
  color: #fff;
  font-family: 'JetBrains Mono', monospace;
}

/* 时间轴指示器 */
.timeline-indicator {
  position: absolute;
  bottom: 12px;
  left: 12px;
  right: 12px;
  z-index: 1000;
}

.timeline-bar {
  height: 4px;
  background: rgba(255, 255, 255, 0.2);
  border-radius: 2px;
  overflow: hidden;
}

.timeline-progress {
  height: 100%;
  background: var(--primary, #ff5722);
  transition: width 0.1s;
}
</style>

<style>
/* 全局样式 - 车辆标记 */
.vehicle-marker {
  width: 30px;
  height: 30px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.vehicle-arrow {
  width: 0;
  height: 0;
  border-left: 8px solid transparent;
  border-right: 8px solid transparent;
  border-bottom: 16px solid #ff5722;
  filter: drop-shadow(0 0 4px #ff5722);
}

/* 起点标记 */
.start-marker {
  width: 24px;
  height: 24px;
  background: #00ff64;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  color: #000;
  font-weight: bold;
  font-size: 12px;
  box-shadow: 0 0 8px rgba(0, 255, 100, 0.6);
}

/* 终点标记 */
.end-marker {
  width: 24px;
  height: 24px;
  background: #ff6400;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  color: #000;
  font-weight: bold;
  font-size: 12px;
  box-shadow: 0 0 8px rgba(255, 100, 0, 0.6);
}

/* Leaflet 覆盖样式 */
.leaflet-container {
  background: #0a0a12;
}

.leaflet-control-zoom {
  border: 1px solid #333 !important;
}

.leaflet-control-zoom a {
  background: #1a1a2e !important;
  color: #888 !important;
  border-bottom-color: #333 !important;
}

.leaflet-control-zoom a:hover {
  background: #252540 !important;
  color: #fff !important;
}
</style>
