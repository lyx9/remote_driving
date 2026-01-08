<template>
  <div class="app-container">
    <Header />
    <AIBar />

    <div class="stage">
      <LeftSidebar />

      <div class="center-view">
        <VideoWall />
        <LidarPanel />
      </div>

      <RightSidebar />
    </div>

    <FleetFooter />
  </div>
</template>

<script setup lang="ts">
import { onMounted, onUnmounted } from 'vue'
import Header from './components/Header.vue'
import AIBar from './components/AIBar.vue'
import LeftSidebar from './components/LeftSidebar.vue'
import RightSidebar from './components/RightSidebar.vue'
import VideoWall from './components/VideoWall.vue'
import LidarPanel from './components/LidarPanel.vue'
import FleetFooter from './components/FleetFooter.vue'
import { useFleetStore } from './stores/fleet'
import { useSystemStore } from './stores/system'
import { useVideoSync } from './composables/useVideoSync'

const fleetStore = useFleetStore()
const systemStore = useSystemStore()

// Initialize video sync
useVideoSync()

let telemetryInterval: number | null = null

const updateTelemetry = () => {
  const vehicle = fleetStore.currentVehicle

  if (vehicle.status === 'ACTIVE') {
    // Update speed
    const speed = Math.floor(30 + (Math.random() - 0.5) * 10)
    fleetStore.updateVehicleSpeed(speed)

    // Update profit randomly
    if (Math.random() > 0.8) {
      fleetStore.addProfit(0.05)
    }

    // Update AI confidence
    const confidence = 80 + Math.random() * 20
    systemStore.updateAIConfidence(confidence)

    // Update steering
    const steering = (Math.random() - 0.5) * 20
    fleetStore.updateVehicleSteering(steering)
  } else {
    fleetStore.updateVehicleSpeed(0)
    fleetStore.updateVehicleSteering(0)
  }
}

onMounted(() => {
  systemStore.addLog('System Ready. Remote Driving Platform initialized.')
  telemetryInterval = window.setInterval(updateTelemetry, 500)
})

onUnmounted(() => {
  if (telemetryInterval) {
    clearInterval(telemetryInterval)
  }
})
</script>

<style scoped>
.app-container {
  display: flex;
  flex-direction: column;
  height: 100%;
  width: 100%;
}

.stage {
  flex: 1;
  display: flex;
  overflow: hidden;
  position: relative;
  min-height: 0;
}

.center-view {
  flex: 1;
  display: flex;
  flex-direction: column;
  background: #000;
  min-width: 0;
  position: relative;
}
</style>
