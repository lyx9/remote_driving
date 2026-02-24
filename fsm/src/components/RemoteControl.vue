<!--
  Guardian Mobility - Remote Control Page

  @description Remote control main page with AI real-time driving suggestions
-->
<template>
  <div class="remote-control-page">
    <NavBar />
    <Header />
    <AIDrivingSuggestions />

    <div class="stage">
      <LeftSidebar />

      <div class="center-view">
        <VideoWall />
        <div v-if="systemStore.ui.showLidar" class="bottom-panels">
          <LidarPanel />
        </div>
      </div>

      <RightSidebar />
    </div>

    <FleetFooter />
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'
import NavBar from './NavBar.vue'
import Header from './Header.vue'
import AIDrivingSuggestions from './AIDrivingSuggestions.vue'
import LeftSidebar from './LeftSidebar.vue'
import RightSidebar from './RightSidebar.vue'
import VideoWall from './VideoWall.vue'
import LidarPanel from './LidarPanel.vue'
import FleetFooter from './FleetFooter.vue'
import { useFleetStore } from '@/stores/fleet'
import { useSystemStore } from '@/stores/system'
import { useVideoSync } from '@/composables/useVideoSync'

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
.remote-control-page {
  display: flex;
  flex-direction: column;
  height: 100vh;
  width: 100%;
  position: relative;
  background: #0a0a12;
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

.bottom-panels {
  display: flex;
  padding: 8px;
  background: #0a0a12;
  min-height: 400px;
}
</style>
