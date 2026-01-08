<template>
  <footer class="fleet-footer">
    <div
      v-for="(vehicle, index) in fleetStore.vehicles"
      :key="vehicle.id"
      :class="['fleet-card', { active: index === fleetStore.currentVehicleIndex }]"
      @click="handleSelectVehicle(index)"
    >
      <div class="card-header">
        <div class="vehicle-id">{{ vehicle.id }}</div>
        <div
          :class="['status-badge', vehicle.status.toLowerCase()]"
        >
          {{ vehicle.status }}
        </div>
      </div>

      <div class="card-footer">
        <span class="vehicle-type">{{ vehicle.type }}</span>
        <span class="profit">${{ vehicle.money.toFixed(2) }}</span>
      </div>
    </div>
  </footer>
</template>

<script setup lang="ts">
import { useFleetStore } from '@/stores/fleet'
import { useSystemStore } from '@/stores/system'
import { useCameraStore } from '@/stores/camera'

const fleetStore = useFleetStore()
const systemStore = useSystemStore()
const cameraStore = useCameraStore()

const handleSelectVehicle = (index: number) => {
  fleetStore.selectVehicle(index)
  systemStore.addLog(`Vehicle [${fleetStore.currentVehicle.id}] Selected.`)
  cameraStore.syncReplay()
}
</script>

<style scoped>
.fleet-footer {
  height: var(--footer-h);
  background: #05080e;
  border-top: 1px solid var(--border);
  display: flex;
  overflow-x: auto;
  padding: 5px;
  gap: 5px;
  flex-shrink: 0;
}

.fleet-card {
  min-width: 150px;
  border: 1px solid #333;
  background: #0a1018;
  padding: 8px;
  cursor: pointer;
  display: flex;
  flex-direction: column;
  justify-content: space-between;
  position: relative;
  transition: all 0.2s ease;
  border-radius: 2px;
}

.fleet-card:hover {
  border-color: #555;
  background: #0d1520;
}

.fleet-card.active {
  border-color: var(--primary);
  background: rgba(0, 242, 255, 0.05);
}

.fleet-card.active::after {
  content: '';
  position: absolute;
  top: 0;
  right: 0;
  width: 0;
  height: 0;
  border-top: 10px solid var(--primary);
  border-left: 10px solid transparent;
}

.card-header {
  display: flex;
  flex-direction: column;
  gap: 4px;
}

.vehicle-id {
  font-weight: bold;
  color: #fff;
  font-size: 12px;
}

.status-badge {
  display: inline-block;
  padding: 2px 6px;
  border-radius: 2px;
  font-size: 9px;
  background: #222;
  align-self: flex-start;
}

.status-badge.active {
  background: var(--success);
  color: #000;
}

.status-badge.idle {
  background: #555;
  color: #aaa;
}

.status-badge.patrol {
  background: var(--warn);
  color: #000;
}

.card-footer {
  display: flex;
  justify-content: space-between;
  align-items: flex-end;
  margin-top: 8px;
}

.vehicle-type {
  font-size: 9px;
  color: #666;
}

.profit {
  color: var(--warn);
  font-weight: bold;
  font-size: 12px;
}
</style>
