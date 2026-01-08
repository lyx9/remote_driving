<template>
  <div class="video-wall">
    <!-- Main Camera -->
    <CameraSlot
      v-if="mainCamera"
      :camera="mainCamera"
      :is-collapsed="false"
    />

    <!-- PIP Cameras -->
    <CameraSlot
      v-for="camera in pipCameras"
      :key="camera.id"
      :camera="camera"
      :is-collapsed="!systemStore.ui.showPIP"
      @toggle-pip="systemStore.toggleUI('showPIP')"
    />
  </div>
</template>

<script setup lang="ts">
import { computed } from 'vue'
import { useCameraStore } from '@/stores/camera'
import { useSystemStore } from '@/stores/system'
import CameraSlot from './CameraSlot.vue'

const cameraStore = useCameraStore()
const systemStore = useSystemStore()

const mainCamera = computed(() => cameraStore.mainCamera)
const pipCameras = computed(() => cameraStore.pipCameras)
</script>

<style scoped>
.video-wall {
  flex: 1;
  position: relative;
  background: #000;
  min-height: 0;
  overflow: hidden;
}
</style>
