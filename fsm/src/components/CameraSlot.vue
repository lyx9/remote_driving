<template>
  <div class="camera-slot" :class="[position, { collapsed: isCollapsed }]">
    <canvas :id="`c-${camera.id}`" ref="canvasRef"></canvas>
    <video
      :id="camera.id"
      ref="videoRef"
      loop
      muted
      playsinline
      @loadedmetadata="handleVideoLoaded"
    ></video>

    <div class="cam-ctrls">
      <div class="ctrl-left">
        <span class="cam-name">{{ camera.name }}</span>
        <span :class="['sync-badge', { active: camera.isSynced }]">
          {{ camera.position === 'main' ? 'MASTER' : 'SYNC' }}
        </span>
      </div>

      <div class="ctrl-right">
        <button
          v-if="camera.position === 'main'"
          class="btn-ui btn-play"
          @click="cameraStore.syncReplay()"
        >
          ↻ REPLAY ALL
        </button>
        <button
          v-if="camera.position === 'main'"
          class="btn-ui"
          @click="togglePlayPause"
        >
          {{ cameraStore.isPlaying ? 'PAUSE' : 'PLAY' }}
        </button>
        <button
          v-if="camera.position !== 'main'"
          class="btn-mini"
          @click="emit('toggle-pip')"
          title="Minimize"
        >
          _
        </button>
        <button class="btn-ui" @click="loadVideo">LOAD</button>
        <input
          :id="`file-${camera.id}`"
          ref="fileInputRef"
          type="file"
          hidden
          accept="video/*"
          @change="handleFileChange"
        />
      </div>
    </div>

    <!-- Crosshair for main camera -->
    <div v-if="camera.position === 'main'" class="crosshair"></div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { useCameraStore } from '@/stores/camera'
import { useSystemStore } from '@/stores/system'
import type { Camera } from '@/types'

interface Props {
  camera: Camera
  isCollapsed?: boolean
}

const props = defineProps<Props>()
const emit = defineEmits<{
  'toggle-pip': []
}>()

const cameraStore = useCameraStore()
const systemStore = useSystemStore()

const videoRef = ref<HTMLVideoElement>()
const canvasRef = ref<HTMLCanvasElement>()
const fileInputRef = ref<HTMLInputElement>()

const position = computed(() => {
  const pos = props.camera.position
  if (pos === 'main') return 'cam-main'

  const parts = pos.split('-')
  if (parts.length === 2) {
    return `cam-pip ${parts[0]} ${parts[1]}`
  }
  return 'cam-pip'
})

let noiseInterval: number | null = null

const drawNoise = () => {
  const canvas = canvasRef.value
  if (!canvas || canvas.style.display === 'none') return

  const ctx = canvas.getContext('2d')
  if (!ctx) return

  const w = canvas.parentElement?.offsetWidth || 640
  const h = canvas.parentElement?.offsetHeight || 480

  if (canvas.width !== w) {
    canvas.width = w
    canvas.height = h
  }

  ctx.fillStyle = '#080808'
  ctx.fillRect(0, 0, w, h)

  for (let i = 0; i < 200; i++) {
    ctx.fillStyle = Math.random() > 0.5 ? '#1a1a1a' : '#000'
    ctx.fillRect(Math.random() * w, Math.random() * h, 2, 2)
  }

  ctx.fillStyle = '#444'
  ctx.textAlign = 'center'
  ctx.font = '14px monospace'
  ctx.fillText('NO SIGNAL', w / 2, h / 2)
}

const handleVideoLoaded = () => {
  if (!videoRef.value || !canvasRef.value) return

  canvasRef.value.style.display = 'none'
  videoRef.value.style.display = 'block'
  videoRef.value.play()

  cameraStore.setCameraLoaded(props.camera.id, true)
  systemStore.addLog(`Loaded source to ${props.camera.id}`)
}

const loadVideo = () => {
  fileInputRef.value?.click()
}

const handleFileChange = (event: Event) => {
  const input = event.target as HTMLInputElement
  const file = input.files?.[0]
  if (!file || !videoRef.value || !canvasRef.value) return

  canvasRef.value.style.display = 'block'
  videoRef.value.style.display = 'none'

  const url = URL.createObjectURL(file)
  videoRef.value.src = url
  videoRef.value.muted = true
}

const togglePlayPause = () => {
  if (cameraStore.isPlaying) {
    cameraStore.pauseAll()
  } else {
    cameraStore.playAll()
  }
}

onMounted(() => {
  if (videoRef.value && canvasRef.value) {
    cameraStore.setCameraElement(props.camera.id, videoRef.value, canvasRef.value)
  }

  // Start noise animation
  noiseInterval = window.setInterval(drawNoise, 100)
  drawNoise()
})

onUnmounted(() => {
  if (noiseInterval) {
    clearInterval(noiseInterval)
  }
})
</script>

<style scoped>
.camera-slot {
  background: #050505;
  border: 1px solid #222;
  overflow: hidden;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  position: relative;
}

.camera-slot video,
.camera-slot canvas {
  position: absolute;
  top: 0;
  left: 0;
  width: 100% !important;
  height: 100% !important;
  object-fit: fill;
  display: block;
}

.camera-slot video {
  display: none;
}

/* Main Camera */
.cam-main {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  z-index: 1;
  border: none;
}

/* PIP Cameras */
.cam-pip {
  position: absolute;
  width: 24%;
  min-width: 240px;
  aspect-ratio: 4 / 3;
  z-index: 20;
  box-shadow: 0 4px 15px rgba(0, 0, 0, 0.8);
  border: 1px solid var(--primary);
  background: #000;
}

.cam-pip.top {
  top: 10px;
}

.cam-pip.bottom {
  bottom: 10px;
}

.cam-pip.left {
  left: 10px;
}

.cam-pip.right {
  right: 10px;
}

/* Collapse animations */
.cam-pip.top.collapsed {
  transform: scale(0.8);
  opacity: 0;
  pointer-events: none;
  top: -100px;
}

.cam-pip.bottom.collapsed {
  transform: scale(0.8);
  opacity: 0;
  pointer-events: none;
  bottom: -100px;
}

/* Controls */
.cam-ctrls {
  position: absolute;
  bottom: 0;
  left: 0;
  width: 100%;
  padding: 5px;
  background: rgba(0, 0, 0, 0.8);
  border-top: 1px solid var(--primary);
  display: flex;
  align-items: center;
  justify-content: space-between;
  transform: translateY(100%);
  transition: transform 0.2s;
  z-index: 20;
  gap: 5px;
}

.camera-slot:hover .cam-ctrls {
  transform: translateY(0);
}

.ctrl-left,
.ctrl-right {
  display: flex;
  align-items: center;
  gap: 5px;
  flex-wrap: wrap;
}

.cam-name {
  font-size: 9px;
  color: var(--primary);
  font-weight: bold;
}

.sync-badge {
  font-size: 9px;
  padding: 2px 4px;
  background: #333;
  color: #888;
  border-radius: 2px;
  transition: all 0.3s;
}

.sync-badge.active {
  background: var(--success);
  color: #000;
  font-weight: bold;
  box-shadow: 0 0 5px var(--success);
}

.crosshair {
  position: absolute;
  top: 50%;
  left: 50%;
  width: 20px;
  height: 20px;
  border: 1px solid rgba(255, 255, 255, 0.4);
  transform: translate(-50%, -50%);
  pointer-events: none;
}
</style>
