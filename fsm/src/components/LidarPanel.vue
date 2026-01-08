<template>
  <Transition name="slide-up">
    <div v-if="systemStore.ui.showLidar" class="lidar-panel">
      <div ref="containerRef" class="lidar-container"></div>
    </div>
  </Transition>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted, watch, nextTick } from 'vue'
import { useSystemStore } from '@/stores/system'
import * as THREE from 'three'

const systemStore = useSystemStore()
const containerRef = ref<HTMLDivElement>()

let renderer: THREE.WebGLRenderer | null = null
let scene: THREE.Scene | null = null
let camera: THREE.PerspectiveCamera | null = null
let animationId: number | null = null
let grid: THREE.GridHelper | null = null
let pathLine: THREE.Line | null = null

const initThree = () => {
  if (!containerRef.value) return

  const container = containerRef.value

  // Scene
  scene = new THREE.Scene()
  scene.fog = new THREE.FogExp2(0x000000, 0.04)

  // Camera
  camera = new THREE.PerspectiveCamera(
    60,
    container.clientWidth / container.clientHeight,
    0.1,
    100
  )
  camera.position.set(0, 4, 6)
  camera.lookAt(0, 0, -4)

  // Renderer
  renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true })
  renderer.setSize(container.clientWidth, container.clientHeight)
  container.appendChild(renderer.domElement)

  // Grid
  grid = new THREE.GridHelper(60, 40, 0x00f2ff, 0x111111)
  scene.add(grid)

  // Car
  const carGeometry = new THREE.BoxGeometry(1, 0.5, 2)
  const carMaterial = new THREE.MeshBasicMaterial({
    color: 0x00f2ff,
    wireframe: true
  })
  const car = new THREE.Mesh(carGeometry, carMaterial)
  car.position.y = 0.25
  scene.add(car)

  // Path Line
  const lineGeometry = new THREE.BufferGeometry()
  const points = new Float32Array(100 * 3)
  lineGeometry.setAttribute('position', new THREE.BufferAttribute(points, 3))
  const lineMaterial = new THREE.LineBasicMaterial({ color: 0x00ff00 })
  pathLine = new THREE.Line(lineGeometry, lineMaterial)
  scene.add(pathLine)

  animate()
}

const animate = () => {
  if (!renderer || !scene || !camera || !grid || !pathLine) return

  animationId = requestAnimationFrame(animate)

  // Animate grid movement
  grid.position.z = (grid.position.z + 0.2) % 1.5

  // Animate path line
  const t = Date.now() * 0.003
  const positions = pathLine.geometry.attributes.position.array as Float32Array

  for (let i = 0; i < 100; i++) {
    positions[i * 3] = Math.sin(i * 0.1 + t) * (i * 0.05)
    positions[i * 3 + 1] = 0.1
    positions[i * 3 + 2] = -i * 0.5
  }

  pathLine.geometry.attributes.position.needsUpdate = true

  renderer.render(scene, camera)
}

const resizeThree = () => {
  if (!containerRef.value || !renderer || !camera) return

  const container = containerRef.value
  if (container.clientHeight === 0) return

  camera.aspect = container.clientWidth / container.clientHeight
  camera.updateProjectionMatrix()
  renderer.setSize(container.clientWidth, container.clientHeight)
}

const cleanup = () => {
  if (animationId) {
    cancelAnimationFrame(animationId)
  }

  if (renderer) {
    renderer.dispose()
    containerRef.value?.removeChild(renderer.domElement)
  }

  scene = null
  camera = null
  renderer = null
  grid = null
  pathLine = null
}

onMounted(async () => {
  await nextTick()
  initThree()
})

onUnmounted(() => {
  cleanup()
})

watch(
  () => systemStore.ui.showLidar,
  async (show) => {
    if (show) {
      await nextTick()
      setTimeout(resizeThree, 350)
    }
  }
)
</script>

<style scoped>
.lidar-panel {
  height: 180px;
  border-top: 1px solid var(--border);
  position: relative;
  background: #000;
  flex-shrink: 0;
  z-index: 30;
}

.lidar-container {
  width: 100%;
  height: 100%;
}

.slide-up-enter-active,
.slide-up-leave-active {
  transition: all 0.3s ease;
}

.slide-up-enter-from {
  transform: translateY(100%);
  opacity: 0;
}

.slide-up-leave-to {
  transform: translateY(100%);
  opacity: 0;
}
</style>
