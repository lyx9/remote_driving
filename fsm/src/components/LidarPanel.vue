<template>
  <div class="lidar-panel-wrapper">
    <!-- Collapse Button -->
    <button
      class="collapse-button"
      @click="toggleLidar"
      :title="'Close LiDAR View'"
    >
      <span class="collapse-icon">▼</span>
      <span class="collapse-label">LiDAR</span>
    </button>

    <!-- Panel Content -->
    <div class="lidar-panel">
      <div ref="containerRef" class="lidar-container"></div>
    </div>
  </div>
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
let obstacles: THREE.Mesh[] = []
let vehicle: THREE.Mesh | null = null

const toggleLidar = () => {
  systemStore.toggleUI('showLidar')
}

const initThree = () => {
  if (!containerRef.value) return

  const container = containerRef.value

  // Scene
  scene = new THREE.Scene()
  scene.fog = new THREE.FogExp2(0x000000, 0.03)

  // Camera - adjusted for better obstacle view
  camera = new THREE.PerspectiveCamera(
    65,
    container.clientWidth / container.clientHeight,
    0.1,
    150
  )
  camera.position.set(0, 8, 12)
  camera.lookAt(0, 0, -5)

  // Renderer
  renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true })
  renderer.setSize(container.clientWidth, container.clientHeight)
  container.appendChild(renderer.domElement)

  // Grid - larger for obstacle course
  grid = new THREE.GridHelper(80, 50, 0xff5722, 0x111111)
  scene.add(grid)

  // Vehicle (main car)
  const carGeometry = new THREE.BoxGeometry(1.2, 0.6, 2.5)
  const carMaterial = new THREE.MeshBasicMaterial({
    color: 0x00f2ff,
    wireframe: true
  })
  vehicle = new THREE.Mesh(carGeometry, carMaterial)
  vehicle.position.set(0, 0.3, 0)
  scene.add(vehicle)

  // Add obstacles (boxes representing other vehicles/objects)
  const obstaclePositions = [
    { x: -3, z: -8 },
    { x: 2, z: -15 },
    { x: -1, z: -22 },
    { x: 3, z: -30 },
    { x: -2.5, z: -38 }
  ]

  obstaclePositions.forEach(pos => {
    const obstacleGeometry = new THREE.BoxGeometry(1.5, 1, 2)
    const obstacleMaterial = new THREE.MeshBasicMaterial({
      color: 0xff5722,
      wireframe: true,
      transparent: true,
      opacity: 0.8
    })
    const obstacle = new THREE.Mesh(obstacleGeometry, obstacleMaterial)
    obstacle.position.set(pos.x, 0.5, pos.z)
    scene.add(obstacle)
    obstacles.push(obstacle)
  })

  // Path Line (avoidance trajectory)
  const lineGeometry = new THREE.BufferGeometry()
  const points = new Float32Array(150 * 3)
  lineGeometry.setAttribute('position', new THREE.BufferAttribute(points, 3))
  const lineMaterial = new THREE.LineBasicMaterial({
    color: 0x00ff88,
    linewidth: 2
  })
  pathLine = new THREE.Line(lineGeometry, lineMaterial)
  scene.add(pathLine)

  // Add ambient light for better visibility
  const ambientLight = new THREE.AmbientLight(0x404040, 0.5)
  scene.add(ambientLight)

  animate()
}

const animate = () => {
  if (!renderer || !scene || !camera || !grid || !pathLine || !vehicle) return

  animationId = requestAnimationFrame(animate)

  const t = Date.now() * 0.0008

  // Animate grid movement (forward motion)
  grid.position.z = (grid.position.z + 0.25) % 1.6

  // Animate vehicle weaving through obstacles
  const vehicleX = Math.sin(t * 1.2) * 2.5
  vehicle.position.x = vehicleX
  vehicle.rotation.y = Math.sin(t * 1.2) * 0.3

  // Animate obstacles moving towards camera (relative motion)
  obstacles.forEach((obstacle, index) => {
    obstacle.position.z += 0.25

    // Reset obstacle position when it passes the camera
    if (obstacle.position.z > 5) {
      obstacle.position.z = -40 - (index * 8)
    }

    // Slight rotation for visual effect
    obstacle.rotation.y += 0.01
  })

  // Animate avoidance path line
  const positions = pathLine.geometry.attributes.position.array as Float32Array

  for (let i = 0; i < 150; i++) {
    const z = -i * 0.4
    const progress = i / 150

    // Create S-curve path for obstacle avoidance
    let x = Math.sin(progress * Math.PI * 3 + t) * 2.5

    // Adjust path to avoid obstacles
    obstacles.forEach(obstacle => {
      const obstacleZ = obstacle.position.z
      if (Math.abs(z - obstacleZ) < 3) {
        const avoidDirection = obstacle.position.x > 0 ? -1 : 1
        x += avoidDirection * 1.5 * (1 - Math.abs(z - obstacleZ) / 3)
      }
    })

    positions[i * 3] = x
    positions[i * 3 + 1] = 0.2
    positions[i * 3 + 2] = z
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
  vehicle = null
  obstacles = []
}

onMounted(async () => {
  await nextTick()
  initThree()
})

onUnmounted(() => {
  cleanup()
})
</script>

<style scoped>
.lidar-panel-wrapper {
  width: 100%;
  background: #000;
  border-top: 1px solid var(--border);
}

.collapse-button {
  width: 100%;
  height: 32px;
  background: linear-gradient(135deg, rgba(255, 87, 34, 0.1) 0%, rgba(255, 87, 34, 0.05) 100%);
  border: none;
  border-bottom: 1px solid #222;
  color: var(--primary);
  font-size: 10px;
  font-weight: bold;
  letter-spacing: 1px;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  transition: all 0.2s ease;
  position: relative;
  z-index: 10;
}

.collapse-button:hover {
  background: linear-gradient(135deg, rgba(255, 87, 34, 0.15) 0%, rgba(255, 87, 34, 0.08) 100%);
  color: #ff6b3d;
}

.collapse-button:active {
  background: linear-gradient(135deg, rgba(255, 87, 34, 0.2) 0%, rgba(255, 87, 34, 0.1) 100%);
}

.collapse-icon {
  font-size: 12px;
  transition: transform 0.3s ease;
}

.collapse-label {
  font-family: 'Courier New', monospace;
}

.lidar-panel {
  height: 400px;
  position: relative;
  background: #000;
  overflow: hidden;
}

.lidar-container {
  width: 100%;
  height: 100%;
}
</style>
