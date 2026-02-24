<!--
  FSM-Pilot V2.0 - RosBag Camera Viewer

  @description 显示 RosBag 中的相机画面
-->
<template>
  <div class="rosbag-camera-viewer">
    <div v-if="imageUrl" class="camera-container">
      <img
        :src="imageUrl"
        :alt="cameraName"
        class="camera-image"
        @error="handleImageError"
      />
      <div class="camera-overlay">
        <div class="camera-label">{{ cameraName }}</div>
        <div class="camera-info">
          <span v-if="imageSize">
            {{ imageSize }}
          </span>
          <span v-if="encoding" class="encoding-badge">
            {{ encoding }}
          </span>
          <span v-if="fps > 0" class="fps-badge">
            {{ fps.toFixed(1) }} FPS
          </span>
        </div>
      </div>
    </div>
    <div v-else class="camera-placeholder">
      <div class="placeholder-icon">📷</div>
      <div class="placeholder-text">
        {{ error || 'No camera data available' }}
      </div>
      <div v-if="topicName" class="placeholder-topic">
        Topic: {{ topicName }}
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, watch, onUnmounted, computed } from 'vue'
import type { RosImage, CompressedImage } from '@/services/rosbagDb3ParserOptimized'
import { convertImageToDataUrl } from '@/services/rosbagDb3ParserOptimized'

const props = defineProps<{
  cameraName: string
  topicName: string | null
  imageData: RosImage | CompressedImage | null
}>()

const imageUrl = ref<string | null>(null)
const error = ref<string | null>(null)
const lastUpdateTime = ref(0)
const frameCount = ref(0)
const fps = ref(0)

// 计算图像尺寸
const imageSize = computed(() => {
  if (!props.imageData) return null
  if ('width' in props.imageData && 'height' in props.imageData) {
    return `${props.imageData.width}×${props.imageData.height}`
  }
  return null
})

// 计算编码格式
const encoding = computed(() => {
  if (!props.imageData) return null
  if ('format' in props.imageData) {
    return props.imageData.format
  }
  if ('encoding' in props.imageData) {
    return props.imageData.encoding
  }
  return null
})

// 监听图像数据变化
watch(() => props.imageData, (newData) => {
  if (!newData) {
    if (imageUrl.value) {
      URL.revokeObjectURL(imageUrl.value)
    }
    imageUrl.value = null
    return
  }

  try {
    // 释放旧的 URL
    if (imageUrl.value) {
      URL.revokeObjectURL(imageUrl.value)
    }

    // 转换新图像
    const dataUrl = convertImageToDataUrl(newData)
    imageUrl.value = dataUrl
    error.value = null

    // 计算 FPS
    const now = Date.now()
    if (lastUpdateTime.value > 0) {
      const deltaTime = (now - lastUpdateTime.value) / 1000
      if (deltaTime > 0) {
        fps.value = 1 / deltaTime
      }
    }
    lastUpdateTime.value = now
    frameCount.value++
  } catch (e) {
    console.error('[RosBag Camera] Failed to convert image:', e)
    error.value = 'Image conversion failed'
  }
}, { immediate: true })

const handleImageError = () => {
  error.value = 'Failed to load image'
}

// 清理
onUnmounted(() => {
  if (imageUrl.value) {
    URL.revokeObjectURL(imageUrl.value)
  }
})
</script>

<style scoped>
.rosbag-camera-viewer {
  width: 100%;
  height: 100%;
  position: relative;
  background: #000;
  border-radius: 4px;
  overflow: hidden;
}

.camera-container {
  width: 100%;
  height: 100%;
  position: relative;
  display: flex;
  align-items: center;
  justify-content: center;
}

.camera-image {
  max-width: 100%;
  max-height: 100%;
  object-fit: contain;
  display: block;
}

.camera-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  background: linear-gradient(180deg, rgba(0,0,0,0.6) 0%, transparent 100%);
  padding: 8px 12px;
  pointer-events: none;
}

.camera-label {
  color: #fff;
  font-size: 14px;
  font-weight: 600;
  text-shadow: 0 1px 3px rgba(0,0,0,0.8);
  margin-bottom: 4px;
}

.camera-info {
  display: flex;
  gap: 8px;
  font-size: 11px;
  color: rgba(255,255,255,0.8);
}

.encoding-badge,
.fps-badge {
  background: rgba(0,242,255,0.2);
  padding: 2px 6px;
  border-radius: 3px;
  border: 1px solid rgba(0,242,255,0.3);
}

.camera-placeholder {
  width: 100%;
  height: 100%;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  color: #666;
  padding: 20px;
  text-align: center;
}

.placeholder-icon {
  font-size: 48px;
  margin-bottom: 12px;
  opacity: 0.5;
}

.placeholder-text {
  font-size: 14px;
  margin-bottom: 8px;
}

.placeholder-topic {
  font-size: 11px;
  color: #888;
  font-family: 'Courier New', monospace;
}
</style>
