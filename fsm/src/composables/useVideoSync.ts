/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      Video Sync Composable
 * @description 多路视频同步控制模块
 */

import { onMounted, onUnmounted } from 'vue'
import { useCameraStore } from '@/stores/camera'
import { useSystemStore } from '@/stores/system'

/**
 * Video Sync Composable
 * Handles synchronization of multiple video feeds
 */
export function useVideoSync() {
  const cameraStore = useCameraStore()
  const systemStore = useSystemStore()

  let masterVideo: HTMLVideoElement | null = null
  const slaveVideos: HTMLVideoElement[] = []

  const initSyncListeners = () => {
    const masterCamera = cameraStore.cameras.find(c => c.id === cameraStore.masterCameraId)
    if (!masterCamera?.videoElement) return

    masterVideo = masterCamera.videoElement

    // Get all slave videos
    slaveVideos.length = 0
    cameraStore.cameras.forEach(cam => {
      if (cam.id !== cameraStore.masterCameraId && cam.videoElement && cam.isLoaded) {
        slaveVideos.push(cam.videoElement)
      }
    })

    // Master play event
    const handlePlay = () => {
      slaveVideos.forEach(v => {
        if (v.paused) v.play()
      })
      cameraStore.setAllSynced(true)
      systemStore.addLog('SYNC: All cameras playing')
    }

    // Master pause event
    const handlePause = () => {
      slaveVideos.forEach(v => {
        if (!v.paused) v.pause()
      })
      cameraStore.setAllSynced(false)
      systemStore.addLog('SYNC: All cameras paused')
    }

    // Master seeked event
    const handleSeeked = () => {
      const currentTime = masterVideo?.currentTime || 0
      slaveVideos.forEach(v => {
        v.currentTime = currentTime
      })
      systemStore.addLog(`SYNC: Time aligned to ${currentTime.toFixed(2)}s`)
    }

    // Add listeners
    masterVideo.addEventListener('play', handlePlay)
    masterVideo.addEventListener('pause', handlePause)
    masterVideo.addEventListener('seeked', handleSeeked)

    // Cleanup function
    return () => {
      masterVideo?.removeEventListener('play', handlePlay)
      masterVideo?.removeEventListener('pause', handlePause)
      masterVideo?.removeEventListener('seeked', handleSeeked)
    }
  }

  let cleanupFn: (() => void) | null = null

  onMounted(() => {
    setTimeout(() => {
      cleanupFn = initSyncListeners() || null
    }, 1000)
  })

  onUnmounted(() => {
    if (cleanupFn) {
      cleanupFn()
    }
  })

  return {
    initSyncListeners
  }
}
