import { defineStore } from 'pinia'
import type { Camera } from '@/types'

export const useCameraStore = defineStore('camera', {
  state: () => ({
    cameras: [
      { id: 'v1', name: 'FRONT_L', position: 'top-left' as const, isLoaded: false, isSynced: false },
      { id: 'v2', name: 'MAIN_CAM', position: 'main' as const, isLoaded: false, isSynced: false },
      { id: 'v3', name: 'FRONT_R', position: 'top-right' as const, isLoaded: false, isSynced: false },
      { id: 'v4', name: 'REAR_L', position: 'bottom-left' as const, isLoaded: false, isSynced: false },
      { id: 'v5', name: 'REAR_R', position: 'bottom-right' as const, isLoaded: false, isSynced: false }
    ] as Camera[],
    isPlaying: false,
    masterCameraId: 'v2'
  }),

  getters: {
    mainCamera: (state): Camera | undefined =>
      state.cameras.find(c => c.position === 'main'),
    pipCameras: (state): Camera[] =>
      state.cameras.filter(c => c.position !== 'main'),
    loadedCameras: (state): Camera[] =>
      state.cameras.filter(c => c.isLoaded)
  },

  actions: {
    setCameraElement(id: string, videoElement: HTMLVideoElement, canvasElement: HTMLCanvasElement) {
      const camera = this.cameras.find(c => c.id === id)
      if (camera) {
        camera.videoElement = videoElement
        camera.canvasElement = canvasElement
      }
    },

    setCameraLoaded(id: string, loaded: boolean) {
      const camera = this.cameras.find(c => c.id === id)
      if (camera) {
        camera.isLoaded = loaded
      }
    },

    setSyncStatus(id: string, synced: boolean) {
      const camera = this.cameras.find(c => c.id === id)
      if (camera) {
        camera.isSynced = synced
      }
    },

    setAllSynced(synced: boolean) {
      this.cameras.forEach(c => {
        c.isSynced = synced
      })
      this.isPlaying = synced
    },

    playAll() {
      this.cameras.forEach(cam => {
        if (cam.videoElement && cam.isLoaded) {
          cam.videoElement.play()
        }
      })
      this.setAllSynced(true)
    },

    pauseAll() {
      this.cameras.forEach(cam => {
        if (cam.videoElement && cam.isLoaded) {
          cam.videoElement.pause()
        }
      })
      this.setAllSynced(false)
    },

    syncReplay() {
      this.cameras.forEach(cam => {
        if (cam.videoElement && cam.isLoaded) {
          cam.videoElement.currentTime = 0
          cam.videoElement.play()
        }
      })
      this.setAllSynced(true)
    }
  }
})
