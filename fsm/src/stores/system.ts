/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 */

import { defineStore } from 'pinia'
import type { LogEntry, RecordingSession } from '@/types'

export const useSystemStore = defineStore('system', {
  state: () => ({
    logs: [] as LogEntry[],
    recording: {
      isRecording: false,
      logs: []
    } as RecordingSession,
    ui: {
      showLeftSidebar: true,
      showRightSidebar: true,
      showAIBar: true,
      showLidar: true,
      showPIP: true
    },
    aiConfidence: 0,

    // Video transmission parameters
    videoTransmission: {
      bandwidth: 0,        // Current bandwidth in Mbps
      latency: 0,          // Current latency in ms
      frameRate: 30,       // Target frame rate (10-60 FPS)
      compression: 75,     // JPEG compression quality (1-100)
      resolution: '1080p' as '720p' | '1080p' | '4K'
    }
  }),

  actions: {
    addLog(message: string, level: 'info' | 'warning' | 'error' = 'info') {
      const entry: LogEntry = {
        timestamp: new Date(),
        message,
        level
      }
      this.logs.unshift(entry)

      // 限制日志数量
      if (this.logs.length > 100) {
        this.logs = this.logs.slice(0, 100)
      }

      // 如果正在录制，添加到录制日志
      if (this.recording.isRecording) {
        this.recording.logs.push(entry)
      }
    },

    startRecording(vehicleId: string) {
      this.recording = {
        isRecording: true,
        startTime: Date.now(),
        vehicleId,
        logs: [
          { timestamp: new Date(), message: '--- REC START ---' },
          { timestamp: new Date(), message: `ID: ${vehicleId}` },
          { timestamp: new Date(), message: `Time: ${new Date().toISOString()}` }
        ]
      }
      this.addLog('Black Box Recording Started (5 Channels)...', 'info')
    },

    stopRecording() {
      this.recording.isRecording = false
      this.addLog('Recording Stopped. Saving data...', 'info')
      this.downloadLogs()
    },

    downloadLogs() {
      const logText = this.recording.logs
        .map(log => `[${log.timestamp.toLocaleTimeString()}] ${log.message}`)
        .join('\n')

      const blob = new Blob([logText], { type: 'text/plain' })
      const url = URL.createObjectURL(blob)
      const a = document.createElement('a')
      a.href = url
      a.download = `${this.recording.vehicleId}_LOG_${Date.now()}.txt`
      document.body.appendChild(a)
      a.click()
      document.body.removeChild(a)
      URL.revokeObjectURL(url)
    },

    toggleUI(key: keyof typeof this.ui) {
      this.ui[key] = !this.ui[key]
    },

    updateAIConfidence(value: number) {
      this.aiConfidence = Math.max(0, Math.min(100, value))
    },

    // Video transmission actions
    updateVideoTransmission(params: {
      bandwidth?: number
      latency?: number
      frameRate?: number
      compression?: number
      resolution?: '720p' | '1080p' | '4K'
    }) {
      if (params.bandwidth !== undefined) {
        this.videoTransmission.bandwidth = params.bandwidth
      }
      if (params.latency !== undefined) {
        this.videoTransmission.latency = params.latency
      }
      if (params.frameRate !== undefined) {
        this.videoTransmission.frameRate = Math.max(10, Math.min(60, params.frameRate))
      }
      if (params.compression !== undefined) {
        this.videoTransmission.compression = Math.max(1, Math.min(100, params.compression))
      }
      if (params.resolution !== undefined) {
        this.videoTransmission.resolution = params.resolution
      }
    },

    setFrameRate(fps: number) {
      this.videoTransmission.frameRate = Math.max(10, Math.min(60, fps))
      this.addLog(`Video frame rate set to ${fps} FPS`)
    },

    setCompression(quality: number) {
      this.videoTransmission.compression = Math.max(1, Math.min(100, quality))
      this.addLog(`Video compression set to ${quality}%`)
    }
  }
})
