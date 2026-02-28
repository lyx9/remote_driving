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
import type { LogEntry, LogLevel, LogCategory, RecordingSession } from '@/types'

let _logIdCounter = 0

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

    // 日志过滤状态
    logFilter: {
      level: null as LogLevel | null,
      category: null as LogCategory | null,
    },

    // Video transmission parameters
    videoTransmission: {
      bandwidth: 0,
      latency: 0,
      frameRate: 30,
      compression: 75,
      resolution: '1080p' as '720p' | '1080p' | '4K'
    }
  }),

  getters: {
    filteredLogs(state) {
      return state.logs.filter(log => {
        if (state.logFilter.level && log.level !== state.logFilter.level) return false
        if (state.logFilter.category && log.category !== state.logFilter.category) return false
        return true
      })
    },
    logStats(state) {
      const stats: Record<string, number> = {
        total: state.logs.length,
        error: 0, warning: 0, info: 0, success: 0, debug: 0,
        mqtt: 0, webrtc: 0, websocket: 0, control: 0,
        camera: 0, telemetry: 0, vehicle: 0, ros: 0, system: 0, auth: 0,
      }
      for (const log of state.logs) {
        stats[log.level] = (stats[log.level] || 0) + 1
        stats[log.category] = (stats[log.category] || 0) + 1
      }
      return stats
    }
  },

  actions: {
    addLog(
      message: string,
      level: LogLevel = 'info',
      category: LogCategory = 'system',
      detail?: string
    ) {
      const entry: LogEntry = {
        id: ++_logIdCounter,
        timestamp: new Date(),
        message,
        level,
        category,
        detail,
      }
      this.logs.unshift(entry)
      if (this.logs.length > 500) {
        this.logs = this.logs.slice(0, 500)
      }
      if (this.recording.isRecording) {
        this.recording.logs.push(entry)
      }
    },

    setLogFilter(level: LogLevel | null, category: LogCategory | null) {
      this.logFilter.level = level
      this.logFilter.category = category
    },

    clearLogs() {
      this.logs = []
    },

    startRecording(vehicleId: string) {
      this.recording = {
        isRecording: true,
        startTime: Date.now(),
        vehicleId,
        logs: []
      }
      this.addLog('Black Box Recording Started (5 Channels)...', 'info', 'system')
    },

    stopRecording() {
      this.recording.isRecording = false
      this.addLog('Recording Stopped. Saving data...', 'info', 'system')
      this.downloadLogs()
    },

    downloadLogs() {
      const logText = this.recording.logs
        .map(log => `[${log.timestamp.toLocaleTimeString()}][${log.category}][${log.level}] ${log.message}`)
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

    updateVideoTransmission(params: {
      bandwidth?: number
      latency?: number
      frameRate?: number
      compression?: number
      resolution?: '720p' | '1080p' | '4K'
    }) {
      if (params.bandwidth !== undefined) this.videoTransmission.bandwidth = params.bandwidth
      if (params.latency !== undefined) this.videoTransmission.latency = params.latency
      if (params.frameRate !== undefined) this.videoTransmission.frameRate = Math.max(10, Math.min(60, params.frameRate))
      if (params.compression !== undefined) this.videoTransmission.compression = Math.max(1, Math.min(100, params.compression))
      if (params.resolution !== undefined) this.videoTransmission.resolution = params.resolution
    },

    setFrameRate(fps: number) {
      this.videoTransmission.frameRate = Math.max(10, Math.min(60, fps))
      this.addLog(`Video frame rate set to ${fps} FPS`, 'info', 'system')
    },

    setCompression(quality: number) {
      this.videoTransmission.compression = Math.max(1, Math.min(100, quality))
      this.addLog(`Video compression set to ${quality}%`, 'info', 'system')
    }
  }
})
