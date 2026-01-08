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
    aiConfidence: 0
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
    }
  }
})
