// 车辆类型定义
export interface Vehicle {
  id: string
  type: 'ROBO-TAXI' | 'LOGISTICS' | 'SECURITY'
  status: 'ACTIVE' | 'IDLE' | 'PATROL' | 'OFFLINE'
  money: number
  mode: 'ECO' | 'SPORT' | 'AI-HUNT'
  location: [number, number]
  path: Array<[number, number]>
  speed?: number
  gear?: 'P' | 'R' | 'N' | 'D'
  steering?: number
}

// 摄像头类型
export interface Camera {
  id: string
  name: string
  position: 'main' | 'top-left' | 'top-right' | 'bottom-left' | 'bottom-right'
  videoElement?: HTMLVideoElement
  canvasElement?: HTMLCanvasElement
  isLoaded: boolean
  isSynced: boolean
}

// 系统日志
export interface LogEntry {
  timestamp: Date
  message: string
  level?: 'info' | 'warning' | 'error'
}

// 录制配置
export interface RecordingSession {
  isRecording: boolean
  startTime?: number
  vehicleId?: string
  logs: LogEntry[]
}

// UI状态
export interface UIState {
  showLeftSidebar: boolean
  showRightSidebar: boolean
  showAIBar: boolean
  showLidar: boolean
  showPIP: boolean
}
