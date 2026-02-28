/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 */

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
export type LogLevel = 'info' | 'warning' | 'error' | 'success' | 'debug'
export type LogCategory =
  | 'system'      // 系统启动/关闭
  | 'mqtt'        // MQTT 连接/断开/消息
  | 'webrtc'      // WebRTC 信令/ICE
  | 'websocket'   // WebSocket 通信
  | 'control'     // 控制指令发送/接收
  | 'camera'      // 摄像头状态/帧率
  | 'telemetry'   // 遥测数据
  | 'vehicle'     // 车辆状态变化
  | 'ros'         // ROS2 Topic 状态
  | 'auth'        // 认证/权限

export interface LogEntry {
  id: number
  timestamp: Date
  message: string
  level: LogLevel
  category: LogCategory
  detail?: string   // 可选的详细信息 (折叠显示)
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
