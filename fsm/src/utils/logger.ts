/**
 * FSM-Pilot - 全局日志工具
 *
 * 所有服务/组件通过此模块写日志，统一接入 systemStore
 * 支持分类、级别、详情，并同步输出到浏览器控制台
 */

import type { LogLevel, LogCategory } from '@/types'

// 延迟导入 store，避免 Pinia 初始化顺序问题
function getStore() {
  const { useSystemStore } = require('@/stores/system')
  return useSystemStore()
}

const CONSOLE_COLORS: Record<LogLevel, string> = {
  info:    '#4fc3f7',
  success: '#69f0ae',
  warning: '#ffcc02',
  error:   '#ef5350',
  debug:   '#90a4ae',
}

const CATEGORY_ICONS: Record<LogCategory, string> = {
  system:    '⚙',
  mqtt:      '📡',
  webrtc:    '🔗',
  websocket: '🌐',
  control:   '🕹',
  camera:    '📷',
  telemetry: '📊',
  vehicle:   '🚗',
  ros:       '🤖',
  auth:      '🔐',
}

export function log(
  message: string,
  level: LogLevel = 'info',
  category: LogCategory = 'system',
  detail?: string
) {
  // 写入 store
  try {
    getStore().addLog(message, level, category, detail)
  } catch {
    // store 未初始化时静默失败
  }

  // 控制台输出
  const icon = CATEGORY_ICONS[category] || '•'
  const color = CONSOLE_COLORS[level] || '#fff'
  const prefix = `%c[${category.toUpperCase()}]`
  const style = `color:${color};font-weight:bold`

  if (level === 'error') {
    console.error(prefix, style, `${icon} ${message}`, detail || '')
  } else if (level === 'warning') {
    console.warn(prefix, style, `${icon} ${message}`, detail || '')
  } else if (level === 'debug') {
    console.debug(prefix, style, `${icon} ${message}`, detail || '')
  } else {
    console.log(prefix, style, `${icon} ${message}`, detail || '')
  }
}

// 快捷方法
export const logger = {
  info:    (msg: string, cat: LogCategory = 'system', detail?: string) => log(msg, 'info',    cat, detail),
  success: (msg: string, cat: LogCategory = 'system', detail?: string) => log(msg, 'success', cat, detail),
  warn:    (msg: string, cat: LogCategory = 'system', detail?: string) => log(msg, 'warning', cat, detail),
  error:   (msg: string, cat: LogCategory = 'system', detail?: string) => log(msg, 'error',   cat, detail),
  debug:   (msg: string, cat: LogCategory = 'system', detail?: string) => log(msg, 'debug',   cat, detail),

  // 分类快捷
  mqtt:      (msg: string, level: LogLevel = 'info', detail?: string) => log(msg, level, 'mqtt',      detail),
  webrtc:    (msg: string, level: LogLevel = 'info', detail?: string) => log(msg, level, 'webrtc',    detail),
  ws:        (msg: string, level: LogLevel = 'info', detail?: string) => log(msg, level, 'websocket', detail),
  control:   (msg: string, level: LogLevel = 'info', detail?: string) => log(msg, level, 'control',   detail),
  camera:    (msg: string, level: LogLevel = 'info', detail?: string) => log(msg, level, 'camera',    detail),
  telemetry: (msg: string, level: LogLevel = 'info', detail?: string) => log(msg, level, 'telemetry', detail),
  vehicle:   (msg: string, level: LogLevel = 'info', detail?: string) => log(msg, level, 'vehicle',   detail),
  ros:       (msg: string, level: LogLevel = 'info', detail?: string) => log(msg, level, 'ros',       detail),
  auth:      (msg: string, level: LogLevel = 'info', detail?: string) => log(msg, level, 'auth',      detail),
}

export default logger
