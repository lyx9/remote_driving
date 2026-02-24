/**
 * FSM-Pilot V2.0 - Logger Utility
 *
 * @description 简单的日志工具
 */

export enum LogLevel {
  DEBUG = 0,
  INFO = 1,
  WARN = 2,
  ERROR = 3
}

class Logger {
  private level: LogLevel = LogLevel.INFO

  setLevel(level: LogLevel) {
    this.level = level
  }

  debug(...args: any[]) {
    if (this.level <= LogLevel.DEBUG) {
      console.log('[DEBUG]', new Date().toISOString(), ...args)
    }
  }

  info(...args: any[]) {
    if (this.level <= LogLevel.INFO) {
      console.log('[INFO]', new Date().toISOString(), ...args)
    }
  }

  warn(...args: any[]) {
    if (this.level <= LogLevel.WARN) {
      console.warn('[WARN]', new Date().toISOString(), ...args)
    }
  }

  error(...args: any[]) {
    if (this.level <= LogLevel.ERROR) {
      console.error('[ERROR]', new Date().toISOString(), ...args)
    }
  }
}

export const logger = new Logger()
