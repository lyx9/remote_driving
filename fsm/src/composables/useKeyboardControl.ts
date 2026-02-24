/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      Keyboard Control Composable
 * @description 键盘控制模块，支持WASD+箭头键控制远程车辆
 */

import { ref, onUnmounted, computed } from 'vue'

// 控制指令类型
export interface ControlCommand {
  timestamp: number
  sequence: number
  vehicle_id: string
  throttle: number
  brake: number
  steering: number
  gear: 'P' | 'R' | 'N' | 'D'
  turn_signal: -1 | 0 | 1
  hazard: boolean
  emergency_stop: boolean
  source: 'keyboard' | 'wheel' | 'auto'
  keys?: KeyboardControlState
}

// 键盘映射配置
export const KEY_MAPPINGS = {
  // 方向控制 (WASD + 箭头)
  FORWARD: ['KeyW', 'ArrowUp'] as string[],
  BACKWARD: ['KeyS', 'ArrowDown'] as string[],
  LEFT: ['KeyA', 'ArrowLeft'] as string[],
  RIGHT: ['KeyD', 'ArrowRight'] as string[],

  // 功能键
  BRAKE: ['Space'] as string[],
  EMERGENCY: ['KeyE'] as string[],
  GEAR_TOGGLE: ['KeyQ'] as string[],
  RESET: ['KeyR'] as string[],

  // 转向灯
  TURN_LEFT: ['KeyZ'] as string[],
  TURN_RIGHT: ['KeyC'] as string[],
  HAZARD: ['KeyX'] as string[],

  // 摄像头切换
  CAMERA_1: ['Digit1'] as string[],
  CAMERA_2: ['Digit2'] as string[],
  CAMERA_3: ['Digit3'] as string[],
  CAMERA_4: ['Digit4'] as string[],
  CAMERA_5: ['Digit5'] as string[],

  // 其他
  VEHICLE_SWITCH: ['Tab'] as string[],
  MAP_FULLSCREEN: ['KeyM'] as string[],
  HORN: ['KeyH'] as string[],
}

export interface KeyboardControlState {
  // 方向控制状态
  forward: boolean
  backward: boolean
  left: boolean
  right: boolean

  // 刹车
  braking: boolean

  // 紧急停车
  emergency: boolean

  // 转向灯
  turnLeft: boolean
  turnRight: boolean
  hazard: boolean

  // 喇叭
  horn: boolean
}

export interface KeyboardControlConfig {
  // 控制参数
  maxThrottle: number      // 最大油门 [0, 1]
  maxSteering: number      // 最大转向 [0, 1]
  throttleRamp: number     // 油门增长速率 (每秒)
  steeringRamp: number     // 转向增长速率 (每秒)
  brakeStrength: number    // 刹车强度 [0, 1]

  // 回正速率
  throttleDecay: number    // 油门衰减速率
  steeringDecay: number    // 转向回正速率

  // 档位序列
  gearSequence: ('P' | 'R' | 'N' | 'D')[]

  // 发送间隔
  sendInterval: number     // 指令发送间隔 (ms)
}

const DEFAULT_CONFIG: KeyboardControlConfig = {
  maxThrottle: 0.8,
  maxSteering: 1.0,
  throttleRamp: 2.0,        // 0.5秒达到最大
  steeringRamp: 3.0,        // 0.33秒达到最大
  brakeStrength: 0.9,
  throttleDecay: 3.0,       // 松开后0.33秒归零
  steeringDecay: 5.0,       // 松开后0.2秒归零
  gearSequence: ['P', 'R', 'N', 'D'],
  sendInterval: 50,         // 20Hz
}

export function useKeyboardControl(
  onCommand: (cmd: ControlCommand) => void,
  config: Partial<KeyboardControlConfig> = {}
) {
  const mergedConfig = { ...DEFAULT_CONFIG, ...config }

  // 键盘状态
  const keyState = ref<KeyboardControlState>({
    forward: false,
    backward: false,
    left: false,
    right: false,
    braking: false,
    emergency: false,
    turnLeft: false,
    turnRight: false,
    hazard: false,
    horn: false,
  })

  // 控制值 (平滑插值)
  const throttle = ref(0)
  const brake = ref(0)
  const steering = ref(0)
  const gearIndex = ref(3) // 默认D档
  const gear = computed(() => mergedConfig.gearSequence[gearIndex.value])

  // 序列号
  let sequence = 0
  let lastUpdateTime = Date.now()
  let sendTimer: ReturnType<typeof setInterval> | null = null

  // 是否启用
  const enabled = ref(false)
  const isActive = ref(false) // 是否有活动输入

  // 按键处理
  const handleKeyDown = (e: KeyboardEvent) => {
    if (!enabled.value) return
    if (e.repeat) return // 忽略按键重复

    const code = e.code

    // 方向控制
    if (KEY_MAPPINGS.FORWARD.includes(code)) {
      keyState.value.forward = true
      e.preventDefault()
    } else if (KEY_MAPPINGS.BACKWARD.includes(code)) {
      keyState.value.backward = true
      e.preventDefault()
    } else if (KEY_MAPPINGS.LEFT.includes(code)) {
      keyState.value.left = true
      e.preventDefault()
    } else if (KEY_MAPPINGS.RIGHT.includes(code)) {
      keyState.value.right = true
      e.preventDefault()
    }

    // 刹车
    else if (KEY_MAPPINGS.BRAKE.includes(code)) {
      keyState.value.braking = true
      e.preventDefault()
    }

    // 紧急停车
    else if (KEY_MAPPINGS.EMERGENCY.includes(code)) {
      keyState.value.emergency = !keyState.value.emergency
      e.preventDefault()
    }

    // 档位切换
    else if (KEY_MAPPINGS.GEAR_TOGGLE.includes(code)) {
      gearIndex.value = (gearIndex.value + 1) % mergedConfig.gearSequence.length
      e.preventDefault()
    }

    // 转向灯
    else if (KEY_MAPPINGS.TURN_LEFT.includes(code)) {
      keyState.value.turnLeft = !keyState.value.turnLeft
      keyState.value.turnRight = false
      e.preventDefault()
    } else if (KEY_MAPPINGS.TURN_RIGHT.includes(code)) {
      keyState.value.turnRight = !keyState.value.turnRight
      keyState.value.turnLeft = false
      e.preventDefault()
    } else if (KEY_MAPPINGS.HAZARD.includes(code)) {
      keyState.value.hazard = !keyState.value.hazard
      e.preventDefault()
    }

    // 喇叭
    else if (KEY_MAPPINGS.HORN.includes(code)) {
      keyState.value.horn = true
      e.preventDefault()
    }
  }

  const handleKeyUp = (e: KeyboardEvent) => {
    if (!enabled.value) return

    const code = e.code

    if (KEY_MAPPINGS.FORWARD.includes(code)) {
      keyState.value.forward = false
    } else if (KEY_MAPPINGS.BACKWARD.includes(code)) {
      keyState.value.backward = false
    } else if (KEY_MAPPINGS.LEFT.includes(code)) {
      keyState.value.left = false
    } else if (KEY_MAPPINGS.RIGHT.includes(code)) {
      keyState.value.right = false
    } else if (KEY_MAPPINGS.BRAKE.includes(code)) {
      keyState.value.braking = false
    } else if (KEY_MAPPINGS.HORN.includes(code)) {
      keyState.value.horn = false
    }
  }

  // 更新控制值 (平滑插值)
  const updateControlValues = () => {
    const now = Date.now()
    const dt = (now - lastUpdateTime) / 1000 // 秒
    lastUpdateTime = now

    // 油门/刹车控制
    if (keyState.value.braking) {
      brake.value = mergedConfig.brakeStrength
      throttle.value = Math.max(0, throttle.value - mergedConfig.throttleDecay * dt)
    } else if (keyState.value.forward) {
      throttle.value = Math.min(
        mergedConfig.maxThrottle,
        throttle.value + mergedConfig.throttleRamp * dt
      )
      brake.value = 0
    } else if (keyState.value.backward) {
      // 倒车模式
      if (gear.value === 'R') {
        throttle.value = Math.min(
          mergedConfig.maxThrottle * 0.5, // 倒车限速
          throttle.value + mergedConfig.throttleRamp * dt
        )
      } else {
        // 非R档，刹车
        brake.value = mergedConfig.brakeStrength * 0.5
      }
    } else {
      // 松开后衰减
      throttle.value = Math.max(0, throttle.value - mergedConfig.throttleDecay * dt)
      brake.value = Math.max(0, brake.value - mergedConfig.throttleDecay * dt)
    }

    // 转向控制
    if (keyState.value.left) {
      steering.value = Math.max(
        -mergedConfig.maxSteering,
        steering.value - mergedConfig.steeringRamp * dt
      )
    } else if (keyState.value.right) {
      steering.value = Math.min(
        mergedConfig.maxSteering,
        steering.value + mergedConfig.steeringRamp * dt
      )
    } else {
      // 自动回正
      if (steering.value > 0) {
        steering.value = Math.max(0, steering.value - mergedConfig.steeringDecay * dt)
      } else if (steering.value < 0) {
        steering.value = Math.min(0, steering.value + mergedConfig.steeringDecay * dt)
      }
    }

    // 判断是否有活动输入
    isActive.value = throttle.value > 0.01 ||
                     brake.value > 0.01 ||
                     Math.abs(steering.value) > 0.01 ||
                     keyState.value.emergency
  }

  // 发送控制指令
  const sendCommand = (vehicleId: string) => {
    updateControlValues()

    // 计算转向灯状态
    let turnSignal: -1 | 0 | 1 = 0
    if (keyState.value.turnLeft) turnSignal = -1
    else if (keyState.value.turnRight) turnSignal = 1

    const command: ControlCommand = {
      timestamp: Date.now(),
      sequence: sequence++,
      vehicle_id: vehicleId,
      throttle: throttle.value,
      brake: brake.value,
      steering: steering.value,
      gear: gear.value,
      turn_signal: turnSignal,
      hazard: keyState.value.hazard,
      emergency_stop: keyState.value.emergency,
      source: 'keyboard',
    }

    onCommand(command)
  }

  // 启动控制
  const start = (vehicleId: string) => {
    enabled.value = true

    // 添加事件监听
    window.addEventListener('keydown', handleKeyDown)
    window.addEventListener('keyup', handleKeyUp)

    // 启动发送定时器
    sendTimer = setInterval(() => {
      if (enabled.value) {
        sendCommand(vehicleId)
      }
    }, mergedConfig.sendInterval)
  }

  // 停止控制
  const stop = () => {
    enabled.value = false

    // 移除事件监听
    window.removeEventListener('keydown', handleKeyDown)
    window.removeEventListener('keyup', handleKeyUp)

    // 停止定时器
    if (sendTimer) {
      clearInterval(sendTimer)
      sendTimer = null
    }

    // 重置状态
    throttle.value = 0
    brake.value = 0
    steering.value = 0
    keyState.value = {
      forward: false,
      backward: false,
      left: false,
      right: false,
      braking: false,
      emergency: false,
      turnLeft: false,
      turnRight: false,
      hazard: false,
      horn: false,
    }
  }

  // 设置档位
  const setGear = (newGear: 'P' | 'R' | 'N' | 'D') => {
    const index = mergedConfig.gearSequence.indexOf(newGear)
    if (index >= 0) {
      gearIndex.value = index
    }
  }

  // 取消紧急停车
  const releaseEmergency = () => {
    keyState.value.emergency = false
  }

  // 组件卸载时清理
  onUnmounted(() => {
    stop()
  })

  return {
    // 状态
    keyState,
    throttle,
    brake,
    steering,
    gear,
    enabled,
    isActive,

    // 方法
    start,
    stop,
    setGear,
    releaseEmergency,

    // 配置
    config: mergedConfig,
    keyMappings: KEY_MAPPINGS,
  }
}

// 显示键盘帮助
export function getKeyboardHelp(): string {
  return `
┌────────────────────────────────────┐
│        键盘控制帮助                │
├────────────────────────────────────┤
│ W / ↑     : 加速/前进             │
│ S / ↓     : 减速/后退             │
│ A / ←     : 左转                  │
│ D / →     : 右转                  │
│ Space     : 刹车                  │
│ E         : 紧急停车 (切换)        │
│ Q         : 切换档位              │
│ R         : 重置                  │
│ Z         : 左转向灯              │
│ C         : 右转向灯              │
│ X         : 双闪                  │
│ H         : 喇叭                  │
│ 1-5       : 切换摄像头            │
│ Tab       : 切换车辆              │
│ M         : 地图全屏              │
└────────────────────────────────────┘
`
}
