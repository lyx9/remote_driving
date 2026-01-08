/**
 * FSM-Pilot 方向盘输入 Composable
 * 支持罗技 G29/G920 等游戏方向盘
 */

import { ref, onMounted, onUnmounted } from 'vue'
import { useSystemStore } from '@/stores/system'

export interface WheelInput {
  steering: number      // -1.0 to 1.0
  throttle: number      // 0.0 to 1.0
  brake: number         // 0.0 to 1.0
  clutch: number        // 0.0 to 1.0
  buttons: {
    emergency: boolean
    horn: boolean
    leftSignal: boolean
    rightSignal: boolean
    gearUp: boolean
    gearDown: boolean
  }
}

export interface WheelConfig {
  deadzone: number
  steeringRatio: number
  throttleCurve: 'linear' | 'exponential'
  brakeCurve: 'linear' | 'exponential'
  invertSteering: boolean
}

const defaultConfig: WheelConfig = {
  deadzone: 0.02,
  steeringRatio: 450,
  throttleCurve: 'linear',
  brakeCurve: 'linear',
  invertSteering: false
}

// 罗技方向盘按钮映射
const LOGITECH_BUTTONS = {
  X: 0,           // 紧急停车
  SQUARE: 1,      // 喇叭
  CIRCLE: 2,
  TRIANGLE: 3,
  L1: 4,          // 左转向灯
  R1: 5,          // 右转向灯
  PADDLE_RIGHT: 6, // 升档
  PADDLE_LEFT: 7   // 降档
}

export function useSteeringWheel(config: Partial<WheelConfig> = {}) {
  const systemStore = useSystemStore()
  const mergedConfig = { ...defaultConfig, ...config }

  // 状态
  const isConnected = ref(false)
  const wheelName = ref('')
  const input = ref<WheelInput>({
    steering: 0,
    throttle: 0,
    brake: 0,
    clutch: 0,
    buttons: {
      emergency: false,
      horn: false,
      leftSignal: false,
      rightSignal: false,
      gearUp: false,
      gearDown: false
    }
  })

  // 回调
  let onInputCallback: ((input: WheelInput) => void) | null = null
  let onEmergencyCallback: (() => void) | null = null
  let onGearChangeCallback: ((direction: 'up' | 'down') => void) | null = null
  let onSignalCallback: ((signal: 'left' | 'right' | 'off') => void) | null = null

  // Gamepad API
  let gamepadIndex: number | null = null
  let animationFrameId: number | null = null

  // 检测方向盘
  function detectWheel() {
    const gamepads = navigator.getGamepads()
    for (let i = 0; i < gamepads.length; i++) {
      const gp = gamepads[i]
      if (gp && isLogitechWheel(gp.id)) {
        gamepadIndex = i
        wheelName.value = gp.id
        isConnected.value = true
        systemStore.addLog(`Steering wheel connected: ${gp.id}`, 'info')
        return true
      }
    }
    return false
  }

  // 判断是否是罗技方向盘
  function isLogitechWheel(id: string): boolean {
    const logitechIds = [
      'Logitech G29',
      'Logitech G920',
      'Logitech G27',
      'Logitech G923',
      'Logitech Driving Force',
      '046d:c24f',  // G29 USB ID
      '046d:c262',  // G920 USB ID
    ]
    return logitechIds.some(lid => id.toLowerCase().includes(lid.toLowerCase()))
  }

  // 应用死区
  function applyDeadzone(value: number): number {
    if (Math.abs(value) < mergedConfig.deadzone) {
      return 0
    }
    // 重新映射死区外的值
    const sign = value > 0 ? 1 : -1
    return sign * (Math.abs(value) - mergedConfig.deadzone) / (1 - mergedConfig.deadzone)
  }

  // 应用曲线
  function applyCurve(value: number, curve: 'linear' | 'exponential'): number {
    if (curve === 'exponential') {
      return Math.sign(value) * Math.pow(Math.abs(value), 2)
    }
    return value
  }

  // 归一化轴值 (从 [-32768, 32767] 或 [-1, 1] 到 [-1, 1] 或 [0, 1])
  function normalizeAxis(value: number, isThrottle: boolean = false): number {
    // Gamepad API 返回 -1 到 1
    if (isThrottle) {
      // 踏板: 从 [-1, 1] 转换为 [0, 1]，-1 = 完全按下，1 = 未按
      return (1 - value) / 2
    }
    return value
  }

  // 读取输入
  function readInput() {
    if (gamepadIndex === null) return

    const gp = navigator.getGamepads()[gamepadIndex]
    if (!gp) {
      isConnected.value = false
      gamepadIndex = null
      systemStore.addLog('Steering wheel disconnected', 'warning')
      return
    }

    // 读取轴
    const rawSteering = gp.axes[0] || 0
    const rawThrottle = gp.axes[2] !== undefined ? gp.axes[2] : (gp.axes[1] || 0)
    const rawBrake = gp.axes[3] !== undefined ? gp.axes[3] : (gp.axes[2] || 0)
    const rawClutch = gp.axes[1] !== undefined ? gp.axes[1] : 0

    // 处理转向
    let steering = applyDeadzone(rawSteering)
    steering = applyCurve(steering, 'linear')
    if (mergedConfig.invertSteering) {
      steering = -steering
    }

    // 处理踏板
    const throttle = applyCurve(normalizeAxis(rawThrottle, true), mergedConfig.throttleCurve)
    const brake = applyCurve(normalizeAxis(rawBrake, true), mergedConfig.brakeCurve)
    const clutch = normalizeAxis(rawClutch, true)

    // 读取按钮
    const prevButtons = { ...input.value.buttons }
    const buttons = {
      emergency: gp.buttons[LOGITECH_BUTTONS.X]?.pressed || false,
      horn: gp.buttons[LOGITECH_BUTTONS.SQUARE]?.pressed || false,
      leftSignal: gp.buttons[LOGITECH_BUTTONS.L1]?.pressed || false,
      rightSignal: gp.buttons[LOGITECH_BUTTONS.R1]?.pressed || false,
      gearUp: gp.buttons[LOGITECH_BUTTONS.PADDLE_RIGHT]?.pressed || false,
      gearDown: gp.buttons[LOGITECH_BUTTONS.PADDLE_LEFT]?.pressed || false
    }

    // 更新状态
    input.value = {
      steering,
      throttle,
      brake,
      clutch,
      buttons
    }

    // 触发回调
    if (onInputCallback) {
      onInputCallback(input.value)
    }

    // 检测按钮变化
    if (buttons.emergency && !prevButtons.emergency && onEmergencyCallback) {
      onEmergencyCallback()
    }

    if (buttons.gearUp && !prevButtons.gearUp && onGearChangeCallback) {
      onGearChangeCallback('up')
    }

    if (buttons.gearDown && !prevButtons.gearDown && onGearChangeCallback) {
      onGearChangeCallback('down')
    }

    if (buttons.leftSignal !== prevButtons.leftSignal && onSignalCallback) {
      onSignalCallback(buttons.leftSignal ? 'left' : 'off')
    }

    if (buttons.rightSignal !== prevButtons.rightSignal && onSignalCallback) {
      onSignalCallback(buttons.rightSignal ? 'right' : 'off')
    }
  }

  // 轮询循环
  function pollLoop() {
    readInput()
    animationFrameId = requestAnimationFrame(pollLoop)
  }

  // 启动轮询
  function startPolling() {
    if (animationFrameId === null) {
      pollLoop()
    }
  }

  // 停止轮询
  function stopPolling() {
    if (animationFrameId !== null) {
      cancelAnimationFrame(animationFrameId)
      animationFrameId = null
    }
  }

  // 设置回调
  function onInput(callback: (input: WheelInput) => void) {
    onInputCallback = callback
  }

  function onEmergency(callback: () => void) {
    onEmergencyCallback = callback
  }

  function onGearChange(callback: (direction: 'up' | 'down') => void) {
    onGearChangeCallback = callback
  }

  function onSignal(callback: (signal: 'left' | 'right' | 'off') => void) {
    onSignalCallback = callback
  }

  // 震动反馈 (如果支持)
  function vibrate(duration: number = 200, weakMagnitude: number = 0.5, strongMagnitude: number = 0.5) {
    if (gamepadIndex === null) return

    const gp = navigator.getGamepads()[gamepadIndex]
    if (gp && 'vibrationActuator' in gp) {
      (gp as any).vibrationActuator.playEffect('dual-rumble', {
        duration,
        weakMagnitude,
        strongMagnitude
      })
    }
  }

  // 监听连接事件
  function handleGamepadConnected(e: GamepadEvent) {
    if (isLogitechWheel(e.gamepad.id)) {
      gamepadIndex = e.gamepad.index
      wheelName.value = e.gamepad.id
      isConnected.value = true
      systemStore.addLog(`Steering wheel connected: ${e.gamepad.id}`, 'info')
      startPolling()
    }
  }

  function handleGamepadDisconnected(e: GamepadEvent) {
    if (e.gamepad.index === gamepadIndex) {
      isConnected.value = false
      wheelName.value = ''
      gamepadIndex = null
      systemStore.addLog('Steering wheel disconnected', 'warning')
    }
  }

  // 生命周期
  onMounted(() => {
    window.addEventListener('gamepadconnected', handleGamepadConnected)
    window.addEventListener('gamepaddisconnected', handleGamepadDisconnected)

    // 尝试检测已连接的方向盘
    if (detectWheel()) {
      startPolling()
    }
  })

  onUnmounted(() => {
    window.removeEventListener('gamepadconnected', handleGamepadConnected)
    window.removeEventListener('gamepaddisconnected', handleGamepadDisconnected)
    stopPolling()
  })

  return {
    // 状态
    isConnected,
    wheelName,
    input,

    // 方法
    detectWheel,
    startPolling,
    stopPolling,
    vibrate,

    // 回调设置
    onInput,
    onEmergency,
    onGearChange,
    onSignal
  }
}
