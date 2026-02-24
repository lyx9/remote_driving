/**
 * FSM-Pilot V2.0 - Gamepad Controller
 *
 * @description 游戏手柄和键盘控制器
 */

import { ref, onMounted, onUnmounted } from 'vue'
import { useSystemStore } from '@/stores/system'

export interface ControlInput {
  steering: number // -1 to 1
  throttle: number // 0 to 1
  brake: number // 0 to 1
  gear: 'P' | 'R' | 'N' | 'D'
  emergency: boolean
}

export function useGamepadController() {
  const systemStore = useSystemStore()

  const controlInput = ref<ControlInput>({
    steering: 0,
    throttle: 0,
    brake: 0,
    gear: 'P',
    emergency: false
  })

  const gamepadConnected = ref(false)
  const gamepadIndex = ref<number | null>(null)

  // 键盘状态
  const keys = new Set<string>()

  // 键盘控制
  const handleKeyDown = (e: KeyboardEvent) => {
    keys.add(e.key.toLowerCase())

    // 档位切换
    if (e.key === 'p' || e.key === 'P') controlInput.value.gear = 'P'
    if (e.key === 'r' || e.key === 'R') controlInput.value.gear = 'R'
    if (e.key === 'n' || e.key === 'N') controlInput.value.gear = 'N'
    if (e.key === 'd' || e.key === 'D') controlInput.value.gear = 'D'

    // 紧急制动
    if (e.key === ' ') {
      e.preventDefault()
      controlInput.value.emergency = true
      controlInput.value.brake = 1
      controlInput.value.throttle = 0
    }
  }

  const handleKeyUp = (e: KeyboardEvent) => {
    keys.delete(e.key.toLowerCase())

    if (e.key === ' ') {
      controlInput.value.emergency = false
      controlInput.value.brake = 0
    }
  }

  // 更新控制输入 (键盘)
  const updateKeyboardInput = () => {
    if (controlInput.value.emergency) return

    // 转向 (A/D 或 左/右箭头)
    let steering = 0
    if (keys.has('a') || keys.has('arrowleft')) steering -= 1
    if (keys.has('d') || keys.has('arrowright')) steering += 1
    controlInput.value.steering = steering

    // 油门 (W 或 上箭头)
    let throttle = 0
    if (keys.has('w') || keys.has('arrowup')) throttle = 1
    controlInput.value.throttle = throttle

    // 刹车 (S 或 下箭头)
    let brake = 0
    if (keys.has('s') || keys.has('arrowdown')) brake = 1
    controlInput.value.brake = brake
  }

  // 游戏手柄连接
  const onGamepadConnected = (e: GamepadEvent) => {
    gamepadConnected.value = true
    gamepadIndex.value = e.gamepad.index
    systemStore.addLog(`Gamepad connected: ${e.gamepad.id}`, 'info')
  }

  const onGamepadDisconnected = (e: GamepadEvent) => {
    if (gamepadIndex.value === e.gamepad.index) {
      gamepadConnected.value = false
      gamepadIndex.value = null
      systemStore.addLog(`Gamepad disconnected: ${e.gamepad.id}`, 'warning')
    }
  }

  // 读取游戏手柄输入
  const updateGamepadInput = () => {
    if (!gamepadConnected.value || gamepadIndex.value === null) return

    const gamepads = navigator.getGamepads()
    const gamepad = gamepads[gamepadIndex.value]

    if (!gamepad) {
      gamepadConnected.value = false
      return
    }

    // Xbox/PS4 控制器映射
    // 左摇杆 X (转向)
    controlInput.value.steering = gamepad.axes[0] || 0

    // RT/R2 (油门) - 按钮 7 或轴 5
    const throttleAxis = gamepad.axes[5]
    if (throttleAxis !== undefined) {
      controlInput.value.throttle = Math.max(0, (throttleAxis + 1) / 2)
    } else if (gamepad.buttons[7]) {
      controlInput.value.throttle = gamepad.buttons[7].value
    }

    // LT/L2 (刹车) - 按钮 6 或轴 4
    const brakeAxis = gamepad.axes[4]
    if (brakeAxis !== undefined) {
      controlInput.value.brake = Math.max(0, (brakeAxis + 1) / 2)
    } else if (gamepad.buttons[6]) {
      controlInput.value.brake = gamepad.buttons[6].value
    }

    // 方向键 - 档位切换
    if (gamepad.buttons[12]?.pressed) controlInput.value.gear = 'D' // 上
    if (gamepad.buttons[13]?.pressed) controlInput.value.gear = 'R' // 下
    if (gamepad.buttons[14]?.pressed) controlInput.value.gear = 'N' // 左
    if (gamepad.buttons[15]?.pressed) controlInput.value.gear = 'P' // 右

    // B/Circle 按钮 - 紧急制动
    if (gamepad.buttons[1]?.pressed) {
      controlInput.value.emergency = true
      controlInput.value.brake = 1
      controlInput.value.throttle = 0
    } else {
      controlInput.value.emergency = false
    }
  }

  // 控制循环
  let animationId: number | null = null
  const controlLoop = () => {
    updateKeyboardInput()
    updateGamepadInput()
    animationId = requestAnimationFrame(controlLoop)
  }

  onMounted(() => {
    // 键盘事件
    window.addEventListener('keydown', handleKeyDown)
    window.addEventListener('keyup', handleKeyUp)

    // 游戏手柄事件
    window.addEventListener('gamepadconnected', onGamepadConnected)
    window.addEventListener('gamepaddisconnected', onGamepadDisconnected)

    // 检查已连接的游戏手柄
    const gamepads = navigator.getGamepads()
    for (let i = 0; i < gamepads.length; i++) {
      if (gamepads[i]) {
        gamepadConnected.value = true
        gamepadIndex.value = i
        systemStore.addLog(`Gamepad detected: ${gamepads[i]!.id}`, 'info')
        break
      }
    }

    // 启动控制循环
    controlLoop()
  })

  onUnmounted(() => {
    window.removeEventListener('keydown', handleKeyDown)
    window.removeEventListener('keyup', handleKeyUp)
    window.removeEventListener('gamepadconnected', onGamepadConnected)
    window.removeEventListener('gamepaddisconnected', onGamepadDisconnected)

    if (animationId) {
      cancelAnimationFrame(animationId)
    }
  })

  return {
    controlInput,
    gamepadConnected,
    gamepadIndex
  }
}
