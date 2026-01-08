/**
 * FSM-Pilot Demo 模式 Composable
 * 提供完整的演示功能，包括 Mock 数据、模拟控制和场景触发
 */

import { ref, reactive, computed, onMounted, onUnmounted, watch } from 'vue'
import { useFleetStore } from '@/stores/fleet'
import { useSystemStore } from '@/stores/system'
import { useMockData, mockApi, createMockCameraStreams } from '@/services/mockData'
import { useSteeringWheel } from '@/composables/useSteeringWheel'

export interface DemoScenario {
  id: string
  name: string
  description: string
  duration: number
  steps: DemoStep[]
}

export interface DemoStep {
  delay: number
  action: () => void | Promise<void>
  description: string
}

export function useDemoMode() {
  const fleetStore = useFleetStore()
  const systemStore = useSystemStore()
  const { isConnected: mockConnected, sendControl, subscribeVehicle } = useMockData()
  const wheel = useSteeringWheel()

  // 状态
  const isDemoMode = ref(true)
  const isRunningScenario = ref(false)
  const currentScenario = ref<string | null>(null)
  const scenarioProgress = ref(0)

  // Mock 视频流
  const mockVideoStreams = ref<Map<string, HTMLCanvasElement>>(new Map())

  // 控制状态
  const controlState = reactive({
    steering: 0,
    throttle: 0,
    brake: 0,
    gear: 3, // D
    turnSignal: 0,
  })

  // 初始化 Mock 视频
  function initMockVideos() {
    mockVideoStreams.value = createMockCameraStreams()
    systemStore.addLog('Mock video streams initialized', 'info')
  }

  // 获取 Mock 视频元素
  function getMockVideoElement(cameraId: string): HTMLCanvasElement | null {
    return mockVideoStreams.value.get(cameraId) || null
  }

  // 连接到车辆 (Demo 模式)
  async function connectVehicle(vehicleId: string) {
    systemStore.addLog(`[Demo] Connecting to ${vehicleId}...`, 'info')

    // 订阅车辆数据
    subscribeVehicle(vehicleId)

    // 更新状态
    fleetStore.selectVehicleById(vehicleId)
    fleetStore.updateConnectionState(vehicleId, 'connecting')

    // 模拟连接延迟
    await new Promise(resolve => setTimeout(resolve, 1000))

    fleetStore.updateConnectionState(vehicleId, 'connected')
    systemStore.addLog(`[Demo] Connected to ${vehicleId}`, 'info')

    return true
  }

  // 断开车辆
  function disconnectVehicle(vehicleId: string) {
    fleetStore.updateConnectionState(vehicleId, 'disconnected')
    systemStore.addLog(`[Demo] Disconnected from ${vehicleId}`, 'info')
  }

  // 发送控制指令
  function sendControlCommand() {
    const vehicle = fleetStore.currentVehicle
    if (!vehicle || vehicle.connectionState !== 'connected') return

    sendControl(vehicle.id, {
      steering: controlState.steering,
      throttle: controlState.throttle,
      brake: controlState.brake,
      gear: controlState.gear,
      turn_signal: controlState.turnSignal,
    })
  }

  // 触发紧急停车
  async function triggerEmergency() {
    const vehicle = fleetStore.currentVehicle
    if (!vehicle) return

    await mockApi.triggerEmergency(vehicle.id, 'trigger')
    systemStore.addLog(`[Demo] Emergency stop triggered for ${vehicle.id}`, 'error')
  }

  // 解除紧急状态
  async function releaseEmergency() {
    const vehicle = fleetStore.currentVehicle
    if (!vehicle) return

    await mockApi.triggerEmergency(vehicle.id, 'release')
    systemStore.addLog(`[Demo] Emergency released for ${vehicle.id}`, 'info')
  }

  // 网络模拟
  async function simulateNetworkDelay() {
    await mockApi.simulateNetwork('delay', { value: 200 })
    systemStore.addLog('[Demo] Network delay simulation started', 'warning')
  }

  async function simulateNetworkJitter() {
    await mockApi.simulateNetwork('jitter', { min: 50, max: 300 })
    systemStore.addLog('[Demo] Network jitter simulation started', 'warning')
  }

  async function simulateNetworkDisconnect() {
    await mockApi.simulateNetwork('disconnect', { duration: 5000 })
    systemStore.addLog('[Demo] Network disconnect simulation (5s)', 'error')
  }

  async function resetNetwork() {
    await mockApi.simulateNetwork('reset')
    systemStore.addLog('[Demo] Network reset to normal', 'info')
  }

  // ============== 预定义演示场景 ==============

  const demoScenarios: DemoScenario[] = [
    {
      id: 'single_vehicle',
      name: '单车远程控制',
      description: '演示单车辆的远程连接和控制',
      duration: 30000,
      steps: [
        {
          delay: 0,
          description: '连接到 FSM-01',
          action: () => connectVehicle('FSM-01'),
        },
        {
          delay: 2000,
          description: '挂入D档',
          action: () => {
            controlState.gear = 3
            sendControlCommand()
          },
        },
        {
          delay: 3000,
          description: '踩油门前进',
          action: () => {
            controlState.throttle = 0.3
            sendControlCommand()
          },
        },
        {
          delay: 5000,
          description: '左转',
          action: () => {
            controlState.steering = -0.5
            sendControlCommand()
          },
        },
        {
          delay: 7000,
          description: '回正方向',
          action: () => {
            controlState.steering = 0
            sendControlCommand()
          },
        },
        {
          delay: 9000,
          description: '右转',
          action: () => {
            controlState.steering = 0.5
            sendControlCommand()
          },
        },
        {
          delay: 11000,
          description: '回正并减速',
          action: () => {
            controlState.steering = 0
            controlState.throttle = 0
            controlState.brake = 0.3
            sendControlCommand()
          },
        },
        {
          delay: 13000,
          description: '停车挂P档',
          action: () => {
            controlState.brake = 0
            controlState.gear = 0
            sendControlCommand()
          },
        },
      ],
    },
    {
      id: 'emergency_demo',
      name: '紧急停车演示',
      description: '演示紧急停车功能',
      duration: 20000,
      steps: [
        {
          delay: 0,
          description: '连接车辆',
          action: () => connectVehicle('FSM-01'),
        },
        {
          delay: 2000,
          description: '开始行驶',
          action: () => {
            controlState.gear = 3
            controlState.throttle = 0.5
            sendControlCommand()
          },
        },
        {
          delay: 5000,
          description: '触发紧急停车',
          action: () => triggerEmergency(),
        },
        {
          delay: 10000,
          description: '解除紧急状态',
          action: () => releaseEmergency(),
        },
      ],
    },
    {
      id: 'network_demo',
      name: '网络异常演示',
      description: '演示网络问题处理',
      duration: 25000,
      steps: [
        {
          delay: 0,
          description: '连接车辆',
          action: () => connectVehicle('FSM-01'),
        },
        {
          delay: 3000,
          description: '模拟网络延迟',
          action: () => simulateNetworkDelay(),
        },
        {
          delay: 8000,
          description: '模拟网络断开',
          action: () => simulateNetworkDisconnect(),
        },
        {
          delay: 15000,
          description: '等待自动重连',
          action: () => {},
        },
        {
          delay: 20000,
          description: '恢复正常',
          action: () => resetNetwork(),
        },
      ],
    },
    {
      id: 'multi_vehicle',
      name: '多车调度演示',
      description: '演示多车辆切换和调度',
      duration: 30000,
      steps: [
        {
          delay: 0,
          description: '连接 FSM-01',
          action: () => connectVehicle('FSM-01'),
        },
        {
          delay: 3000,
          description: '连接 FSM-02',
          action: () => connectVehicle('FSM-02'),
        },
        {
          delay: 6000,
          description: '连接 FSM-03',
          action: () => connectVehicle('FSM-03'),
        },
        {
          delay: 10000,
          description: '切换到 FSM-02',
          action: () => fleetStore.selectVehicleById('FSM-02'),
        },
        {
          delay: 15000,
          description: 'FSM-03 紧急升级',
          action: () => mockApi.triggerEmergency('FSM-03', 'trigger'),
        },
        {
          delay: 18000,
          description: '观察调度变化',
          action: () => {},
        },
        {
          delay: 23000,
          description: '解除紧急状态',
          action: () => mockApi.triggerEmergency('FSM-03', 'release'),
        },
      ],
    },
  ]

  // 运行演示场景
  async function runScenario(scenarioId: string) {
    const scenario = demoScenarios.find(s => s.id === scenarioId)
    if (!scenario || isRunningScenario.value) return

    isRunningScenario.value = true
    currentScenario.value = scenarioId
    scenarioProgress.value = 0

    systemStore.addLog(`[Demo] Starting scenario: ${scenario.name}`, 'info')

    for (let i = 0; i < scenario.steps.length; i++) {
      const step = scenario.steps[i]

      // 等待延迟
      if (step.delay > 0) {
        await new Promise(resolve => setTimeout(resolve, step.delay - (i > 0 ? scenario.steps[i - 1].delay : 0)))
      }

      // 执行步骤
      systemStore.addLog(`[Demo] ${step.description}`, 'info')
      await step.action()

      scenarioProgress.value = ((i + 1) / scenario.steps.length) * 100
    }

    isRunningScenario.value = false
    currentScenario.value = null
    systemStore.addLog(`[Demo] Scenario completed: ${scenario.name}`, 'info')
  }

  // 停止演示
  function stopScenario() {
    isRunningScenario.value = false
    currentScenario.value = null
    scenarioProgress.value = 0

    // 重置状态
    controlState.steering = 0
    controlState.throttle = 0
    controlState.brake = 0

    resetNetwork()
  }

  // 监听方向盘输入
  watch(() => wheel.input.value, (input) => {
    if (!isDemoMode.value) return

    controlState.steering = input.steering
    controlState.throttle = input.throttle
    controlState.brake = input.brake

    sendControlCommand()
  }, { deep: true })

  // 方向盘紧急按钮
  wheel.onEmergency(() => {
    if (isDemoMode.value) {
      triggerEmergency()
    }
  })

  // 方向盘档位
  wheel.onGearChange((direction) => {
    if (direction === 'up' && controlState.gear < 3) {
      controlState.gear++
    } else if (direction === 'down' && controlState.gear > 0) {
      controlState.gear--
    }
    sendControlCommand()
  })

  // 生命周期
  onMounted(() => {
    initMockVideos()
    systemStore.addLog('[Demo] Demo mode initialized', 'info')
  })

  return {
    // 状态
    isDemoMode,
    isRunningScenario,
    currentScenario,
    scenarioProgress,
    mockConnected,
    mockVideoStreams,
    controlState,

    // 方法
    connectVehicle,
    disconnectVehicle,
    getMockVideoElement,
    sendControlCommand,

    // 紧急操作
    triggerEmergency,
    releaseEmergency,

    // 网络模拟
    simulateNetworkDelay,
    simulateNetworkJitter,
    simulateNetworkDisconnect,
    resetNetwork,

    // 场景
    demoScenarios,
    runScenario,
    stopScenario,

    // 方向盘
    wheelConnected: wheel.isConnected,
    wheelInput: wheel.input,
  }
}
