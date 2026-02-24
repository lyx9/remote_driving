/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      RosBag Player Service
 * @description RosBag回放服务，支持Autoware.universe实车数据回放
 *              支持mcap和db3格式的RosBag文件
 */

import { ref, reactive, computed } from 'vue'

// ======================== 类型定义 ========================

/**
 * ROS2 消息类型定义
 */
export interface RosMessage {
  topic: string
  timestamp: number  // nanoseconds
  data: any
  type: string
}

/**
 * Autoware 车辆状态
 */
export interface AutowareVehicleStatus {
  header: {
    stamp: { sec: number; nanosec: number }
    frame_id: string
  }
  // 速度 (m/s)
  longitudinal_velocity: number
  lateral_velocity: number
  // 加速度 (m/s^2)
  longitudinal_acceleration: number
  lateral_acceleration: number
  // 航向角速度 (rad/s)
  heading_rate: number
  // 转向角 (rad)
  steering_tire_angle: number
}

/**
 * Autoware 定位信息 (来自 /localization/kinematic_state)
 */
export interface AutowareKinematicState {
  header: {
    stamp: { sec: number; nanosec: number }
    frame_id: string
  }
  pose: {
    pose: {
      position: { x: number; y: number; z: number }
      orientation: { x: number; y: number; z: number; w: number }
    }
    covariance: number[]
  }
  twist: {
    twist: {
      linear: { x: number; y: number; z: number }
      angular: { x: number; y: number; z: number }
    }
    covariance: number[]
  }
}

/**
 * Autoware 感知目标
 */
export interface AutowareDetectedObject {
  existence_probability: number
  classification: Array<{
    label: number
    probability: number
  }>
  kinematics: {
    pose_with_covariance: {
      pose: {
        position: { x: number; y: number; z: number }
        orientation: { x: number; y: number; z: number; w: number }
      }
    }
    twist_with_covariance: {
      twist: {
        linear: { x: number; y: number; z: number }
        angular: { x: number; y: number; z: number }
      }
    }
  }
  shape: {
    type: number
    dimensions: { x: number; y: number; z: number }
  }
}

/**
 * Autoware 感知目标数组
 */
export interface AutowareDetectedObjects {
  header: {
    stamp: { sec: number; nanosec: number }
    frame_id: string
  }
  objects: AutowareDetectedObject[]
}

/**
 * 点云数据 (PointCloud2)
 */
export interface PointCloud2 {
  header: {
    stamp: { sec: number; nanosec: number }
    frame_id: string
  }
  height: number
  width: number
  fields: Array<{
    name: string
    offset: number
    datatype: number
    count: number
  }>
  is_bigendian: boolean
  point_step: number
  row_step: number
  data: Uint8Array
  is_dense: boolean
}

/**
 * 压缩图像
 */
export interface CompressedImage {
  header: {
    stamp: { sec: number; nanosec: number }
    frame_id: string
  }
  format: string
  data: Uint8Array
}

/**
 * RosBag 文件信息
 */
export interface RosBagInfo {
  id: string
  name: string
  path: string
  format: 'mcap' | 'db3'
  size: number
  duration: number  // seconds
  startTime: number  // nanoseconds
  endTime: number    // nanoseconds
  messageCount: number
  topics: TopicInfo[]
  metadata?: {
    vehicle_id?: string
    location?: string
    date?: string
    description?: string
  }
}

/**
 * Topic 信息
 */
export interface TopicInfo {
  name: string
  type: string
  messageCount: number
  frequency: number  // Hz
}

/**
 * 回放状态
 */
export interface PlaybackState {
  isPlaying: boolean
  isPaused: boolean
  currentTime: number  // nanoseconds
  playbackRate: number
  loopEnabled: boolean
}

/**
 * Autoware 常用 Topic 列表
 */
export const AUTOWARE_TOPICS = {
  // 定位
  KINEMATIC_STATE: '/localization/kinematic_state',
  POSE: '/localization/pose_twist_fusion_filter/pose',

  // 感知
  DETECTED_OBJECTS: '/perception/object_recognition/detection/objects',
  TRACKED_OBJECTS: '/perception/object_recognition/tracking/objects',
  PREDICTED_OBJECTS: '/perception/object_recognition/prediction/objects',

  // 点云
  LIDAR_CONCATENATED: '/sensing/lidar/concatenated/pointcloud',
  LIDAR_LEFT: '/sensing/lidar/left/pointcloud_raw',
  LIDAR_RIGHT: '/sensing/lidar/right/pointcloud_raw',
  LIDAR_TOP: '/sensing/lidar/top/pointcloud_raw',

  // 相机
  CAMERA_FRONT: '/sensing/camera/front/image_raw/compressed',
  CAMERA_LEFT: '/sensing/camera/left/image_raw/compressed',
  CAMERA_RIGHT: '/sensing/camera/right/image_raw/compressed',
  CAMERA_REAR: '/sensing/camera/rear/image_raw/compressed',

  // 车辆状态
  VEHICLE_STATUS: '/vehicle/status/velocity_status',
  STEERING_STATUS: '/vehicle/status/steering_status',
  GEAR_STATUS: '/vehicle/status/gear_status',
  TURN_INDICATORS: '/vehicle/status/turn_indicators_status',

  // 控制
  CONTROL_CMD: '/control/command/control_cmd',
  TRAJECTORY: '/planning/scenario_planning/trajectory',

  // 诊断
  DIAGNOSTICS: '/diagnostics',
  SYSTEM_STATUS: '/autoware/state',
}

// ======================== Mock RosBag 数据生成 ========================

/**
 * 生成 Mock RosBag 信息
 */
export function generateMockRosBagInfo(): RosBagInfo {
  const duration = 120 + Math.random() * 180  // 2-5分钟
  const startTime = Date.now() * 1e6  // 转换为纳秒

  return {
    id: `rosbag_${Date.now()}`,
    name: `autoware_drive_${new Date().toISOString().slice(0, 10)}.mcap`,
    path: '/data/rosbags/autoware_drive.mcap',
    format: 'mcap',
    size: Math.floor(500 + Math.random() * 2000) * 1024 * 1024,  // 500MB - 2.5GB
    duration,
    startTime,
    endTime: startTime + duration * 1e9,
    messageCount: Math.floor(duration * 100),  // ~100 msg/s
    topics: [
      { name: AUTOWARE_TOPICS.KINEMATIC_STATE, type: 'nav_msgs/msg/Odometry', messageCount: Math.floor(duration * 50), frequency: 50 },
      { name: AUTOWARE_TOPICS.DETECTED_OBJECTS, type: 'autoware_auto_perception_msgs/msg/DetectedObjects', messageCount: Math.floor(duration * 10), frequency: 10 },
      { name: AUTOWARE_TOPICS.LIDAR_CONCATENATED, type: 'sensor_msgs/msg/PointCloud2', messageCount: Math.floor(duration * 10), frequency: 10 },
      { name: AUTOWARE_TOPICS.CAMERA_FRONT, type: 'sensor_msgs/msg/CompressedImage', messageCount: Math.floor(duration * 30), frequency: 30 },
      { name: AUTOWARE_TOPICS.CAMERA_LEFT, type: 'sensor_msgs/msg/CompressedImage', messageCount: Math.floor(duration * 30), frequency: 30 },
      { name: AUTOWARE_TOPICS.CAMERA_RIGHT, type: 'sensor_msgs/msg/CompressedImage', messageCount: Math.floor(duration * 30), frequency: 30 },
      { name: AUTOWARE_TOPICS.VEHICLE_STATUS, type: 'autoware_auto_vehicle_msgs/msg/VelocityReport', messageCount: Math.floor(duration * 100), frequency: 100 },
      { name: AUTOWARE_TOPICS.CONTROL_CMD, type: 'autoware_auto_control_msgs/msg/AckermannControlCommand', messageCount: Math.floor(duration * 30), frequency: 30 },
      { name: AUTOWARE_TOPICS.TRAJECTORY, type: 'autoware_auto_planning_msgs/msg/Trajectory', messageCount: Math.floor(duration * 10), frequency: 10 },
    ],
    metadata: {
      vehicle_id: 'FSM-01',
      location: 'Shanghai Pudong Test Zone',
      date: new Date().toISOString(),
      description: 'Autoware.universe autonomous driving test'
    }
  }
}

/**
 * 生成 Mock 车辆状态消息
 */
function generateMockVehicleStatus(timestamp: number): AutowareVehicleStatus {
  const time = timestamp / 1e9  // 转换为秒
  // 模拟变化的速度
  const baseSpeed = 10 + 5 * Math.sin(time * 0.1)

  return {
    header: {
      stamp: {
        sec: Math.floor(timestamp / 1e9),
        nanosec: timestamp % 1e9
      },
      frame_id: 'base_link'
    },
    longitudinal_velocity: baseSpeed + Math.random() * 0.5,
    lateral_velocity: Math.random() * 0.1 - 0.05,
    longitudinal_acceleration: Math.sin(time * 0.2) * 0.5,
    lateral_acceleration: Math.random() * 0.2 - 0.1,
    heading_rate: Math.sin(time * 0.3) * 0.05,
    steering_tire_angle: Math.sin(time * 0.2) * 0.1
  }
}

/**
 * 生成 Mock 定位消息
 */
function generateMockKinematicState(timestamp: number, startPos: { x: number; y: number }): AutowareKinematicState {
  const time = timestamp / 1e9
  // 模拟沿路径移动
  const progress = time * 10  // 10 m/s
  const x = startPos.x + progress * Math.cos(time * 0.05)
  const y = startPos.y + progress * Math.sin(time * 0.05)
  const yaw = Math.atan2(Math.sin(time * 0.05), Math.cos(time * 0.05))

  return {
    header: {
      stamp: {
        sec: Math.floor(timestamp / 1e9),
        nanosec: timestamp % 1e9
      },
      frame_id: 'map'
    },
    pose: {
      pose: {
        position: { x, y, z: 0 },
        orientation: {
          x: 0,
          y: 0,
          z: Math.sin(yaw / 2),
          w: Math.cos(yaw / 2)
        }
      },
      covariance: new Array(36).fill(0)
    },
    twist: {
      twist: {
        linear: { x: 10 + Math.random() * 2, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: Math.sin(time * 0.3) * 0.1 }
      },
      covariance: new Array(36).fill(0)
    }
  }
}

/**
 * 生成 Mock 感知目标
 */
function generateMockDetectedObjects(timestamp: number, vehiclePos: { x: number; y: number }): AutowareDetectedObjects {
  const objectCount = 3 + Math.floor(Math.random() * 5)
  const objects: AutowareDetectedObject[] = []

  for (let i = 0; i < objectCount; i++) {
    const angle = Math.random() * Math.PI * 2
    const distance = 10 + Math.random() * 50

    objects.push({
      existence_probability: 0.8 + Math.random() * 0.2,
      classification: [{
        label: Math.floor(Math.random() * 3),  // 0: car, 1: pedestrian, 2: cyclist
        probability: 0.7 + Math.random() * 0.3
      }],
      kinematics: {
        pose_with_covariance: {
          pose: {
            position: {
              x: vehiclePos.x + Math.cos(angle) * distance,
              y: vehiclePos.y + Math.sin(angle) * distance,
              z: 0
            },
            orientation: { x: 0, y: 0, z: 0, w: 1 }
          }
        },
        twist_with_covariance: {
          twist: {
            linear: { x: Math.random() * 5, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
          }
        }
      },
      shape: {
        type: 0,  // BOUNDING_BOX
        dimensions: {
          x: 4 + Math.random() * 2,
          y: 1.8 + Math.random() * 0.4,
          z: 1.5 + Math.random() * 0.5
        }
      }
    })
  }

  return {
    header: {
      stamp: {
        sec: Math.floor(timestamp / 1e9),
        nanosec: timestamp % 1e9
      },
      frame_id: 'base_link'
    },
    objects
  }
}

// ======================== RosBag 播放器服务 ========================

export function useRosBagPlayer() {
  // 状态
  const isLoading = ref(false)
  const error = ref<string | null>(null)
  const currentBag = ref<RosBagInfo | null>(null)

  // 回放状态
  const playbackState = reactive<PlaybackState>({
    isPlaying: false,
    isPaused: false,
    currentTime: 0,
    playbackRate: 1.0,
    loopEnabled: false
  })

  // 当前数据
  const currentVehicleStatus = ref<AutowareVehicleStatus | null>(null)
  const currentKinematicState = ref<AutowareKinematicState | null>(null)
  const currentDetectedObjects = ref<AutowareDetectedObjects | null>(null)
  const currentImages = reactive<Record<string, string>>({})  // topic -> base64 image

  // 消息回调
  const messageCallbacks = new Map<string, ((msg: RosMessage) => void)[]>()

  // 内部状态
  let playbackTimer: ReturnType<typeof setInterval> | null = null
  let startPosition = { x: 0, y: 0 }

  // ==================== 计算属性 ====================

  const progress = computed(() => {
    if (!currentBag.value) return 0
    const duration = currentBag.value.endTime - currentBag.value.startTime
    if (duration === 0) return 0
    return (playbackState.currentTime - currentBag.value.startTime) / duration
  })

  const currentTimeFormatted = computed(() => {
    const seconds = playbackState.currentTime / 1e9
    const mins = Math.floor(seconds / 60)
    const secs = Math.floor(seconds % 60)
    const ms = Math.floor((seconds % 1) * 100)
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}.${ms.toString().padStart(2, '0')}`
  })

  const durationFormatted = computed(() => {
    if (!currentBag.value) return '00:00.00'
    const seconds = currentBag.value.duration
    const mins = Math.floor(seconds / 60)
    const secs = Math.floor(seconds % 60)
    return `${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}.00`
  })

  // ==================== 加载 RosBag ====================

  /**
   * 加载 RosBag 文件
   */
  const loadBag = async (bagInfo: RosBagInfo): Promise<boolean> => {
    isLoading.value = true
    error.value = null

    try {
      // 实际实现中需要通过后端解析 RosBag 文件
      // 这里使用 Mock 数据
      currentBag.value = bagInfo
      playbackState.currentTime = bagInfo.startTime
      playbackState.isPlaying = false
      playbackState.isPaused = false

      // 初始化起始位置
      startPosition = { x: Math.random() * 100, y: Math.random() * 100 }

      console.log(`[RosBag] Loaded: ${bagInfo.name}`)
      isLoading.value = false
      return true
    } catch (e) {
      error.value = `Failed to load bag: ${e}`
      isLoading.value = false
      return false
    }
  }

  /**
   * 加载 Mock RosBag
   */
  const loadMockBag = async (): Promise<boolean> => {
    const mockInfo = generateMockRosBagInfo()
    return loadBag(mockInfo)
  }

  // ==================== 回放控制 ====================

  /**
   * 开始播放
   */
  const play = () => {
    if (!currentBag.value) return

    playbackState.isPlaying = true
    playbackState.isPaused = false

    // 启动回放定时器 (100Hz)
    const interval = 10 / playbackState.playbackRate

    playbackTimer = setInterval(() => {
      if (!currentBag.value || !playbackState.isPlaying) return

      // 更新时间 (每10ms前进10ms * 回放速率)
      playbackState.currentTime += 10 * 1e6 * playbackState.playbackRate

      // 检查是否到达结尾
      if (playbackState.currentTime >= currentBag.value.endTime) {
        if (playbackState.loopEnabled) {
          playbackState.currentTime = currentBag.value.startTime
        } else {
          pause()
          playbackState.currentTime = currentBag.value.endTime
          return
        }
      }

      // 生成并发布消息
      publishMockMessages()
    }, interval)
  }

  /**
   * 暂停播放
   */
  const pause = () => {
    playbackState.isPlaying = false
    playbackState.isPaused = true

    if (playbackTimer) {
      clearInterval(playbackTimer)
      playbackTimer = null
    }
  }

  /**
   * 停止播放
   */
  const stop = () => {
    pause()
    if (currentBag.value) {
      playbackState.currentTime = currentBag.value.startTime
    }
    playbackState.isPaused = false
  }

  /**
   * 跳转到指定时间
   */
  const seek = (timeNs: number) => {
    if (!currentBag.value) return

    playbackState.currentTime = Math.max(
      currentBag.value.startTime,
      Math.min(currentBag.value.endTime, timeNs)
    )

    // 立即发布当前时间的消息
    publishMockMessages()
  }

  /**
   * 跳转到进度百分比
   */
  const seekToProgress = (percent: number) => {
    if (!currentBag.value) return

    const duration = currentBag.value.endTime - currentBag.value.startTime
    const targetTime = currentBag.value.startTime + duration * Math.max(0, Math.min(1, percent))
    seek(targetTime)
  }

  /**
   * 设置回放速率
   */
  const setPlaybackRate = (rate: number) => {
    playbackState.playbackRate = Math.max(0.1, Math.min(10, rate))

    // 如果正在播放，重启定时器
    if (playbackState.isPlaying) {
      pause()
      play()
    }
  }

  /**
   * 切换循环模式
   */
  const toggleLoop = () => {
    playbackState.loopEnabled = !playbackState.loopEnabled
  }

  /**
   * 单步前进
   */
  const stepForward = () => {
    if (!currentBag.value) return
    seek(playbackState.currentTime + 100 * 1e6)  // 前进100ms
  }

  /**
   * 单步后退
   */
  const stepBackward = () => {
    if (!currentBag.value) return
    seek(playbackState.currentTime - 100 * 1e6)  // 后退100ms
  }

  // ==================== 消息发布 ====================

  /**
   * 发布 Mock 消息
   */
  const publishMockMessages = () => {
    const timestamp = playbackState.currentTime
    const relativeTime = currentBag.value
      ? (timestamp - currentBag.value.startTime) / 1e9
      : 0

    // 计算当前位置
    const currentPos = {
      x: startPosition.x + relativeTime * 10 * Math.cos(relativeTime * 0.05),
      y: startPosition.y + relativeTime * 10 * Math.sin(relativeTime * 0.05)
    }

    // 生成消息
    const vehicleStatus = generateMockVehicleStatus(timestamp)
    const kinematicState = generateMockKinematicState(timestamp, startPosition)
    const detectedObjects = generateMockDetectedObjects(timestamp, currentPos)

    // 更新状态
    currentVehicleStatus.value = vehicleStatus
    currentKinematicState.value = kinematicState
    currentDetectedObjects.value = detectedObjects

    // 触发回调
    publishMessage({
      topic: AUTOWARE_TOPICS.VEHICLE_STATUS,
      timestamp,
      data: vehicleStatus,
      type: 'autoware_auto_vehicle_msgs/msg/VelocityReport'
    })

    publishMessage({
      topic: AUTOWARE_TOPICS.KINEMATIC_STATE,
      timestamp,
      data: kinematicState,
      type: 'nav_msgs/msg/Odometry'
    })

    publishMessage({
      topic: AUTOWARE_TOPICS.DETECTED_OBJECTS,
      timestamp,
      data: detectedObjects,
      type: 'autoware_auto_perception_msgs/msg/DetectedObjects'
    })
  }

  /**
   * 发布消息到订阅者
   */
  const publishMessage = (msg: RosMessage) => {
    const callbacks = messageCallbacks.get(msg.topic)
    if (callbacks) {
      callbacks.forEach(cb => cb(msg))
    }

    // 也触发通配符订阅
    const wildcardCallbacks = messageCallbacks.get('*')
    if (wildcardCallbacks) {
      wildcardCallbacks.forEach(cb => cb(msg))
    }
  }

  // ==================== 订阅管理 ====================

  /**
   * 订阅 Topic
   */
  const subscribe = (topic: string, callback: (msg: RosMessage) => void) => {
    if (!messageCallbacks.has(topic)) {
      messageCallbacks.set(topic, [])
    }
    messageCallbacks.get(topic)!.push(callback)

    return () => {
      const callbacks = messageCallbacks.get(topic)
      if (callbacks) {
        const index = callbacks.indexOf(callback)
        if (index >= 0) {
          callbacks.splice(index, 1)
        }
      }
    }
  }

  /**
   * 取消所有订阅
   */
  const unsubscribeAll = () => {
    messageCallbacks.clear()
  }

  // ==================== 清理 ====================

  const dispose = () => {
    stop()
    unsubscribeAll()
    currentBag.value = null
    currentVehicleStatus.value = null
    currentKinematicState.value = null
    currentDetectedObjects.value = null
  }

  return {
    // 状态
    isLoading,
    error,
    currentBag,
    playbackState,
    progress,
    currentTimeFormatted,
    durationFormatted,

    // 当前数据
    currentVehicleStatus,
    currentKinematicState,
    currentDetectedObjects,
    currentImages,

    // 加载
    loadBag,
    loadMockBag,

    // 回放控制
    play,
    pause,
    stop,
    seek,
    seekToProgress,
    setPlaybackRate,
    toggleLoop,
    stepForward,
    stepBackward,

    // 订阅
    subscribe,
    unsubscribeAll,

    // 清理
    dispose,

    // 常量
    AUTOWARE_TOPICS
  }
}

// ======================== 辅助函数 ========================

/**
 * 格式化文件大小
 */
export function formatFileSize(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`
  if (bytes < 1024 * 1024 * 1024) return `${(bytes / (1024 * 1024)).toFixed(1)} MB`
  return `${(bytes / (1024 * 1024 * 1024)).toFixed(2)} GB`
}

/**
 * 格式化时长
 */
export function formatDuration(seconds: number): string {
  const hrs = Math.floor(seconds / 3600)
  const mins = Math.floor((seconds % 3600) / 60)
  const secs = Math.floor(seconds % 60)

  if (hrs > 0) {
    return `${hrs}:${mins.toString().padStart(2, '0')}:${secs.toString().padStart(2, '0')}`
  }
  return `${mins}:${secs.toString().padStart(2, '0')}`
}

/**
 * 四元数转欧拉角
 */
export function quaternionToEuler(q: { x: number; y: number; z: number; w: number }): { roll: number; pitch: number; yaw: number } {
  const { x, y, z, w } = q

  // Roll (x-axis rotation)
  const sinr_cosp = 2 * (w * x + y * z)
  const cosr_cosp = 1 - 2 * (x * x + y * y)
  const roll = Math.atan2(sinr_cosp, cosr_cosp)

  // Pitch (y-axis rotation)
  const sinp = 2 * (w * y - z * x)
  const pitch = Math.abs(sinp) >= 1
    ? Math.sign(sinp) * Math.PI / 2
    : Math.asin(sinp)

  // Yaw (z-axis rotation)
  const siny_cosp = 2 * (w * z + x * y)
  const cosy_cosp = 1 - 2 * (y * y + z * z)
  const yaw = Math.atan2(siny_cosp, cosy_cosp)

  return { roll, pitch, yaw }
}

/**
 * 弧度转角度
 */
export function radToDeg(rad: number): number {
  return rad * 180 / Math.PI
}
