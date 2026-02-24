/**
 * FSM-Pilot V2.0 - Camera Configuration
 *
 * @description 相机 Topic 映射配置
 *              定义前端窗口与 ROS Topic 的映射关系
 */

export interface CameraConfig {
  id: string
  name: string
  position: 'front' | 'rear' | 'left' | 'right' | 'top'
  topicPattern: string // Topic 名称匹配模式
  preferredType: 'compressed' | 'raw' // 优先使用的图像类型
  enabled: boolean
}

/**
 * 默认相机配置
 * 用户可以在设置中修改这些配置
 */
export const DEFAULT_CAMERA_CONFIG: CameraConfig[] = [
  {
    id: 'camera_front',
    name: 'Front Camera',
    position: 'front',
    topicPattern: '/camera/front',
    preferredType: 'compressed',
    enabled: true
  },
  {
    id: 'camera_rear',
    name: 'Rear Camera',
    position: 'rear',
    topicPattern: '/camera/rear',
    preferredType: 'compressed',
    enabled: true
  },
  {
    id: 'camera_left',
    name: 'Left Camera',
    position: 'left',
    topicPattern: '/camera/left',
    preferredType: 'compressed',
    enabled: true
  },
  {
    id: 'camera_right',
    name: 'Right Camera',
    position: 'right',
    topicPattern: '/camera/right',
    preferredType: 'compressed',
    enabled: true
  },
  {
    id: 'camera_top',
    name: 'Top Camera',
    position: 'top',
    topicPattern: '/camera/top',
    preferredType: 'compressed',
    enabled: false
  }
]

/**
 * 根据配置查找匹配的 topic
 */
export function findMatchingTopic(
  config: CameraConfig,
  availableTopics: string[]
): string | null {
  // 首先尝试精确匹配
  const exactMatch = availableTopics.find(t => t === config.topicPattern)
  if (exactMatch) return exactMatch

  // 尝试部分匹配
  const partialMatch = availableTopics.find(t =>
    t.toLowerCase().includes(config.topicPattern.toLowerCase())
  )
  if (partialMatch) return partialMatch

  // 尝试根据位置关键词匹配
  const positionMatch = availableTopics.find(t =>
    t.toLowerCase().includes(config.position)
  )
  if (positionMatch) return positionMatch

  return null
}

/**
 * 自动映射相机配置到可用 topics
 */
export function autoMapCameraTopics(
  configs: CameraConfig[],
  availableTopics: string[]
): Map<string, string> {
  const mapping = new Map<string, string>()

  for (const config of configs) {
    if (!config.enabled) continue

    const topic = findMatchingTopic(config, availableTopics)
    if (topic) {
      mapping.set(config.id, topic)
      console.log(`[Camera Config] Mapped ${config.id} -> ${topic}`)
    } else {
      console.warn(`[Camera Config] No topic found for ${config.id} (pattern: ${config.topicPattern})`)
    }
  }

  return mapping
}
