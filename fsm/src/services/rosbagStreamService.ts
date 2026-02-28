import logger from '@/utils/logger'
/**
 * FSM-Pilot V2.0 - RosBag Streaming Client Service
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      RosBag Streaming Client
 * @description WebSocket client for streaming large RosBag files from backend
 */

import { ref, computed, type Ref } from 'vue'

// ======================== Types ========================

export interface RosBagInfo {
  name: string
  path: string
  size: number
  format?: string
  db3_files?: number
  message_count?: number
  duration?: number
  // Legacy ROS1 properties (optional for backward compatibility)
  messageCount?: number
  topicCount?: number
  sizeGB?: string
}

export interface TopicInfo {
  name: string
  type: string
  messageCount?: number
  message_count?: number
}

export interface RosBagMessage {
  topic: string
  topicType: string
  timestamp: number
  data: string // base64 encoded
}

export interface StreamOptions {
  topics: string[]
  startTime?: number
  endTime?: number
  playbackRate?: number
}

type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error'
type StreamState = 'idle' | 'streaming' | 'paused' | 'stopped'

// ======================== RosBag Stream Service ========================

class RosBagStreamService {
  private ws: WebSocket | null = null
  private serverUrl: string
  private reconnectAttempts = 0
  private maxReconnectAttempts = 5
  private reconnectDelay = 2000

  // Reactive state
  public connectionState: Ref<ConnectionState> = ref('disconnected')
  public streamState: Ref<StreamState> = ref('idle')
  public availableBags: Ref<RosBagInfo[]> = ref([])
  public currentBag: Ref<RosBagInfo | null> = ref(null)
  public topics: Ref<TopicInfo[]> = ref([])
  public messageCount: Ref<number> = ref(0)
  public error: Ref<string | null> = ref(null)

  // Message handlers
  private messageHandlers: Map<string, (data: any) => void> = new Map()

  constructor(serverUrl: string = 'ws://localhost:8765') {
    this.serverUrl = serverUrl
  }

  // ======================== Connection Management ========================

  async connect(): Promise<void> {
    if (this.ws?.readyState === WebSocket.OPEN) {
      logger.info('[RosBagStreamService] Already connected', 'ros')
      return
    }

    return new Promise((resolve, reject) => {
      try {
        logger.info(`[RosBagStreamService] Connecting to ${this.serverUrl}`, 'ros')
        this.connectionState.value = 'connecting'
        this.error.value = null

        this.ws = new WebSocket(this.serverUrl)

        this.ws.onopen = () => {
          logger.info('[RosBagStreamService] Connected', 'ros')
          this.connectionState.value = 'connected'
          this.reconnectAttempts = 0
          resolve()
        }

        this.ws.onmessage = (event) => {
          this.handleMessage(event.data)
        }

        this.ws.onerror = (error) => {
          logger.error('[RosBagStreamService] WebSocket error:', error, 'ros')
          this.connectionState.value = 'error'
          this.error.value = 'WebSocket connection error'
          reject(error)
        }

        this.ws.onclose = () => {
          logger.info('[RosBagStreamService] Disconnected', 'ros')
          this.connectionState.value = 'disconnected'
          this.attemptReconnect()
        }
      } catch (error) {
        logger.error('[RosBagStreamService] Connection error:', error, 'ros')
        this.connectionState.value = 'error'
        this.error.value = error instanceof Error ? error.message : 'Connection failed'
        reject(error)
      }
    })
  }

  disconnect(): void {
    if (this.ws) {
      logger.info('[RosBagStreamService] Disconnecting', 'ros')
      this.ws.close()
      this.ws = null
      this.connectionState.value = 'disconnected'
    }
  }

  private attemptReconnect(): void {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      logger.info('[RosBagStreamService] Max reconnect attempts reached', 'ros')
      this.error.value = 'Failed to reconnect to server'
      return
    }

    this.reconnectAttempts++
    logger.ros(`Reconnecting in ${this.reconnectDelay}ms (attempt ${this.reconnectAttempts}/${this.maxReconnectAttempts})`)

    setTimeout(() => {
      this.connect().catch((error) => {
        logger.error('[RosBagStreamService] Reconnect failed:', error, 'ros')
      })
    }, this.reconnectDelay)
  }

  // ======================== Message Handling ========================

  private handleMessage(data: string): void {
    try {
      const message = JSON.parse(data)
      logger.info('[RosBagStreamService] Received message:', message.type, 'ros')

      switch (message.type) {
        case 'bag_list':
          logger.info('[RosBagStreamService] Setting availableBags, count:', message.bags?.length, 'ros')
          this.availableBags.value = message.bags
          break

        case 'bag_opened':
          this.topics.value = message.topics
          break

        case 'topics':
          this.topics.value = message.topics
          break

        case 'stream_started':
          this.streamState.value = 'streaming'
          this.messageCount.value = 0
          break

        case 'message':
          this.handleRosBagMessage(message)
          break

        case 'stream_progress':
          this.messageCount.value = message.messageCount
          break

        case 'stream_complete':
          this.streamState.value = 'stopped'
          logger.info(`[RosBagStreamService] Stream complete: ${message.messageCount} messages`, 'ros')
          break

        case 'stream_stopped':
          this.streamState.value = 'stopped'
          break

        case 'bag_closed':
          this.currentBag.value = null
          this.topics.value = []
          break

        case 'error':
          logger.error('[RosBagStreamService] Server error:', message.message, 'ros')
          this.error.value = message.message
          break

        default:
          logger.warn('[RosBagStreamService] Unknown message type:', message.type, 'ros')
      }

      // Call registered handlers
      const handler = this.messageHandlers.get(message.type)
      if (handler) {
        handler(message)
      }
    } catch (error) {
      logger.error('[RosBagStreamService] Error handling message:', error, 'ros')
    }
  }

  private handleRosBagMessage(message: RosBagMessage): void {
    this.messageCount.value++

    // Call topic-specific handlers
    const handler = this.messageHandlers.get(`topic:${message.topic}`)
    if (handler) {
      handler(message)
    }

    // Call general message handler
    const generalHandler = this.messageHandlers.get('message')
    if (generalHandler) {
      generalHandler(message)
    }
  }

  // ======================== Public API ========================

  async listBags(): Promise<RosBagInfo[]> {
    logger.info('[RosBagStreamService] Sending list_bags command', 'ros')
    this.sendCommand({ command: 'list_bags' })
    return new Promise((resolve) => {
      const handler = (data: any) => {
        logger.info('[RosBagStreamService] list_bags handler called, bags:', data.bags?.length, 'ros')
        this.messageHandlers.delete('bag_list')
        resolve(data.bags)
      }
      this.messageHandlers.set('bag_list', handler)
    })
  }

  async openBag(bagPath: string): Promise<void> {
    logger.info(`[RosBagStreamService] Opening bag: ${bagPath}`, 'ros')
    this.sendCommand({ command: 'open_bag', bagPath })

    return new Promise((resolve, reject) => {
      const handler = (data: any) => {
        this.messageHandlers.delete('bag_opened')
        if (data.type === 'error') {
          reject(new Error(data.message))
        } else {
          this.currentBag.value = this.availableBags.value.find((b) => b.path === bagPath) || null
          resolve()
        }
      }
      this.messageHandlers.set('bag_opened', handler)
    })
  }

  closeBag(): void {
    this.sendCommand({ command: 'close_bag' })
    this.currentBag.value = null
    this.topics.value = []
  }

  async getTopics(): Promise<TopicInfo[]> {
    this.sendCommand({ command: 'get_topics' })
    return new Promise((resolve) => {
      const handler = (data: any) => {
        this.messageHandlers.delete('topics')
        resolve(data.topics)
      }
      this.messageHandlers.set('topics', handler)
    })
  }

  streamTopics(options: StreamOptions): void {
    logger.info('[RosBagStreamService] Starting stream:', options, 'ros')
    this.sendCommand({
      command: 'stream_topics',
      ...options
    })
  }

  stopStream(): void {
    logger.info('[RosBagStreamService] Stopping stream', 'ros')
    this.sendCommand({ command: 'stop_stream' })
    this.streamState.value = 'stopped'
  }

  // ======================== Event Handlers ========================

  onMessage(callback: (message: RosBagMessage) => void): void {
    this.messageHandlers.set('message', callback)
  }

  onTopic(topicName: string, callback: (message: RosBagMessage) => void): void {
    this.messageHandlers.set(`topic:${topicName}`, callback)
  }

  removeMessageHandler(): void {
    this.messageHandlers.delete('message')
  }

  removeTopicHandler(topicName: string): void {
    this.messageHandlers.delete(`topic:${topicName}`)
  }

  // ======================== Utilities ========================

  private sendCommand(command: any): void {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      logger.error('[RosBagStreamService] WebSocket not connected', 'ros')
      this.error.value = 'Not connected to server'
      return
    }

    try {
      this.ws.send(JSON.stringify(command))
    } catch (error) {
      logger.error('[RosBagStreamService] Error sending command:', error, 'ros')
      this.error.value = error instanceof Error ? error.message : 'Failed to send command'
    }
  }

  get isConnected(): boolean {
    return this.connectionState.value === 'connected'
  }

  get isStreaming(): boolean {
    return this.streamState.value === 'streaming'
  }
}

// ======================== Singleton Instance ========================

let serviceInstance: RosBagStreamService | null = null

export function getRosBagStreamService(serverUrl?: string): RosBagStreamService {
  if (!serviceInstance) {
    serviceInstance = new RosBagStreamService(serverUrl)
  }
  return serviceInstance
}

export default RosBagStreamService
