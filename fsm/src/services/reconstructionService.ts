/**
 * Guardian Mobility v0.0 - 3DGS Reconstruction Service
 *
 * 3D Gaussian Splatting重建服务API
 *
 * @author Li Yixiang
 * @institution City University of Hong Kong
 */

export interface ReconstructionConfig {
  rosbagPath: string
  outputDir: string
  imageTopic?: string
  cameraInfoTopic?: string
  maxImages?: number
  colmapQuality?: 'low' | 'medium' | 'high' | 'extreme'
  colmapMatcher?: 'exhaustive' | 'sequential'
  iterations?: number
  resolution?: number
  shDegree?: number
  useGpu?: boolean
  gpuId?: number
  downsampleFactor?: number
}

export interface ReconstructionResult {
  success: boolean
  message: string
  outputPath?: string
  colmapPath?: string
  gaussianPath?: string
  viewerUrl?: string
  stats?: {
    image_count: number
    duration_seconds: number
    start_time: string
    end_time: string
  }
  error?: string
}

export interface ReconstructionProgress {
  phase: string
  percentage: number
  message: string
  elapsedTime: number
  estimatedRemaining: number
}

/**
 * 3DGS重建服务
 */
class ReconstructionService {
  private apiEndpoint = 'http://localhost:5000/api/reconstruction'
  private ws: WebSocket | null = null
  private progressCallback: ((progress: ReconstructionProgress) => void) | null = null
  private logCallback: ((log: { level: string; message: string }) => void) | null = null

  /**
   * 开始重建
   */
  async startReconstruction(config: ReconstructionConfig): Promise<{ jobId: string }> {
    try {
      const response = await fetch(`${this.apiEndpoint}/start`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(config)
      })

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`)
      }

      const result = await response.json()
      return result
    } catch (error) {
      console.error('[Reconstruction] 启动失败:', error)
      throw error
    }
  }

  /**
   * 连接WebSocket监听进度
   */
  connectProgress(
    jobId: string,
    onProgress: (progress: ReconstructionProgress) => void,
    onLog: (log: { level: string; message: string }) => void
  ): void {
    this.progressCallback = onProgress
    this.logCallback = onLog

    const wsUrl = `ws://localhost:5000/ws/reconstruction/${jobId}`
    this.ws = new WebSocket(wsUrl)

    this.ws.onopen = () => {
      console.log('[Reconstruction] WebSocket连接成功')
    }

    this.ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data)

        if (data.type === 'progress' && this.progressCallback) {
          this.progressCallback(data.data)
        } else if (data.type === 'log' && this.logCallback) {
          this.logCallback(data.data)
        }
      } catch (error) {
        console.error('[Reconstruction] 解析消息失败:', error)
      }
    }

    this.ws.onerror = (error) => {
      console.error('[Reconstruction] WebSocket错误:', error)
    }

    this.ws.onclose = () => {
      console.log('[Reconstruction] WebSocket连接关闭')
    }
  }

  /**
   * 断开WebSocket
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close()
      this.ws = null
    }
    this.progressCallback = null
    this.logCallback = null
  }

  /**
   * 获取重建状态
   */
  async getStatus(jobId: string): Promise<ReconstructionProgress> {
    try {
      const response = await fetch(`${this.apiEndpoint}/status/${jobId}`)

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`)
      }

      return await response.json()
    } catch (error) {
      console.error('[Reconstruction] 获取状态失败:', error)
      throw error
    }
  }

  /**
   * 获取重建结果
   */
  async getResult(jobId: string): Promise<ReconstructionResult> {
    try {
      const response = await fetch(`${this.apiEndpoint}/result/${jobId}`)

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`)
      }

      return await response.json()
    } catch (error) {
      console.error('[Reconstruction] 获取结果失败:', error)
      throw error
    }
  }

  /**
   * 取消重建
   */
  async cancelReconstruction(jobId: string): Promise<void> {
    try {
      const response = await fetch(`${this.apiEndpoint}/cancel/${jobId}`, {
        method: 'POST'
      })

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`)
      }
    } catch (error) {
      console.error('[Reconstruction] 取消失败:', error)
      throw error
    }
  }

  /**
   * 获取历史记录
   */
  async getHistory(): Promise<Array<ReconstructionResult>> {
    try {
      const response = await fetch(`${this.apiEndpoint}/history`)

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`)
      }

      return await response.json()
    } catch (error) {
      console.error('[Reconstruction] 获取历史失败:', error)
      throw error
    }
  }

  /**
   * 删除重建结果
   */
  async deleteResult(jobId: string): Promise<void> {
    try {
      const response = await fetch(`${this.apiEndpoint}/delete/${jobId}`, {
        method: 'DELETE'
      })

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`)
      }
    } catch (error) {
      console.error('[Reconstruction] 删除失败:', error)
      throw error
    }
  }

  /**
   * 上传ROS bag文件
   */
  async uploadRosbag(
    file: File,
    onProgress?: (percentage: number) => void
  ): Promise<{ path: string }> {
    try {
      const formData = new FormData()
      formData.append('file', file)

      const xhr = new XMLHttpRequest()

      return new Promise((resolve, reject) => {
        xhr.upload.addEventListener('progress', (event) => {
          if (event.lengthComputable && onProgress) {
            const percentage = (event.loaded / event.total) * 100
            onProgress(percentage)
          }
        })

        xhr.addEventListener('load', () => {
          if (xhr.status === 200) {
            const result = JSON.parse(xhr.responseText)
            resolve(result)
          } else {
            reject(new Error(`Upload failed: ${xhr.statusText}`))
          }
        })

        xhr.addEventListener('error', () => {
          reject(new Error('Upload failed'))
        })

        xhr.open('POST', `${this.apiEndpoint}/upload`)
        xhr.send(formData)
      })
    } catch (error) {
      console.error('[Reconstruction] 上传失败:', error)
      throw error
    }
  }

  /**
   * 检查系统依赖
   */
  async checkDependencies(): Promise<{
    ros: boolean
    colmap: boolean
    gaussian_splatting: boolean
    gpu: boolean
  }> {
    try {
      const response = await fetch(`${this.apiEndpoint}/check-dependencies`)

      if (!response.ok) {
        throw new Error(`HTTP ${response.status}: ${response.statusText}`)
      }

      return await response.json()
    } catch (error) {
      console.error('[Reconstruction] 检查依赖失败:', error)
      throw error
    }
  }
}

// 单例
let reconstructionService: ReconstructionService | null = null

export function getReconstructionService(): ReconstructionService {
  if (!reconstructionService) {
    reconstructionService = new ReconstructionService()
  }
  return reconstructionService
}

export default ReconstructionService
