import logger from '@/utils/logger'
/**
 * FSM-Pilot V2.0 - Doubao LLM Service
 *
 * Integration with ByteDance Doubao (豆包) LLM for:
 * - Driving scenario analysis
 * - Safety suggestions
 * - Risk assessment explanations
 * - Operator guidance
 *
 * @author Li Yixiang
 * @institution City University of Hong Kong
 */

import { getAPIConfig, isAPIConfigured } from '@/config/apiConfig'
import type { RiskScore, VehicleRiskFeatures } from './riskScoringService'
import type { ControlMode } from './adaptiveControlModeService'

export interface ScenarioContext {
  vehicleId: string
  location: string
  scenario: string
  weather: string
  riskScore: RiskScore
  riskFeatures: VehicleRiskFeatures
  controlMode: ControlMode
  speed: number
  trafficDensity: number
}

export interface LLMAnalysis {
  scenarioDescription: string      // 场景描述
  riskExplanation: string          // 风险解释
  drivingSuggestions: string[]     // 驾驶建议
  operatorGuidance: string         // 安全员指导
  urgencyLevel: 'critical' | 'high' | 'medium' | 'low'
  keyFactors: string[]             // 关键因素
  timestamp: number
  processingTime: number           // 处理时间(ms)
}

export interface DoubaoMessage {
  role: 'system' | 'user' | 'assistant'
  content: string
}

export interface DoubaoRequest {
  model: string
  messages: DoubaoMessage[]
  temperature?: number
  max_tokens?: number
  top_p?: number
}

export interface DoubaoResponse {
  id: string
  object: string
  created: number
  model: string
  choices: Array<{
    index: number
    message: {
      role: string
      content: string
    }
    finish_reason: string
  }>
  usage: {
    prompt_tokens: number
    completion_tokens: number
    total_tokens: number
  }
}

/**
 * Doubao LLM Service
 */
class DoubaoLLMService {
  private apiConfig = getAPIConfig()
  private requestCache = new Map<string, LLMAnalysis>()
  private cacheExpiration = 5 * 60 * 1000  // 5分钟缓存

  /**
   * Check if Doubao LLM is available
   */
  isAvailable(): boolean {
    return isAPIConfigured('doubao')
  }

  /**
   * Analyze driving scenario and provide suggestions
   */
  async analyzeScenario(context: ScenarioContext): Promise<LLMAnalysis> {
    const startTime = performance.now()

    // Check if Doubao is configured
    if (!this.isAvailable()) {
      logger.warn('[Doubao] LLM not configured, using fallback analysis', 'system')
      return this.generateFallbackAnalysis(context, startTime)
    }

    // Check cache
    const cacheKey = this.getCacheKey(context)
    const cached = this.requestCache.get(cacheKey)
    if (cached && Date.now() - cached.timestamp < this.cacheExpiration) {
      logger.info('[Doubao] Using cached analysis', 'system')
      return cached
    }

    try {
      // Call Doubao API
      const analysis = await this.callDoubaoAPI(context)
      analysis.processingTime = performance.now() - startTime

      // Cache result
      this.requestCache.set(cacheKey, analysis)

      return analysis
    } catch (error) {
      logger.error('[Doubao] API call failed:', error, 'system')
      return this.generateFallbackAnalysis(context, startTime)
    }
  }

  /**
   * Call Doubao API
   */
  private async callDoubaoAPI(context: ScenarioContext): Promise<LLMAnalysis> {
    const prompt = this.buildPrompt(context)

    const request: DoubaoRequest = {
      model: this.apiConfig.doubao.model,
      messages: [
        {
          role: 'system',
          content: '你是一个专业的自动驾驶远程接管系统AI助手。你的任务是分析车辆当前状态、识别风险因素、并提供专业的驾驶建议和安全员指导。请用简洁、专业、易懂的语言回答。'
        },
        {
          role: 'user',
          content: prompt
        }
      ],
      temperature: 0.7,
      max_tokens: 1000,
      top_p: 0.9
    }

    const response = await fetch(`${this.apiConfig.doubao.endpoint}/chat/completions`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.apiConfig.doubao.apiKey}`
      },
      body: JSON.stringify(request)
    })

    if (!response.ok) {
      throw new Error(`Doubao API error: ${response.status} ${response.statusText}`)
    }

    const data: DoubaoResponse = await response.json()
    const content = data.choices[0]?.message?.content || ''

    return this.parseDoubaoResponse(content, context)
  }

  /**
   * Build prompt for Doubao
   */
  private buildPrompt(context: ScenarioContext): string {
    const { vehicleId, scenario, weather, riskScore, riskFeatures, controlMode, speed, trafficDensity } = context

    return `
请分析以下自动驾驶车辆的实时状态并提供专业建议：

【车辆信息】
- 车辆ID: ${vehicleId}
- 当前位置: ${context.location}

【驾驶场景】
- 场景类型: ${this.translateScenario(scenario)}
- 天气条件: ${this.translateWeather(weather)}
- 当前车速: ${speed} km/h
- 交通密度: ${trafficDensity}辆车

【风险评估】
- 综合风险分数: ${(riskScore.overallScore * 100).toFixed(1)}%
- 紧急等级: ${this.translateUrgency(riskScore.urgencyLevel)}
- 车辆风险: ${(riskScore.components.vehicleRisk * 100).toFixed(1)}%
- 环境风险: ${(riskScore.components.environmentRisk * 100).toFixed(1)}%
- 系统风险: ${(riskScore.components.systemRisk * 100).toFixed(1)}%
- 安全员风险: ${(riskScore.components.operatorRisk * 100).toFixed(1)}%

【系统状态】
- 控制模式: ${this.translateControlMode(controlMode)}
- 系统健康度: ${(riskFeatures.systemHealth * 100).toFixed(0)}%
- 传感器状态: ${(riskFeatures.sensorStatus * 100).toFixed(0)}%
- 网络延迟: ${riskFeatures.networkLatency}ms
- 轨迹偏离: ${riskFeatures.trajectoryDeviation.toFixed(1)}m
- 紧急制动次数: ${riskFeatures.emergencyBrakeCount}

请按以下格式回答（使用JSON格式）：
{
  "scenarioDescription": "用1-2句话描述当前驾驶场景",
  "riskExplanation": "解释当前主要风险因素和原因",
  "drivingSuggestions": ["建议1", "建议2", "建议3"],
  "operatorGuidance": "给安全员的具体操作指导",
  "keyFactors": ["关键因素1", "关键因素2", "关键因素3"]
}
`.trim()
  }

  /**
   * Parse Doubao response
   */
  private parseDoubaoResponse(content: string, context: ScenarioContext): LLMAnalysis {
    try {
      // Try to extract JSON from response
      const jsonMatch = content.match(/\{[\s\S]*\}/)
      if (jsonMatch) {
        const parsed = JSON.parse(jsonMatch[0])
        return {
          scenarioDescription: parsed.scenarioDescription || '',
          riskExplanation: parsed.riskExplanation || '',
          drivingSuggestions: parsed.drivingSuggestions || [],
          operatorGuidance: parsed.operatorGuidance || '',
          urgencyLevel: context.riskScore.urgencyLevel,
          keyFactors: parsed.keyFactors || [],
          timestamp: Date.now(),
          processingTime: 0
        }
      }
    } catch (error) {
      logger.warn('[Doubao] Failed to parse JSON response, using fallback', 'system')
    }

    // Fallback: use plain text parsing
    return this.parseTextResponse(content, context)
  }

  /**
   * Parse plain text response
   */
  private parseTextResponse(content: string, context: ScenarioContext): LLMAnalysis {
    const lines = content.split('\n').filter(l => l.trim())

    return {
      scenarioDescription: lines[0] || '当前驾驶场景分析中...',
      riskExplanation: lines[1] || context.riskScore.recommendation,
      drivingSuggestions: lines.slice(2, 5).filter(l => l.length > 0),
      operatorGuidance: lines[lines.length - 1] || '保持警惕，随时准备接管',
      urgencyLevel: context.riskScore.urgencyLevel,
      keyFactors: [],
      timestamp: Date.now(),
      processingTime: 0
    }
  }

  /**
   * Generate fallback analysis (when Doubao is not available)
   */
  private generateFallbackAnalysis(context: ScenarioContext, startTime: number): LLMAnalysis {
    const { scenario, weather, riskScore, riskFeatures, controlMode } = context

    // Scenario description
    const scenarioDesc = `车辆正在${this.translateScenario(scenario)}行驶，天气${this.translateWeather(weather)}，使用${this.translateControlMode(controlMode)}模式`

    // Risk explanation
    const riskFactors: string[] = []
    if (riskScore.components.vehicleRisk > 0.5) {
      riskFactors.push('车辆控制不稳定')
    }
    if (riskScore.components.environmentRisk > 0.5) {
      riskFactors.push('环境复杂度较高')
    }
    if (riskScore.components.systemRisk > 0.5) {
      riskFactors.push('系统健康度下降')
    }
    if (riskFeatures.emergencyBrakeCount > 0) {
      riskFactors.push(`已紧急制动${riskFeatures.emergencyBrakeCount}次`)
    }

    const riskExplanation = riskFactors.length > 0
      ? `主要风险因素：${riskFactors.join('、')}`
      : '当前驾驶状况正常，风险可控'

    // Driving suggestions
    const suggestions: string[] = []
    if (riskScore.urgencyLevel === 'critical' || riskScore.urgencyLevel === 'high') {
      suggestions.push('建议立即准备接管车辆')
      suggestions.push('降低车速，增加安全裕度')
      suggestions.push('密切监控车辆状态和周围环境')
    } else if (riskScore.urgencyLevel === 'medium') {
      suggestions.push('保持高度警惕，准备随时干预')
      suggestions.push('关注车辆轨迹和系统状态')
      suggestions.push('预判潜在风险场景')
    } else {
      suggestions.push('维持当前控制模式')
      suggestions.push('定期检查系统状态')
      suggestions.push('保持对路况的监控')
    }

    // Operator guidance
    let operatorGuidance = ''
    if (controlMode === 'direct') {
      operatorGuidance = '使用直接控制模式，实时操控车辆。注意方向盘和油门刹车的协调配合'
    } else if (controlMode === 'trajectory') {
      operatorGuidance = '使用轨迹确认模式，审核并确认车辆规划的行驶路径'
    } else {
      operatorGuidance = '使用语义指令模式，通过高层命令指导车辆行驶'
    }

    // Key factors
    const keyFactors: string[] = []
    if (riskFeatures.speed > 80) keyFactors.push('高速行驶')
    if (riskFeatures.trafficDensity > 10) keyFactors.push('交通密集')
    if (riskFeatures.weatherSeverity > 0.5) keyFactors.push('恶劣天气')
    if (riskFeatures.systemHealth < 0.7) keyFactors.push('系统异常')
    if (riskFeatures.networkLatency > 200) keyFactors.push('网络延迟')

    return {
      scenarioDescription: scenarioDesc,
      riskExplanation: riskExplanation,
      drivingSuggestions: suggestions,
      operatorGuidance: operatorGuidance,
      urgencyLevel: riskScore.urgencyLevel,
      keyFactors: keyFactors.length > 0 ? keyFactors : ['正常行驶'],
      timestamp: Date.now(),
      processingTime: performance.now() - startTime
    }
  }

  /**
   * Helper: Generate cache key
   */
  private getCacheKey(context: ScenarioContext): string {
    return `${context.vehicleId}-${context.riskScore.urgencyLevel}-${Math.floor(Date.now() / 30000)}`
  }

  /**
   * Helper: Translate scenario
   */
  private translateScenario(scenario: string): string {
    const map: Record<string, string> = {
      highway: '高速公路',
      urban: '城市道路',
      residential: '住宅区',
      parking: '停车场'
    }
    return map[scenario] || scenario
  }

  /**
   * Helper: Translate weather
   */
  private translateWeather(weather: string): string {
    const map: Record<string, string> = {
      clear: '晴朗',
      rain: '雨天',
      snow: '雪天',
      fog: '雾天'
    }
    return map[weather] || weather
  }

  /**
   * Helper: Translate urgency
   */
  private translateUrgency(urgency: string): string {
    const map: Record<string, string> = {
      critical: '紧急',
      high: '高',
      medium: '中',
      low: '低'
    }
    return map[urgency] || urgency
  }

  /**
   * Helper: Translate control mode
   */
  private translateControlMode(mode: ControlMode): string {
    const map: Record<ControlMode, string> = {
      direct: '直接控制',
      trajectory: '轨迹确认',
      semantic: '语义指令'
    }
    return map[mode] || mode
  }

  /**
   * Clear cache
   */
  clearCache(): void {
    this.requestCache.clear()
  }

  /**
   * Get cache statistics
   */
  getCacheStats() {
    return {
      size: this.requestCache.size,
      keys: Array.from(this.requestCache.keys())
    }
  }
}

// Singleton instance
let doubaoLLMService: DoubaoLLMService | null = null

export function getDoubaoLLMService(): DoubaoLLMService {
  if (!doubaoLLMService) {
    doubaoLLMService = new DoubaoLLMService()
  }
  return doubaoLLMService
}

export { DoubaoLLMService }
