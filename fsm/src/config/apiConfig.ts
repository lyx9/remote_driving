/**
 * FSM-Pilot V2.0 - API Configuration
 *
 * Configurable API keys and endpoints for external services
 *
 * @author Li Yixiang
 * @institution City University of Hong Kong
 */

export interface APIConfig {
  // 豆包 LLM API配置
  doubao: {
    enabled: boolean
    apiKey: string
    endpoint: string
    model: string
  }

  // 高德地图 API配置
  amap: {
    enabled: boolean
    apiKey: string
    securityJsCode: string  // Web服务API密钥
  }
}

/**
 * 默认API配置
 *
 * 使用说明:
 * 1. 豆包LLM: 访问 https://console.volcengine.com/ark 获取API密钥
 * 2. 高德地图: 访问 https://console.amap.com 获取Web服务密钥
 */
export const DEFAULT_API_CONFIG: APIConfig = {
  doubao: {
    enabled: false,  // 设为true启用豆包LLM
    apiKey: import.meta.env.VITE_DOUBAO_API_KEY || '',
    endpoint: 'https://ark.cn-beijing.volces.com/api/v3',
    model: 'doubao-pro-32k'  // 可选: doubao-lite-4k, doubao-pro-4k, doubao-pro-32k
  },

  amap: {
    enabled: true,  // 设为true启用高德地图
    apiKey: import.meta.env.VITE_AMAP_API_KEY || 'fa4c4bc1d796891d00472871682f6628',
    securityJsCode: import.meta.env.VITE_AMAP_JS_CODE || '215104d11967ff4b9b17366e0bd56f0f'
  }
}

/**
 * 获取API配置
 * 优先从环境变量读取，如果环境变量未设置则使用默认值
 */
export function getAPIConfig(): APIConfig {
  return {
    doubao: {
      enabled: DEFAULT_API_CONFIG.doubao.enabled,
      apiKey: DEFAULT_API_CONFIG.doubao.apiKey,
      endpoint: DEFAULT_API_CONFIG.doubao.endpoint,
      model: DEFAULT_API_CONFIG.doubao.model
    },
    amap: {
      enabled: DEFAULT_API_CONFIG.amap.enabled,
      apiKey: DEFAULT_API_CONFIG.amap.apiKey,
      securityJsCode: DEFAULT_API_CONFIG.amap.securityJsCode
    }
  }
}

/**
 * 检查API是否配置完整
 */
export function isAPIConfigured(service: 'doubao' | 'amap'): boolean {
  const config = getAPIConfig()

  if (service === 'doubao') {
    return config.doubao.enabled && config.doubao.apiKey.length > 0
  }

  if (service === 'amap') {
    return config.amap.enabled && config.amap.apiKey.length > 0
  }

  return false
}
