# FSM-Pilot V2.0 - API Configuration Guide

## Overview

FSM-Pilot V2.0 integrates two powerful external APIs to enhance the remote driving platform:

1. **豆包 (Doubao) LLM** - AI-powered scenario analysis and driving suggestions
2. **高德地图 (Amap)** - Real-time vehicle location and mapping

Both APIs are **optional** and configurable. The system will gracefully degrade to local fallback functionality when APIs are not configured.

---

## Quick Start

### 1. Create Environment Configuration

Copy the example environment file:

```bash
cp .env.example .env.local
```

### 2. Configure API Keys

Edit `.env.local` and add your API keys:

```bash
# Doubao LLM API
VITE_DOUBAO_API_KEY=your_doubao_api_key_here

# Amap (高德地图) API
VITE_AMAP_API_KEY=your_amap_key_here
VITE_AMAP_JS_CODE=your_amap_js_code_here
```

### 3. Restart Development Server

```bash
npm run dev
```

The system will automatically detect and enable configured APIs.

---

## Doubao LLM API Configuration

### What is Doubao?

豆包 (Doubao) is ByteDance's large language model platform, providing powerful AI capabilities for natural language understanding and generation.

### How to Obtain API Key

1. **Visit Doubao Console**:
   - URL: https://console.volcengine.com/ark
   - Register for an account if you don't have one

2. **Create API Key**:
   - Navigate to API Keys section
   - Click "Create API Key"
   - Copy the generated key

3. **Configure in Environment**:
   ```bash
   VITE_DOUBAO_API_KEY=ep-20240115xxxxxxxxxxxxxx
   ```

### Features Enabled by Doubao

When Doubao API is configured, the AI Suggestion Panel provides:

- **Intelligent Scenario Analysis** - Contextual understanding of driving situations
- **Risk Factor Identification** - AI-powered risk assessment with explanations
- **Driving Suggestions** - Real-time recommendations based on scenario
- **Operator Guidance** - Specific instructions for remote operators
- **Natural Language Explanations** - Human-readable analysis in Chinese

### API Endpoints

- **Endpoint**: `https://ark.cn-beijing.volces.com/api/v3/chat/completions`
- **Model**: `doubao-pro-32k`
- **Method**: POST
- **Authentication**: Bearer Token

### Request Format

```json
{
  "model": "doubao-pro-32k",
  "messages": [
    {
      "role": "system",
      "content": "你是一个专业的自动驾驶远程接管系统AI助手..."
    },
    {
      "role": "user",
      "content": "场景分析请求..."
    }
  ],
  "temperature": 0.7,
  "max_tokens": 1000
}
```

### Response Format

The LLM returns a JSON response with:

```json
{
  "urgencyLevel": "high",
  "scenarioDescription": "车辆正在城市道路上行驶...",
  "riskExplanation": "当前存在以下风险因素...",
  "keyFactors": ["高速行驶", "交通密集", "天气不佳"],
  "drivingSuggestions": [
    "建议降低车速至40km/h以下",
    "保持与前车的安全距离",
    "密切关注周围交通状况"
  ],
  "operatorGuidance": "建议安全员保持高度警惕，随时准备接管..."
}
```

### Fallback Behavior

When Doubao API is not configured or unavailable:

- System uses **local rule-based analysis**
- Analysis based on risk score thresholds
- Predefined suggestion templates
- Reduced processing time (no network calls)
- UI displays "离线" (Offline) status indicator

### Cost Considerations

- **Free Tier**: 豆包 provides a free tier for testing
- **Pricing**: Check https://console.volcengine.com/ark for current pricing
- **Caching**: System caches responses for 5 minutes to reduce API calls
- **Rate Limiting**: Implement rate limiting in production environments

---

## Amap (高德地图) API Configuration

### What is Amap?

高德地图 (Amap) is one of China's leading mapping and navigation services, providing comprehensive location-based services.

### How to Obtain API Keys

1. **Visit Amap Console**:
   - URL: https://console.amap.com
   - Register/login with your account

2. **Create Application**:
   - Navigate to "应用管理" (Application Management)
   - Click "创建新应用" (Create New Application)
   - Enter application name and description

3. **Get API Key**:
   - Add "Web服务" (Web Service) key
   - Add "Web端(JS API)" key
   - Copy both keys

4. **Get Security JS Code** (Recommended):
   - Navigate to key settings
   - Generate security verification code
   - Copy the JS security code

5. **Configure in Environment**:
   ```bash
   VITE_AMAP_API_KEY=your_32_char_api_key
   VITE_AMAP_JS_CODE=your_security_code
   ```

### Features Enabled by Amap

When Amap API is configured, the map component provides:

- **Real-time Vehicle Location Display** - Live vehicle positions on map
- **Operator Location Tracking** - Safety operator positions
- **Route Visualization** - Planned and actual routes
- **Geocoding** - Convert addresses to coordinates
- **Reverse Geocoding** - Convert coordinates to addresses
- **Traffic Information** - Real-time traffic conditions
- **Map Styles** - Dark/Light theme switching

### API Integration Details

**JavaScript API Version**: 2.0

**Loaded Plugins**:
- `AMap.Geocoder` - Address and coordinate conversion
- `AMap.Marker` - Map markers for vehicles/operators
- `AMap.Polyline` - Route drawing

**Script Loading**:
```javascript
https://webapi.amap.com/maps?v=2.0&key=YOUR_KEY&plugin=AMap.Geocoder,AMap.Marker,AMap.Polyline
```

**Security Configuration**:
```javascript
window._AMapSecurityConfig = {
  securityJsCode: 'YOUR_SECURITY_CODE'
}
```

### Map Configuration

Default map settings in `AmapVehicleLocation.vue`:

```typescript
{
  zoom: 12,
  center: [114.17, 22.32],  // Hong Kong
  viewMode: '3D',
  pitch: 40,
  mapStyle: 'amap://styles/darkblue'
}
```

### Marker Types

**Vehicle Markers**:
- Color-coded by urgency level:
  - 🔴 Critical: `#ff3333`
  - 🟠 High: `#ff9933`
  - 🟡 Medium: `#ffdd33`
  - 🟢 Low: `#33ff99`
- Size: 24×24px circular markers
- Custom HTML content rendering

**Operator Markers**:
- Color-coded by status:
  - 🔵 Idle: `#00f2ff`
  - ⚫ Busy: `#888`
- Size: 32×32px circular markers
- Emoji icon: 👤

### Fallback Behavior

When Amap API is not configured:

- System displays **configuration notice** overlay
- Instructions for obtaining API keys
- Direct link to Amap console
- UI shows "未配置" (Not Configured) status
- No map functionality available

### Cost Considerations

- **Free Tier**: 100,000 requests/day for standard APIs
- **Pricing**: Check https://lbs.amap.com/pricing for current pricing
- **Optimization**: Marker updates are batched to reduce API calls
- **Best Practices**:
  - Cache geocoding results
  - Limit map refresh rate
  - Use appropriate zoom levels

---

## API Status Indicators

Both services provide visual status indicators in the UI:

### Doubao LLM Status

Located in AI Suggestion Panel header:

- 🟢 **在线** (Online) - API configured and responding
- ⚫ **离线** (Offline) - Using local fallback analysis
- Processing time display (e.g., "850ms")

### Amap Status

Located in map status bar:

- 🟢 **地图就绪** (Map Ready) - API configured and initialized
- ⚫ **未配置** (Not Configured) - API keys not set

---

## Development vs Production

### Development Environment

Use `.env.local` for local development:

```bash
# .env.local (not tracked by git)
VITE_DOUBAO_API_KEY=ep-20240115xxxxxxxxxxxxxx
VITE_AMAP_API_KEY=your_dev_key
VITE_AMAP_JS_CODE=your_dev_security_code
```

### Production Environment

Set environment variables in your deployment platform:

**Vercel**:
```bash
vercel env add VITE_DOUBAO_API_KEY production
vercel env add VITE_AMAP_API_KEY production
vercel env add VITE_AMAP_JS_CODE production
```

**Docker**:
```dockerfile
ENV VITE_DOUBAO_API_KEY=your_key
ENV VITE_AMAP_API_KEY=your_key
ENV VITE_AMAP_JS_CODE=your_security_code
```

**Nginx**:
```nginx
location / {
    fastcgi_param VITE_DOUBAO_API_KEY your_key;
    fastcgi_param VITE_AMAP_API_KEY your_key;
    fastcgi_param VITE_AMAP_JS_CODE your_security_code;
}
```

---

## Security Best Practices

### API Key Management

1. **Never commit API keys to git**:
   - `.env.local` is in `.gitignore`
   - Use `.env.example` for templates only

2. **Rotate keys regularly**:
   - Generate new keys every 90 days
   - Revoke old keys after rotation

3. **Use environment-specific keys**:
   - Separate keys for dev/staging/production
   - Different keys for different services

4. **Monitor API usage**:
   - Set up usage alerts in provider consoles
   - Track costs and request volumes
   - Detect unusual activity

### Security Configuration

**Amap Security JS Code**:
- Prevents API key abuse
- Whitelist domains in Amap console
- Rotate security codes periodically

**Doubao API**:
- Use HTTPS only
- Implement rate limiting
- Log all API requests
- Monitor for suspicious patterns

### Frontend Security

1. **API Key Exposure**:
   - Vite exposes `VITE_*` variables to client
   - Acceptable for frontend-only APIs (Amap)
   - Consider backend proxy for sensitive APIs (Doubao)

2. **Recommended Architecture** (Production):
   ```
   Frontend → Backend Proxy → Doubao API
   Frontend → Amap API (direct)
   ```

3. **Backend Proxy Benefits**:
   - Hide API keys from client
   - Implement custom rate limiting
   - Add authentication/authorization
   - Log and monitor requests

---

## Troubleshooting

### Doubao LLM Issues

**Problem**: "AI分析失败" (AI Analysis Failed)

**Solutions**:
1. Check API key is correctly set in `.env.local`
2. Verify API key is valid in Doubao console
3. Check network connectivity to `ark.cn-beijing.volces.com`
4. Review browser console for detailed error messages
5. Verify API quota hasn't been exceeded

**Problem**: Slow response times

**Solutions**:
1. Check network latency to Doubao endpoint
2. Reduce `max_tokens` in request (default: 1000)
3. Implement request debouncing
4. Use caching more aggressively

### Amap Issues

**Problem**: "高德地图未配置" (Amap Not Configured)

**Solutions**:
1. Verify `VITE_AMAP_API_KEY` is set in `.env.local`
2. Restart development server after adding keys
3. Check browser console for script loading errors
4. Verify key is enabled in Amap console

**Problem**: Map doesn't display

**Solutions**:
1. Check `VITE_AMAP_JS_CODE` is set (security verification)
2. Whitelist your domain in Amap console
3. Check browser console for CORS errors
4. Verify map container has valid dimensions
5. Check if `AMap` object is loaded: `console.log(window.AMap)`

**Problem**: Markers not appearing

**Solutions**:
1. Verify marker positions are valid coordinates
2. Check if markers are outside current view (use "适应视图" button)
3. Review browser console for marker creation errors
4. Ensure `zoom` level is appropriate

---

## API Configuration Checkers

The system provides helper functions to check API configuration:

### `isAPIConfigured()`

```typescript
import { isAPIConfigured } from '@/config/apiConfig'

// Check if Doubao is configured
if (isAPIConfigured('doubao')) {
  console.log('Doubao LLM is ready')
}

// Check if Amap is configured
if (isAPIConfigured('amap')) {
  console.log('Amap is ready')
}
```

### Service Availability Methods

**DoubaoLLMService**:
```typescript
const doubaoService = getDoubaoLLMService()
if (doubaoService.isAvailable()) {
  // Use LLM features
}
```

**AmapService**:
```typescript
const amapService = getAmapService()
if (amapService.isAvailable()) {
  // Use map features
}
```

---

## Testing Without APIs

Both services include fallback functionality for testing without API keys:

### Test Doubao Features

Without API key, the system will:
- Generate rule-based analysis
- Use predefined suggestion templates
- Display "离线" status
- Process faster (no network calls)

### Test Amap Features

Without API key, the system will:
- Show configuration notice
- Display instructions for setup
- Maintain UI structure
- Allow testing of other features

---

## Support and Resources

### Doubao (豆包)

- **Console**: https://console.volcengine.com/ark
- **Documentation**: https://www.volcengine.com/docs/82379
- **Pricing**: https://www.volcengine.com/docs/82379/1099320
- **Support**: support@volcengine.com

### Amap (高德地图)

- **Console**: https://console.amap.com
- **Documentation**: https://lbs.amap.com/api/javascript-api-v2/summary
- **Pricing**: https://lbs.amap.com/pricing
- **Forum**: https://lbs.amap.com/forum
- **Support**: https://lbs.amap.com/service/help

---

## Summary

FSM-Pilot V2.0's API integration provides powerful features while maintaining system functionality when APIs are unavailable:

| Feature | With API | Without API |
|---------|----------|-------------|
| **AI Analysis** | Doubao LLM-powered | Rule-based fallback |
| **Map Display** | Amap with full features | Configuration notice |
| **Vehicle Tracking** | Real-time on map | List view available |
| **Risk Assessment** | AI-enhanced | XGBoost model |
| **Suggestions** | Natural language | Template-based |

Configure APIs for the best experience, but the platform remains fully functional with local alternatives.

---

**Last Updated**: 2026-01-15
**Version**: FSM-Pilot V2.0
**Author**: Li Yixiang, City University of Hong Kong
