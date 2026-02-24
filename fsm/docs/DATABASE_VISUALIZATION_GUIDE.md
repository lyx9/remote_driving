# FSM-Pilot V2.0 - 数据库可视化系统

**最后更新**: 2026-01-16
**版本**: 2.0

---

## 🎯 系统概述

数据库可视化系统是 FSM-Pilot V2.0 的核心数据管理模块，提供完整的自动驾驶场景数据存储、查询、可视化和导入导出功能。

### 核心功能

- ✅ **IndexedDB 本地存储** - 浏览器端高性能数据库
- ✅ **多维度数据展示** - 车辆、接管、AI分析、操作员
- ✅ **数据导入导出** - JSON 格式完整备份
- ✅ **模拟数据生成** - 一键生成测试数据
- ✅ **实时统计展示** - 关键指标卡片
- ✅ **响应式界面** - 现代化深色主题

---

## 🚀 快速开始

### 方式一：一键演示（推荐）⭐

```bash
# 1. 运行演示脚本
./demo_database.sh

# 2. 浏览器自动打开
# 3. 自动登录 cityu/2026
# 4. 导航至数据库可视化页面
```

### 方式二：手动启动

```bash
# 1. 启动开发服务器
npm run dev

# 2. 打开浏览器
# http://localhost:3000

# 3. 登录
# 用户名: cityu
# 密码: 2026

# 4. 点击顶部导航
# 🗄️ Database
```

---

## 📊 数据结构

### 1. 车辆记录 (VehicleRecord)

存储车辆的实时状态和位置信息。

```typescript
interface VehicleRecord {
  id: string                 // 车辆ID，如 "V-001"
  timestamp: number          // 记录时间戳
  location: {                // 地理位置
    longitude: number        // 经度
    latitude: number         // 纬度
    street?: string          // 街道
    district?: string        // 区域
  }
  riskScore: number          // 风险评分 (0-100)
  urgencyLevel: string       // 紧急程度: critical/high/medium/low
  scenario: string           // 场景: highway/urban/residential
  weather: string            // 天气: clear/rainy/foggy/snowy
  speed: number              // 速度 (km/h)
  controlMode: string        // 控制模式: autonomous/assisted/manual
}
```

**示例数据**:
```json
{
  "id": "V-001",
  "timestamp": 1705389600000,
  "location": {
    "longitude": 114.1693,
    "latitude": 22.3193,
    "street": "Nathan Road",
    "district": "Yau Tsim Mong"
  },
  "riskScore": 45.2,
  "urgencyLevel": "medium",
  "scenario": "urban",
  "weather": "clear",
  "speed": 55.3,
  "controlMode": "autonomous"
}
```

### 2. 接管事件 (TakeoverEvent)

记录远程接管操作的详细信息。

```typescript
interface TakeoverEvent {
  id: string                 // 事件ID，如 "TO-001"
  vehicleId: string          // 车辆ID
  operatorId: string         // 操作员ID
  timestamp: number          // 事件时间
  reason: string             // 接管原因
  riskScore: number          // 当时风险评分
  duration: number           // 接管持续时间（秒）
  outcome: string            // 结果: success/failed/cancelled
  aiAnalysis?: string        // AI分析（可选）
}
```

**示例数据**:
```json
{
  "id": "TO-001",
  "vehicleId": "V-023",
  "operatorId": "OP-003",
  "timestamp": 1705389700000,
  "reason": "检测到高风险情况",
  "riskScore": 92.3,
  "duration": 45,
  "outcome": "success",
  "aiAnalysis": "前方障碍物，已成功避让"
}
```

### 3. AI分析记录 (AIAnalysisRecord)

存储 AI 系统对场景的分析结果。

```typescript
interface AIAnalysisRecord {
  id: string                 // 分析ID，如 "AI-001"
  vehicleId: string          // 车辆ID
  timestamp: number          // 分析时间
  incidentDescription: string // 事件描述
  riskFactors: Array<{       // 风险因素
    factor: string           // 因素名称
    severity: string         // 严重程度
    value: any               // 具体值
  }>
  recommendations: string[]  // 建议列表
  confidence: number         // 置信度 (0-1)
}
```

**示例数据**:
```json
{
  "id": "AI-001",
  "vehicleId": "V-023",
  "timestamp": 1705389650000,
  "incidentDescription": "车辆检测到前方障碍物",
  "riskFactors": [
    {
      "factor": "speed",
      "severity": "high",
      "value": 85
    },
    {
      "factor": "distance",
      "severity": "critical",
      "value": 45
    }
  ],
  "recommendations": [
    "降低车速",
    "准备接管"
  ],
  "confidence": 0.92
}
```

### 4. 操作员记录 (OperatorRecord)

记录安全操作员的状态和性能。

```typescript
interface OperatorRecord {
  id: string                 // 操作员ID，如 "OP-001"
  name: string               // 姓名
  status: string             // 状态: idle/busy/offline
  assignedVehicles: string[] // 分配的车辆ID列表
  totalTakeovers: number     // 总接管次数
  successRate: number        // 成功率 (0-1)
  averageResponseTime: number // 平均响应时间（秒）
}
```

**示例数据**:
```json
{
  "id": "OP-003",
  "name": "张伟",
  "status": "busy",
  "assignedVehicles": ["V-023", "V-045"],
  "totalTakeovers": 127,
  "successRate": 0.97,
  "averageResponseTime": 2.3
}
```

---

## 🎨 界面功能说明

### 统计卡片

页面顶部显示四个实时统计卡片：

- **🚗 车辆记录** - 数据库中车辆记录总数
- **🎮 接管事件** - 接管事件总数
- **🤖 AI分析** - AI分析记录总数
- **👥 操作员** - 操作员总数

### 操作按钮

#### 1. 🔄 刷新数据
- 重新加载数据库中的所有数据
- 更新统计信息

#### 2. 📥 导入数据
- 点击打开文件选择对话框
- 支持 `.json` 格式
- 自动验证数据格式
- 导入成功后自动刷新

#### 3. 📤 导出数据
- 一键导出所有数据为 JSON 文件
- 文件名格式: `fsm-pilot-data-{timestamp}.json`
- 包含完整数据和统计信息

#### 4. 🗑️ 清空数据
- 删除所有数据库记录
- 需要确认操作
- **不可恢复**，请谨慎使用

#### 5. 生成模拟数据
- 快速生成测试数据
- 包含: 10车辆 + 5接管 + 3AI分析 + 3操作员

### 数据表格

#### 标签页切换
- **🚗 车辆记录** - 显示所有车辆状态
- **🎮 接管事件** - 显示接管历史
- **🤖 AI分析** - 显示AI分析结果
- **👥 操作员** - 显示操作员信息

#### 表格特性
- 实时数据刷新
- 悬停高亮行
- 颜色编码:
  - 🔴 紧急/失败 (红色)
  - 🟠 高风险 (橙色)
  - 🟡 中等 (黄色)
  - 🟢 低风险/成功 (绿色)

---

## 📤 导出数据格式

导出的 JSON 文件结构：

```json
{
  "version": "2.0",
  "exportTime": "2026-01-16T10:30:00.000Z",
  "data": {
    "vehicles": [...],      // 车辆记录数组
    "takeovers": [...],     // 接管事件数组
    "aiAnalyses": [...],    // AI分析数组
    "operators": [...]      // 操作员数组
  },
  "stats": {
    "totalVehicles": 10,
    "totalTakeovers": 5,
    "totalAIAnalyses": 3,
    "totalOperators": 3,
    "dateRange": {
      "start": 1705389600000,
      "end": 1705390200000
    }
  }
}
```

---

## 📥 导入数据

### 支持格式

导入文件必须是有效的 JSON 格式，包含以下字段：

```json
{
  "version": "2.0",           // 可选
  "data": {
    "vehicles": [],           // 必需
    "takeovers": [],          // 可选
    "aiAnalyses": [],         // 可选
    "operators": []           // 可选
  }
}
```

### 导入步骤

1. 点击 **📥 导入数据** 按钮
2. 在对话框中点击 **📁 选择 JSON 文件**
3. 选择要导入的文件
4. 查看文件信息（文件名、大小）
5. 点击 **确认导入**
6. 等待导入完成
7. 数据自动刷新

### 错误处理

导入失败可能的原因：
- ❌ 文件格式不正确
- ❌ JSON 解析错误
- ❌ 缺少必需字段
- ❌ 数据类型不匹配

---

## 🎲 生成模拟数据

点击 **生成模拟数据** 按钮会创建：

### 车辆记录 (10条)
- ID: V-001 到 V-010
- 随机位置、速度、风险评分
- 多种场景和天气组合

### 接管事件 (5条)
- ID: TO-001 到 TO-005
- 随机关联车辆和操作员
- 不同接管原因和结果

### AI分析 (3条)
- ID: AI-001 到 AI-003
- 模拟风险因素分析
- 随机置信度

### 操作员 (3条)
- ID: OP-001 到 OP-003
- 不同状态和性能指标
- 随机分配车辆

---

## 🔍 数据查询

### IndexedDB 表结构

数据库名: `FSM_Pilot_DB`

#### 表 1: vehicles
- 主键: `id`
- 索引: `timestamp`, `urgencyLevel`, `scenario`

#### 表 2: takeover_events
- 主键: `id`
- 索引: `vehicleId`, `operatorId`, `timestamp`, `outcome`

#### 表 3: ai_analyses
- 主键: `id`
- 索引: `vehicleId`, `timestamp`

#### 表 4: operators
- 主键: `id`
- 索引: `status`

#### 表 5: telemetry
- 主键: `id` (自增)
- 索引: `vehicleId`, `timestamp`

### 浏览器 DevTools 查看

1. 按 **F12** 打开开发者工具
2. 切换到 **Application** 标签页
3. 左侧找到 **IndexedDB**
4. 展开 **FSM_Pilot_DB**
5. 点击任意表查看数据

---

## 🛠️ API 使用

### 初始化数据库

```typescript
import databaseService from '@/services/databaseService'

// 初始化
await databaseService.initialize()
```

### 保存数据

```typescript
// 保存车辆记录
await databaseService.saveVehicle({
  id: 'V-001',
  timestamp: Date.now(),
  location: { longitude: 114.17, latitude: 22.32 },
  riskScore: 45.2,
  urgencyLevel: 'medium',
  scenario: 'urban',
  weather: 'clear',
  speed: 55.3,
  controlMode: 'autonomous'
})

// 保存接管事件
await databaseService.saveTakeoverEvent({
  id: 'TO-001',
  vehicleId: 'V-001',
  operatorId: 'OP-001',
  timestamp: Date.now(),
  reason: '高风险情况',
  riskScore: 85.5,
  duration: 45,
  outcome: 'success'
})
```

### 查询数据

```typescript
// 获取所有车辆
const vehicles = await databaseService.getAllVehicles()

// 获取所有接管事件
const takeovers = await databaseService.getAllTakeovers()

// 获取统计信息
const stats = await databaseService.getStats()
```

### 导入导出

```typescript
// 导出为 JSON
const jsonData = await databaseService.exportToJSON()

// 导入 JSON
await databaseService.importFromJSON(jsonString)

// 清空所有数据
await databaseService.clearAll()
```

---

## 🎯 使用场景

### 场景 1: 数据备份

```bash
# 1. 运行系统并生成数据
./demo_database.sh

# 2. 在浏览器中导出数据
点击 📤 导出数据

# 3. 保存 JSON 文件到安全位置
mv ~/Downloads/fsm-pilot-data-*.json ./backups/
```

### 场景 2: 数据恢复

```bash
# 1. 启动系统
./demo_database.sh

# 2. 清空当前数据（可选）
点击 🗑️ 清空数据

# 3. 导入备份文件
点击 📥 导入数据
选择备份的 JSON 文件
```

### 场景 3: 演示准备

```bash
# 1. 生成模拟数据
点击 「生成模拟数据」

# 2. 浏览各个标签页
🚗 车辆记录 → 🎮 接管事件 → 🤖 AI分析 → 👥 操作员

# 3. 展示导入导出功能
导出 JSON → 清空 → 重新导入
```

### 场景 4: 数据分析

```bash
# 1. 导出数据
./demo_database.sh
点击 📤 导出数据

# 2. 使用外部工具分析
python analyze_data.py fsm-pilot-data-*.json

# 3. 可视化结果
# 使用 Python/R 等工具进行数据分析
```

---

## 🔐 数据安全

### 本地存储

- 数据存储在浏览器的 IndexedDB 中
- 仅限当前域名访问
- 不会自动上传到服务器

### 数据隐私

- 所有数据保存在本地
- 导出文件由用户控制
- 可以随时清空数据

### 备份建议

1. **定期导出**: 每周导出一次数据
2. **多重备份**: 保存到云端和本地
3. **版本控制**: 使用时间戳命名文件
4. **验证完整性**: 导入测试确保备份可用

---

## 📊 性能优化

### IndexedDB 优化

- 使用索引加速查询
- 批量操作减少事务次数
- 异步操作不阻塞 UI

### UI 优化

- 虚拟滚动处理大数据量
- 分页加载避免一次加载过多
- 防抖/节流优化搜索

### 内存管理

- 及时清理不用的数据
- 使用 WeakMap 避免内存泄漏
- 监控数据库大小

---

## 🐛 故障排查

### 问题 1: 数据不显示

**解决方案**:
```
1. 检查浏览器控制台是否有错误
2. 点击 🔄 刷新数据
3. F12 → Application → IndexedDB 检查数据是否存在
4. 尝试生成模拟数据
```

### 问题 2: 导入失败

**解决方案**:
```
1. 检查 JSON 文件格式是否正确
2. 确认文件包含 "data" 字段
3. 使用 JSON 验证工具检查文件
4. 查看浏览器控制台的错误信息
```

### 问题 3: 导出为空

**解决方案**:
```
1. 确认数据库中有数据
2. 点击生成模拟数据
3. 刷新后重试导出
```

### 问题 4: 浏览器兼容性

**支持的浏览器**:
- ✅ Chrome 90+
- ✅ Firefox 88+
- ✅ Edge 90+
- ✅ Safari 14+

---

## 📞 技术支持

**遇到问题？**

1. 查看浏览器控制台 (F12 → Console)
2. 检查 IndexedDB 状态 (F12 → Application)
3. 阅读本文档的故障排查部分
4. 联系技术支持: li.yixiang@cityu.edu.hk

---

## 🎉 总结

数据库可视化系统提供了完整的数据管理解决方案：

- ✅ **简单易用** - 一键演示，无需配置
- ✅ **功能完整** - 增删改查导入导出全支持
- ✅ **性能优异** - IndexedDB 高效存储
- ✅ **界面美观** - 现代化深色主题
- ✅ **文档完善** - 详细的使用说明

立即开始使用：
```bash
./demo_database.sh
```

---

**最后更新**: 2026-01-16
**版本**: 2.0
**作者**: Li Yixiang, City University of Hong Kong
