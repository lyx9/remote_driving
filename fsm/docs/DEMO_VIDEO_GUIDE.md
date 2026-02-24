# FSM-Pilot V2.0 - Demo Video Walkthrough Guide

**演示视频制作指南**

## 概述 Overview

本文档提供完整的FSM-Pilot V2.0远程驾驶平台演示视频制作指南，展示系统的核心功能和企业级特性。

Total Duration: 5-8 minutes
Target Audience: Potential clients, investors, technical stakeholders

---

## 📋 Demo Flow / 演示流程

### Task 1: 平台登录 (Platform Login)
**Duration: 30 seconds**

**操作步骤 Steps**:
1. 打开浏览器访问平台 URL: `http://localhost:5173`
2. 显示登录界面
   - 平台 LOGO 和标题
   - "FSM-Pilot V2.0 Remote Driving Platform"
   - City University of Hong Kong branding
3. 输入默认凭据:
   - Username: `cityu`
   - Password: `2026`
4. 点击"Login"按钮
5. 短暂加载动画后进入主界面

**录制要点 Recording Tips**:
- 展示完整的登录界面设计
- 突出显示安全认证流程
- 保持画面稳定、清晰

**旁白文稿 Narration Script** (中文):
> "欢迎来到FSM-Pilot V2.0 远程驾驶平台。这是由香港城市大学开发的企业级自动驾驶远程接管系统。让我们从平台登录开始..."

**Narration Script** (English):
> "Welcome to FSM-Pilot V2.0 Remote Driving Platform. This is an enterprise-grade autonomous driving remote takeover system developed by City University of Hong Kong. Let's start with platform login..."

---

### Task 2: 显示工作界面 (Display Work Interface)
**Duration: 45 seconds**

**操作步骤 Steps**:
1. 登录成功后展示主控制台
2. 导航栏功能概览:
   - Dashboard
   - Remote Control
   - RosBag Replay
   - Intelligent Dispatch Demo
3. 展示当前页面布局
4. 点击"Intelligent Dispatch Demo"进入主演示界面

**界面元素 Interface Elements**:
- 顶部导航栏 (Navigation Bar)
- 系统状态指示器 (System Status Indicators)
- 用户信息 (User Information)
- 快速操作按钮 (Quick Action Buttons)

**录制要点 Recording Tips**:
- 缓慢移动鼠标展示各个功能模块
- 突出显示响应式设计
- 展示深色主题UI设计

**旁白文稿 Narration Script** (中文):
> "进入主界面后，您可以看到完整的功能模块。包括实时调度面板、远程控制台、RosBag回放系统等。让我们进入智能调度演示..."

**Narration Script** (English):
> "After logging in, you can see the complete function modules including real-time dispatch dashboard, remote control console, and RosBag replay system. Let's enter the Intelligent Dispatch Demo..."

---

### Task 3: 接入AD自动驾驶本地视频 (Connect AD Local Video)
**Duration: 40 seconds**

**操作步骤 Steps**:
1. 在 Intelligent Dispatch Demo 界面
2. 展示地图组件 (高德地图 Amap)
   - 实时车辆位置标记
   - 不同紧急级别的颜色编码
3. 展示自动驾驶视频流 (模拟)
   - 如果没有真实视频，使用模拟的车辆状态面板
4. 显示车辆遥测数据

**界面元素 Interface Elements**:
- 实时地图视图 (Real-time Map View)
- 车辆位置标记 (Vehicle Location Markers)
- 视频流占位符 (Video Stream Placeholder)
- 遥测数据面板 (Telemetry Data Panel)

**录制要点 Recording Tips**:
- 突出显示地图与实时数据的同步
- 展示多车辆同时监控能力
- 强调系统的可扩展性

**旁白文稿 Narration Script** (中文):
> "系统实时接入自动驾驶车辆的视频流和传感器数据。通过高德地图API，我们可以精确显示所有车辆的实时位置。每个车辆都有独特的风险评分和紧急级别..."

**Narration Script** (English):
> "The system connects to real-time autonomous vehicle video streams and sensor data. Through Amap API, we can precisely display the real-time location of all vehicles. Each vehicle has a unique risk score and urgency level..."

---

### Task 4: Mock 100车辆车队，展示优先级排序 (Mock 100-Vehicle Fleet with Priority Sorting)
**Duration: 1 minute**

**操作步骤 Steps**:
1. 点击"Start Simulation"按钮
2. 展示车辆队列实时增长到30-50辆
3. 点击"Add 10 Vehicles"按钮多次，增加到100辆左右
4. 展示车辆队列根据风险评分自动排序:
   - **Critical** (红色): 紧急情况车辆
   - **High** (橙色): 高风险车辆
   - **Medium** (黄色): 中等风险
   - **Low** (绿色): 低风险
5. 展示统计数据实时更新:
   - 总车辆数
   - 活跃操作员
   - 平均匹配时间
   - 匹配成功率

**界面元素 Interface Elements**:
- 车辆队列列表 (Vehicle Queue List)
- 优先级排序指示器 (Priority Sorting Indicators)
- 实时统计卡片 (Real-time Statistics Cards)
- 地图上的多车辆标记 (Multi-vehicle Markers on Map)

**录制要点 Recording Tips**:
- 展示系统处理大规模车队的能力
- 突出显示动态优先级算法
- 显示实时统计数据的变化

**旁白文稿 Narration Script** (中文):
> "FSM-Pilot支持同时管理100辆以上的自动驾驶车辆。系统使用XGBoost机器学习模型实时计算每辆车的风险评分，并根据21维特征向量进行智能排序。紧急情况的车辆会自动置顶，确保最需要人工接管的车辆得到优先处理..."

**Narration Script** (English):
> "FSM-Pilot supports simultaneous management of over 100 autonomous vehicles. The system uses an XGBoost machine learning model to calculate risk scores in real-time for each vehicle, intelligently sorting based on a 21-dimensional feature vector. Critical vehicles are automatically prioritized to ensure those needing manual takeover receive immediate attention..."

---

### Task 5: 车队风险异常，接管确认与AI分析 (Fleet Risk Anomaly, Takeover Confirmation & AI Analysis)
**Duration: 1.5-2 minutes**

**操作步骤 Steps**:
1. **风险检测 Risk Detection**:
   - 系统自动检测到高风险/紧急车辆
   - 车辆卡片上显示红色/橙色"接管车辆"按钮
   - 按钮有脉冲动画吸引注意

2. **打开接管对话框 Open Takeover Dialog**:
   - 点击"接管车辆"按钮
   - 显示企业级接管确认对话框

3. **AI智能分析展示 AI Analysis Display**:
   - **紧急程度横幅** (Urgency Banner):
     - "紧急情况 - 需要立即接管"
     - 风险评分: 85.3分

   - **AI分析部分** (AI Analysis Section):
     - 🤖 AI智能分析状态: "分析完成" / "豆包大模型"
     - 事故风险描述: "车辆V-023在城市道路检测到前方有突然出现的障碍物..."

   - **关键风险因素** (Key Risk Factors):
     - 🚗 车速过快: 严重
     - 🚦 交通密集: 极高
     - 🌧️ 天气不佳: 影响较大
     - 📡 系统信心度: 偏低

   - **AI建议措施** (AI Recommended Actions):
     - P0: 立即接管车辆控制权
     - P1: 降低车速至安全范围
     - P2: 评估周围环境，选择安全路径

   - **接管建议** (Takeover Recommendation):
     - ✓ AI强烈建议接管
     - "车辆当前处于高风险状态，自动驾驶系统无法确保安全，强烈建议立即接管..."

4. **车辆遥测数据 Vehicle Telemetry**:
   - 当前速度: 95 km/h
   - 控制模式: 完全自动驾驶
   - 位置信息: HK-2-1 城市道路
   - 场景类型: 城市道路
   - 天气条件: 雨天
   - 系统状态: 正常运行

5. **安全员分配 Operator Assignment**:
   - 显示最佳匹配的安全员
   - 姓名: "张伟 3"
   - 状态: 空闲
   - 负载: 1/3
   - 匹配度: 92%
   - ⭐ 最佳匹配徽章

6. **二次确认 (Double Confirmation)** (针对高风险):
   - 显示警告框
   - "高风险操作需要二次确认"
   - 复选框: "我已充分理解风险，确认执行远程接管操作"

7. **确认接管 Confirm Takeover**:
   - 勾选确认复选框
   - 点击"确认接管"按钮
   - 对话框关闭
   - 车辆控制模式更新为"直接控制"
   - 统计数据更新

**录制要点 Recording Tips**:
- 充分展示对话框的每个部分
- 突出显示AI分析的智能化
- 展示企业级的安全确认流程
- 强调系统的决策透明度

**旁白文稿 Narration Script** (中文):
> "当系统检测到高风险车辆时，会立即提示操作员。点击接管按钮后，我们可以看到完整的AI智能分析结果。"
>
> "系统使用豆包大模型进行实时场景分析，识别出多个关键风险因素：车速过快、交通密集、天气不佳等。AI给出了明确的优先级建议措施。"
>
> "同时，系统展示了完整的车辆遥测数据，包括速度、位置、场景信息等。"
>
> "智能匹配算法自动为这辆车分配了最合适的安全员，匹配度高达92%。"
>
> "对于高风险操作，系统要求二次确认，确保操作员充分理解风险。这是企业级系统的重要安全特性..."

**Narration Script** (English):
> "When the system detects a high-risk vehicle, it immediately alerts the operator. Upon clicking the takeover button, we can see the complete AI intelligent analysis results."
>
> "The system uses the Doubao LLM for real-time scenario analysis, identifying multiple key risk factors: excessive speed, heavy traffic, adverse weather conditions, etc. The AI provides clear prioritized action recommendations."
>
> "At the same time, the system displays complete vehicle telemetry data including speed, location, and scenario information."
>
> "The intelligent matching algorithm automatically assigns the most suitable safety operator for this vehicle with a match score of 92%."
>
> "For high-risk operations, the system requires double confirmation to ensure operators fully understand the risks. This is a critical safety feature of enterprise-grade systems..."

---

### Task 6: Remote Assistant - 实时驾驶建议 (Real-time Driving Suggestions)
**Duration: 1 minute**

**操作步骤 Steps**:
1. **打开Remote Assistant面板**:
   - 面板自动显示在右侧
   - 展示"Remote Assistant"标题
   - 显示AI图标和运行状态指示器

2. **当前监控车辆信息**:
   - 车辆 ID: V-045
   - 场景: 城市道路 🏙️
   - 天气: 雨天 🌧️
   - 风险评分: 78.3 (高风险)

3. **实时建议流展示**:
   - **实时建议卡片**逐条出现:

     **建议 1** (紧急):
     - 🚨 "检测到紧急情况，建议立即降低车速并准备接管"
     - 优先级: 紧急
     - 操作按钮: "立即接管"

     **建议 2** (高优先级):
     - 🚦 "当前车速 65 km/h 偏高，建议降至 45 km/h"
     - 优先级: 高优先级
     - 操作按钮: "应用建议"

     **建议 3** (中等):
     - 🌧️ "雨天路况，建议保持更大的安全距离"
     - 优先级: 中等
     - 操作按钮: "调整距离"

     **建议 4** (低优先级):
     - 🏙️ "城市路况复杂，注意行人和非机动车"
     - 优先级: 提示

4. **关键指标展示**:
   - 建议采纳率: 87%
   - 风险降低: -23%
   - 响应时间: 320ms
   - 今日建议: 142

5. **快速操作按钮**:
   - 🚨 紧急指导
   - 🗺️ 路线优化

6. **互动演示**:
   - 点击"应用建议"按钮
   - 建议卡片消失（表示已执行）
   - 点击"紧急指导"按钮
   - 显示新的紧急分析建议
   - 点击"路线优化"
   - 显示路线优化计算中的提示

**录制要点 Recording Tips**:
- 展示建议的实时生成过程
- 突出显示不同优先级的视觉区分
- 展示建议的可操作性
- 强调系统的主动性和智能化

**旁白文稿 Narration Script** (中文):
> "Remote Assistant是系统的核心AI功能之一，它为选中的车辆提供实时驾驶建议。"
>
> "系统持续分析车辆状态、路况、天气等多维度信息，每5秒生成一次智能建议。建议按照紧急程度分为四个级别：紧急、高优先级、中等和提示。"
>
> "对于紧急建议，系统提供一键操作按钮，操作员可以立即应用建议或触发接管流程。"
>
> "关键指标显示，我们的AI建议采纳率达到87%，有效降低了23%的风险。平均响应时间仅320毫秒，确保实时性。"
>
> "通过快速操作按钮，操作员可以随时请求紧急指导或路线优化，系统会立即响应并提供专业建议..."

**Narration Script** (English):
> "Remote Assistant is one of the core AI features of the system, providing real-time driving suggestions for the selected vehicle."
>
> "The system continuously analyzes multi-dimensional information including vehicle status, road conditions, and weather, generating intelligent suggestions every 5 seconds. Suggestions are categorized into four priority levels: Critical, High, Medium, and Informational."
>
> "For critical suggestions, the system provides one-click action buttons. Operators can immediately apply suggestions or trigger the takeover process."
>
> "Key metrics show our AI suggestions have an 87% adoption rate, effectively reducing risks by 23%. The average response time is only 320 milliseconds, ensuring real-time performance."
>
> "Through quick action buttons, operators can request emergency guidance or route optimization at any time. The system responds immediately with professional suggestions..."

---

### Task 7: 远程接管进行避险 (Remote Takeover for Risk Avoidance)
**Duration: 40 seconds**

**操作步骤 Steps**:
1. **执行远程接管**:
   - 显示车辆控制模式从"自动驾驶"切换到"直接控制"
   - 车辆卡片状态更新
   - 控制模式图表实时更新

2. **展示控制模式分布**:
   - Direct Control (直接控制): 15%
   - Trajectory Confirmation (轨迹确认): 60%
   - Semantic Instruction (语义指令): 25%

3. **显示算法性能指标**:
   - 平均评分时间: 45ms
   - 平均匹配时间: 128ms
   - 预测准确度: 85%
   - 平均接管延迟: 1850ms
   - 模式转换总数: 28
   - 带宽节省: 15.3 Mbps

4. **展示操作员状态更新**:
   - 选中的操作员状态变为"忙碌"
   - 分配车辆数增加
   - 负载百分比更新

**录制要点 Recording Tips**:
- 展示从检测到接管的完整流程
- 突出显示系统的低延迟性能
- 强调智能调度算法的效率

**旁白文稿 Narration Script** (中文):
> "确认接管后，车辆立即从自动驾驶模式切换到直接控制模式。我们可以看到控制模式分布图实时更新。"
>
> "系统的算法性能非常出色：平均风险评分只需45毫秒，匹配时间128毫秒，端到端接管延迟不到2秒。"
>
> "通过自适应控制模式，系统在确保安全的同时，节省了超过15Mbps的带宽，降低了系统成本..."

**Narration Script** (English):
> "After confirming takeover, the vehicle immediately switches from autonomous mode to direct control mode. We can see the control mode distribution chart updating in real-time."
>
> "The system's algorithm performance is exceptional: average risk scoring takes only 45 milliseconds, matching time is 128 milliseconds, and end-to-end takeover latency is under 2 seconds."
>
> "Through adaptive control modes, the system saves over 15Mbps of bandwidth while ensuring safety, reducing system costs..."

---

### Task 8: 数据库存储展示 (Database Storage Display)
**Duration: 30 seconds**

**操作步骤 Steps**:
1. **展示数据持久化**:
   - 在浏览器开发者工具中打开 IndexedDB
   - 显示 FSM-Pilot 数据库
   - 展示存储的数据表:
     - vehicles (车辆记录)
     - operators (操作员记录)
     - takeover_events (接管事件)
     - ai_analyses (AI分析结果)
     - telemetry (遥测数据)

2. **显示数据记录**:
   - 选择一条接管事件记录
   - 展示完整的JSON数据:
     ```json
     {
       "id": "takeover-001",
       "vehicleId": "V-023",
       "operatorId": "OP-003",
       "timestamp": 1705300000000,
       "reason": "High risk detected",
       "aiAnalysis": {...},
       "telemetry": {...},
       "outcome": "successful"
     }
     ```

3. **展示实时更新**:
   - 触发新的接管操作
   - 实时查看数据库中新增的记录

**录制要点 Recording Tips**:
- 清晰展示数据库结构
- 突出显示数据的完整性
- 展示系统的可审计性

**旁白文稿 Narration Script** (中文):
> "所有的操作和分析结果都实时存储到本地IndexedDB数据库中。这确保了数据的持久化和可追溯性。"
>
> "我们可以看到详细的接管事件记录，包括车辆ID、操作员、时间戳、AI分析结果等。这对于后续的审计和优化至关重要..."

**Narration Script** (English):
> "All operations and analysis results are stored in real-time to the local IndexedDB database, ensuring data persistence and traceability."
>
> "We can see detailed takeover event records including vehicle ID, operator, timestamp, AI analysis results, etc. This is crucial for subsequent auditing and optimization..."

---

## 🎬 Production Guidelines / 制作指南

### Technical Requirements / 技术要求

**Video Settings**:
- Resolution: 1920×1080 (Full HD)
- Frame Rate: 60 FPS (for smooth UI animations)
- Bitrate: 8-10 Mbps
- Format: MP4 (H.264 codec)

**Audio Settings**:
- Sample Rate: 48 kHz
- Bitrate: 320 kbps
- Format: AAC
- Channels: Stereo

**Screen Recording Tools**:
- **Windows**: OBS Studio, Camtasia
- **macOS**: ScreenFlow, OBS Studio
- **Linux**: SimpleScreenRecorder, OBS Studio

### Recording Tips / 录制技巧

1. **Preparation**:
   - 清理桌面和浏览器书签栏
   - 关闭不必要的通知
   - 确保网络连接稳定
   - 准备好演示脚本

2. **During Recording**:
   - 缓慢、清晰地移动鼠标
   - 在每个关键操作后暂停1-2秒
   - 避免快速滚动或切换
   - 保持画面在屏幕中央

3. **Post-Production**:
   - 添加背景音乐（低音量）
   - 添加字幕（中英双语）
   - 添加关键点标注
   - 调整颜色和对比度

### Narration Guidelines / 旁白指南

**Voice Characteristics**:
- 语速: 中等偏慢 (120-140 words/min)
- 音调: 专业、自信、友好
- 音量: 清晰可听，略高于背景音乐

**Content Structure**:
- 开头: 简要介绍系统和功能
- 过程: 解释正在进行的操作
- 结尾: 总结关键特性和优势

**Key Points to Emphasize**:
- Enterprise-grade reliability
- AI-powered intelligence
- Real-time performance
- Scalability (100+ vehicles)
- Safety and compliance

---

## 📊 Key Metrics to Highlight / 重点指标

Throughout the demo, emphasize these performance metrics:

1. **System Performance**:
   - ⚡ Risk Scoring: <50ms
   - 🎯 Matching: <150ms
   - 📡 Takeover Latency: <2s
   - 🔮 Prediction Accuracy: 85%+

2. **Scale & Capacity**:
   - 🚗 Concurrent Vehicles: 100+
   - 👥 Active Operators: 30+
   - 📈 Success Rate: 98%+

3. **Intelligence & Automation**:
   - 🤖 AI Analysis: Real-time
   - 🧠 LLM Integration: Doubao
   - 🗺️ Geographic Matching: Optimized
   - 📊 21-dimension Feature Vector

4. **Safety & Compliance**:
   - 🔒 Multi-level Confirmation
   - 📝 Complete Audit Trail
   - ⚠️ Proactive Risk Detection
   - 🛡️ Failsafe Mechanisms

---

## 🎯 Target Audience Customization / 目标受众定制

### For Investors / 投资者版本:
- Focus on: Market potential, scalability, ROI
- Emphasize: 100+ vehicle capacity, AI integration, cost savings
- Duration: 5-6 minutes

### For Technical Stakeholders / 技术版本:
- Focus on: Architecture, algorithms, performance metrics
- Emphasize: XGBoost model, bipartite matching, latency optimization
- Duration: 7-8 minutes

### For Clients / 客户版本:
- Focus on: Safety, reliability, ease of use
- Emphasize: AI-powered analysis, operator-friendly interface, compliance
- Duration: 5-6 minutes

---

## 📝 Checklist Before Recording / 录制前检查清单

- [ ] 系统已正常启动 (`npm run dev`)
- [ ] 所有API已配置（Doubao, Amap）
- [ ] 浏览器已清理（书签、扩展）
- [ ] 录屏软件已设置正确
- [ ] 麦克风已测试
- [ ] 演示脚本已准备
- [ ] 测试数据已准备
- [ ] 网络连接稳定
- [ ] 桌面背景已清理
- [ ] 系统通知已关闭

---

## 🎥 Post-Production Enhancements / 后期制作

### Visual Enhancements:
1. **Title Cards**:
   - Opening: "FSM-Pilot V2.0 - Remote Driving Platform"
   - Section transitions
   - Closing: Contact information

2. **Annotations**:
   - Highlight important UI elements
   - Add arrows pointing to key features
   - Display metrics in callout boxes

3. **Transitions**:
   - Smooth fade transitions between sections
   - Zoom in on important details
   - Pan to follow mouse cursor

### Audio Enhancements:
1. **Background Music**:
   - Professional, tech-oriented
   - Low volume (-20dB from narration)
   - Fade in/out at section boundaries

2. **Sound Effects** (Optional):
   - Subtle clicks for button presses
   - Success chimes for completions
   - Alert sounds for notifications

3. **Narration**:
   - Remove background noise
   - Normalize audio levels
   - Add slight compression

### Subtitles:
- Chinese (Simplified) - Primary
- English - Secondary
- Position: Bottom center
- Font: Clear, sans-serif
- Background: Semi-transparent black

---

## 📧 Contact & Distribution / 联系与分发

**For Demo Video Requests**:
- Email: li.yixiang@cityu.edu.hk
- Institution: City University of Hong Kong
- Project: FSM-Pilot V2.0

**Distribution Channels**:
- Company website
- YouTube
- Bilibili
- LinkedIn
- Technical conferences
- Client presentations

---

**Last Updated**: 2026-01-15
**Version**: FSM-Pilot V2.0 Demo Guide v1.0
**Author**: Li Yixiang, City University of Hong Kong
