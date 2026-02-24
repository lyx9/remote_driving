# FSM-Pilot V2.0 - Demo Script Usage Guide

## Overview

The `start_demo.sh` script provides a complete one-click enterprise demonstration of the FSM-Pilot V2.0 platform. It automatically:

- ✅ Checks system environment (Node.js, npm, browser)
- ✅ Generates mock collision scenarios
- ✅ Creates 100-vehicle fleet data
- ✅ Starts the development server
- ✅ Opens browser automatically
- ✅ Executes automated 3-minute demo flow
- ✅ Displays performance metrics
- ✅ Handles cleanup on exit

---

## Quick Start

### 1. Make Script Executable

```bash
chmod +x start_demo.sh
```

### 2. Run the Demo

```bash
./start_demo.sh
```

### 3. Select Demo Mode

You'll see an interactive menu:

```
╔════════════════════════════════════════════════════════════════╗
║          FSM-Pilot V2.0 - Enterprise Demo System              ║
║     Autonomous Driving Remote Takeover Platform               ║
╚════════════════════════════════════════════════════════════════╝

Demo Options:
1) Full Automated Demo (3 minutes)
2) Quick Demo (1 minute)
3) Custom Scenario
4) Performance Metrics Only
5) Exit

Please select an option [1-5]:
```

---

## Demo Modes

### Mode 1: Full Automated Demo (Recommended)

**Duration**: 3 minutes

**What it does**:
1. **Platform Login (0-30s)**
   - Automatic login with credentials: `cityu / 2026`
   - Displays main dashboard

2. **Work Interface Display (30-60s)**
   - Shows navigation menu
   - Displays Intelligent Dispatch Demo interface

3. **AD Video Integration (60-90s)**
   - Connects to mock autonomous driving video stream
   - Shows vehicle telemetry data on Amap

4. **100-Vehicle Fleet Simulation (90-120s)**
   - Generates 100 mock vehicles
   - Displays priority-based queue sorting
   - Shows real-time statistics

5. **Collision Risk Alert & Takeover (120-150s)**
   - Triggers high-risk vehicle (V-023)
   - Opens takeover confirmation dialog
   - Displays AI analysis from Doubao LLM:
     - Incident description
     - Risk factors (speed, traffic, weather)
     - Recommended actions (P0, P1, P2)
   - Shows operator assignment with 92% match score
   - Executes double confirmation for high-risk operation

6. **Remote Assistant - Real-time Suggestions (150-165s)**
   - Displays Remote Assistant panel
   - Shows 4-level priority suggestions:
     - 🚨 Critical: "检测到紧急情况，建议立即降低车速并准备接管"
     - ⚠️ High: "当前车速 65 km/h 偏高，建议降至 45 km/h"
     - ⚡ Medium: "雨天路况，建议保持更大的安全距离"
     - ℹ️ Low: "城市路况复杂，注意行人和非机动车"
   - Updates metrics: adoption rate, risk reduction, response time

7. **Remote Takeover Execution (165-175s)**
   - Confirms takeover operation
   - Updates vehicle control mode to "Direct Control"
   - Updates operator status to "Busy"
   - Shows control mode distribution chart

8. **Database Storage Display (175-180s)**
   - Opens browser DevTools (F12)
   - Shows IndexedDB storage
   - Displays takeover event records

**Usage**:
```bash
./start_demo.sh
# Select option 1
```

---

### Mode 2: Quick Demo

**Duration**: 1 minute

**What it does**:
- Compressed version of full demo
- Key highlights only:
  - Login → Interface → Fleet → Takeover → Complete

**Best for**:
- Quick system verification
- Fast demonstrations
- Testing environment setup

**Usage**:
```bash
./start_demo.sh
# Select option 2
```

---

### Mode 3: Custom Scenario

**What it does**:
- Interactive scenario builder
- Allows custom configuration:
  - Number of vehicles (10-200)
  - Risk level distribution
  - Collision scenario details
  - Weather conditions
  - Traffic density

**Usage**:
```bash
./start_demo.sh
# Select option 3
# Follow interactive prompts
```

**Example Configuration**:
```
Enter number of vehicles [10-200]: 150
Enter critical risk percentage [0-100]: 20
Enter collision distance (meters) [10-100]: 35
Select weather: 1) Clear 2) Rainy 3) Foggy 4) Snowy
Traffic density [low/medium/high]: high
```

---

### Mode 4: Performance Metrics Only

**What it does**:
- Runs system checks
- Displays current performance metrics:
  - ⚡ Risk Scoring Time: <50ms
  - 🎯 Matching Time: <150ms
  - 📡 Takeover Latency: <2s
  - 🔮 Prediction Accuracy: 85%+
  - 🚗 Concurrent Vehicles: 100+
  - 👥 Active Operators: 30+
  - 📈 Success Rate: 98%+

**Best for**:
- System health check
- Performance monitoring
- Pre-demo verification

**Usage**:
```bash
./start_demo.sh
# Select option 4
```

---

## Mock Data Generated

### 1. Collision Scenario Data

**File**: `/tmp/fsm_collision_scenario.json`

**Content**:
```json
{
  "vehicle": {
    "id": "V-023",
    "location": {
      "longitude": 114.1693,
      "latitude": 22.3193,
      "street": "Nathan Road",
      "district": "Yau Tsim Mong",
      "region": "Hong Kong"
    },
    "speed": 85.5,
    "risk_score": 92.3,
    "urgency": "critical",
    "scenario": "urban",
    "weather": "rainy"
  },
  "collision_risk": {
    "type": "frontal_obstacle",
    "distance_meters": 45,
    "time_to_collision_seconds": 1.8,
    "collision_probability": 0.89,
    "obstacle_type": "pedestrian_crossing",
    "avoidance_difficulty": "high"
  },
  "ai_analysis": {
    "incident_description": "车辆V-023在城市道路检测到前方有突然出现的障碍物，距离仅45米，以当前速度85.5km/h行驶，预计1.8秒后将发生碰撞。雨天路况进一步增加了制动距离和风险。",
    "risk_factors": [
      {
        "factor": "vehicle_speed",
        "severity": "critical",
        "value": 85.5,
        "description": "车速过快 (85.5 km/h)"
      },
      {
        "factor": "traffic_density",
        "severity": "high",
        "value": 0.89,
        "description": "交通密集度极高"
      },
      {
        "factor": "weather_condition",
        "severity": "high",
        "value": "rainy",
        "description": "雨天路况影响制动"
      },
      {
        "factor": "system_confidence",
        "severity": "medium",
        "value": 0.23,
        "description": "系统信心度偏低"
      }
    ],
    "recommended_actions": [
      {
        "priority": "P0",
        "action": "immediate_takeover",
        "description": "立即接管车辆控制权"
      },
      {
        "priority": "P1",
        "action": "reduce_speed",
        "target_speed": 30,
        "description": "紧急降低车速至30 km/h以下"
      },
      {
        "priority": "P2",
        "action": "evaluate_environment",
        "description": "评估周围环境，选择最安全的避让路径"
      }
    ],
    "takeover_recommendation": {
      "should_takeover": true,
      "confidence": 0.96,
      "reasoning": "车辆当前处于高风险状态，自动驾驶系统无法确保安全通过，强烈建议立即进行人工接管。"
    }
  },
  "operator_assignment": {
    "operator_id": "OP-003",
    "operator_name": "张伟 3",
    "match_score": 0.92,
    "status": "idle",
    "current_load": 1,
    "max_capacity": 3,
    "distance_km": 2.3,
    "estimated_response_time_seconds": 15
  },
  "timestamp": 1705300000000,
  "simulation_metadata": {
    "generated_by": "FSM-Pilot Demo Script",
    "scenario_type": "collision_avoidance",
    "demo_purpose": "enterprise_presentation"
  }
}
```

### 2. Fleet Data

**File**: `/tmp/fsm_fleet_data.json`

**Content**:
```json
{
  "fleet_size": 100,
  "vehicles": [
    {
      "id": "V-001",
      "location": { "longitude": 114.15, "latitude": 22.28 },
      "riskScore": {
        "overallScore": 45.2,
        "urgencyLevel": "medium"
      },
      "scenario": "highway",
      "weather": "clear",
      "controlMode": "full-autonomous",
      "telemetry": {
        "speed": 75.3,
        "acceleration": 0.5,
        "heading": 145
      }
    }
    // ... 99 more vehicles
  ],
  "statistics": {
    "critical": 8,
    "high": 22,
    "medium": 45,
    "low": 25
  },
  "operators": [
    {
      "id": "OP-001",
      "name": "李明 1",
      "status": "idle",
      "location": { "longitude": 114.17, "latitude": 22.32 },
      "skills": {
        "scenarioExpertise": {
          "highway": 0.95,
          "urban": 0.88,
          "residential": 0.76
        },
        "successRate": 0.97,
        "averageResponseTime": 3.2,
        "experience": 5.5
      },
      "maxConcurrentVehicles": 3,
      "currentLoad": 0,
      "assignedVehicles": []
    }
    // ... 29 more operators
  ]
}
```

---

## System Requirements

### Minimum Requirements

- **Node.js**: v18.0.0 or higher
- **npm**: v9.0.0 or higher
- **Browser**: Chrome, Firefox, or Safari
- **OS**: Linux, macOS, or Windows (with WSL)
- **RAM**: 4GB minimum
- **Disk Space**: 2GB free space

### Recommended Requirements

- **Node.js**: v20.0.0 or higher
- **RAM**: 8GB or more
- **CPU**: 4 cores or more
- **Network**: Stable internet connection for API calls

---

## Environment Setup

### 1. Install Dependencies

```bash
cd /home/lyx/fsm
npm install
```

### 2. Configure APIs (Optional)

Create `.env.local` file:

```bash
cp .env.example .env.local
```

Edit `.env.local`:

```bash
# Doubao LLM API (Optional)
VITE_DOUBAO_API_KEY=your_doubao_api_key_here

# Amap API (Optional)
VITE_AMAP_API_KEY=your_amap_key_here
VITE_AMAP_JS_CODE=your_amap_js_code_here
```

**Note**: APIs are optional. The system will use local fallback functionality if not configured.

### 3. Verify Setup

```bash
./start_demo.sh
# Select option 4 (Performance Metrics)
```

---

## Script Features

### Automatic System Checks

The script automatically verifies:

✅ Node.js installation and version
✅ npm availability
✅ Browser detection (Chrome > Firefox > Safari)
✅ Port 5173 availability
✅ Project dependencies
✅ Build process health

### Mock Data Generation

Generates realistic data:

✅ Collision scenarios with AI analysis
✅ 100-vehicle fleet with telemetry
✅ 30 operators with skills
✅ Risk score distribution
✅ Geographic locations (Hong Kong area)

### Automated Browser Control

✅ Cross-platform browser launching
✅ Automatic login execution
✅ Navigation to demo interface
✅ Scenario triggering
✅ Dialog interactions

### Performance Monitoring

Tracks key metrics:

✅ Risk scoring time (<50ms)
✅ Operator matching time (<150ms)
✅ Takeover latency (<2s)
✅ Prediction accuracy (85%+)
✅ System success rate (98%+)

### Cleanup & Error Handling

✅ Graceful shutdown on Ctrl+C
✅ Temporary file cleanup
✅ Server process termination
✅ Error logging
✅ Recovery suggestions

---

## Troubleshooting

### Problem 1: Port Already in Use

**Error**:
```
Port 5173 is already in use
```

**Solution**:
```bash
# Find and kill process using port 5173
lsof -ti:5173 | xargs kill -9

# Or use a different port
export DEMO_PORT=5174
./start_demo.sh
```

### Problem 2: Browser Not Opening

**Error**:
```
Could not detect browser
```

**Solution**:
```bash
# Manually specify browser
export BROWSER_CMD="google-chrome"  # or "firefox", "safari"
./start_demo.sh

# Or open manually after script starts
# Navigate to: http://localhost:5173
```

### Problem 3: Dependencies Not Installed

**Error**:
```
node_modules not found
```

**Solution**:
```bash
cd /home/lyx/fsm
npm install
./start_demo.sh
```

### Problem 4: Build Fails

**Error**:
```
Build failed with errors
```

**Solution**:
```bash
# Clear cache and rebuild
rm -rf node_modules/.vite
npm run build
./start_demo.sh
```

### Problem 5: API Not Configured

**Warning**:
```
Doubao LLM API not configured - using fallback
Amap API not configured - map features disabled
```

**Solution**:
This is normal! The system works with local fallbacks.

To enable full features:
1. Follow [API_CONFIGURATION.md](./API_CONFIGURATION.md)
2. Create `.env.local` with API keys
3. Restart the demo script

---

## Demo Flow Timeline

### Full Demo (3 minutes)

| Time | Task | Action |
|------|------|--------|
| **0:00-0:30** | Platform Login | Automatic login with cityu/2026 |
| **0:30-1:00** | Work Interface | Display navigation and modules |
| **1:00-1:30** | AD Video Integration | Show vehicle telemetry on map |
| **1:30-2:00** | 100-Vehicle Fleet | Generate and display sorted queue |
| **2:00-2:30** | Risk Alert & AI | Show collision scenario + AI analysis |
| **2:30-2:45** | Remote Assistant | Display real-time suggestions |
| **2:45-2:55** | Takeover Execution | Confirm and execute remote control |
| **2:55-3:00** | Database Logging | Show IndexedDB records |

### Quick Demo (1 minute)

| Time | Task | Action |
|------|------|--------|
| **0:00-0:15** | Login + Interface | Fast navigation |
| **0:15-0:30** | Fleet Display | Show 50 vehicles |
| **0:30-0:45** | Takeover | Single high-risk vehicle |
| **0:45-1:00** | Complete | Show results |

---

## Advanced Usage

### Custom Script Variables

Edit the script header to customize:

```bash
# Demo configuration
DEMO_PORT=5173              # Server port
DEMO_URL="http://localhost:${DEMO_PORT}"
SCENARIO_DURATION=180       # Total demo time (seconds)
AUTO_LOGIN=true             # Automatic login
AUTO_NAVIGATE=true          # Automatic navigation
MOCK_COLLISION=true         # Include collision scenario
```

### Debug Mode

Enable verbose output:

```bash
export DEBUG=1
./start_demo.sh
```

### Headless Mode

Run without browser (for CI/CD):

```bash
export HEADLESS=1
./start_demo.sh
```

### Custom Scenario File

Use your own scenario data:

```bash
export SCENARIO_FILE="/path/to/custom_scenario.json"
./start_demo.sh
```

---

## Integration with Other Tools

### CI/CD Integration

#### GitHub Actions

```yaml
name: Demo Test
on: [push]
jobs:
  demo:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 20
      - run: npm install
      - run: chmod +x start_demo.sh
      - run: export HEADLESS=1 && ./start_demo.sh
```

#### GitLab CI

```yaml
demo_test:
  script:
    - npm install
    - chmod +x start_demo.sh
    - export HEADLESS=1 && ./start_demo.sh
```

### Docker Integration

```dockerfile
FROM node:20-alpine
WORKDIR /app
COPY . .
RUN npm install
RUN chmod +x start_demo.sh
CMD ["./start_demo.sh"]
```

### Video Recording

Record demo with `ffmpeg`:

```bash
# Start recording
ffmpeg -f x11grab -s 1920x1080 -i :0.0 -r 30 demo.mp4 &
FFMPEG_PID=$!

# Run demo
./start_demo.sh

# Stop recording
kill $FFMPEG_PID
```

---

## Performance Benchmarks

### Expected Metrics (Full Demo)

| Metric | Target | Typical |
|--------|--------|---------|
| Risk Scoring | <50ms | 35-45ms |
| Operator Matching | <150ms | 110-140ms |
| Takeover Latency | <2000ms | 1700-1900ms |
| Prediction Accuracy | >85% | 87-92% |
| Success Rate | >98% | 98.5-99.2% |
| Bandwidth Savings | >15 Mbps | 15-20 Mbps |

### System Load

| Resource | Usage (Idle) | Usage (Demo) |
|----------|--------------|--------------|
| CPU | 5-10% | 30-50% |
| RAM | 200-300 MB | 600-800 MB |
| Network | Minimal | 2-5 MB/s |
| Disk I/O | <1 MB/s | 5-10 MB/s |

---

## Best Practices

### For Presentations

1. **Pre-Demo Setup** (30 minutes before):
   - Run `./start_demo.sh` option 4 (metrics check)
   - Verify API configurations
   - Test browser launch
   - Check network connectivity

2. **During Demo**:
   - Use Full Automated Demo (option 1)
   - Avoid manual interruptions
   - Let the script handle timing
   - Be ready to explain each step

3. **Post-Demo**:
   - Show database records (F12 → IndexedDB)
   - Display performance metrics
   - Answer questions with data

### For Development

1. **Quick Testing**:
   - Use Quick Demo (option 2)
   - Iterate rapidly
   - Test specific features

2. **Custom Testing**:
   - Use Custom Scenario (option 3)
   - Test edge cases
   - Validate specific conditions

3. **Integration Testing**:
   - Use Metrics Only (option 4)
   - Monitor performance
   - Track regressions

---

## Support & Resources

### Documentation

- [API Configuration Guide](./API_CONFIGURATION.md)
- [Demo Video Guide](./DEMO_VIDEO_GUIDE.md)
- [System Architecture](./ARCHITECTURE.md)

### Contact

- **Email**: li.yixiang@cityu.edu.hk
- **Institution**: City University of Hong Kong
- **Project**: FSM-Pilot V2.0

### Links

- **GitHub**: [Repository URL]
- **Documentation**: [Docs URL]
- **Demo Video**: [Video URL]

---

## Changelog

### Version 2.0 (2026-01-15)

- ✨ Initial enterprise demo script
- ✨ Automated 3-minute demo flow
- ✨ Mock collision scenario generation
- ✨ 100-vehicle fleet simulation
- ✨ AI analysis integration
- ✨ Remote Assistant feature
- ✨ Database logging display
- ✨ Interactive menu system
- ✨ Performance metrics tracking
- ✨ Cross-platform support

---

**Last Updated**: 2026-01-15
**Version**: 2.0
**Author**: Li Yixiang, City University of Hong Kong
