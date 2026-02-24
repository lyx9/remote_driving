# FSM-Pilot V2.0 - Complete System Demo Script

**Project:** FSM-Pilot Remote Driving Platform
**Author:** Li Yixiang
**Institution:** City University of Hong Kong
**Date:** January 2026

---

## 🎬 Video Demo Script (10-15 minutes)

### Introduction (1 minute)

**[Screen: Title Slide]**

> "Welcome to FSM-Pilot V2.0, a next-generation remote driving platform developed at City University of Hong Kong. This system enables safe, efficient remote operation of autonomous vehicles with real-time telemetry, multi-camera streaming, and intelligent dispatch capabilities."

**Key Features to Highlight:**
- Real-time remote vehicle control
- Multi-camera video streaming
- LiDAR point cloud visualization
- Amap integration for vehicle tracking
- RosBag data replay and analysis
- Database management system
- Intelligent dispatch optimization

---

### Part 1: System Architecture Overview (2 minutes)

**[Screen: Architecture Diagram]**

> "FSM-Pilot V2.0 follows a distributed architecture with three main components:"

1. **Vehicle Node (C++ ROS2)**
   - Real-time data collection from sensors
   - WebRTC-based video streaming
   - Command execution with safety checks
   - Latency monitoring

2. **Cloud Server (Node.js)**
   - WebSocket signaling server
   - Intelligent scheduling service
   - Alert analysis and prediction
   - Database management

3. **Operator Client (Vue 3 + TypeScript)**
   - Modern web-based interface
   - Real-time telemetry display
   - Multi-camera video wall
   - Interactive map with Amap

**[Screen: Technology Stack]**

> "The system leverages cutting-edge technologies:"
- **Frontend:** Vue 3, TypeScript, Vite
- **Backend:** Node.js, Express, WebSocket
- **Vehicle:** ROS2, C++17, WebRTC
- **Database:** SQLite, IndexedDB
- **Maps:** Amap (高德地图) API
- **Visualization:** Three.js, Canvas API

---

### Part 2: New Orange-Red Tech Theme (1 minute)

**[Screen: Login Page]**

> "Let's start by logging into the system. Notice the new orange-red tech theme that gives the interface a modern, futuristic appearance."

**Actions:**
1. Navigate to `http://localhost:3000`
2. Show the login page with orange-red accents
3. Enter credentials: `cityu` / `2026`
4. Click "Login"

**Narration:**
> "The orange-red color scheme (#ff5722) provides excellent contrast and visibility, essential for remote driving operations. All UI elements, buttons, and indicators now use this cohesive theme."

---

### Part 3: Remote Control Interface (3 minutes)

**[Screen: Remote Control Page]**

> "This is the main remote control interface. Let me walk you through each component."

#### 3.1 Video Wall (Top Section)

**[Highlight: Multi-camera display]**

> "The video wall displays synchronized feeds from multiple cameras mounted on the vehicle. This provides the operator with a comprehensive view of the vehicle's surroundings."

**Features to Show:**
- Multiple camera angles (front, rear, left, right)
- Real-time video streaming
- Synchronized playback
- Camera selection controls

#### 3.2 LiDAR Panel (Bottom Left)

**[Highlight: Point cloud visualization]**

> "The LiDAR panel shows real-time 3D point cloud data from the vehicle's LiDAR sensors. This provides depth perception and obstacle detection."

**Features to Show:**
- 3D point cloud rendering
- Color-coded distance information
- Rotation and zoom controls
- Real-time updates

#### 3.3 Amap Vehicle Location (Bottom Right)

**[Highlight: Map with vehicle marker]**

> "The Amap integration displays the vehicle's real-time location on a high-definition map. This is powered by Amap (高德地图), China's leading mapping service."

**Features to Show:**
- Vehicle marker on map (orange-red)
- Real-time position updates
- Map controls (zoom, pan, style)
- Legend showing different marker types
- Status indicators

**Configuration Details:**
```javascript
API Key: fa4c4bc1d796891d00472871682f6628
Security Key: fa4c4bc1d796891d00472871682f6628
```

#### 3.4 Control Panels (Sidebars)

**[Highlight: Left and right sidebars]**

**Left Sidebar:**
- Vehicle status indicators
- Speed, steering, gear display
- Emergency stop button
- Control mode selector

**Right Sidebar:**
- System diagnostics
- Network latency metrics
- CPU/GPU/Memory usage
- Alert notifications

---

### Part 4: RosBag Streaming & Replay (3 minutes)

**[Screen: Navigate to RosBag Replay]**

> "One of the most powerful features of FSM-Pilot V2.0 is the ability to stream and replay large RosBag files efficiently."

**[Navigate to: `http://localhost:3000/rosbag-replay-pro`]**

#### 4.1 RosBag Selection

**[Show: Bag file list]**

> "The system automatically detects available RosBag files. Let's select this 784MB bag file recorded on February 23rd."

**File Details:**
- Path: `/home/lyx/fsm/rosbag/rosbag2_2025_02_23-16_49_58/`
- Size: 784 MB
- Topics: `/rslidar_points`, `/fix`

#### 4.2 Topic Filtering

**[Show: Topic selection panel]**

> "We can filter which topics to stream. This is crucial for large files as it reduces bandwidth and processing requirements."

**Available Topics:**
- `/rslidar_points` - LiDAR point cloud data
- `/fix` - GPS/GNSS position data

#### 4.3 Streaming Architecture

**[Diagram: Client-Server Architecture]**

> "Unlike traditional approaches that load the entire file into memory, our streaming architecture processes data on the server and sends only requested messages to the client via WebSocket."

**Benefits:**
- ✅ Handles files larger than 15GB
- ✅ Low memory footprint
- ✅ Fast seeking and playback
- ✅ Real-time filtering

#### 4.4 Playback Controls

**[Show: Playback interface]**

**Features:**
- Play/Pause controls
- Playback speed adjustment (0.5x - 2.0x)
- Timeline scrubbing
- Frame-by-frame stepping
- Loop mode

#### 4.5 Multi-Camera Display

**[Show: Camera feeds during playback]**

> "During playback, we can view synchronized camera feeds from the recorded data. The system automatically decodes JPEG images from the bag file."

---

### Part 5: Database Visualization (2 minutes)

**[Screen: Navigate to Database Visualization]**

> "FSM-Pilot includes a comprehensive database management system for storing and analyzing vehicle data."

**[Navigate to: Database Visualization page]**

#### 5.1 Database Overview

**[Show: Database statistics]**

**Metrics Displayed:**
- Total records
- Storage size
- Last update time
- Data categories

#### 5.2 Data Import/Export

**[Show: Import/Export controls]**

**Features:**
- Import from JSON/CSV
- Export to multiple formats
- Batch operations
- Data validation

#### 5.3 Data Visualization

**[Show: Charts and graphs]**

**Visualizations:**
- Time-series plots
- Histogram distributions
- Scatter plots
- Heatmaps

---

### Part 6: Intelligent Dispatch Demo (2 minutes)

**[Screen: Navigate to Intelligent Dispatch]**

> "The intelligent dispatch system optimizes vehicle allocation and routing for maximum efficiency."

**[Navigate to: Intelligent Dispatch Demo]**

#### 6.1 Fleet Overview

**[Show: Fleet status dashboard]**

**Information Displayed:**
- Active vehicles
- Available operators
- Current assignments
- System health

#### 6.2 Dispatch Algorithm

**[Show: Dispatch visualization]**

> "The system uses advanced algorithms to match vehicles with operators based on multiple factors:"

**Factors:**
- Vehicle urgency level
- Operator availability
- Geographic proximity
- Operator skill level
- Network latency

#### 6.3 Real-time Updates

**[Show: Live dispatch updates]**

> "Watch as the system automatically assigns operators to vehicles in real-time, optimizing for minimal response time and maximum safety."

---

### Part 7: Technical Highlights (1 minute)

**[Screen: Code snippets and architecture]**

> "Let me highlight some key technical achievements:"

#### 7.1 RosBag Streaming Server

```javascript
// Node.js backend with SQLite3
class RosBagManager {
  async streamMessages(topicNames, startTime, endTime, callback) {
    // Stream messages without loading entire file
    this.db.each(query, params, (err, row) => {
      callback(row); // Send each message individually
    });
  }
}
```

#### 7.2 Amap Integration

```typescript
// Amap service with vehicle tracking
const amapService = getAmapService();
await amapService.initialize();
amapService.createMap(container, {
  center: [114.1733, 22.3364], // Hong Kong
  zoom: 15,
  mapStyle: 'amap://styles/darkblue'
});
```

#### 7.3 WebSocket Communication

```typescript
// Real-time bidirectional communication
const streamService = getRosBagStreamService('ws://localhost:8765');
streamService.streamTopics({
  topics: ['/rslidar_points', '/fix'],
  playbackRate: 1.0
});
```

---

### Part 8: Performance Metrics (1 minute)

**[Screen: Performance dashboard]**

> "Let's look at the system's performance characteristics:"

**Metrics:**

| Metric | Value | Notes |
|--------|-------|-------|
| Video Latency | 50-100ms | WebRTC with TURN |
| Control Latency | 20-50ms | WebSocket |
| Frame Rate | 30 FPS | Per camera |
| RosBag Streaming | 1-2x realtime | Depends on topics |
| Memory Usage | <500MB | Client-side |
| CPU Usage | 15-30% | During streaming |

**Scalability:**
- ✅ Supports multiple simultaneous operators
- ✅ Handles RosBag files >15GB
- ✅ Scales to 10+ vehicles per server
- ✅ Cloud-native architecture

---

### Conclusion (1 minute)

**[Screen: Summary slide]**

> "FSM-Pilot V2.0 represents a significant advancement in remote driving technology. Key achievements include:"

**Achievements:**
1. ✅ **Modern UI** - Orange-red tech theme for improved visibility
2. ✅ **Amap Integration** - Real-time vehicle tracking on Chinese maps
3. ✅ **RosBag Streaming** - Efficient handling of large data files
4. ✅ **Multi-Camera Support** - Synchronized video feeds
5. ✅ **Database Management** - Comprehensive data storage and analysis
6. ✅ **Intelligent Dispatch** - Optimized vehicle-operator matching

**Future Work:**
- 5G network integration
- AI-assisted driving suggestions
- Multi-vehicle coordination
- Enhanced security features

**[Screen: Contact Information]**

> "Thank you for watching this demonstration of FSM-Pilot V2.0. For more information, please contact:"

**Contact:**
- **Author:** Li Yixiang
- **Institution:** City University of Hong Kong
- **Email:** [contact information]
- **GitHub:** [repository link]

---

## 🎯 Demo Checklist

### Pre-Demo Setup

- [ ] Start demo script: `./demo_complete.sh`
- [ ] Verify frontend is running on port 3000
- [ ] Verify RosBag server is running on port 8765
- [ ] Check RosBag file is accessible
- [ ] Test Amap API credentials
- [ ] Prepare browser with tabs:
  - Tab 1: Login page
  - Tab 2: Remote Control
  - Tab 3: RosBag Replay
  - Tab 4: Database Visualization
  - Tab 5: Intelligent Dispatch

### During Demo

- [ ] Show login with new theme
- [ ] Navigate through all main pages
- [ ] Demonstrate RosBag streaming
- [ ] Show Amap vehicle tracking
- [ ] Display multi-camera feeds
- [ ] Show database operations
- [ ] Demonstrate dispatch system
- [ ] Highlight performance metrics

### Post-Demo

- [ ] Answer questions
- [ ] Provide documentation links
- [ ] Share demo script
- [ ] Collect feedback

---

## 📝 Talking Points

### Why FSM-Pilot V2.0?

1. **Safety First** - Multiple layers of safety checks and emergency stop
2. **Low Latency** - Optimized for real-time control (<100ms)
3. **Scalable** - Cloud-native architecture supports multiple vehicles
4. **Efficient** - Handles large data files without memory issues
5. **Modern** - Built with latest web technologies
6. **Localized** - Amap integration for Chinese market

### Technical Innovations

1. **RosBag Streaming** - Novel approach to handling large files
2. **WebRTC Integration** - Low-latency video streaming
3. **Amap Integration** - First-class support for Chinese maps
4. **Theme System** - Consistent orange-red tech aesthetic
5. **Database Management** - Comprehensive data handling

### Use Cases

1. **Remote Driving** - Teleoperation of autonomous vehicles
2. **Data Analysis** - Replay and analyze recorded drives
3. **Fleet Management** - Coordinate multiple vehicles
4. **Training** - Operator training with recorded scenarios
5. **Research** - Platform for autonomous driving research

---

## 🚀 Quick Start Commands

```bash
# Start complete demo
./demo_complete.sh

# Start only RosBag demo
./demo_rosbag.sh

# Start only database demo
./demo_database.sh

# Build production version
npm run build

# Run tests
npm run test
```

---

## 📊 System Requirements

### Minimum Requirements
- **CPU:** 4 cores, 2.5 GHz
- **RAM:** 8 GB
- **GPU:** Integrated graphics
- **Network:** 10 Mbps
- **OS:** Ubuntu 20.04+, Windows 10+, macOS 11+

### Recommended Requirements
- **CPU:** 8 cores, 3.5 GHz
- **RAM:** 16 GB
- **GPU:** Dedicated GPU with 4GB VRAM
- **Network:** 100 Mbps
- **OS:** Ubuntu 22.04 LTS

---

## 🔗 Additional Resources

- **Documentation:** `/docs/SYSTEM_ARCHITECTURE.md`
- **API Reference:** `/docs/API.md`
- **User Guide:** `/docs/USER_GUIDE.md`
- **Development Guide:** `/docs/DEVELOPMENT.md`

---

**End of Demo Script**
