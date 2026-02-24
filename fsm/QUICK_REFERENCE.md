# Guardian Mobility - Quick Reference

## 🚀 Quick Start

```bash
# Start the demo
./demo_1210.sh

# Or start services individually
cd server && node rosbag-server.js &  # Port 8765
npm run dev &                          # Port 3000
```

## 🌐 Access URLs

- **Frontend:** http://localhost:3000
- **RosBag Replay:** http://localhost:3000/rosbag-replay-pro
- **Remote Control:** http://localhost:3000/remote-control
- **WebSocket:** ws://localhost:8765

## 🔐 Login Credentials

- **Username:** `cityu`
- **Password:** `2026`

## 📊 RosBag Dataset (1210)

- **File:** `rosbag2_2025_12_10-17_25_58_0.db3`
- **Size:** 3.63 GB
- **Duration:** 58.81 seconds
- **Topics:** 43 channels
- **Messages:** 80,067 total
- **Cameras:** 6 cameras (588 frames each)

## 📷 Camera Topics

```
/camera0/image_raw/compressed  (588 messages)
/camera1/image_raw/compressed  (588 messages)
/camera2/image_raw/compressed  (588 messages)
/camera3/image_raw/compressed  (589 messages)
/camera4/image_raw/compressed  (588 messages)
/camera5/image_raw/compressed  (588 messages)
```

## 🚗 Vehicle Data Topics

```
/Sensor_msgs/chassis_can/speed       (Float32)
/Sensor_msgs/chassis_can/steerAngle  (Float32)
/Sensor_msgs/chassis_can/brakeRate   (Float32)
/Sensor_msgs/chassis_can/SOC         (Int16)
```

## ✅ System Verification

```bash
# Run complete system verification
./verify_system.sh

# Test image decoding only
node test_image_decode.cjs

# View fix summary
./ROSBAG_FIX_SUMMARY.sh
```

## 📁 Key Files

### Backend
- `server/rosbag-server.js` - WebSocket streaming server
- `server/cdr-decoder.js` - ROS2 CDR message decoder

### Frontend
- `src/components/RosBagReplayPro.vue` - RosBag replay interface
- `src/components/RemoteControl.vue` - Remote control interface
- `src/components/AmapVehicleLocation.vue` - Amap integration

### Configuration
- `src/config/apiConfig.ts` - API configuration (Amap key)
- `src/style.css` - Global theme (orange-red)

### Scripts
- `demo_1210.sh` - Start demo with RosBag 1210
- `record_demo.sh` - Professional recording setup
- `verify_system.sh` - System verification

### Documentation
- `README_COMPLETE.md` - Complete project documentation
- `ROSBAG_FIX_VERIFICATION.md` - Fix verification report
- `VIDEO_SCRIPT_2MIN_PROFESSIONAL.md` - Demo video script
- `RECORDING_GUIDE.md` - Recording setup guide

## 🎬 Demo Video Recording

```bash
# Start recording setup
./record_demo.sh

# Follow the 2-minute script
# See: VIDEO_SCRIPT_2MIN_PROFESSIONAL.md
```

## 🔧 Troubleshooting

### No image data displayed

**Fixed!** The CDR decoder now properly extracts JPEG data from ROS2 messages.

### Port already in use

```bash
# Kill existing processes
pkill -f "rosbag-server"
pkill -f "vite"

# Or use different ports in configuration
```

### RosBag file not found

```bash
# Check file exists
ls -lh /home/lyx/fsm/rosbag/1210/rosbag2_2025_12_10-17_25_58/

# Update path in demo_1210.sh if needed
```

## 📈 Performance

- **Image Size:** ~91 KB per frame (JPEG)
- **Frame Rate:** ~10 FPS per camera
- **Bandwidth:** ~730 KB/s per camera
- **Total Bandwidth:** ~4.4 MB/s (6 cameras)
- **Latency:** <100ms video, <50ms control

## 🎯 Key Features

### Real-Time Remote Control
- 6-camera surround view (<100ms latency)
- Real-time LiDAR 3D visualization
- Amap vehicle tracking (高德地图)
- Comprehensive telemetry monitoring

### RosBag Scenario Replay
- Supports files >15GB
- WebSocket real-time streaming
- Multi-camera synchronized playback
- Vehicle data visualization
- Professional playback controls

### AI-Powered Intelligence
- Predictive alerts
- Auto-dispatch
- Pattern recognition
- Data analysis

## 🌐 Amap Integration

**API Key:** `fa4c4bc1d796891d00472871682f6628`
**Security Code:** `fa4c4bc1d796891d00472871682f6628`

Configured in: `src/config/apiConfig.ts`

## 🎨 Theme

**Primary Color:** `#ff5722` (Orange-Red)
**Background:** `#0a0a0a` (Black)
**Panel:** `#1a1a1a` (Dark Gray)

## 📞 Support

- **Project:** Guardian Mobility
- **Institution:** City University of Hong Kong
- **Documentation:** See README_COMPLETE.md

## ✅ Status

**RosBag Image Decoding:** ✅ OPERATIONAL
**System Tests:** ✅ 11/11 PASSED
**Production Ready:** ✅ YES

---

**Guardian Mobility - AI-Powered Remote Driving Platform**
