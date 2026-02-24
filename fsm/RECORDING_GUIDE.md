# Guardian Mobility - Professional Recording Guide

**Product:** Guardian Mobility - AI-Powered Remote Driving Platform
**Target:** Production-quality 2-minute demo video
**Audience:** Investors, Enterprise Clients, Industry Leaders

---

## 🎯 Recording Objective

Create a **professional, investor-grade 2-minute demo video** showcasing Guardian Mobility's key features with emphasis on:
1. Real-time remote control capabilities
2. RosBag scenario replay (3.7GB dataset)
3. Multi-camera synchronized playback
4. AI-powered data analysis
5. Enterprise-grade safety features

---

## 📋 Pre-Recording Checklist

### System Preparation

```bash
# 1. Close unnecessary applications
killall chrome firefox slack discord telegram

# 2. Enable Do Not Disturb
# Ubuntu: Settings → Notifications → Do Not Disturb
# macOS: Control Center → Do Not Disturb

# 3. Set screen resolution
xrandr --output HDMI-1 --mode 1920x1080 --rate 60

# 4. Clear browser cache
# Chrome: Ctrl+Shift+Delete → Clear data

# 5. Disable desktop effects (for performance)
# Ubuntu: Settings → Appearance → Animations: Off
```

### Audio Setup

```bash
# Test microphone
arecord -d 5 test.wav && aplay test.wav

# Adjust levels
alsamixer
# Set microphone to 80-90%
# Set desktop audio to 60-70%
```

---

## 🎥 OBS Studio Setup (Recommended)

### Installation

```bash
# Ubuntu/Debian
sudo apt install obs-studio

# macOS
brew install --cask obs

# Or download from: https://obsproject.com/
```

### OBS Configuration

#### 1. Scene Setup

**Scene Name:** "Guardian Mobility Demo"

**Sources (in order):**
1. **Window Capture**
   - Window: Chrome/Firefox (Guardian Mobility)
   - Capture Method: Automatic
   - Crop: None

2. **Audio Input Capture**
   - Device: Your Microphone
   - Volume: -6dB to -3dB

3. **Audio Output Capture**
   - Device: Desktop Audio
   - Volume: -12dB to -9dB

#### 2. Video Settings

```
Settings → Video

Base (Canvas) Resolution: 1920x1080
Output (Scaled) Resolution: 1920x1080
Downscale Filter: Lanczos (Sharpened scaling, 36 samples)
Common FPS Values: 60
```

#### 3. Output Settings

```
Settings → Output

Output Mode: Advanced

Recording:
  Type: Standard
  Recording Path: ~/Videos/guardian_mobility/
  Recording Format: mp4

  Encoder: x264
  Rate Control: CBR
  Bitrate: 15000 Kbps
  Keyframe Interval: 2
  CPU Usage Preset: veryfast
  Profile: high
  Tune: none

Audio Track: 1
```

#### 4. Audio Settings

```
Settings → Audio

Sample Rate: 48 kHz

Desktop Audio:
  Device: Default

Mic/Auxiliary Audio:
  Device: Your Microphone
```

#### 5. Hotkeys

```
Settings → Hotkeys

Start Recording: F9
Stop Recording: F10
Pause Recording: F11
```

---

## 🎬 Recording Workflow

### Step 1: Start Demo Environment

```bash
cd /home/lyx/fsm
./record_demo.sh
```

Wait for:
- ✓ RosBag server ready (port 8765)
- ✓ Frontend server ready (port 3000)
- ✓ Browser opens automatically

### Step 2: Browser Setup

1. **Open Chrome/Firefox in full-screen**
   ```
   Press F11 for full-screen mode
   ```

2. **Navigate to Guardian Mobility**
   ```
   http://localhost:3000
   ```

3. **Login**
   ```
   Username: cityu
   Password: 2026
   ```

4. **Prepare tabs**
   - Tab 1: Remote Control (main page)
   - Tab 2: RosBag Replay Pro

### Step 3: OBS Recording Setup

1. **Open OBS Studio**

2. **Select Scene**
   - "Guardian Mobility Demo"

3. **Position Window Capture**
   - Fit to screen (1920x1080)
   - No black bars

4. **Test Audio**
   - Speak into microphone
   - Check levels (green, not red)
   - Play desktop audio
   - Check levels (green, not red)

5. **Test Recording**
   - Press F9 to start
   - Record for 5 seconds
   - Press F10 to stop
   - Review test recording
   - Delete test file

### Step 4: Rehearsal

**Do 2-3 practice runs:**

1. **Timing Check**
   - Use stopwatch
   - Aim for 2:00-2:10 total
   - Adjust pace as needed

2. **Narration Practice**
   - Read script aloud
   - Mark difficult words
   - Practice transitions

3. **Mouse Movement**
   - Smooth, deliberate movements
   - Highlight key features
   - No erratic movements

### Step 5: Final Recording

1. **Deep breath, relax**

2. **Start OBS recording (F9)**

3. **Count down silently**
   - 3... 2... 1... Begin

4. **Follow script exactly**
   - Refer to: VIDEO_SCRIPT_2MIN_PROFESSIONAL.md
   - Maintain professional tone
   - Speak clearly and confidently

5. **Stop recording (F10)**

6. **Review immediately**
   - Check audio sync
   - Check visual quality
   - Check timing
   - Note any issues

7. **Re-record if needed**
   - Don't settle for "good enough"
   - Aim for perfection

---

## 📊 Recording Timeline (Detailed)

### 0:00-0:10 | Opening (10 seconds)

**Actions:**
1. Browser shows login page
2. Type username: `cityu`
3. Type password: `2026`
4. Click "Login" button
5. Smooth transition to dashboard

**Narration:**
> "Guardian Mobility - The future of autonomous vehicle remote operation."

**Mouse Movement:**
- Smooth, professional
- Highlight login button
- No unnecessary movements

---

### 0:10-0:40 | Remote Control (30 seconds)

#### 0:10-0:18 | Multi-Camera (8 seconds)

**Actions:**
1. Hover over video wall
2. Highlight each camera feed
3. Show latency indicator (78ms)
4. Show frame rate (30 FPS)

**Narration:**
> "Six synchronized cameras provide complete 360-degree awareness with sub-100 millisecond latency."

**Visual Cues:**
- Pulse effect on cameras (if available)
- Cursor circles each camera briefly

#### 0:18-0:25 | LiDAR (7 seconds)

**Actions:**
1. Move to LiDAR panel
2. Show 3D point cloud
3. Rotate view slightly
4. Highlight distance indicators

**Narration:**
> "Real-time LiDAR visualization provides precise 3D environmental awareness."

**Visual Cues:**
- Smooth rotation of point cloud
- Cursor highlights key features

#### 0:25-0:32 | Amap (7 seconds)

**Actions:**
1. Move to map panel
2. Show vehicle marker (orange-red)
3. Zoom in slightly
4. Show real-time position

**Narration:**
> "Integrated Amap tracking ensures precise vehicle location and route optimization."

**Visual Cues:**
- Cursor circles vehicle marker
- Brief zoom animation

#### 0:32-0:40 | Telemetry (8 seconds)

**Actions:**
1. Move to telemetry panel
2. Highlight speed gauge
3. Highlight steering angle
4. Highlight battery level

**Narration:**
> "Comprehensive telemetry monitoring ensures complete vehicle awareness and control."

**Visual Cues:**
- Cursor highlights each metric
- Smooth transitions

---

### 0:40-1:30 | RosBag Replay (50 seconds)

#### 0:40-0:48 | Data Set Intro (8 seconds)

**Actions:**
1. Navigate to RosBag Replay Pro
2. Show file browser
3. Select 1210 RosBag file
4. Show file statistics

**Narration:**
> "Guardian Mobility handles massive data sets - this 3.7 gigabyte RosBag contains real-world driving scenarios."

**Visual Cues:**
- Cursor selects file
- Statistics appear smoothly

#### 0:48-0:58 | Streaming Architecture (10 seconds)

**Actions:**
1. Show topic list (43 topics)
2. Highlight camera topics
3. Highlight LiDAR topics
4. Show streaming indicator

**Narration:**
> "Our proprietary streaming architecture handles files over 15 gigabytes without loading into memory."

**Visual Cues:**
- Scroll through topics
- Cursor highlights key topics

#### 0:58-1:15 | Multi-Camera Playback (17 seconds)

**Actions:**
1. Select 6 camera topics
2. Click "Start Streaming"
3. Show all 6 cameras playing
4. Adjust playback speed (1.0x → 1.5x)
5. Show timeline scrubbing

**Narration:**
> "Six synchronized camera feeds replay real driving scenarios. Perfect for training, analysis, and incident review."

**Visual Cues:**
- All cameras start simultaneously
- Smooth playback
- Speed control demonstration

#### 1:15-1:25 | Data Visualization (10 seconds)

**Actions:**
1. Show vehicle data panel
2. Highlight speed graph
3. Highlight steering graph
4. Show AI insights badge

**Narration:**
> "Real-time data visualization reveals vehicle behavior patterns and enables AI-powered insights."

**Visual Cues:**
- Graphs animate
- Data updates in real-time

#### 1:25-1:30 | Playback Controls (5 seconds)

**Actions:**
1. Show playback controls
2. Demonstrate pause/play
3. Show timeline scrubbing
4. Show speed control

**Narration:**
> "Professional-grade controls enable detailed scenario analysis."

**Visual Cues:**
- Quick control demonstration
- Smooth interactions

---

### 1:30-1:50 | Key Differentiators (20 seconds)

**Actions:**
1. Return to main dashboard
2. Show all panels simultaneously
3. Highlight key metrics
4. Show system health indicators

**Narration:**
> "Guardian Mobility combines enterprise-grade safety, ultra-low latency, AI-powered intelligence, and big data capabilities - all in one platform."

**Visual Cues:**
- Cursor highlights each feature
- Professional pacing

---

### 1:50-2:00 | Use Cases (10 seconds)

**Actions:**
1. Quick montage of features
2. Show different panels
3. Demonstrate versatility

**Narration:**
> "From robotaxis to mining operations - Guardian Mobility scales across industries."

**Visual Cues:**
- Quick transitions
- Professional flow

---

### 2:00-2:10 | Closing (10 seconds)

**Actions:**
1. Return to main dashboard
2. Show full interface
3. Fade to black (if editing)

**Narration:**
> "Guardian Mobility. The future of safe, intelligent remote driving."

**Visual Cues:**
- Professional ending
- Smooth fade

---

## 🎨 Visual Guidelines

### Mouse Cursor

**Do:**
- ✓ Smooth, deliberate movements
- ✓ Highlight key features
- ✓ Pause briefly on important elements
- ✓ Professional pace

**Don't:**
- ✗ Erratic movements
- ✗ Unnecessary circling
- ✗ Too fast movements
- ✗ Shaking or jittering

### Transitions

**Between Sections:**
- Smooth scrolling
- Professional pacing
- Clear visual flow
- No abrupt jumps

**Between Pages:**
- Click navigation clearly
- Wait for page load
- Smooth transitions

### Highlighting

**Methods:**
- Cursor hover (2-3 seconds)
- Gentle circling (if needed)
- Brief pause on feature
- Clear visual focus

---

## 🎤 Narration Guidelines

### Voice Quality

**Tone:**
- Professional
- Confident
- Enthusiastic (but not over-the-top)
- Clear enunciation

**Pace:**
- Moderate speed
- Clear pauses between sections
- Emphasis on key words
- Natural rhythm

**Volume:**
- Consistent level
- No sudden changes
- Clear and audible
- Professional quality

### Script Delivery

**Tips:**
1. **Practice 5-10 times** before recording
2. **Mark difficult words** in script
3. **Breathe naturally** between sentences
4. **Smile while speaking** (improves tone)
5. **Stand while recording** (better voice projection)
6. **Use hand gestures** (even though not visible - helps energy)

### Common Mistakes to Avoid

- ✗ Speaking too fast
- ✗ Monotone delivery
- ✗ Filler words (um, uh, like)
- ✗ Breathing into microphone
- ✗ Background noise
- ✗ Inconsistent volume

---

## 📁 File Management

### Recording Files

**Location:**
```
~/Videos/guardian_mobility/
├── raw/
│   ├── take_01.mp4
│   ├── take_02.mp4
│   └── take_03.mp4
├── edited/
│   └── guardian_mobility_demo_final.mp4
└── exports/
    ├── guardian_mobility_demo_1080p.mp4
    ├── guardian_mobility_demo_720p.mp4
    └── guardian_mobility_demo_thumbnail.jpg
```

### Naming Convention

```
guardian_mobility_demo_[version]_[date].mp4

Examples:
- guardian_mobility_demo_v1_20260119.mp4
- guardian_mobility_demo_final_20260119.mp4
- guardian_mobility_demo_investor_20260119.mp4
```

---

## ✂️ Post-Production (Optional)

### Basic Editing

**Software Options:**
1. **DaVinci Resolve** (Free, Professional)
2. **Kdenlive** (Free, Linux)
3. **iMovie** (Free, macOS)
4. **Camtasia** (Paid, Easy)

### Editing Checklist

- [ ] Trim beginning/end
- [ ] Remove mistakes/pauses
- [ ] Add intro title (3 seconds)
- [ ] Add outro with contact info (5 seconds)
- [ ] Add on-screen annotations
- [ ] Add background music (subtle)
- [ ] Color grading (enhance orange-red)
- [ ] Audio normalization
- [ ] Export in multiple formats

### On-Screen Annotations

**Add text overlays for:**
- Feature names
- Key metrics
- Performance indicators
- Contact information

**Style:**
- Font: Sans-serif, bold
- Color: White or orange-red
- Background: Semi-transparent black
- Animation: Fade in/out

---

## 📤 Export Settings

### High Quality (Investor/Conference)

```
Format: MP4 (H.264)
Resolution: 1920x1080
Frame Rate: 60 FPS
Bitrate: 15 Mbps (CBR)
Audio: AAC, 320 kbps, 48 kHz
File Size: ~150-200 MB
```

### Medium Quality (Web/Social)

```
Format: MP4 (H.264)
Resolution: 1920x1080
Frame Rate: 30 FPS
Bitrate: 8 Mbps (CBR)
Audio: AAC, 192 kbps, 48 kHz
File Size: ~80-100 MB
```

### Low Quality (Email/Mobile)

```
Format: MP4 (H.264)
Resolution: 1280x720
Frame Rate: 30 FPS
Bitrate: 4 Mbps (CBR)
Audio: AAC, 128 kbps, 48 kHz
File Size: ~40-50 MB
```

---

## 🚀 Quick Start

### One-Command Setup

```bash
# Start recording environment
cd /home/lyx/fsm
./record_demo.sh
```

### Recording Checklist

```
Pre-Recording:
[ ] System prepared (notifications off, apps closed)
[ ] Audio tested (microphone + desktop)
[ ] OBS configured (1080p60, 15Mbps)
[ ] Browser ready (full-screen, logged in)
[ ] Script reviewed (practiced 3+ times)

During Recording:
[ ] OBS recording started (F9)
[ ] Following script timing
[ ] Smooth mouse movements
[ ] Clear narration
[ ] Professional pace

Post-Recording:
[ ] Recording stopped (F10)
[ ] File saved successfully
[ ] Immediate review completed
[ ] Quality check passed
[ ] Backup created
```

---

## 📞 Support

**Questions or Issues?**
- Review: VIDEO_SCRIPT_2MIN_PROFESSIONAL.md
- Check: OBS Studio documentation
- Test: Audio/video before final recording

---

## 🎯 Success Criteria

**Your recording is ready when:**
- ✓ Duration: 2:00-2:10 minutes
- ✓ Audio: Clear, professional, no background noise
- ✓ Video: Smooth, 1080p60, no lag
- ✓ Content: All key features demonstrated
- ✓ Narration: Clear, confident, well-paced
- ✓ Timing: Matches script exactly
- ✓ Quality: Investor-grade, professional

---

**Good luck with your recording! 🎬**

**Guardian Mobility - The future of safe, intelligent remote driving.**
