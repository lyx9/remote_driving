# FSM-Pilot V2.0 - Verification Checklist

**Date**: 2026-01-15
**Version**: 2.0

Use this checklist to verify the system is properly set up and functioning.

---

## ✅ Pre-Demo Verification

### 1. System Environment

- [ ] **Node.js Version**: v18.0.0 or higher
  ```bash
  node --version
  ```

- [ ] **npm Installed**: v9.0.0 or higher
  ```bash
  npm --version
  ```

- [ ] **Dependencies Installed**:
  ```bash
  cd /home/lyx/fsm
  npm install
  ```

- [ ] **Port 5173 Available**:
  ```bash
  lsof -i:5173  # Should show nothing
  ```

### 2. File Structure

- [ ] **Demo Script Exists**:
  ```bash
  ls -lh start_demo.sh
  # Should show: -rwxrwxr-x ... start_demo.sh
  ```

- [ ] **Documentation Files**:
  ```bash
  ls -lh docs/
  # Should include:
  # - API_CONFIGURATION.md
  # - DEMO_VIDEO_GUIDE.md
  # - DEMO_SCRIPT_USAGE.md
  # - LATEST_UPDATES.md
  ```

- [ ] **Environment Template**:
  ```bash
  ls -lh .env.example
  # Should exist
  ```

- [ ] **New Components**:
  ```bash
  ls -lh src/components/
  # Should include:
  # - TakeoverConfirmDialog.vue
  # - RemoteAssistantPanel.vue
  # - AmapVehicleLocation.vue
  ```

- [ ] **New Services**:
  ```bash
  ls -lh src/services/
  # Should include:
  # - doubaoLLMService.ts
  # - amapService.ts
  ```

### 3. Build Verification

- [ ] **TypeScript Compilation**:
  ```bash
  npm run build
  # Should complete without errors
  # ✓ 160 modules transformed
  ```

- [ ] **No Type Errors**:
  - Check build output for any TypeScript errors
  - All should be resolved

### 4. API Configuration (Optional)

- [ ] **Environment File Created**:
  ```bash
  cp .env.example .env.local
  ```

- [ ] **API Keys Configured** (if available):
  - [ ] VITE_DOUBAO_API_KEY set
  - [ ] VITE_AMAP_API_KEY set
  - [ ] VITE_AMAP_JS_CODE set

**Note**: APIs are optional. System works with local fallbacks.

---

## ✅ Demo Script Verification

### 1. Script Execution

- [ ] **Script is Executable**:
  ```bash
  chmod +x start_demo.sh
  ./start_demo.sh
  ```

- [ ] **Menu Displays**:
  - Should show 5 options
  - Banner displays correctly
  - No error messages

### 2. System Checks (Option 4)

- [ ] **Run Performance Metrics**:
  ```bash
  ./start_demo.sh
  # Select option 4: Performance Metrics Only
  ```

- [ ] **All Checks Pass**:
  - [x] Node.js version check
  - [x] npm availability
  - [x] Browser detection
  - [x] Port availability
  - [x] Dependencies check

### 3. Quick Demo (Option 2)

- [ ] **Run Quick Demo**:
  ```bash
  ./start_demo.sh
  # Select option 2: Quick Demo
  ```

- [ ] **Demo Completes**:
  - Server starts successfully
  - Browser opens automatically
  - Demo runs for ~1 minute
  - No crashes or errors

### 4. Full Demo (Option 1)

- [ ] **Run Full Demo**:
  ```bash
  ./start_demo.sh
  # Select option 1: Full Automated Demo
  ```

- [ ] **All 8 Tasks Execute**:
  1. Platform Login (0-30s)
  2. Work Interface Display (30-60s)
  3. AD Video Integration (60-90s)
  4. 100-Vehicle Fleet (90-120s)
  5. Risk Alert & AI Analysis (120-150s)
  6. Remote Assistant (150-165s)
  7. Takeover Execution (165-175s)
  8. Database Storage (175-180s)

---

## ✅ UI Component Verification

### 1. Login Page

- [ ] **Navigate to**: http://localhost:5173

- [ ] **Login Form Displays**:
  - FSM-Pilot V2.0 logo
  - Username field
  - Password field
  - Login button
  - City University of Hong Kong branding

- [ ] **Login Works**:
  - Username: `cityu`
  - Password: `2026`
  - Redirects to dashboard after login

### 2. Intelligent Dispatch Demo

- [ ] **Navigate to**: Intelligent Dispatch Demo page

- [ ] **Main Components Visible**:
  - Vehicle queue section
  - Statistics cards (Total Vehicles, Active Operators, etc.)
  - Control mode distribution chart
  - Algorithm performance metrics
  - Amap vehicle location (if configured)

- [ ] **Add Vehicles Button Works**:
  - Click "Add 10 Vehicles"
  - Vehicle count increases
  - Cards appear in queue

- [ ] **Priority Sorting Works**:
  - Critical vehicles (red) appear at top
  - High risk (orange) below critical
  - Medium (yellow) and Low (green) at bottom

### 3. Takeover Confirmation Dialog

- [ ] **Trigger Dialog**:
  - Find a high-risk or critical vehicle
  - Click "接管车辆" button
  - Dialog appears

- [ ] **Dialog Components Display**:
  - [ ] Urgency banner with risk score
  - [ ] AI Analysis section:
    - Incident description
    - Risk factors (4 cards)
    - Recommended actions (P0, P1, P2)
    - Takeover recommendation
  - [ ] Vehicle telemetry section:
    - Speed, control mode, location
    - Scenario, weather, system status
  - [ ] Operator assignment section:
    - Operator name and ID
    - Match score percentage
    - Status and load
    - "最佳匹配" badge
  - [ ] Double confirmation checkbox (for high-risk)
  - [ ] Confirm and Cancel buttons

- [ ] **Dialog Interaction**:
  - [ ] Checkbox works
  - [ ] Confirm button disabled until checkbox checked
  - [ ] Cancel button closes dialog
  - [ ] Confirm button executes takeover

### 4. Remote Assistant Panel

- [ ] **Panel Visibility**:
  - Should appear on right side (or can be toggled)
  - Shows "Remote Assistant" header
  - Displays AI status indicator

- [ ] **Select Vehicle**:
  - Click on any vehicle card
  - Remote Assistant updates for that vehicle

- [ ] **Suggestions Appear**:
  - [ ] Real-time suggestions display
  - [ ] Different priority levels visible:
    - 🚨 Critical (red)
    - ⚠️ High (orange)
    - ⚡ Medium (yellow)
    - ℹ️ Low (blue)
  - [ ] Suggestions have text and icons
  - [ ] Action buttons visible

- [ ] **Metrics Display**:
  - [ ] Adoption rate percentage
  - [ ] Risk reduction percentage
  - [ ] Response time (ms)
  - [ ] Today's suggestion count

- [ ] **Quick Actions Work**:
  - [ ] "紧急指导" button clickable
  - [ ] "路线优化" button clickable

### 5. Amap Integration

**If Amap API is configured**:

- [ ] **Map Loads**:
  - High-definition map appears
  - Dark theme applied
  - Hong Kong area centered

- [ ] **Vehicle Markers**:
  - [ ] Circular markers appear
  - [ ] Color-coded by urgency
  - [ ] Markers update in real-time

- [ ] **Operator Markers**:
  - [ ] Blue circular markers with 👤
  - [ ] Status-based coloring

- [ ] **Map Controls**:
  - [ ] Zoom in/out works
  - [ ] Pan/drag works
  - [ ] "适应视图" button works

**If Amap API is NOT configured**:

- [ ] **Configuration Notice**:
  - Message displays explaining API not configured
  - Instructions for setup visible
  - Link to console provided

---

## ✅ Data & Storage Verification

### 1. Mock Data Generation

- [ ] **Collision Scenario File**:
  ```bash
  cat /tmp/fsm_collision_scenario.json
  # Should contain complete scenario with AI analysis
  ```

- [ ] **Fleet Data File**:
  ```bash
  cat /tmp/fsm_fleet_data.json | head -n 50
  # Should contain 100 vehicles and 30 operators
  ```

### 2. IndexedDB Storage

- [ ] **Open Browser DevTools**: F12

- [ ] **Navigate to Application Tab**:
  - Storage → IndexedDB
  - Should see "FSM-Pilot" database

- [ ] **Check Tables**:
  - [ ] vehicles table
  - [ ] operators table
  - [ ] takeover_events table
  - [ ] ai_analyses table (if AI used)
  - [ ] telemetry table

- [ ] **Verify Records**:
  - Click on any table
  - Should see stored records
  - Data should be properly structured JSON

---

## ✅ API Integration Verification

### 1. Doubao LLM (If Configured)

- [ ] **Service Initialization**:
  - Check browser console
  - Should see: "Doubao LLM service initialized"

- [ ] **Analysis Requests**:
  - Trigger takeover dialog
  - AI Analysis section should show LLM-generated content
  - Status indicator shows "在线" (Online)
  - Processing time displayed (e.g., "850ms")

- [ ] **Fallback (If Not Configured)**:
  - AI Analysis uses rule-based content
  - Status indicator shows "离线" (Offline)
  - Faster processing time (<100ms)

### 2. Amap (If Configured)

- [ ] **Map API Loads**:
  - Check browser console
  - Should see: "Amap initialized successfully"
  - No CORS errors

- [ ] **Markers Render**:
  - Vehicles appear on map
  - Operators appear on map
  - Colors match urgency levels

- [ ] **Security Code**:
  - No security verification errors
  - API key accepted

- [ ] **Fallback (If Not Configured)**:
  - Configuration notice appears
  - Rest of UI still functional
  - Vehicle list view still works

---

## ✅ Performance Verification

### 1. Response Times

- [ ] **Risk Scoring**: <50ms
  - Check algorithm performance metrics panel
  - "平均评分时间" value

- [ ] **Operator Matching**: <150ms
  - Check "平均匹配时间" value

- [ ] **Takeover Latency**: <2000ms
  - Measure from button click to confirmation

### 2. UI Performance

- [ ] **Smooth Animations**:
  - Dialog transitions smooth
  - Vehicle card animations fluid
  - No janky scrolling

- [ ] **Responsive Updates**:
  - Statistics update in real-time
  - Charts redraw smoothly
  - No lag with 100+ vehicles

### 3. Memory & CPU

- [ ] **Open Browser Task Manager**:
  - Chrome: Shift+Esc
  - Firefox: about:performance

- [ ] **Check Resource Usage**:
  - [ ] Memory: <800MB
  - [ ] CPU: <50% during active use
  - [ ] No memory leaks over time

---

## ✅ Documentation Verification

### 1. README.md

- [ ] **Quick Start Section**:
  - Installation steps clear
  - Commands correct

- [ ] **Demo Script Section**:
  - Instructions accurate
  - Commands work

- [ ] **Links Work**:
  - All internal links navigate correctly

### 2. API_CONFIGURATION.md

- [ ] **Doubao Section**:
  - API endpoint correct
  - Example requests valid

- [ ] **Amap Section**:
  - Setup instructions clear
  - Security code explanation accurate

### 3. DEMO_VIDEO_GUIDE.md

- [ ] **8 Tasks Documented**:
  - All tasks have timing
  - Narration scripts provided
  - Screenshots would be helpful (optional)

### 4. DEMO_SCRIPT_USAGE.md

- [ ] **Usage Instructions**:
  - Commands accurate
  - Examples work

- [ ] **Troubleshooting Section**:
  - Common issues covered
  - Solutions provided

---

## ✅ Error Handling Verification

### 1. Graceful Degradation

- [ ] **No API Keys**:
  - System still works
  - Fallback functionality active
  - No crashes

- [ ] **Network Errors**:
  - API timeouts handled
  - Error messages displayed
  - User can continue working

- [ ] **Invalid Data**:
  - Form validation works
  - Invalid inputs rejected
  - Helpful error messages

### 2. Edge Cases

- [ ] **Empty Vehicle Queue**:
  - UI displays empty state
  - No errors in console

- [ ] **All Operators Busy**:
  - System handles gracefully
  - Queue continues to function

- [ ] **High Load (100+ vehicles)**:
  - Performance remains acceptable
  - No crashes or freezes

---

## ✅ Browser Compatibility

### 1. Chrome

- [ ] Login works
- [ ] All features functional
- [ ] No console errors
- [ ] Performance good

### 2. Firefox

- [ ] Login works
- [ ] All features functional
- [ ] No console errors
- [ ] Performance acceptable

### 3. Safari (macOS)

- [ ] Login works
- [ ] All features functional
- [ ] No console errors

---

## 🎯 Final Checklist

Before considering the system verified, ensure:

- [ ] All 8 demo tasks execute successfully
- [ ] No TypeScript compilation errors
- [ ] No runtime errors in browser console
- [ ] All major UI components render correctly
- [ ] API integrations work (or fallback correctly)
- [ ] Documentation is accurate and complete
- [ ] Demo script runs without issues
- [ ] Performance metrics meet targets
- [ ] Data is properly stored in IndexedDB

---

## 📝 Issue Reporting

If you find any issues during verification:

1. **Note the issue**: Describe what went wrong
2. **Check logs**: Browser console, terminal output
3. **Review docs**: Check troubleshooting sections
4. **Report**: Contact li.yixiang@cityu.edu.hk

---

## ✅ Sign-off

**Verification completed by**: _______________
**Date**: _______________
**Version tested**: 2.0
**All checks passed**: [ ] Yes [ ] No

**Notes**:
```
[Any additional observations or issues found]
```

---

**Last Updated**: 2026-01-15
**Version**: 2.0
