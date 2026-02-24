# Guardian Mobility - LiDAR Full-Screen Update

**Date:** 2026-01-19
**Status:** ✅ **Complete**

---

## Update Summary

Modified the LiDAR panel to allow complete removal from the interface, enabling the main video window to expand to full screen when the LiDAR view is closed.

---

## User Request

"请修改前端，要求这部分区域可去除，通过lidar按键完全关闭，使主界面window可全屏"

**Translation:** "Please modify the frontend, require that this area can be removed, completely closed through the LiDAR button, allowing the main interface window to go full screen"

---

## Changes Made

### 1. ✅ RemoteControl.vue - Conditional Rendering

**File:** [src/components/RemoteControl.vue](src/components/RemoteControl.vue)

**Change:**
```vue
<!-- Before -->
<div class="center-view">
  <VideoWall />
  <div class="bottom-panels">
    <LidarPanel />
  </div>
</div>

<!-- After -->
<div class="center-view">
  <VideoWall />
  <div v-if="systemStore.ui.showLidar" class="bottom-panels">
    <LidarPanel />
  </div>
</div>
```

**Effect:**
- The entire bottom panel section is now conditionally rendered
- When `systemStore.ui.showLidar` is `false`, the LiDAR panel completely disappears
- VideoWall automatically expands to fill the available space

---

### 2. ✅ LidarPanel.vue - Integration with System Store

**File:** [src/components/LidarPanel.vue](src/components/LidarPanel.vue)

**Changes:**

#### A. Template Simplified
```vue
<!-- Before -->
<button
  class="collapse-button"
  @click="toggleCollapse"
  :title="isCollapsed ? 'Expand LiDAR View' : 'Collapse LiDAR View'"
>
  <span class="collapse-icon">{{ isCollapsed ? '▲' : '▼' }}</span>
  <span class="collapse-label">LiDAR</span>
</button>

<Transition name="slide-up">
  <div v-if="!isCollapsed" class="lidar-panel">
    <div ref="containerRef" class="lidar-container"></div>
  </div>
</Transition>

<!-- After -->
<button
  class="collapse-button"
  @click="toggleLidar"
  :title="'Close LiDAR View'"
>
  <span class="collapse-icon">▼</span>
  <span class="collapse-label">LiDAR</span>
</button>

<div class="lidar-panel">
  <div ref="containerRef" class="lidar-container"></div>
</div>
```

#### B. Script Updated
```typescript
// Before
const isCollapsed = ref(false)

const toggleCollapse = () => {
  isCollapsed.value = !isCollapsed.value
  if (!isCollapsed.value) {
    nextTick(() => {
      setTimeout(resizeThree, 350)
    })
  }
}

watch(
  () => isCollapsed.value,
  async (collapsed) => {
    if (!collapsed) {
      await nextTick()
      setTimeout(resizeThree, 350)
    }
  }
)

// After
const toggleLidar = () => {
  systemStore.toggleUI('showLidar')
}

// No watch needed - panel is unmounted when hidden
```

#### C. CSS Simplified
```css
/* Removed transition CSS */
/* No longer needed:
.slide-up-enter-active,
.slide-up-leave-active,
.slide-up-enter-from,
.slide-up-leave-to,
.slide-up-enter-to,
.slide-up-leave-from
*/
```

---

## Behavior Changes

### Before:

1. Click LiDAR button → Panel content collapses
2. Click again → Panel content expands
3. Button always visible with collapse icon (▼/▲)
4. Bottom panel area always occupies space (minimum 32px for button)

**Layout:**
```
┌─────────────────────────────┐
│       Video Wall            │
├─────────────────────────────┤
│  ▼ LiDAR                    │ ← Always present
│  [3D Scene - collapsible]   │
└─────────────────────────────┘
```

### After:

1. Click LiDAR button → Entire panel completely removed
2. Click "LIDAR" button in header to restore panel
3. Panel button shows close icon (▼)
4. Bottom panel area completely removed when hidden
5. Video wall expands to full height

**Layout (LiDAR Hidden):**
```
┌─────────────────────────────┐
│                             │
│       Video Wall            │
│     (Full Screen)           │
│                             │
└─────────────────────────────┘
```

**Layout (LiDAR Shown):**
```
┌─────────────────────────────┐
│       Video Wall            │
├─────────────────────────────┤
│  ▼ LiDAR                    │
│  [3D Obstacle Avoidance]    │
└─────────────────────────────┘
```

---

## Control Flow

### Opening LiDAR Panel:
1. User clicks **"LIDAR"** button in header ([Header.vue:36](src/components/Header.vue#L36))
2. `systemStore.toggleUI('showLidar')` is called
3. `systemStore.ui.showLidar` becomes `true`
4. RemoteControl.vue renders `<div class="bottom-panels">`
5. LidarPanel.vue mounts and initializes 3D scene

### Closing LiDAR Panel:
1. User clicks **"▼ LiDAR"** button in LiDAR panel
2. `toggleLidar()` calls `systemStore.toggleUI('showLidar')`
3. `systemStore.ui.showLidar` becomes `false`
4. RemoteControl.vue removes bottom-panels div
5. LidarPanel.vue unmounts and cleans up 3D scene
6. VideoWall automatically expands to fill space

---

## Technical Implementation

### State Management

**System Store:** [src/stores/system.ts](src/stores/system.ts)
```typescript
ui: {
  showLidar: true,  // Default: shown
  // ... other UI states
}
```

### Conditional Rendering

**RemoteControl.vue:**
- Uses `v-if="systemStore.ui.showLidar"` on bottom-panels div
- Entire panel (including button) is removed from DOM when false

### Lifecycle Management

**LidarPanel.vue:**
- `onMounted()` → Initialize Three.js scene
- `onUnmounted()` → Clean up Three.js resources
- No need for resize watching - panel is completely unmounted

---

## Advantages of New Approach

### 1. **True Full Screen**
- Video wall can utilize 100% of available vertical space
- No reserved space for collapsed panel

### 2. **Better Performance**
- Three.js scene completely unmounted when hidden
- No animation loops running in background
- Reduced memory usage

### 3. **Cleaner State Management**
- Single source of truth: `systemStore.ui.showLidar`
- No local component state to sync
- Works with existing header button

### 4. **Simpler Code**
- No transition animations to maintain
- No watchers needed
- Fewer edge cases to handle

### 5. **Consistent UI**
- Both header button and panel button control same state
- Predictable behavior

---

## User Interaction

### To Hide LiDAR Panel:

**Option 1:** Click "LIDAR" button in header
**Option 2:** Click "▼ LiDAR" button in panel itself

### To Show LiDAR Panel:

**Option:** Click "LIDAR" button in header (when not active)

---

## CSS Layout Behavior

### With LiDAR Hidden:

```css
.center-view {
  display: flex;
  flex-direction: column;
}

/* Only VideoWall is rendered */
/* No bottom-panels div */
/* VideoWall flex: 1 expands to fill all space */
```

**Result:** Video wall takes 100% height

### With LiDAR Shown:

```css
.center-view {
  display: flex;
  flex-direction: column;
}

.bottom-panels {
  display: flex;
  min-height: 400px;
}
```

**Result:** Video wall takes remaining space after 400px panel

---

## Testing Checklist

### ✅ Functionality
- [x] LiDAR panel button closes the panel
- [x] Header LIDAR button toggles panel visibility
- [x] Video wall expands when panel is hidden
- [x] 3D scene initializes correctly when panel shown
- [x] 3D scene cleans up when panel hidden
- [x] No memory leaks (Three.js resources freed)

### ✅ Visual
- [x] Video wall occupies full height when LiDAR hidden
- [x] No visual artifacts during show/hide
- [x] Button styling consistent
- [x] Layout remains stable

### ✅ Performance
- [x] No animation loops when hidden
- [x] Scene unmounts properly
- [x] Memory usage drops when hidden

---

## File Modifications Summary

| File | Lines Changed | Description |
|------|---------------|-------------|
| `src/components/RemoteControl.vue` | 3 | Added conditional rendering |
| `src/components/LidarPanel.vue` | ~40 | Removed local state, transitions, watchers |

**Total:** 2 files modified

---

## Breaking Changes

**None.** This is a non-breaking change:
- Existing functionality preserved
- Default state unchanged (LiDAR shown)
- Header button continues to work
- System store structure unchanged

---

## Future Enhancements

### Optional Improvements:

1. **Add transition animation** when showing/hiding panel
   - Use Vue's `<Transition>` on bottom-panels div
   - Smooth height animation

2. **Remember user preference**
   - Store showLidar state in localStorage
   - Persist across sessions

3. **Keyboard shortcut**
   - Add hotkey (e.g., "L") to toggle LiDAR
   - Improve accessibility

4. **Full-screen API**
   - Add true browser full-screen mode
   - Use Fullscreen API for video wall

---

## Verification

### Test Steps:

1. Open http://localhost:3000/remote-control
2. Login with cityu / 2026
3. Observe LiDAR panel at bottom showing 3D scene
4. Click **"▼ LiDAR"** button in panel
5. ✅ Verify entire panel disappears
6. ✅ Verify video wall expands to full height
7. Click **"LIDAR"** button in header
8. ✅ Verify panel reappears with 3D scene
9. ✅ Verify scene animates correctly

---

## Completion Status

### ✅ All Requirements Met

1. ✅ LiDAR area can be completely removed
2. ✅ Controlled via LiDAR button
3. ✅ Main video window goes full screen
4. ✅ Clean integration with existing UI
5. ✅ No performance degradation

### 🎯 Production Ready

The LiDAR panel now provides true full-screen capability for the main video window. Users can toggle between focused video viewing and 3D obstacle avoidance visualization seamlessly.

---

**Guardian Mobility v0.0 - AI-Powered Remote Driving Platform**
*City University of Hong Kong*
*2026-01-19*
