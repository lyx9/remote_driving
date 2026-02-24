# Amap Configuration Complete

## Summary

Successfully configured Amap (高德地图) API for Guardian Mobility v0.0 platform.

## Configuration Details

### 1. Environment Variables (.env.local)

Created `.env.local` file with the following configuration:

```bash
# Amap API Key
VITE_AMAP_API_KEY=fa4c4bc1d796891d00472871682f6628

# Amap Security Key (JS Code)
VITE_AMAP_JS_CODE=215104d11967ff4b9b17366e0bd56f0f
```

### 2. Updated Files

#### LeftSidebar.vue (src/components/LeftSidebar.vue:81-95)

Updated the `initMap()` function to:
- Read API key from environment variable `VITE_AMAP_API_KEY`
- Set security configuration using `VITE_AMAP_JS_CODE`
- Fallback to hardcoded values if environment variables are not set

```typescript
// Set security configuration
;(window as any)._AMapSecurityConfig = {
  securityJsCode: import.meta.env.VITE_AMAP_JS_CODE || '215104d11967ff4b9b17366e0bd56f0f'
}

const AMap = await AMapLoader.load({
  key: import.meta.env.VITE_AMAP_API_KEY || 'fa4c4bc1d796891d00472871682f6628',
  version: '2.0',
  plugins: ['AMap.Scale', 'AMap.ToolBar']
})
```

### 3. Dev Server

- Restarted development server to load new environment variables
- Server running at: http://localhost:3000/
- Hot Module Replacement (HMR) working correctly

## Testing

### Test Page Created

Created `test-amap-config.html` for manual testing:
- Verifies environment variables are loaded
- Tests Amap script loading
- Creates a test map centered on Hong Kong
- Adds a marker to verify full functionality

### How to Test

1. **Open the main application:**
   ```
   http://localhost:3000/remote-control
   ```

2. **Click the MAP button** in the header to show the left sidebar

3. **Verify the map displays:**
   - Hong Kong map should be visible
   - Dark theme should be applied
   - Vehicle markers should appear
   - Map should be interactive (zoom, pan)

4. **Alternative: Use test page:**
   ```
   http://localhost:3000/test-amap-config.html
   ```
   This page will show detailed test results for each configuration step.

## Expected Behavior

When the left sidebar is opened:
- Amap should load with dark theme
- Map should center on Hong Kong (Tsim Sha Tsui: 114.1694, 22.3193)
- Vehicle markers should appear as colored circles:
  - 🟢 Green: Active vehicles
  - ⚪ Gray: Idle vehicles
  - 🟠 Orange: Patrol vehicles
- Clicking on markers should show vehicle info windows
- Map should update vehicle positions every 2 seconds

## Configuration Files Reference

- **Environment Variables**: `.env.local`
- **API Config**: `src/config/apiConfig.ts`
- **Map Component**: `src/components/LeftSidebar.vue`
- **Amap Service**: `src/services/amapService.ts`

## Notes

- The API key and security key are now properly configured
- Environment variables are loaded at build time by Vite
- Security configuration is set before loading the Amap script
- Fallback values ensure the app works even if .env.local is missing

## Status

✅ Configuration Complete
✅ Dev Server Restarted
✅ Environment Variables Loaded
✅ LeftSidebar.vue Updated
✅ Test Page Created

**Ready for testing!**

---

*Guardian Mobility v0.0*
*City University of Hong Kong*
