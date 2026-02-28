import logger from '@/utils/logger'
/**
 * FSM-Pilot V2.0 - Amap (高德地图) Service
 *
 * Integration with Amap for:
 * - Real-time vehicle location display
 * - Route visualization
 * - Traffic information
 * - Geocoding and reverse geocoding
 *
 * @author Li Yixiang
 * @institution City University of Hong Kong
 */

import { getAPIConfig, isAPIConfigured } from '@/config/apiConfig'

export interface AmapLocation {
  longitude: number
  latitude: number
  address?: string
  city?: string
  district?: string
}

export interface AmapMarker {
  id: string
  position: [number, number]  // [longitude, latitude]
  title: string
  icon?: string
  label?: string
  extData?: any
}

export interface AmapRoute {
  id: string
  path: Array<[number, number]>
  color: string
  width: number
}

/**
 * Amap Service
 */
class AmapService {
  private apiConfig: any
  private mapInstance: any = null
  private markers: Map<string, any> = new Map()
  private routes: Map<string, any> = new Map()
  private initialized = false

  constructor() {
    this.apiConfig = getAPIConfig()
    logger.info('[Amap Service] Constructor - API Config loaded:', this.apiConfig.amap, 'system')
  }

  /**
   * Check if Amap is available
   */
  isAvailable(): boolean {
    const configured = isAPIConfigured('amap')
    logger.info(`Amap isAvailable: configured=${configured}`, 'system')
    return configured
  }

  /**
   * Initialize Amap
   * Must be called before using map features
   */
  async initialize(): Promise<boolean> {
    logger.info('[Amap Service] Initialize called, initialized:', this.initialized, 'system')

    if (this.initialized) {
      return true
    }

    const available = this.isAvailable()
    logger.info('[Amap Service] isAvailable result:', available, 'system')

    if (!available) {
      logger.warn('[Amap] API not configured', 'system')
      logger.warn('[Amap] Config:', this.apiConfig.amap, 'system')
      return false
    }

    try {
      // Load Amap JavaScript API
      await this.loadAmapScript()
      this.initialized = true
      logger.info('[Amap] Initialized successfully', 'system')
      return true
    } catch (error) {
      logger.error('[Amap] Initialization failed:', error, 'system')
      return false
    }
  }

  /**
   * Load Amap JavaScript API
   */
  private loadAmapScript(): Promise<void> {
    return new Promise((resolve, reject) => {
      // Check if already loaded
      if (window.AMap) {
        logger.info('[Amap] Script already loaded', 'system')
        resolve()
        return
      }

      logger.info(`Amap loading script with apiKey=${this.apiConfig.amap.apiKey.slice(0,6)}...`, 'system')

      // Set security verification BEFORE loading script
      if (this.apiConfig.amap.securityJsCode) {
        window._AMapSecurityConfig = {
          securityJsCode: this.apiConfig.amap.securityJsCode
        }
        logger.info('[Amap] Security config set:', window._AMapSecurityConfig, 'system')
      }

      const scriptUrl = `https://webapi.amap.com/maps?v=2.0&key=${this.apiConfig.amap.apiKey}&plugin=AMap.Geocoder,AMap.Marker,AMap.Polyline`
      logger.info('[Amap] Script URL:', scriptUrl, 'system')

      const script = document.createElement('script')
      script.type = 'text/javascript'
      script.src = scriptUrl

      script.onload = () => {
        logger.info('[Amap] Script loaded successfully', 'system')
        logger.info('[Amap] window.AMap available:', !!window.AMap, 'system')
        resolve()
      }

      script.onerror = (error) => {
        logger.error('[Amap] Script load error:', error, 'system')
        logger.error('[Amap] Failed URL:', scriptUrl, 'system')
        reject(new Error('Failed to load Amap script'))
      }

      document.head.appendChild(script)
      logger.info('[Amap] Script tag appended to head', 'system')
    })
  }

  /**
   * Create map instance
   */
  createMap(container: string | HTMLElement, options?: any): any {
    if (!this.initialized || !window.AMap) {
      logger.error('[Amap] Not initialized', 'system')
      return null
    }

    const defaultOptions = {
      zoom: 12,
      center: [114.17, 22.32],  // Hong Kong default center
      viewMode: '3D',
      pitch: 40,
      skyColor: '#0a0a12',
      mapStyle: 'amap://styles/darkblue',  // Dark style
      ...options
    }

    this.mapInstance = new window.AMap.Map(container, defaultOptions)
    logger.info('[Amap] Map created', 'system')
    return this.mapInstance
  }

  /**
   * Add marker to map
   */
  addMarker(marker: AmapMarker): any {
    if (!this.mapInstance || !window.AMap) {
      logger.error('[Amap] Map not initialized', 'system')
      return null
    }

    // Remove existing marker with same id
    this.removeMarker(marker.id)

    const amapMarker = new window.AMap.Marker({
      position: new window.AMap.LngLat(marker.position[0], marker.position[1]),
      title: marker.title,
      extData: { id: marker.id, ...marker.extData }
    })

    // Set custom icon if provided
    if (marker.icon) {
      amapMarker.setIcon(marker.icon)
    }

    // Set label if provided
    if (marker.label) {
      amapMarker.setLabel({
        content: marker.label,
        direction: 'top'
      })
    }

    this.mapInstance.add(amapMarker)
    this.markers.set(marker.id, amapMarker)

    return amapMarker
  }

  /**
   * Remove marker from map
   */
  removeMarker(id: string): void {
    const marker = this.markers.get(id)
    if (marker && this.mapInstance) {
      this.mapInstance.remove(marker)
      this.markers.delete(id)
    }
  }

  /**
   * Update marker position
   */
  updateMarkerPosition(id: string, position: [number, number]): void {
    const marker = this.markers.get(id)
    if (marker) {
      marker.setPosition(new window.AMap.LngLat(position[0], position[1]))
    }
  }

  /**
   * Add route (polyline) to map
   */
  addRoute(route: AmapRoute): any {
    if (!this.mapInstance || !window.AMap) {
      logger.error('[Amap] Map not initialized', 'system')
      return null
    }

    // Remove existing route with same id
    this.removeRoute(route.id)

    const path = route.path.map(p => new window.AMap.LngLat(p[0], p[1]))

    const polyline = new window.AMap.Polyline({
      path: path,
      strokeColor: route.color,
      strokeWeight: route.width,
      strokeOpacity: 0.8,
      extData: { id: route.id }
    })

    this.mapInstance.add(polyline)
    this.routes.set(route.id, polyline)

    return polyline
  }

  /**
   * Remove route from map
   */
  removeRoute(id: string): void {
    const route = this.routes.get(id)
    if (route && this.mapInstance) {
      this.mapInstance.remove(route)
      this.routes.delete(id)
    }
  }

  /**
   * Set map center
   */
  setCenter(longitude: number, latitude: number, zoom?: number): void {
    if (this.mapInstance) {
      this.mapInstance.setCenter([longitude, latitude])
      if (zoom !== undefined) {
        this.mapInstance.setZoom(zoom)
      }
    }
  }

  /**
   * Fit map to show all markers
   */
  fitView(): void {
    if (this.mapInstance) {
      this.mapInstance.setFitView()
    }
  }

  /**
   * Geocoding: Address to coordinates
   */
  async geocode(address: string): Promise<AmapLocation | null> {
    if (!window.AMap) {
      logger.error('[Amap] Not initialized', 'system')
      return null
    }

    return new Promise((resolve) => {
      const geocoder = new window.AMap.Geocoder()

      geocoder.getLocation(address, (status: string, result: any) => {
        if (status === 'complete' && result.geocodes.length > 0) {
          const location = result.geocodes[0].location
          resolve({
            longitude: location.lng,
            latitude: location.lat,
            address: result.geocodes[0].formattedAddress,
            city: result.geocodes[0].city,
            district: result.geocodes[0].district
          })
        } else {
          logger.error('[Amap] Geocoding failed:', status, 'system')
          resolve(null)
        }
      })
    })
  }

  /**
   * Reverse Geocoding: Coordinates to address
   */
  async reverseGeocode(longitude: number, latitude: number): Promise<AmapLocation | null> {
    if (!window.AMap) {
      logger.error('[Amap] Not initialized', 'system')
      return null
    }

    return new Promise((resolve) => {
      const geocoder = new window.AMap.Geocoder()
      const lnglat = new window.AMap.LngLat(longitude, latitude)

      geocoder.getAddress(lnglat, (status: string, result: any) => {
        if (status === 'complete' && result.regeocode) {
          resolve({
            longitude,
            latitude,
            address: result.regeocode.formattedAddress,
            city: result.regeocode.addressComponent.city,
            district: result.regeocode.addressComponent.district
          })
        } else {
          logger.error('[Amap] Reverse geocoding failed:', status, 'system')
          resolve(null)
        }
      })
    })
  }

  /**
   * Clear all markers
   */
  clearMarkers(): void {
    for (const marker of this.markers.values()) {
      if (this.mapInstance) {
        this.mapInstance.remove(marker)
      }
    }
    this.markers.clear()
  }

  /**
   * Clear all routes
   */
  clearRoutes(): void {
    for (const route of this.routes.values()) {
      if (this.mapInstance) {
        this.mapInstance.remove(route)
      }
    }
    this.routes.clear()
  }

  /**
   * Clear map
   */
  clear(): void {
    this.clearMarkers()
    this.clearRoutes()
  }

  /**
   * Destroy map instance
   */
  destroy(): void {
    if (this.mapInstance) {
      this.clear()
      this.mapInstance.destroy()
      this.mapInstance = null
    }
  }

  /**
   * Get statistics
   */
  getStats() {
    return {
      initialized: this.initialized,
      available: this.isAvailable(),
      markers: this.markers.size,
      routes: this.routes.size
    }
  }
}

// Singleton instance
let amapService: AmapService | null = null

export function getAmapService(): AmapService {
  if (!amapService) {
    amapService = new AmapService()
  }
  return amapService
}

export { AmapService }

// Declare global AMap types
declare global {
  interface Window {
    AMap: any
    _AMapSecurityConfig: any
  }
}
