/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 */

import { createApp } from 'vue'
import { createPinia } from 'pinia'
import App from './App.vue'
import router from './router'
import './style.css'

// 测试Amap配置
import { getAPIConfig, isAPIConfigured } from './config/apiConfig'
console.log('=== Guardian Mobility - Amap Configuration ===')
console.log('VITE_AMAP_API_KEY:', import.meta.env.VITE_AMAP_API_KEY)
console.log('VITE_AMAP_JS_CODE:', import.meta.env.VITE_AMAP_JS_CODE)
const config = getAPIConfig()
console.log('Amap Config:', config.amap)
console.log('Is Amap Configured:', isAPIConfigured('amap'))
console.log('==============================================')

const app = createApp(App)
const pinia = createPinia()

app.use(pinia)
app.use(router)
app.mount('#app')
