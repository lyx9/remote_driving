/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      Router
 * @description Vue Router 配置和路由守卫
 */

import { createRouter, createWebHistory, type RouteRecordRaw } from 'vue-router'
import { getAuthService } from '@/services/authService'

// ======================== 路由定义 ========================

const routes: RouteRecordRaw[] = [
  {
    path: '/login',
    name: 'Login',
    component: () => import('@/components/LoginPage.vue'),
    meta: {
      requiresAuth: false,
      title: 'Login - FSM-Pilot'
    }
  },
  {
    path: '/',
    redirect: '/remote-control'
  },
  {
    path: '/remote-control',
    name: 'RemoteControl',
    component: () => import('@/components/RemoteControl.vue'),
    meta: {
      requiresAuth: true,
      permissions: ['vehicle:control', 'vehicle:view'],
      title: 'Remote Control - FSM-Pilot'
    }
  },
  {
    path: '/mqtt-remote-control',
    name: 'MQTTRemoteControl',
    component: () => import('@/components/MQTTRemoteControl.vue'),
    meta: {
      requiresAuth: true,
      permissions: ['vehicle:control'],
      title: 'MQTT Remote Control - FSM-Pilot'
    }
  },
  {
    path: '/rosbag-replay-pro',
    name: 'RosBagReplayPro',
    component: () => import('@/components/RosBagReplayPro.vue'),
    meta: {
      requiresAuth: true,
      permissions: ['rosbag:replay'],
      title: 'RosBag Replay - FSM-Pilot'
    }
  },
  {
    path: '/dispatch-dashboard',
    name: 'DispatchDashboard',
    component: () => import('@/components/DispatchDashboard.vue'),
    meta: {
      requiresAuth: true,
      permissions: ['dispatch:view'],
      title: 'Dispatch Dashboard - FSM-Pilot'
    }
  },
  {
    path: '/intelligent-dispatch-demo',
    name: 'IntelligentDispatchDemo',
    component: () => import('@/components/IntelligentDispatchDemo.vue'),
    meta: {
      requiresAuth: true,
      permissions: ['dispatch:view'],
      title: 'Intelligent Dispatch Demo - FSM-Pilot'
    }
  },
  {
    path: '/database-visualization',
    name: 'DatabaseVisualization',
    component: () => import('@/components/DatabaseVisualization.vue'),
    meta: {
      requiresAuth: true,
      permissions: ['dispatch:view'],
      title: 'Database Visualization - FSM-Pilot'
    }
  },
  {
    path: '/:pathMatch(.*)*',
    name: 'NotFound',
    redirect: '/login'
  }
]

// ======================== 创建路由实例 ========================

const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes
})

// ======================== 全局前置守卫 ========================

router.beforeEach((to, from, next) => {
  const authService = getAuthService()

  // 设置页面标题
  if (to.meta.title) {
    document.title = to.meta.title as string
  }

  // 检查路由是否需要认证
  const requiresAuth = to.meta.requiresAuth !== false

  if (!requiresAuth) {
    // 不需要认证的页面
    if (to.name === 'Login' && authService.isAuthenticated.value) {
      // 已登录用户访问登录页，重定向到主页
      next('/remote-control')
    } else {
      next()
    }
    return
  }

  // 需要认证的页面
  if (!authService.isAuthenticated.value) {
    // 未登录，重定向到登录页
    console.log('[Router] Authentication required, redirecting to login')
    next({
      name: 'Login',
      query: { redirect: to.fullPath }
    })
    return
  }

  // 检查权限
  const requiredPermissions = to.meta.permissions as string[] | undefined
  if (requiredPermissions && requiredPermissions.length > 0) {
    const hasPermission = authService.hasAnyPermission(requiredPermissions)

    if (!hasPermission) {
      console.warn('[Router] Permission denied:', requiredPermissions)
      // 权限不足，重定向到首页或显示错误
      next('/remote-control')
      return
    }
  }

  // 通过所有检查，允许访问
  next()
})

// ======================== 全局后置钩子 ========================

router.afterEach((to, from) => {
  console.log(`[Router] Navigated from ${from.path} to ${to.path}`)
})

export default router
