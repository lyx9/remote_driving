import logger from '@/utils/logger'
/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      AuthenticationService
 * @description 用户认证服务
 */

import { ref, computed, readonly } from 'vue'
import { USER_WHITELIST, SESSION_CONFIG, type User } from '@/config/userConfig'

// ======================== 类型定义 ========================

export interface LoginCredentials {
  username: string
  password: string
  rememberMe?: boolean
}

export interface AuthSession {
  user: User
  token: string
  expiresAt: number
  createdAt: number
}

export interface LoginAttempt {
  username: string
  attempts: number
  lastAttempt: number
  lockedUntil?: number
}

// ======================== 本地存储键 ========================

const STORAGE_KEYS = {
  SESSION: 'fsm_pilot_session',
  REMEMBER_ME: 'fsm_pilot_remember',
  LOGIN_ATTEMPTS: 'fsm_pilot_login_attempts'
}

// ======================== 认证服务 ========================

export function useAuthService() {
  // 状态
  const currentUser = ref<User | null>(null)
  const isAuthenticated = ref(false)
  const sessionToken = ref<string | null>(null)
  const sessionExpiresAt = ref<number | null>(null)

  // 登录尝试记录
  const loginAttempts = ref<Map<string, LoginAttempt>>(new Map())

  // 计算属性
  const userRole = computed(() => currentUser.value?.role || null)
  const userDisplayName = computed(() => currentUser.value?.displayName || '')
  const userPermissions = computed(() => currentUser.value?.permissions || [])

  const isAdmin = computed(() => userRole.value === 'admin')
  const isOperator = computed(() => userRole.value === 'operator')
  const isViewer = computed(() => userRole.value === 'viewer')

  const sessionTimeRemaining = computed(() => {
    if (!sessionExpiresAt.value) return 0
    return Math.max(0, sessionExpiresAt.value - Date.now())
  })

  // ======================== 初始化 ========================

  /**
   * 初始化认证服务
   */
  function initialize() {
    // 尝试从本地存储恢复会话
    restoreSession()

    // 加载登录尝试记录
    loadLoginAttempts()

    // 设置会话检查定时器
    setInterval(checkSessionValidity, 60 * 1000) // 每分钟检查一次
  }

  /**
   * 从本地存储恢复会话
   */
  function restoreSession(): boolean {
    try {
      const sessionData = localStorage.getItem(STORAGE_KEYS.SESSION)
      if (!sessionData) return false

      const session: AuthSession = JSON.parse(sessionData)

      // 检查会话是否过期
      if (Date.now() > session.expiresAt) {
        clearSession()
        return false
      }

      // 恢复会话
      currentUser.value = session.user
      sessionToken.value = session.token
      sessionExpiresAt.value = session.expiresAt
      isAuthenticated.value = true

      logger.info('[Auth] Session restored for user:', session.user.username, 'auth')
      return true
    } catch (error) {
      logger.error('[Auth] Failed to restore session:', error, 'auth')
      clearSession()
      return false
    }
  }

  /**
   * 加载登录尝试记录
   */
  function loadLoginAttempts() {
    try {
      const data = localStorage.getItem(STORAGE_KEYS.LOGIN_ATTEMPTS)
      if (data) {
        const attempts = JSON.parse(data)
        loginAttempts.value = new Map(Object.entries(attempts))
      }
    } catch (error) {
      logger.error('[Auth] Failed to load login attempts:', error, 'auth')
    }
  }

  /**
   * 保存登录尝试记录
   */
  function saveLoginAttempts() {
    try {
      const attempts = Object.fromEntries(loginAttempts.value)
      localStorage.setItem(STORAGE_KEYS.LOGIN_ATTEMPTS, JSON.stringify(attempts))
    } catch (error) {
      logger.error('[Auth] Failed to save login attempts:', error, 'auth')
    }
  }

  // ======================== 认证方法 ========================

  /**
   * 用户登录
   */
  async function login(credentials: LoginCredentials): Promise<{
    success: boolean
    message?: string
  }> {
    const { username, password, rememberMe } = credentials

    // 账户锁定功能已禁用 - 直接验证用户

    // 查找用户
    const user = USER_WHITELIST.find(
      u => u.username === username && u.enabled
    )

    // 验证密码
    if (!user || user.password !== password) {
      // 不再记录失败登录
      return {
        success: false,
        message: 'Invalid username or password'
      }
    }

    // 不再需要清除失败记录

    // 生成会话
    const session = createSession(user, rememberMe)

    // 保存会话
    saveSession(session)

    // 更新状态
    currentUser.value = user
    sessionToken.value = session.token
    sessionExpiresAt.value = session.expiresAt
    isAuthenticated.value = true

    // 更新最后登录时间
    user.lastLogin = new Date().toISOString()

    logger.info('[Auth] Login successful:', username, 'auth')

    return {
      success: true,
      message: 'Login successful'
    }
  }

  /**
   * 用户登出
   */
  function logout() {
    logger.info('[Auth] Logging out user:', currentUser.value?.username, 'auth')

    clearSession()

    currentUser.value = null
    sessionToken.value = null
    sessionExpiresAt.value = null
    isAuthenticated.value = false
  }

  /**
   * 创建会话
   */
  function createSession(user: User, rememberMe = false): AuthSession {
    const now = Date.now()
    const timeout = rememberMe
      ? SESSION_CONFIG.REMEMBER_ME_TIMEOUT
      : SESSION_CONFIG.SESSION_TIMEOUT

    return {
      user,
      token: generateToken(),
      expiresAt: now + timeout,
      createdAt: now
    }
  }

  /**
   * 生成会话 Token
   */
  function generateToken(): string {
    const timestamp = Date.now()
    const random = Math.random().toString(36).substring(2)
    return `fsm_${timestamp}_${random}`
  }

  /**
   * 保存会话到本地存储
   */
  function saveSession(session: AuthSession) {
    try {
      localStorage.setItem(STORAGE_KEYS.SESSION, JSON.stringify(session))
    } catch (error) {
      logger.error('[Auth] Failed to save session:', error, 'auth')
    }
  }

  /**
   * 清除会话
   */
  function clearSession() {
    localStorage.removeItem(STORAGE_KEYS.SESSION)
    localStorage.removeItem(STORAGE_KEYS.REMEMBER_ME)
  }

  /**
   * 检查会话有效性
   */
  function checkSessionValidity() {
    if (!isAuthenticated.value || !sessionExpiresAt.value) return

    if (Date.now() > sessionExpiresAt.value) {
      logger.info('[Auth] Session expired', 'auth')
      logout()
    }
  }

  /**
   * 刷新会话
   */
  function refreshSession(): boolean {
    if (!currentUser.value) return false

    const session = createSession(currentUser.value, true)
    saveSession(session)

    sessionToken.value = session.token
    sessionExpiresAt.value = session.expiresAt

    logger.info('[Auth] Session refreshed', 'auth')
    return true
  }

  // ======================== 登录尝试管理 ========================

  /**
   * 检查账户锁定状态
   */
  function checkAccountLockout(username: string): {
    locked: boolean
    remainingTime: number
  } {
    const attempt = loginAttempts.value.get(username)
    if (!attempt || !attempt.lockedUntil) {
      return { locked: false, remainingTime: 0 }
    }

    const now = Date.now()
    if (now < attempt.lockedUntil) {
      return {
        locked: true,
        remainingTime: attempt.lockedUntil - now
      }
    }

    // 锁定时间已过，清除锁定
    attempt.lockedUntil = undefined
    attempt.attempts = 0
    saveLoginAttempts()

    return { locked: false, remainingTime: 0 }
  }

  /**
   * 记录失败的登录尝试
   */
  function recordFailedLogin(username: string) {
    const now = Date.now()
    let attempt = loginAttempts.value.get(username)

    if (!attempt) {
      attempt = {
        username,
        attempts: 0,
        lastAttempt: now
      }
      loginAttempts.value.set(username, attempt)
    }

    attempt.attempts++
    attempt.lastAttempt = now

    // 检查是否需要锁定账户
    if (attempt.attempts >= SESSION_CONFIG.MAX_LOGIN_ATTEMPTS) {
      attempt.lockedUntil = now + SESSION_CONFIG.LOCKOUT_DURATION
      logger.warn(`[Auth] Account locked due to too many failed attempts: ${username}`, 'auth')
    }

    saveLoginAttempts()
  }

  /**
   * 清除失败登录记录
   */
  function clearFailedLogins(username: string) {
    loginAttempts.value.delete(username)
    saveLoginAttempts()
  }

  // ======================== 权限检查 ========================

  /**
   * 检查是否有指定权限
   */
  function hasPermission(permission: string): boolean {
    if (!currentUser.value) return false

    // Admin 拥有所有权限
    if (currentUser.value.permissions.includes('*')) return true

    return currentUser.value.permissions.includes(permission)
  }

  /**
   * 检查是否有任一权限
   */
  function hasAnyPermission(permissions: string[]): boolean {
    return permissions.some(p => hasPermission(p))
  }

  /**
   * 检查是否有所有权限
   */
  function hasAllPermissions(permissions: string[]): boolean {
    return permissions.every(p => hasPermission(p))
  }

  /**
   * 要求权限（如果没有则抛出错误）
   */
  function requirePermission(permission: string): void {
    if (!hasPermission(permission)) {
      throw new Error(`Permission denied: ${permission}`)
    }
  }

  // ======================== 用户管理 ========================

  /**
   * 获取所有用户（仅管理员）
   */
  function getAllUsers(): User[] {
    if (!isAdmin.value) {
      throw new Error('Permission denied: admin only')
    }
    return USER_WHITELIST
  }

  /**
   * 获取用户信息
   */
  function getUserInfo(username: string): User | undefined {
    return USER_WHITELIST.find(u => u.username === username)
  }

  // 初始化
  initialize()

  return {
    // 只读状态
    currentUser: readonly(currentUser),
    isAuthenticated: readonly(isAuthenticated),
    userRole: readonly(userRole),
    userDisplayName: readonly(userDisplayName),
    userPermissions: readonly(userPermissions),
    isAdmin: readonly(isAdmin),
    isOperator: readonly(isOperator),
    isViewer: readonly(isViewer),
    sessionTimeRemaining: readonly(sessionTimeRemaining),

    // 认证方法
    login,
    logout,
    refreshSession,

    // 权限检查
    hasPermission,
    hasAnyPermission,
    hasAllPermissions,
    requirePermission,

    // 用户管理
    getAllUsers,
    getUserInfo
  }
}

// 创建全局单例
let authServiceInstance: ReturnType<typeof useAuthService> | null = null

export function getAuthService() {
  if (!authServiceInstance) {
    authServiceInstance = useAuthService()
  }
  return authServiceInstance
}

export default useAuthService
