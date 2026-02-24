<!--
  Guardian Mobility v0.0 - Login Page

  @description 用户登录界面
-->
<template>
  <div class="login-page">
    <div class="login-background">
      <div class="bg-grid"></div>
      <div class="bg-gradient"></div>
    </div>

    <div class="login-container">
      <div class="login-card">
        <!-- Logo 和标题 -->
        <div class="login-header">
          <div class="logo">
            <span class="logo-icon">🚗</span>
            <span class="logo-text">Guardian Mobility</span>
          </div>
          <h1 class="title">Remote Driving Platform</h1>
          <p class="subtitle">Secure Operator Login</p>
        </div>

        <!-- 登录表单 -->
        <form class="login-form" @submit.prevent="handleLogin">
          <!-- 错误提示 -->
          <div v-if="errorMessage" class="error-alert">
            <span class="error-icon">⚠️</span>
            <span class="error-text">{{ errorMessage }}</span>
          </div>

          <!-- 成功提示 -->
          <div v-if="successMessage" class="success-alert">
            <span class="success-icon">✓</span>
            <span class="success-text">{{ successMessage }}</span>
          </div>

          <!-- 锁定提示 -->
          <div v-if="isLocked" class="warning-alert">
            <span class="warning-icon">🔒</span>
            <span class="warning-text">
              Account locked due to too many failed attempts.
              Please try again in {{ lockTimeRemaining }} minutes.
            </span>
          </div>

          <!-- 用户名输入 -->
          <div class="form-group">
            <label for="username" class="form-label">
              <span class="label-icon">👤</span>
              Username
            </label>
            <input
              id="username"
              v-model="credentials.username"
              type="text"
              class="form-input"
              placeholder="Enter your username"
              autocomplete="username"
              required
              :disabled="isLoading || isLocked"
            />
          </div>

          <!-- 密码输入 -->
          <div class="form-group">
            <label for="password" class="form-label">
              <span class="label-icon">🔑</span>
              Password
            </label>
            <div class="password-input-wrapper">
              <input
                id="password"
                v-model="credentials.password"
                :type="showPassword ? 'text' : 'password'"
                class="form-input"
                placeholder="Enter your password"
                autocomplete="current-password"
                required
                :disabled="isLoading || isLocked"
              />
              <button
                type="button"
                class="password-toggle"
                @click="showPassword = !showPassword"
                tabindex="-1"
              >
                {{ showPassword ? '👁️' : '🙈' }}
              </button>
            </div>
          </div>

          <!-- 记住我 -->
          <div class="form-group checkbox-group">
            <label class="checkbox-label">
              <input
                v-model="credentials.rememberMe"
                type="checkbox"
                class="checkbox-input"
                :disabled="isLoading || isLocked"
              />
              <span class="checkbox-text">Remember me for 30 days</span>
            </label>
          </div>

          <!-- 登录按钮 -->
          <button
            type="submit"
            class="login-button"
            :disabled="isLoading || isLocked || !isFormValid"
          >
            <span v-if="isLoading" class="loading-spinner">⟳</span>
            <span v-else>{{ isLocked ? 'Account Locked' : 'Sign In' }}</span>
          </button>
        </form>

        <!-- 快速登录提示 -->
        <div class="quick-login">
          <div class="divider">
            <span>Quick Access (Demo)</span>
          </div>
          <div class="demo-accounts">
            <button
              v-for="account in demoAccounts"
              :key="account.username"
              class="demo-btn"
              @click="fillCredentials(account)"
              :disabled="isLoading"
            >
              <span class="demo-role">{{ account.role }}</span>
              <span class="demo-username">{{ account.username }}</span>
            </button>
          </div>
        </div>

        <!-- 页脚信息 -->
        <div class="login-footer">
          <p class="version">Guardian Mobility v0.0</p>
          <p class="copyright">
            © 2025 City University of Hong Kong. All rights reserved.
          </p>
        </div>
      </div>

      <!-- 侧边信息 -->
      <div class="info-panel">
        <div class="info-content">
          <h2 class="info-title">Welcome Back</h2>
          <p class="info-text">
            Access the Guardian Mobility Remote Driving Platform to manage and monitor
            100+ autonomous vehicles in real-time.
          </p>

          <div class="info-features">
            <div class="feature-item">
              <span class="feature-icon">🚗</span>
              <span class="feature-text">100+ Vehicle Fleet</span>
            </div>
            <div class="feature-item">
              <span class="feature-icon">⚡</span>
              <span class="feature-text">Low Latency (&lt;120ms)</span>
            </div>
            <div class="feature-item">
              <span class="feature-icon">📹</span>
              <span class="feature-text">Multi-Camera Streaming</span>
            </div>
            <div class="feature-item">
              <span class="feature-icon">🔒</span>
              <span class="feature-text">Enterprise Security</span>
            </div>
          </div>

          <div class="info-stats">
            <div class="stat-item">
              <div class="stat-value">100+</div>
              <div class="stat-label">Vehicles</div>
            </div>
            <div class="stat-item">
              <div class="stat-value">70ms</div>
              <div class="stat-label">Avg Latency</div>
            </div>
            <div class="stat-item">
              <div class="stat-value">99.9%</div>
              <div class="stat-label">Uptime</div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue'
import { useRouter } from 'vue-router'
import { useAuthService } from '@/services/authService'

const router = useRouter()
const authService = useAuthService()

// 状态
const credentials = ref({
  username: '',
  password: '',
  rememberMe: false
})

const showPassword = ref(false)
const isLoading = ref(false)
const errorMessage = ref('')
const successMessage = ref('')
const isLocked = ref(false)
const lockTimeRemaining = ref(0)

// 演示账户
const demoAccounts = [
  { username: 'cityu', password: '2026', role: 'Admin' },
  { username: 'operator1', password: 'Operator@123', role: 'Operator' },
  { username: 'viewer', password: 'Viewer@2025', role: 'Viewer' }
]

// 计算属性
const isFormValid = computed(() => {
  return credentials.value.username.length > 0 &&
         credentials.value.password.length > 0
})

// 方法
async function handleLogin() {
  if (!isFormValid.value || isLoading.value || isLocked.value) return

  isLoading.value = true
  errorMessage.value = ''
  successMessage.value = ''

  try {
    const result = await authService.login(credentials.value)

    if (result.success) {
      successMessage.value = 'Login successful! Redirecting...'

      // 延迟跳转以显示成功消息
      setTimeout(() => {
        router.push('/remote-control')
      }, 1000)
    } else {
      errorMessage.value = result.message || 'Login failed'

      // 检查是否被锁定
      if (result.message?.includes('locked')) {
        isLocked.value = true
        // 从消息中提取剩余时间
        const match = result.message.match(/(\d+) minutes/)
        if (match) {
          lockTimeRemaining.value = parseInt(match[1])

          // 倒计时
          const countdown = setInterval(() => {
            lockTimeRemaining.value--
            if (lockTimeRemaining.value <= 0) {
              isLocked.value = false
              clearInterval(countdown)
              errorMessage.value = ''
            }
          }, 60000) // 每分钟更新一次
        }
      }
    }
  } catch (error) {
    console.error('[Login] Error:', error)
    errorMessage.value = 'An unexpected error occurred. Please try again.'
  } finally {
    isLoading.value = false
  }
}

function fillCredentials(account: typeof demoAccounts[0]) {
  credentials.value.username = account.username
  credentials.value.password = account.password
  errorMessage.value = ''
  successMessage.value = ''
}
</script>

<style scoped>
.login-page {
  min-height: 100vh;
  display: flex;
  align-items: center;
  justify-content: center;
  background: #0a0a12;
  position: relative;
  overflow: hidden;
}

/* 背景 */
.login-background {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  pointer-events: none;
}

.bg-grid {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-image:
    linear-gradient(rgba(255, 87, 34, 0.03) 1px, transparent 1px),
    linear-gradient(90deg, rgba(255, 87, 34, 0.03) 1px, transparent 1px);
  background-size: 50px 50px;
  animation: gridMove 20s linear infinite;
}

@keyframes gridMove {
  0% {
    transform: translate(0, 0);
  }
  100% {
    transform: translate(50px, 50px);
  }
}

.bg-gradient {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: radial-gradient(
    ellipse at top left,
    rgba(255, 87, 34, 0.1) 0%,
    transparent 50%
  ),
  radial-gradient(
    ellipse at bottom right,
    rgba(138, 43, 226, 0.1) 0%,
    transparent 50%
  );
}

/* 登录容器 */
.login-container {
  display: flex;
  gap: 60px;
  max-width: 1200px;
  width: 100%;
  padding: 40px;
  position: relative;
  z-index: 1;
}

/* 登录卡片 */
.login-card {
  flex: 1;
  max-width: 480px;
  background: rgba(20, 20, 31, 0.95);
  border: 1px solid #2a2a3a;
  border-radius: 16px;
  padding: 48px;
  backdrop-filter: blur(20px);
  box-shadow: 0 20px 60px rgba(0, 0, 0, 0.5);
}

/* 头部 */
.login-header {
  text-align: center;
  margin-bottom: 40px;
}

.logo {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 12px;
  margin-bottom: 16px;
}

.logo-icon {
  font-size: 40px;
}

.logo-text {
  font-size: 32px;
  font-weight: 700;
  background: linear-gradient(135deg, var(--primary, #ff5722), #00d4e0);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.title {
  font-size: 24px;
  font-weight: 600;
  color: #fff;
  margin: 0 0 8px 0;
}

.subtitle {
  font-size: 14px;
  color: #888;
  margin: 0;
}

/* 表单 */
.login-form {
  margin-bottom: 32px;
}

.form-group {
  margin-bottom: 24px;
}

.form-label {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 13px;
  font-weight: 600;
  color: #aaa;
  margin-bottom: 8px;
}

.label-icon {
  font-size: 16px;
}

.form-input {
  width: 100%;
  padding: 14px 16px;
  background: #1a1a2e;
  border: 1px solid #3a3a4a;
  border-radius: 8px;
  color: #fff;
  font-size: 14px;
  transition: all 0.2s;
}

.form-input:focus {
  outline: none;
  border-color: var(--primary, #ff5722);
  box-shadow: 0 0 0 3px rgba(255, 87, 34, 0.1);
}

.form-input:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.password-input-wrapper {
  position: relative;
}

.password-toggle {
  position: absolute;
  right: 12px;
  top: 50%;
  transform: translateY(-50%);
  background: none;
  border: none;
  font-size: 20px;
  cursor: pointer;
  padding: 4px;
  opacity: 0.6;
  transition: opacity 0.2s;
}

.password-toggle:hover {
  opacity: 1;
}

.checkbox-group {
  margin-bottom: 32px;
}

.checkbox-label {
  display: flex;
  align-items: center;
  gap: 8px;
  cursor: pointer;
  user-select: none;
}

.checkbox-input {
  width: 18px;
  height: 18px;
  accent-color: var(--primary, #ff5722);
  cursor: pointer;
}

.checkbox-text {
  font-size: 13px;
  color: #aaa;
}

/* 按钮 */
.login-button {
  width: 100%;
  padding: 16px;
  background: linear-gradient(135deg, var(--primary, #ff5722), #00d4e0);
  border: none;
  border-radius: 8px;
  color: #000;
  font-size: 15px;
  font-weight: 700;
  cursor: pointer;
  transition: all 0.3s;
  text-transform: uppercase;
  letter-spacing: 0.5px;
}

.login-button:hover:not(:disabled) {
  transform: translateY(-2px);
  box-shadow: 0 8px 24px rgba(255, 87, 34, 0.3);
}

.login-button:disabled {
  opacity: 0.5;
  cursor: not-allowed;
  transform: none;
}

.loading-spinner {
  display: inline-block;
  animation: spin 1s linear infinite;
}

@keyframes spin {
  from { transform: rotate(0deg); }
  to { transform: rotate(360deg); }
}

/* 提示框 */
.error-alert,
.success-alert,
.warning-alert {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 12px 16px;
  border-radius: 8px;
  margin-bottom: 24px;
  font-size: 13px;
}

.error-alert {
  background: rgba(255, 68, 68, 0.1);
  border: 1px solid rgba(255, 68, 68, 0.3);
  color: #ff8888;
}

.success-alert {
  background: rgba(68, 255, 68, 0.1);
  border: 1px solid rgba(68, 255, 68, 0.3);
  color: #88ff88;
}

.warning-alert {
  background: rgba(255, 193, 7, 0.1);
  border: 1px solid rgba(255, 193, 7, 0.3);
  color: #ffc107;
}

/* 快速登录 */
.quick-login {
  margin-bottom: 32px;
}

.divider {
  text-align: center;
  margin: 24px 0;
  position: relative;
}

.divider span {
  background: rgba(20, 20, 31, 0.95);
  padding: 0 16px;
  font-size: 11px;
  color: #666;
  text-transform: uppercase;
  position: relative;
  z-index: 1;
}

.divider::before {
  content: '';
  position: absolute;
  top: 50%;
  left: 0;
  right: 0;
  height: 1px;
  background: #2a2a3a;
}

.demo-accounts {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 8px;
}

.demo-btn {
  padding: 12px;
  background: #1a1a2e;
  border: 1px solid #2a2a3a;
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.2s;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 4px;
}

.demo-btn:hover:not(:disabled) {
  background: #2a2a3a;
  border-color: var(--primary, #ff5722);
}

.demo-btn:disabled {
  opacity: 0.3;
  cursor: not-allowed;
}

.demo-role {
  font-size: 10px;
  color: #666;
  text-transform: uppercase;
}

.demo-username {
  font-size: 12px;
  color: #aaa;
  font-family: 'JetBrains Mono', monospace;
}

/* 页脚 */
.login-footer {
  text-align: center;
  padding-top: 24px;
  border-top: 1px solid #2a2a3a;
}

.version {
  font-size: 11px;
  color: var(--primary, #ff5722);
  margin: 0 0 8px 0;
  font-weight: 600;
}

.copyright {
  font-size: 10px;
  color: #666;
  margin: 0;
}

/* 侧边信息面板 */
.info-panel {
  flex: 1;
  max-width: 480px;
  display: flex;
  align-items: center;
}

.info-content {
  width: 100%;
}

.info-title {
  font-size: 36px;
  font-weight: 700;
  color: #fff;
  margin: 0 0 16px 0;
}

.info-text {
  font-size: 16px;
  line-height: 1.6;
  color: #aaa;
  margin: 0 0 32px 0;
}

.info-features {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 16px;
  margin-bottom: 40px;
}

.feature-item {
  display: flex;
  align-items: center;
  gap: 12px;
  padding: 16px;
  background: rgba(255, 255, 255, 0.02);
  border: 1px solid #2a2a3a;
  border-radius: 8px;
}

.feature-icon {
  font-size: 24px;
}

.feature-text {
  font-size: 13px;
  color: #ddd;
}

.info-stats {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 24px;
}

.stat-item {
  text-align: center;
}

.stat-value {
  font-size: 32px;
  font-weight: 700;
  background: linear-gradient(135deg, var(--primary, #ff5722), #00d4e0);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
  margin-bottom: 8px;
}

.stat-label {
  font-size: 12px;
  color: #888;
  text-transform: uppercase;
}

/* 响应式 */
@media (max-width: 1024px) {
  .info-panel {
    display: none;
  }

  .login-container {
    justify-content: center;
  }
}

@media (max-width: 640px) {
  .login-card {
    padding: 32px 24px;
  }

  .demo-accounts {
    grid-template-columns: 1fr;
  }
}
</style>
