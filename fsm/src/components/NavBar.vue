<!--
  Guardian Mobility v0.0 - Navigation Bar

  @description 顶部导航栏，包含用户信息和登出功能
-->
<template>
  <nav class="navbar">
    <div class="navbar-left">
      <router-link to="/remote-control" class="nav-link">
        🚗 Remote Control
      </router-link>
      <router-link to="/rosbag-replay-pro" class="nav-link">
        📹 RosBag Replay
      </router-link>
      <router-link to="/dispatch-dashboard" class="nav-link">
        📊 Dispatch
      </router-link>
      <router-link to="/intelligent-dispatch-demo" class="nav-link">
        🧠 AI Dispatch Demo
      </router-link>
      <router-link to="/database-visualization" class="nav-link">
        🗄️ Database
      </router-link>
    </div>

    <div class="navbar-right">
      <div class="user-info">
        <span class="user-icon">👤</span>
        <span class="user-name">{{ auth.userDisplayName }}</span>
        <span class="user-role">{{ auth.userRole }}</span>
      </div>

      <button class="logout-btn" @click="handleLogout">
        <span class="logout-icon">🚪</span>
        Logout
      </button>
    </div>
  </nav>
</template>

<script setup lang="ts">
import { useRouter } from 'vue-router'
import { getAuthService } from '@/services/authService'

const router = useRouter()
const auth = getAuthService()

function handleLogout() {
  if (confirm('Are you sure you want to logout?')) {
    auth.logout()
    router.push('/login')
  }
}
</script>

<style scoped>
.navbar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 24px;
  background: #14141f;
  border-bottom: 1px solid #2a2a3a;
  position: sticky;
  top: 0;
  z-index: 1000;
}

.navbar-left {
  display: flex;
  gap: 8px;
}

.nav-link {
  padding: 8px 16px;
  background: transparent;
  border: 1px solid transparent;
  border-radius: 6px;
  color: #888;
  text-decoration: none;
  font-size: 13px;
  transition: all 0.2s;
  display: flex;
  align-items: center;
  gap: 6px;
}

.nav-link:hover {
  background: rgba(255, 255, 255, 0.05);
  color: #fff;
  border-color: #3a3a4a;
}

.nav-link.router-link-active {
  background: rgba(255, 87, 34, 0.1);
  color: var(--primary, #ff5722);
  border-color: var(--primary, #ff5722);
}

.navbar-right {
  display: flex;
  align-items: center;
  gap: 16px;
}

.user-info {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 6px 12px;
  background: rgba(255, 255, 255, 0.02);
  border-radius: 6px;
  border: 1px solid #2a2a3a;
}

.user-icon {
  font-size: 16px;
}

.user-name {
  font-size: 13px;
  color: #fff;
  font-weight: 500;
}

.user-role {
  font-size: 10px;
  color: var(--primary, #ff5722);
  text-transform: uppercase;
  padding: 2px 6px;
  background: rgba(255, 87, 34, 0.1);
  border-radius: 4px;
}

.logout-btn {
  padding: 8px 16px;
  background: #2a2a3a;
  border: 1px solid #3a3a4a;
  border-radius: 6px;
  color: #aaa;
  font-size: 12px;
  cursor: pointer;
  transition: all 0.2s;
  display: flex;
  align-items: center;
  gap: 6px;
}

.logout-btn:hover {
  background: #ff4444;
  border-color: #ff4444;
  color: #fff;
}

.logout-icon {
  font-size: 14px;
}
</style>
