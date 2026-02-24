# 用户认证系统文档

## 概述

FSM-Pilot V2.0 新增了完整的用户认证系统，支持基于白名单的用户登录、权限管理和会话控制。

## 功能特性

### ✅ 已实现功能

1. **用户白名单管理**
   - 可配置的用户列表
   - 角色分类（Admin / Operator / Viewer）
   - 权限细粒度控制

2. **登录安全**
   - 密码验证
   - 失败登录限制（5次）
   - 账户自动锁定（15分钟）
   - 记住我功能（30天）

3. **会话管理**
   - 自动会话超时（8小时）
   - 会话刷新机制
   - 本地存储持久化

4. **路由守卫**
   - 自动登录检查
   - 权限验证
   - 未授权重定向

5. **用户界面**
   - 现代化登录页面
   - 顶部导航栏
   - 用户信息显示
   - 一键登出

## 默认用户账户

### 管理员账户
```
用户名: admin
密码: FSM@Admin2025
角色: Admin
权限: 所有权限
```

### 运营商账户 1
```
用户名: operator1
密码: Operator@123
角色: Operator
权限: 车辆控制、视频查看、紧急操作
```

### 运营商账户 2
```
用户名: operator2
密码: Operator@456
角色: Operator
权限: 车辆控制、视频查看、紧急操作
```

### 查看者账户
```
用户名: viewer
密码: Viewer@2025
角色: Viewer
权限: 车辆查看、视频查看、RosBag回放
```

### 演示账户
```
用户名: demo
密码: Demo@123
角色: Viewer
权限: 车辆查看、RosBag回放
```

## 角色权限说明

### Admin（管理员）
- ✅ 所有系统权限
- ✅ 用户管理
- ✅ 系统配置
- ✅ 调度管理
- ✅ 车辆控制
- ✅ 视频录制/查看
- ✅ RosBag操作
- ✅ 紧急操作

### Operator（运营商）
- ✅ 车辆控制
- ✅ 车辆查看
- ✅ 视频查看
- ✅ 视频录制
- ✅ 紧急制动
- ✅ RosBag回放
- ✅ 调度查看
- ❌ 用户管理
- ❌ 系统配置

### Viewer（查看者）
- ✅ 车辆查看
- ✅ 视频查看
- ✅ RosBag回放
- ✅ 调度查看
- ❌ 车辆控制
- ❌ 视频录制
- ❌ 紧急操作
- ❌ 用户管理

## 使用指南

### 首次登录

1. 启动系统
   ```bash
   npm run dev
   ```

2. 访问登录页面
   ```
   http://localhost:5173
   ```
   系统会自动重定向到登录页面

3. 选择账户登录
   - 方式1：手动输入用户名和密码
   - 方式2：点击"Quick Access"区域的演示账户按钮快速填充

4. 可选：勾选"Remember me"以保持30天登录状态

5. 点击"Sign In"登录

### 页面访问

登录后可访问以下页面（根据权限）：

- **Remote Control** (`/remote-control`) - 远程控制界面
  - 需要权限：`vehicle:control` 或 `vehicle:view`
  - 角色：Admin, Operator, Viewer

- **RosBag Replay** (`/rosbag-replay-pro`) - RosBag回放
  - 需要权限：`rosbag:replay`
  - 角色：Admin, Operator, Viewer

- **Dispatch Dashboard** (`/dispatch-dashboard`) - 调度管理
  - 需要权限：`dispatch:view`
  - 角色：Admin, Operator, Viewer

### 退出登录

点击右上角导航栏的"Logout"按钮即可退出。

## 安全配置

### 会话超时设置

在 `src/config/userConfig.ts` 中配置：

```typescript
export const SESSION_CONFIG = {
  SESSION_TIMEOUT: 8 * 60 * 60 * 1000,        // 8小时
  REMEMBER_ME_TIMEOUT: 30 * 24 * 60 * 60 * 1000, // 30天
  MAX_LOGIN_ATTEMPTS: 5,                      // 最大失败次数
  LOCKOUT_DURATION: 15 * 60 * 1000,          // 锁定15分钟
  TOKEN_REFRESH_INTERVAL: 30 * 60 * 1000     // 30分钟刷新
}
```

### 添加新用户

在 `src/config/userConfig.ts` 的 `USER_WHITELIST` 数组中添加：

```typescript
{
  username: 'newuser',
  password: 'SecurePassword@123',
  role: 'operator',  // 'admin' | 'operator' | 'viewer'
  displayName: 'New User',
  email: 'newuser@fsm-pilot.com',
  permissions: [
    'vehicle:control',
    'vehicle:view',
    'video:view',
    'emergency:trigger'
  ],
  enabled: true,
  createdAt: '2025-01-13T00:00:00Z'
}
```

### 修改用户权限

编辑对应用户的 `permissions` 数组：

```typescript
permissions: [
  'vehicle:control',    // 车辆控制
  'vehicle:view',       // 车辆查看
  'video:view',         // 视频查看
  'video:record',       // 视频录制
  'emergency:trigger',  // 紧急操作
  'rosbag:replay',      // RosBag回放
  'rosbag:record',      // RosBag录制
  'rosbag:export',      // RosBag导出
  'dispatch:view',      // 调度查看
  'dispatch:manage',    // 调度管理
  'user:manage',        // 用户管理
  'system:config'       // 系统配置
]
```

## 技术实现

### 核心文件

1. **用户配置** - `src/config/userConfig.ts`
   - 用户白名单定义
   - 权限常量
   - 会话配置

2. **认证服务** - `src/services/authService.ts`
   - 登录/登出逻辑
   - 会话管理
   - 权限检查
   - 失败登录追踪

3. **登录页面** - `src/components/LoginPage.vue`
   - 登录表单UI
   - 错误提示
   - 快速登录

4. **导航栏** - `src/components/NavBar.vue`
   - 页面导航
   - 用户信息显示
   - 登出按钮

5. **路由配置** - `src/router/index.ts`
   - 路由定义
   - 认证守卫
   - 权限检查

### 认证流程

```
1. 用户访问系统
   ↓
2. 路由守卫检查认证状态
   ↓
3a. 未登录 → 重定向到登录页
3b. 已登录 → 检查权限
   ↓
4a. 无权限 → 重定向到首页
4b. 有权限 → 允许访问
```

### 权限检查示例

在组件中使用认证服务：

```typescript
import { getAuthService } from '@/services/authService'

const auth = getAuthService()

// 检查单个权限
if (auth.hasPermission('vehicle:control')) {
  // 允许控制车辆
}

// 检查任一权限
if (auth.hasAnyPermission(['vehicle:control', 'vehicle:view'])) {
  // 允许查看或控制车辆
}

// 检查所有权限
if (auth.hasAllPermissions(['video:view', 'video:record'])) {
  // 允许查看和录制视频
}

// 要求权限（无权限会抛出异常）
try {
  auth.requirePermission('emergency:trigger')
  // 执行紧急操作
} catch (error) {
  console.error('权限不足')
}
```

### 会话存储

会话数据存储在 `localStorage` 中：

```typescript
// 存储键
{
  'fsm_pilot_session': {     // 当前会话
    user: User,
    token: string,
    expiresAt: number,
    createdAt: number
  },
  'fsm_pilot_login_attempts': {  // 登录尝试记录
    [username]: {
      attempts: number,
      lastAttempt: number,
      lockedUntil?: number
    }
  }
}
```

## 安全建议（生产环境）

### ⚠️ 重要提示

当前实现适用于开发和演示环境。**生产环境部署前必须实施以下安全措施**：

1. **密码加密**
   ```typescript
   // 使用 bcrypt 或类似库
   import bcrypt from 'bcrypt'

   // 存储哈希而非明文
   const hashedPassword = await bcrypt.hash(password, 10)

   // 验证密码
   const isValid = await bcrypt.compare(password, user.passwordHash)
   ```

2. **环境变量管理**
   ```bash
   # .env
   VITE_SESSION_SECRET=your-secret-key
   VITE_SESSION_TIMEOUT=28800000
   ```

3. **后端认证集成**
   - 将认证逻辑移至后端API
   - 使用JWT tokens
   - 实现token刷新机制

4. **HTTPS通信**
   - 强制使用HTTPS
   - 设置Secure Cookie flags

5. **审计日志**
   - 记录所有登录尝试
   - 记录权限变更
   - 记录敏感操作

6. **双因素认证（2FA）**
   - 添加TOTP支持
   - 短信验证码

7. **密码策略**
   - 最小长度要求
   - 复杂度要求
   - 定期更换

8. **会话安全**
   - 使用HttpOnly cookies
   - CSRF保护
   - XSS防护

## 故障排除

### 问题：无法登录

**症状**：输入正确的用户名和密码后显示"Invalid username or password"

**解决方案**：
1. 确认用户名和密码拼写正确（区分大小写）
2. 检查用户的 `enabled` 字段是否为 `true`
3. 清除浏览器localStorage：`localStorage.clear()`
4. 刷新页面重试

### 问题：账户被锁定

**症状**：显示"Account locked. Try again in X minutes"

**解决方案**：
1. 等待锁定时间过期（默认15分钟）
2. 或手动清除：
   ```javascript
   localStorage.removeItem('fsm_pilot_login_attempts')
   ```

### 问题：会话过期

**症状**：自动跳转到登录页面

**解决方案**：
1. 重新登录
2. 勾选"Remember me"以延长会话时间
3. 调整 `SESSION_TIMEOUT` 配置

### 问题：权限不足

**症状**：无法访问某些页面或功能

**解决方案**：
1. 确认用户角色和权限配置
2. 使用Admin账户登录进行管理操作
3. 在 `userConfig.ts` 中为用户添加所需权限

## 更新日志

### V2.0.0 (2025-01-13)
- ✅ 新增用户认证系统
- ✅ 实现基于白名单的用户管理
- ✅ 添加路由守卫和权限检查
- ✅ 创建登录页面UI
- ✅ 实现会话管理
- ✅ 添加登录失败限制
- ✅ 集成导航栏和登出功能

---

**FSM-Pilot V2.0** - Enterprise Remote Driving Platform with Secure Authentication
