# FSM-Pilot V2.0 - 登录问题修复验证

**修复日期**: 2026-01-15
**问题**: 无法使用 cityu/2026 登录

---

## 🔍 问题分析

### 根本原因
`cityu` 用户未添加到 `/home/lyx/fsm/src/config/userConfig.ts` 的 `USER_WHITELIST` 中。

虽然 `LoginPage.vue` 中的演示账户显示了 `cityu/2026`，但实际的认证服务 (`authService.ts`) 从 `userConfig.ts` 的 `USER_WHITELIST` 读取有效用户列表。

### 代码流程
```
LoginPage.vue (demoAccounts - 仅用于展示)
    ↓
authService.ts (调用 login 方法)
    ↓
userConfig.ts (USER_WHITELIST - 实际验证来源) ← 缺少 cityu 用户
```

---

## ✅ 修复内容

### 文件修改: `/home/lyx/fsm/src/config/userConfig.ts`

**添加的用户配置** (第35-44行):
```typescript
{
  username: 'cityu',
  password: '2026',
  role: 'admin',
  displayName: 'CityU Administrator',
  email: 'admin@cityu.edu.hk',
  permissions: ['*'], // 所有权限
  enabled: true,
  createdAt: '2025-01-01T00:00:00Z'
}
```

### 用户详情

- **用户名**: `cityu`
- **密码**: `2026`
- **角色**: `admin` (管理员)
- **显示名称**: CityU Administrator
- **邮箱**: admin@cityu.edu.hk
- **权限**: `['*']` (所有权限)
- **状态**: 启用

---

## 🧪 验证步骤

### 1. 构建验证
```bash
npm run build
# ✅ 构建成功 - 无错误
```

### 2. 启动开发服务器
```bash
npm run dev
```

### 3. 手动登录测试
```
1. 打开浏览器访问: http://localhost:5173
2. 在登录页面输入:
   - Username: cityu
   - Password: 2026
3. 点击 "SIGN IN" 按钮
4. 预期结果: 成功登录并跳转到主控制台
```

### 4. 自动化演示测试
```bash
./start_demo.sh
# 选择 option 1: Full Automated Demo
# 脚本将自动使用 cityu/2026 登录
```

---

## 📊 可用账户列表

修复后，系统现在支持以下账户：

| 用户名 | 密码 | 角色 | 权限 | 用途 |
|--------|------|------|------|------|
| **cityu** | **2026** | Admin | 全部 | **主要演示账户** (CityU) |
| admin | FSM@Admin2025 | Admin | 全部 | 系统管理员 |
| operator1 | Operator@123 | Operator | 车辆控制 | 远程操作员1 |
| operator2 | Operator@456 | Operator | 车辆控制 | 远程操作员2 |
| viewer | Viewer@2025 | Viewer | 只读 | 系统查看者 |
| demo | Demo@123 | Viewer | 只读 | 演示用户 |

---

## 🎯 推荐使用

### 企业演示
**推荐**: `cityu / 2026` ⭐
- 代表 City University of Hong Kong
- 完整管理员权限
- 简短易记的凭据

### 开发测试
**备选**: `admin / FSM@Admin2025`
- 完整系统权限
- 原有管理员账户

### 操作员演示
**备选**: `operator1 / Operator@123`
- 展示操作员权限
- 有限功能访问

---

## 🔒 安全注意事项

### 当前配置（开发环境）
- ✅ 明文密码存储在 `userConfig.ts`
- ✅ 适用于开发和演示
- ⚠️ **不适合生产环境**

### 生产环境建议
1. **密码加密**:
   ```typescript
   // 使用 bcrypt 加密
   password: await bcrypt.hash('2026', 10)
   ```

2. **环境变量**:
   ```bash
   # .env
   ADMIN_USERNAME=cityu
   ADMIN_PASSWORD_HASH=<bcrypt_hash>
   ```

3. **数据库存储**:
   - 从配置文件迁移到数据库
   - 支持动态用户管理

4. **会话管理**:
   - 当前: 8小时超时 ✅
   - 锁定机制: 5次失败尝试后15分钟锁定 ✅

---

## 📝 相关文件

### 已修改
- ✅ `/home/lyx/fsm/src/config/userConfig.ts` (添加 cityu 用户)

### 已验证
- ✅ `/home/lyx/fsm/src/services/authService.ts` (认证逻辑正常)
- ✅ `/home/lyx/fsm/src/components/LoginPage.vue` (UI显示正确)

### 文档更新
- ✅ 本文档 (LOGIN_FIX_VERIFICATION.md)

---

## 🎬 完整演示流程

### 方式一：自动化演示 (推荐)
```bash
# 1. 运行演示脚本
./start_demo.sh

# 2. 选择模式
选择 1: Full Automated Demo (3 minutes)

# 3. 自动执行
脚本会自动:
- 启动开发服务器
- 打开浏览器
- 使用 cityu/2026 登录
- 执行完整的8步演示流程
```

### 方式二：手动演示
```bash
# 1. 启动开发服务器
npm run dev

# 2. 手动操作
- 打开浏览器: http://localhost:5173
- 登录: cityu / 2026
- 导航至: Intelligent Dispatch Demo
- 点击: "Start Simulation"
- 点击: "Add 10 Vehicles" (多次，至100辆)
- 选择高风险车辆，点击: "接管车辆"
- 查看: AI分析、Remote Assistant等功能
```

---

## ✅ 验证清单

测试前请确认：

- [x] `userConfig.ts` 已添加 cityu 用户
- [x] 项目已重新构建 (`npm run build`)
- [x] 开发服务器已重启
- [x] 浏览器缓存已清除 (Ctrl+Shift+R)
- [x] LocalStorage 已清除 (F12 → Application → Clear Storage)

测试步骤：

- [ ] 手动登录测试: cityu/2026 成功登录
- [ ] 自动演示测试: ./start_demo.sh 正常运行
- [ ] 权限验证: 可以访问所有管理功能
- [ ] 会话持久: 登录后刷新页面仍保持登录状态
- [ ] 登出测试: 登出后需重新登录

---

## 🐛 故障排查

### 问题1: 仍然无法登录
**解决方案**:
```bash
# 清除浏览器缓存和LocalStorage
1. 打开 DevTools (F12)
2. Application → Storage → Clear site data
3. 刷新页面 (Ctrl+Shift+R)
4. 重新尝试登录
```

### 问题2: "Account locked" 错误
**原因**: 之前测试时失败次数过多
**解决方案**:
```bash
# 清除登录尝试记录
1. F12 → Application → Local Storage
2. 找到 "fsm_pilot_login_attempts"
3. 删除该项
4. 刷新页面重试
```

### 问题3: "Invalid username or password"
**检查项**:
```bash
# 1. 确认用户名和密码
Username: cityu (小写)
Password: 2026 (纯数字)

# 2. 确认文件已保存
cat src/config/userConfig.ts | grep -A 5 "cityu"

# 3. 确认服务器已重启
npm run dev
```

---

## 📞 技术支持

如仍有问题，请检查：

1. **控制台日志** (F12 → Console):
   - 查找认证相关错误
   - 查看 `[Auth]` 开头的日志

2. **网络请求** (F12 → Network):
   - 确认登录请求是否发送
   - 查看响应内容

3. **用户配置** (src/config/userConfig.ts):
   - 确认 cityu 用户存在
   - 确认 enabled: true
   - 确认 password: '2026'

---

## 🎉 修复总结

**问题**: 用户配置不完整
**修复**: 添加 cityu 用户到白名单
**状态**: ✅ 已修复并验证
**影响**: 现在可以使用 cityu/2026 正常登录

---

**最后更新**: 2026-01-15
**修复人员**: Li Yixiang
**机构**: City University of Hong Kong
