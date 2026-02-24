/**
 * FSM-Pilot V2.0 - Remote Driving System
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      UserConfig
 * @description 用户白名单配置
 */

export interface User {
  username: string
  password: string
  role: 'admin' | 'operator' | 'viewer'
  displayName: string
  email?: string
  permissions: string[]
  enabled: boolean
  createdAt: string
  lastLogin?: string
}

/**
 * 用户白名单配置
 *
 * 注意：生产环境中应该：
 * 1. 将密码改为加密哈希存储（如 bcrypt）
 * 2. 使用环境变量或安全配置管理
 * 3. 实现密码策略（复杂度、过期等）
 */
export const USER_WHITELIST: User[] = [
  {
    username: 'cityu',
    password: '2026',
    role: 'admin',
    displayName: 'CityU Administrator',
    email: 'admin@cityu.edu.hk',
    permissions: ['*'], // 所有权限
    enabled: true,
    createdAt: '2025-01-01T00:00:00Z'
  },
  {
    username: 'admin',
    password: 'FSM@Admin2025',  // 生产环境应使用哈希
    role: 'admin',
    displayName: 'System Administrator',
    email: 'admin@fsm-pilot.com',
    permissions: ['*'], // 所有权限
    enabled: true,
    createdAt: '2025-01-01T00:00:00Z'
  },
  {
    username: 'operator1',
    password: 'Operator@123',
    role: 'operator',
    displayName: 'Remote Operator 1',
    email: 'operator1@fsm-pilot.com',
    permissions: [
      'vehicle:control',
      'vehicle:view',
      'video:view',
      'emergency:trigger'
    ],
    enabled: true,
    createdAt: '2025-01-01T00:00:00Z'
  },
  {
    username: 'operator2',
    password: 'Operator@456',
    role: 'operator',
    displayName: 'Remote Operator 2',
    email: 'operator2@fsm-pilot.com',
    permissions: [
      'vehicle:control',
      'vehicle:view',
      'video:view',
      'emergency:trigger'
    ],
    enabled: true,
    createdAt: '2025-01-01T00:00:00Z'
  },
  {
    username: 'viewer',
    password: 'Viewer@2025',
    role: 'viewer',
    displayName: 'System Viewer',
    email: 'viewer@fsm-pilot.com',
    permissions: [
      'vehicle:view',
      'video:view',
      'rosbag:replay'
    ],
    enabled: true,
    createdAt: '2025-01-01T00:00:00Z'
  },
  {
    username: 'demo',
    password: 'Demo@123',
    role: 'viewer',
    displayName: 'Demo User',
    permissions: [
      'vehicle:view',
      'rosbag:replay'
    ],
    enabled: true,
    createdAt: '2025-01-01T00:00:00Z'
  }
]

/**
 * 权限定义
 */
export const PERMISSIONS = {
  // 车辆控制权限
  VEHICLE_CONTROL: 'vehicle:control',
  VEHICLE_VIEW: 'vehicle:view',

  // 视频权限
  VIDEO_VIEW: 'video:view',
  VIDEO_RECORD: 'video:record',

  // 紧急操作
  EMERGENCY_TRIGGER: 'emergency:trigger',

  // RosBag 操作
  ROSBAG_REPLAY: 'rosbag:replay',
  ROSBAG_RECORD: 'rosbag:record',
  ROSBAG_EXPORT: 'rosbag:export',

  // 调度管理
  DISPATCH_VIEW: 'dispatch:view',
  DISPATCH_MANAGE: 'dispatch:manage',

  // 系统管理
  USER_MANAGE: 'user:manage',
  SYSTEM_CONFIG: 'system:config'
} as const

/**
 * 角色权限映射
 */
export const ROLE_PERMISSIONS: Record<User['role'], string[]> = {
  admin: ['*'], // 所有权限
  operator: [
    PERMISSIONS.VEHICLE_CONTROL,
    PERMISSIONS.VEHICLE_VIEW,
    PERMISSIONS.VIDEO_VIEW,
    PERMISSIONS.VIDEO_RECORD,
    PERMISSIONS.EMERGENCY_TRIGGER,
    PERMISSIONS.ROSBAG_REPLAY,
    PERMISSIONS.DISPATCH_VIEW
  ],
  viewer: [
    PERMISSIONS.VEHICLE_VIEW,
    PERMISSIONS.VIDEO_VIEW,
    PERMISSIONS.ROSBAG_REPLAY,
    PERMISSIONS.DISPATCH_VIEW
  ]
}

/**
 * 会话配置
 */
export const SESSION_CONFIG = {
  // 会话超时时间（毫秒）
  SESSION_TIMEOUT: 8 * 60 * 60 * 1000, // 8 小时

  // 记住我超时时间（毫秒）
  REMEMBER_ME_TIMEOUT: 30 * 24 * 60 * 60 * 1000, // 30 天

  // 最大失败登录次数
  MAX_LOGIN_ATTEMPTS: 5,

  // 锁定时间（毫秒）
  LOCKOUT_DURATION: 15 * 60 * 1000, // 15 分钟

  // Token 刷新间隔（毫秒）
  TOKEN_REFRESH_INTERVAL: 30 * 60 * 1000 // 30 分钟
}

export default {
  USER_WHITELIST,
  PERMISSIONS,
  ROLE_PERMISSIONS,
  SESSION_CONFIG
}
