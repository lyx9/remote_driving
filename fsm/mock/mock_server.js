/**
 * FSM-Pilot Mock 数据服务器
 * 用于演示和测试，模拟车端数据和云端服务
 */

const http = require('http');
const WebSocket = require('ws');
const url = require('url');

// 配置
const CONFIG = {
  HTTP_PORT: 3001,
  WS_PORT: 3002,
  UPDATE_INTERVAL: 100,  // 遥测更新间隔 (ms)
  PING_INTERVAL: 1000,   // 心跳间隔 (ms)
};

// Mock 车辆数据
const mockVehicles = {
  'FSM-01': {
    id: 'FSM-01',
    type: 'ROBO-TAXI',
    status: 'online',
    name: '测试车辆 1',
    // 位置 (上海)
    location: { lat: 31.2304, lng: 121.4737 },
    heading: 45,
    // 车辆状态
    speed: 0,
    steering_angle: 0,
    gear: 'P',
    turn_signal: 0,
    // 系统状态
    battery_level: 85,
    cpu_usage: 35,
    gpu_usage: 45,
    memory_usage: 60,
    // 传感器
    cameras_online: 5,
    cameras_total: 5,
    lidar_online: true,
    gps_fix: true,
    // 连接
    latency_ms: 45,
    connection_state: 'connected',
    // 调度
    emergency_level: 0,
    priority_score: 60,
    task_status: 'ACTIVE',
  },
  'FSM-02': {
    id: 'FSM-02',
    type: 'LOGISTICS',
    status: 'online',
    name: '物流车辆 2',
    location: { lat: 31.235, lng: 121.480 },
    heading: 180,
    speed: 0,
    steering_angle: 0,
    gear: 'P',
    turn_signal: 0,
    battery_level: 45,
    cpu_usage: 20,
    gpu_usage: 15,
    memory_usage: 40,
    cameras_online: 5,
    cameras_total: 5,
    lidar_online: true,
    gps_fix: true,
    latency_ms: 120,
    connection_state: 'connected',
    emergency_level: 0,
    priority_score: 40,
    task_status: 'IDLE',
  },
  'FSM-03': {
    id: 'FSM-03',
    type: 'SECURITY',
    status: 'online',
    name: '安防车辆 3',
    location: { lat: 31.220, lng: 121.460 },
    heading: 270,
    speed: 0,
    steering_angle: 0,
    gear: 'D',
    turn_signal: 0,
    battery_level: 92,
    cpu_usage: 55,
    gpu_usage: 60,
    memory_usage: 70,
    cameras_online: 4,
    cameras_total: 5,
    lidar_online: true,
    gps_fix: true,
    latency_ms: 65,
    connection_state: 'connected',
    emergency_level: 3,
    priority_score: 85,
    task_status: 'PATROL',
  },
};

// 控制指令状态
const controlState = {
  activeVehicle: null,
  steering: 0,
  throttle: 0,
  brake: 0,
};

// 网络模拟状态
const networkSimulation = {
  enabled: false,
  type: null,
  delayMs: 0,
  jitterMin: 0,
  jitterMax: 0,
  disconnected: false,
};

// 告警列表
const alerts = [];
let alertIdCounter = 0;

// WebSocket 客户端列表
const wsClients = new Set();

// ============== HTTP API ==============

const httpServer = http.createServer((req, res) => {
  const parsedUrl = url.parse(req.url, true);
  const path = parsedUrl.pathname;
  const method = req.method;

  // CORS 头
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
  res.setHeader('Access-Control-Allow-Headers', 'Content-Type');
  res.setHeader('Content-Type', 'application/json');

  if (method === 'OPTIONS') {
    res.writeHead(200);
    res.end();
    return;
  }

  // 路由
  if (path === '/api/v1/vehicles' && method === 'GET') {
    // 获取所有车辆
    const vehicles = Object.values(mockVehicles).map(v => ({
      id: v.id,
      type: v.type,
      status: v.status,
      name: v.name,
      location: v.location,
    }));
    res.writeHead(200);
    res.end(JSON.stringify(vehicles));

  } else if (path.match(/^\/api\/v1\/vehicles\/[\w-]+$/) && method === 'GET') {
    // 获取单个车辆
    const vehicleId = path.split('/').pop();
    const vehicle = mockVehicles[vehicleId];
    if (vehicle) {
      res.writeHead(200);
      res.end(JSON.stringify(vehicle));
    } else {
      res.writeHead(404);
      res.end(JSON.stringify({ error: 'Vehicle not found' }));
    }

  } else if (path.match(/^\/api\/v1\/vehicles\/[\w-]+\/status$/) && method === 'GET') {
    // 获取车辆状态
    const vehicleId = path.split('/')[4];
    const vehicle = mockVehicles[vehicleId];
    if (vehicle) {
      res.writeHead(200);
      res.end(JSON.stringify({
        vehicle_id: vehicle.id,
        timestamp: Date.now(),
        speed: vehicle.speed,
        steering: vehicle.steering_angle,
        gear: vehicle.gear,
        location: vehicle.location,
        latency_ms: vehicle.latency_ms,
        battery_level: vehicle.battery_level,
      }));
    } else {
      res.writeHead(404);
      res.end(JSON.stringify({ error: 'Vehicle not found' }));
    }

  } else if (path === '/api/v1/scheduling/queue' && method === 'GET') {
    // 获取调度队列
    const queue = Object.values(mockVehicles)
      .sort((a, b) => b.priority_score - a.priority_score)
      .map(v => ({
        vehicle_id: v.id,
        priority_score: v.priority_score,
        emergency_level: v.emergency_level,
        latency_ms: v.latency_ms,
        battery_level: v.battery_level,
        task_status: v.task_status,
      }));
    res.writeHead(200);
    res.end(JSON.stringify({
      vehicles: queue,
      algorithm: 'weighted_priority',
      updated_at: new Date().toISOString(),
    }));

  } else if (path === '/api/v1/alerts' && method === 'GET') {
    // 获取告警列表
    res.writeHead(200);
    res.end(JSON.stringify(alerts));

  } else if (path === '/mock/simulate' && method === 'POST') {
    // 网络模拟
    let body = '';
    req.on('data', chunk => { body += chunk; });
    req.on('end', () => {
      const data = JSON.parse(body);
      handleNetworkSimulation(data);
      res.writeHead(200);
      res.end(JSON.stringify({ status: 'ok', simulation: networkSimulation }));
    });

  } else if (path === '/mock/emergency' && method === 'POST') {
    // 紧急状态模拟
    let body = '';
    req.on('data', chunk => { body += chunk; });
    req.on('end', () => {
      const data = JSON.parse(body);
      handleEmergencySimulation(data);
      res.writeHead(200);
      res.end(JSON.stringify({ status: 'ok' }));
    });

  } else if (path === '/mock/control' && method === 'POST') {
    // 接收控制指令
    let body = '';
    req.on('data', chunk => { body += chunk; });
    req.on('end', () => {
      const data = JSON.parse(body);
      handleControlCommand(data);
      res.writeHead(200);
      res.end(JSON.stringify({ status: 'ok' }));
    });

  } else {
    res.writeHead(404);
    res.end(JSON.stringify({ error: 'Not found' }));
  }
});

// ============== WebSocket Server ==============

const wss = new WebSocket.Server({ port: CONFIG.WS_PORT });

wss.on('connection', (ws, req) => {
  console.log('[WS] Client connected');
  wsClients.add(ws);

  // 发送初始车辆列表
  ws.send(JSON.stringify({
    event: 'vehicles_list',
    data: Object.values(mockVehicles).map(v => ({
      id: v.id,
      type: v.type,
      status: v.status,
    })),
  }));

  ws.on('message', (message) => {
    try {
      const data = JSON.parse(message);
      handleWsMessage(ws, data);
    } catch (e) {
      console.error('[WS] Parse error:', e);
    }
  });

  ws.on('close', () => {
    console.log('[WS] Client disconnected');
    wsClients.delete(ws);
  });
});

function handleWsMessage(ws, data) {
  switch (data.type) {
    case 'subscribe':
      // 订阅车辆数据
      ws.subscribedVehicle = data.vehicle_id;
      console.log(`[WS] Subscribed to ${data.vehicle_id}`);
      break;

    case 'control':
      // 控制指令
      handleControlCommand(data);
      break;

    case 'ping':
      // 心跳响应
      ws.send(JSON.stringify({
        type: 'pong',
        sequence: data.sequence,
        timestamp: Date.now(),
        original_timestamp: data.timestamp,
      }));
      break;
  }
}

// ============== 数据模拟逻辑 ==============

function handleNetworkSimulation(data) {
  switch (data.type) {
    case 'delay':
      networkSimulation.enabled = true;
      networkSimulation.type = 'delay';
      networkSimulation.delayMs = data.value || 200;
      // 更新所有车辆延迟
      Object.values(mockVehicles).forEach(v => {
        v.latency_ms = Math.max(v.latency_ms, networkSimulation.delayMs);
      });
      addAlert('high_latency', 'FSM-01', 'warning', '网络延迟升高');
      break;

    case 'jitter':
      networkSimulation.enabled = true;
      networkSimulation.type = 'jitter';
      networkSimulation.jitterMin = data.min || 50;
      networkSimulation.jitterMax = data.max || 300;
      break;

    case 'disconnect':
      networkSimulation.enabled = true;
      networkSimulation.type = 'disconnect';
      networkSimulation.disconnected = true;
      Object.values(mockVehicles).forEach(v => {
        v.connection_state = 'reconnecting';
      });
      addAlert('connection_lost', 'FSM-01', 'critical', '网络连接中断');
      // 恢复定时
      setTimeout(() => {
        networkSimulation.disconnected = false;
        Object.values(mockVehicles).forEach(v => {
          v.connection_state = 'connected';
        });
      }, data.duration || 5000);
      break;

    case 'reset':
      networkSimulation.enabled = false;
      networkSimulation.type = null;
      networkSimulation.delayMs = 0;
      networkSimulation.disconnected = false;
      // 恢复正常延迟
      mockVehicles['FSM-01'].latency_ms = 45;
      mockVehicles['FSM-02'].latency_ms = 120;
      mockVehicles['FSM-03'].latency_ms = 65;
      Object.values(mockVehicles).forEach(v => {
        v.connection_state = 'connected';
      });
      break;
  }
}

function handleEmergencySimulation(data) {
  const vehicle = mockVehicles[data.vehicle_id];
  if (!vehicle) return;

  if (data.action === 'trigger') {
    vehicle.emergency_level = 4;
    vehicle.speed = 0;
    vehicle.gear = 'P';
    vehicle.priority_score = 100;
    addAlert('emergency_active', data.vehicle_id, 'critical', '紧急停车已触发');
    broadcastEmergency(data.vehicle_id, true);
  } else if (data.action === 'release') {
    vehicle.emergency_level = 0;
    vehicle.priority_score = calculatePriority(vehicle);
    broadcastEmergency(data.vehicle_id, false);
  }
}

function handleControlCommand(data) {
  const vehicleId = data.vehicle_id || controlState.activeVehicle;
  const vehicle = mockVehicles[vehicleId];
  if (!vehicle) return;

  // 更新控制状态
  if (data.steering !== undefined) {
    controlState.steering = data.steering;
    vehicle.steering_angle = data.steering * 35;  // 最大 35 度
  }
  if (data.throttle !== undefined) {
    controlState.throttle = data.throttle;
  }
  if (data.brake !== undefined) {
    controlState.brake = data.brake;
  }
  if (data.gear !== undefined) {
    vehicle.gear = ['P', 'R', 'N', 'D'][data.gear] || vehicle.gear;
  }
  if (data.turn_signal !== undefined) {
    vehicle.turn_signal = data.turn_signal;
  }
  if (data.emergency) {
    handleEmergencySimulation({ vehicle_id: vehicleId, action: 'trigger' });
  }

  console.log(`[Control] ${vehicleId}: steering=${controlState.steering.toFixed(2)}, throttle=${controlState.throttle.toFixed(2)}`);
}

function calculatePriority(vehicle) {
  const weights = {
    emergency: 0.35,
    latency: 0.25,
    battery: 0.10,
  };

  let score = 50;  // 基础分
  score += vehicle.emergency_level * 10 * weights.emergency;
  score += Math.max(0, (300 - vehicle.latency_ms) / 3) * weights.latency;
  score += (100 - vehicle.battery_level) * 0.1 * weights.battery;

  return Math.round(score);
}

function addAlert(type, vehicleId, severity, message) {
  const alert = {
    id: `alert_${++alertIdCounter}`,
    vehicle_id: vehicleId,
    type: type,
    severity: severity,
    message: message,
    timestamp: Date.now(),
    acknowledged: false,
  };
  alerts.unshift(alert);

  // 广播告警
  broadcastAlert(alert);
}

// ============== 广播函数 ==============

function broadcastToAll(message) {
  const data = JSON.stringify(message);
  wsClients.forEach(ws => {
    if (ws.readyState === WebSocket.OPEN) {
      ws.send(data);
    }
  });
}

function broadcastVehicleStatus(vehicle) {
  broadcastToAll({
    event: 'vehicle_status',
    data: {
      vehicle_id: vehicle.id,
      timestamp: Date.now(),
      speed: vehicle.speed,
      steering: vehicle.steering_angle,
      gear: vehicle.gear,
      turn_signal: vehicle.turn_signal,
      location: vehicle.location,
      latency_ms: vehicle.latency_ms,
      battery_level: vehicle.battery_level,
      connection_state: vehicle.connection_state,
    },
  });
}

function broadcastSchedulingUpdate() {
  const queue = Object.values(mockVehicles)
    .sort((a, b) => b.priority_score - a.priority_score)
    .map(v => ({
      vehicle_id: v.id,
      priority: v.priority_score,
      emergency_level: v.emergency_level,
    }));

  broadcastToAll({
    event: 'scheduling_update',
    data: { queue },
  });
}

function broadcastAlert(alert) {
  broadcastToAll({
    event: 'alert',
    data: alert,
  });
}

function broadcastEmergency(vehicleId, active) {
  broadcastToAll({
    event: 'emergency',
    data: {
      vehicle_id: vehicleId,
      active: active,
      timestamp: Date.now(),
    },
  });
}

function broadcastLatency() {
  Object.values(mockVehicles).forEach(vehicle => {
    broadcastToAll({
      event: 'latency_update',
      data: {
        vehicle_id: vehicle.id,
        rtt_ms: vehicle.latency_ms,
        video_latency_ms: vehicle.latency_ms + 30,
        control_latency_ms: vehicle.latency_ms + 10,
        jitter_ms: Math.random() * 10,
      },
    });
  });
}

// ============== 模拟更新循环 ==============

function updateSimulation() {
  Object.values(mockVehicles).forEach(vehicle => {
    // 更新速度 (基于油门和刹车)
    if (vehicle.gear === 'D') {
      if (controlState.throttle > 0 && vehicle.id === controlState.activeVehicle) {
        vehicle.speed = Math.min(60, vehicle.speed + controlState.throttle * 2);
      } else if (controlState.brake > 0 && vehicle.id === controlState.activeVehicle) {
        vehicle.speed = Math.max(0, vehicle.speed - controlState.brake * 5);
      } else {
        vehicle.speed = Math.max(0, vehicle.speed - 0.5);  // 自然减速
      }
    } else if (vehicle.gear === 'R') {
      vehicle.speed = Math.min(10, vehicle.speed + controlState.throttle * 0.5);
    } else {
      vehicle.speed = 0;
    }

    // 更新位置 (简单模拟)
    if (vehicle.speed > 0) {
      const heading = vehicle.heading * Math.PI / 180;
      const distance = vehicle.speed / 3600;  // km per tick
      vehicle.location.lat += Math.cos(heading) * distance * 0.01;
      vehicle.location.lng += Math.sin(heading) * distance * 0.01;
      vehicle.heading += vehicle.steering_angle * 0.1;
    }

    // 更新延迟 (网络模拟)
    if (networkSimulation.type === 'jitter') {
      const jitter = networkSimulation.jitterMin +
        Math.random() * (networkSimulation.jitterMax - networkSimulation.jitterMin);
      vehicle.latency_ms = Math.round(jitter);
    }

    // 更新优先级
    vehicle.priority_score = calculatePriority(vehicle);

    // 电池消耗 (非常慢)
    if (vehicle.speed > 0) {
      vehicle.battery_level = Math.max(0, vehicle.battery_level - 0.001);
    }

    // 系统负载波动
    vehicle.cpu_usage = Math.max(10, Math.min(90, vehicle.cpu_usage + (Math.random() - 0.5) * 5));
    vehicle.gpu_usage = Math.max(10, Math.min(90, vehicle.gpu_usage + (Math.random() - 0.5) * 5));

    // 广播状态
    if (!networkSimulation.disconnected) {
      broadcastVehicleStatus(vehicle);
    }
  });

  // 调度队列更新
  broadcastSchedulingUpdate();
}

// ============== 启动 ==============

httpServer.listen(CONFIG.HTTP_PORT, () => {
  console.log(`[HTTP] Mock API server running on http://localhost:${CONFIG.HTTP_PORT}`);
});

console.log(`[WS] Mock WebSocket server running on ws://localhost:${CONFIG.WS_PORT}`);

// 定时更新
setInterval(updateSimulation, CONFIG.UPDATE_INTERVAL);
setInterval(broadcastLatency, CONFIG.PING_INTERVAL);

// 设置默认活动车辆
controlState.activeVehicle = 'FSM-01';

console.log('');
console.log('╔════════════════════════════════════════╗');
console.log('║     FSM-Pilot Mock Server Started      ║');
console.log('╠════════════════════════════════════════╣');
console.log('║  HTTP API: http://localhost:3001       ║');
console.log('║  WebSocket: ws://localhost:3002        ║');
console.log('║                                        ║');
console.log('║  Mock Vehicles:                        ║');
console.log('║    - FSM-01 (ROBO-TAXI)               ║');
console.log('║    - FSM-02 (LOGISTICS)               ║');
console.log('║    - FSM-03 (SECURITY)                ║');
console.log('╚════════════════════════════════════════╝');
