/**
 * 阿里云 IoT MQTT 配置
 * 修改此文件中的参数以连接到你的阿里云设备
 */

import type { MQTTConfig } from '@/services/mqttService'

/**
 * 阿里云 IoT MQTT over WebSocket endpoint 格式:
 *   wss://{productKey}.iot-as-mqtt.{region}.aliyuncs.com/mqtt
 *
 * 认证信息从 vehicle_ros2/gen_auth.py 生成
 */
export const ALIYUN_MQTT_CONFIG: MQTTConfig = {
  brokerUrl: import.meta.env.VITE_MQTT_BROKER_URL ||
    'wss://YOUR_PRODUCT_KEY.iot-as-mqtt.cn-shanghai.aliyuncs.com/mqtt',

  clientId: import.meta.env.VITE_MQTT_CLIENT_ID ||
    'web-operator-001|securemode=3,signmethod=hmacsha256,timestamp=1700000000|',

  username: import.meta.env.VITE_MQTT_USERNAME ||
    'web-operator-001&YOUR_PRODUCT_KEY',

  password: import.meta.env.VITE_MQTT_PASSWORD ||
    'YOUR_GENERATED_PASSWORD',

  vehicleId: import.meta.env.VITE_VEHICLE_ID || 'FSM-01',
}

/**
 * 多车配置 (展示用)
 */
export const VEHICLE_LIST = [
  { id: 'FSM-01', name: '测试车 01', type: 'ROBO-TAXI' },
  { id: 'FSM-02', name: '测试车 02', type: 'LOGISTICS' },
]
