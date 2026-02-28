#!/usr/bin/env python3
"""
FSM-Pilot 车端 MQTT Bridge
连接 Autoware.Universe ROS2 与阿里云 IoT MQTT

功能:
  - 6路摄像头图像压缩后发布到 MQTT
  - 车辆遥测数据 (速度/位置/状态) 发布到 MQTT
  - 订阅 MQTT 控制指令并转发给 Autoware

依赖:
  pip install paho-mqtt pyyaml opencv-python numpy
  ROS2 packages: rclpy, sensor_msgs, geometry_msgs, autoware_auto_control_msgs

用法:
  python3 mqtt_bridge.py --config config.yaml
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String

# Autoware 控制消息
try:
    from autoware_auto_control_msgs.msg import AckermannControlCommand
    from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport, GearReport
    HAS_AUTOWARE = True
except ImportError:
    HAS_AUTOWARE = False
    print("[WARN] Autoware msgs not found, using generic msgs")

import paho.mqtt.client as mqtt
import ssl
import json
import time
import threading
import yaml
import argparse
import cv2
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path


# ======================== MQTT Topic 定义 ========================

def make_topics(vehicle_id: str):
    base = f"fsm/{vehicle_id}"
    return {
        # 上行 (车→云→前端)
        "cam":       [f"{base}/cam/{i}" for i in range(6)],
        "telemetry": f"{base}/telemetry",
        "status":    f"{base}/status",
        "lidar":     f"{base}/lidar",
        # 下行 (前端→云→车)
        "control":   f"{base}/control",
        "cmd":       f"{base}/cmd",
    }


# ======================== 摄像头配置 ========================

CAMERA_TOPICS = [
    "/sensing/camera/camera0/image_raw",
    "/sensing/camera/camera1/image_raw",
    "/sensing/camera/camera2/image_raw",
    "/sensing/camera/camera3/image_raw",
    "/sensing/camera/camera4/image_raw",
    "/sensing/camera/camera5/image_raw",
]

# 也支持 CompressedImage
CAMERA_COMPRESSED_TOPICS = [
    "/sensing/camera/camera0/image_raw/compressed",
    "/sensing/camera/camera1/image_raw/compressed",
    "/sensing/camera/camera2/image_raw/compressed",
    "/sensing/camera/camera3/image_raw/compressed",
    "/sensing/camera/camera4/image_raw/compressed",
    "/sensing/camera/camera5/image_raw/compressed",
]


class MQTTBridgeNode(Node):
    """车端 ROS2 → MQTT 桥接节点"""

    def __init__(self, config: dict):
        super().__init__('mqtt_bridge')
        self.config = config
        self.vehicle_id = config['vehicle']['id']
        self.topics = make_topics(self.vehicle_id)
        self.bridge = CvBridge()

        # 图像质量配置
        self.jpeg_quality = config.get('camera', {}).get('jpeg_quality', 60)
        self.max_width = config.get('camera', {}).get('max_width', 640)
        self.cam_fps = config.get('camera', {}).get('fps', 10)

        # 帧率控制
        self._last_frame_time = [0.0] * 6
        self._frame_interval = 1.0 / self.cam_fps

        # 遥测缓存
        self._telemetry = {
            "vehicle_id": self.vehicle_id,
            "speed": 0.0,
            "steering": 0.0,
            "gear": "N",
            "lat": 0.0,
            "lng": 0.0,
            "heading": 0.0,
            "battery": 100.0,
            "timestamp": 0,
        }
        self._telemetry_lock = threading.Lock()

        # 初始化 MQTT
        self._setup_mqtt()

        # 初始化 ROS2 订阅
        self._setup_ros_subscribers()

        # 初始化 Autoware 控制发布者
        self._setup_ros_publishers()

        # 定时发布遥测
        self.create_timer(0.1, self._publish_telemetry)  # 10Hz
        self.create_timer(1.0, self._publish_status)     # 1Hz

        self.get_logger().info(f"[MQTTBridge] Vehicle {self.vehicle_id} started")

    # ======================== MQTT 初始化 ========================

    def _setup_mqtt(self):
        cfg = self.config['aliyun']
        self.mqtt_client = mqtt.Client(
            client_id=cfg['client_id'],
            protocol=mqtt.MQTTv311
        )
        self.mqtt_client.username_pw_set(cfg['username'], cfg['password'])

        # TLS (阿里云 IoT 必须)
        if cfg.get('tls', True):
            self.mqtt_client.tls_set(
                ca_certs=cfg.get('ca_cert', None),
                tls_version=ssl.PROTOCOL_TLS
            )

        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message

        self.get_logger().info(f"[MQTT] Connecting to {cfg['host']}:{cfg['port']}")
        self.mqtt_client.connect_async(cfg['host'], cfg['port'], keepalive=60)
        self.mqtt_client.loop_start()

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("[MQTT] Connected to Aliyun IoT")
            # 订阅控制指令
            client.subscribe(self.topics['control'], qos=1)
            client.subscribe(self.topics['cmd'], qos=1)
            self.get_logger().info(f"[MQTT] Subscribed: {self.topics['control']}")
        else:
            self.get_logger().error(f"[MQTT] Connect failed, rc={rc}")

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f"[MQTT] Disconnected rc={rc}, will reconnect")

    def _on_mqtt_message(self, client, userdata, msg):
        """处理来自前端的控制指令"""
        try:
            payload = json.loads(msg.payload.decode())
            topic = msg.topic

            if topic == self.topics['control']:
                self._handle_control_command(payload)
            elif topic == self.topics['cmd']:
                self._handle_system_command(payload)
        except Exception as e:
            self.get_logger().error(f"[MQTT] Message error: {e}")

    # ======================== ROS2 订阅 ========================

    def _setup_ros_subscribers(self):
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 6路摄像头
        for i, topic in enumerate(CAMERA_TOPICS):
            idx = i
            self.create_subscription(
                Image, topic,
                lambda msg, n=idx: self._on_camera_image(msg, n),
                qos_sensor
            )
            # 同时订阅压缩版本 (优先使用)
            self.create_subscription(
                CompressedImage, CAMERA_COMPRESSED_TOPICS[i],
                lambda msg, n=idx: self._on_camera_compressed(msg, n),
                qos_sensor
            )

        # 速度
        if HAS_AUTOWARE:
            self.create_subscription(
                VelocityReport,
                '/vehicle/status/velocity_status',
                self._on_velocity,
                qos_sensor
            )
            self.create_subscription(
                SteeringReport,
                '/vehicle/status/steering_status',
                self._on_steering,
                qos_sensor
            )
            self.create_subscription(
                GearReport,
                '/vehicle/status/gear_status',
                self._on_gear,
                qos_sensor
            )
        else:
            # 通用 Twist
            self.create_subscription(
                TwistStamped,
                '/vehicle/twist',
                self._on_twist,
                qos_sensor
            )

        # 定位 (Autoware EKF 输出)
        self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self._on_odometry,
            qos_reliable
        )

    # ======================== ROS2 发布者 ========================

    def _setup_ros_publishers(self):
        if HAS_AUTOWARE:
            self._control_pub = self.create_publisher(
                AckermannControlCommand,
                '/control/command/control_cmd',
                10
            )
        else:
            self._twist_pub = self.create_publisher(
                TwistStamped,
                '/cmd_vel',
                10
            )

    # ======================== 摄像头回调 ========================

    def _on_camera_image(self, msg: Image, cam_idx: int):
        """处理原始图像，压缩后发布到 MQTT"""
        now = time.time()
        if now - self._last_frame_time[cam_idx] < self._frame_interval:
            return
        self._last_frame_time[cam_idx] = now

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._publish_camera_frame(cv_img, cam_idx, msg.header.stamp.sec)
        except Exception as e:
            self.get_logger().warn(f"[CAM{cam_idx}] Image convert error: {e}")

    def _on_camera_compressed(self, msg: CompressedImage, cam_idx: int):
        """处理已压缩图像，直接转发或重新压缩"""
        now = time.time()
        if now - self._last_frame_time[cam_idx] < self._frame_interval:
            return
        self._last_frame_time[cam_idx] = now

        try:
            # 解码
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_img is not None:
                self._publish_camera_frame(cv_img, cam_idx, msg.header.stamp.sec)
        except Exception as e:
            self.get_logger().warn(f"[CAM{cam_idx}] Compressed image error: {e}")

    def _publish_camera_frame(self, cv_img, cam_idx: int, timestamp: int):
        """缩放 + JPEG 压缩 + MQTT 发布"""
        # 缩放
        h, w = cv_img.shape[:2]
        if w > self.max_width:
            scale = self.max_width / w
            cv_img = cv2.resize(cv_img, (self.max_width, int(h * scale)))

        # JPEG 编码
        ret, buf = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ret:
            return

        # MQTT 发布 (二进制 payload)
        topic = self.topics['cam'][cam_idx]
        self.mqtt_client.publish(topic, buf.tobytes(), qos=0, retain=False)

    # ======================== 遥测回调 ========================

    def _on_velocity(self, msg):
        with self._telemetry_lock:
            self._telemetry['speed'] = msg.longitudinal_velocity * 3.6  # m/s → km/h

    def _on_steering(self, msg):
        with self._telemetry_lock:
            self._telemetry['steering'] = msg.steering_tire_angle  # rad

    def _on_gear(self, msg):
        gear_map = {1: 'P', 2: 'R', 3: 'N', 4: 'D'}
        with self._telemetry_lock:
            self._telemetry['gear'] = gear_map.get(msg.report, 'N')

    def _on_twist(self, msg):
        with self._telemetry_lock:
            self._telemetry['speed'] = msg.twist.linear.x * 3.6

    def _on_odometry(self, msg):
        # 从 EKF 输出提取位置和朝向
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        # 四元数转 yaw
        import math
        siny = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny, cosy)

        with self._telemetry_lock:
            # 注意: Autoware 使用局部坐标系，需要配合 GNSS 转换为经纬度
            # 这里先存 x/y，实际部署时替换为 GNSS 经纬度
            self._telemetry['lat'] = pos.y   # 替换为真实纬度
            self._telemetry['lng'] = pos.x   # 替换为真实经度
            self._telemetry['heading'] = math.degrees(yaw)
            self._telemetry['timestamp'] = int(time.time() * 1000)

    # ======================== 定时发布 ========================

    def _publish_telemetry(self):
        with self._telemetry_lock:
            payload = json.dumps(self._telemetry)
        self.mqtt_client.publish(self.topics['telemetry'], payload, qos=0)

    def _publish_status(self):
        status = {
            "vehicle_id": self.vehicle_id,
            "online": True,
            "timestamp": int(time.time() * 1000),
            "autoware": HAS_AUTOWARE,
        }
        self.mqtt_client.publish(self.topics['status'], json.dumps(status), qos=1)

    # ======================== 控制指令处理 ========================

    def _handle_control_command(self, payload: dict):
        """
        前端发来的控制指令:
        {
          "seq": 123,
          "steering": -0.3,   # -1.0 ~ 1.0
          "throttle": 0.5,    # 0 ~ 1.0
          "brake": 0.0,       # 0 ~ 1.0
          "gear": 3,          # 0=P,1=R,2=N,3=D
          "emergency": false
        }
        """
        if payload.get('emergency'):
            self.get_logger().warn("[CONTROL] EMERGENCY STOP")
            self._send_emergency_stop()
            return

        steering = float(payload.get('steering', 0.0))
        throttle = float(payload.get('throttle', 0.0))
        brake = float(payload.get('brake', 0.0))

        if HAS_AUTOWARE:
            self._send_autoware_control(steering, throttle, brake)
        else:
            self._send_twist_control(steering, throttle, brake)

        # 发送 ACK
        ack = {"seq": payload.get('seq', 0), "applied": True, "ts": int(time.time() * 1000)}
        self.mqtt_client.publish(
            f"fsm/{self.vehicle_id}/control_ack",
            json.dumps(ack), qos=0
        )

    def _send_autoware_control(self, steering: float, throttle: float, brake: float):
        """发送 Autoware AckermannControlCommand"""
        from rclpy.time import Time
        msg = AckermannControlCommand()
        msg.stamp = self.get_clock().now().to_msg()

        # 转向角: 前端 -1~1 → 实际转向角 (最大 ±0.5 rad ≈ ±28.6°)
        max_steer = self.config.get('vehicle', {}).get('max_steer_rad', 0.5)
        msg.lateral.steering_tire_angle = steering * max_steer

        # 纵向: throttle/brake → 加速度
        max_accel = self.config.get('vehicle', {}).get('max_accel', 3.0)
        max_decel = self.config.get('vehicle', {}).get('max_decel', -5.0)
        if brake > 0.01:
            msg.longitudinal.acceleration = brake * max_decel
        else:
            msg.longitudinal.acceleration = throttle * max_accel

        self._control_pub.publish(msg)

    def _send_twist_control(self, steering: float, throttle: float, brake: float):
        """发送通用 TwistStamped (非 Autoware 模式)"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        max_speed = self.config.get('vehicle', {}).get('max_speed_mps', 10.0)
        msg.twist.linear.x = throttle * max_speed - brake * max_speed
        msg.twist.angular.z = -steering * 1.0
        self._twist_pub.publish(msg)

    def _send_emergency_stop(self):
        if HAS_AUTOWARE:
            msg = AckermannControlCommand()
            msg.stamp = self.get_clock().now().to_msg()
            msg.longitudinal.acceleration = -10.0
            msg.lateral.steering_tire_angle = 0.0
            self._control_pub.publish(msg)
        else:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            self._twist_pub.publish(msg)

    def _handle_system_command(self, payload: dict):
        cmd = payload.get('cmd')
        self.get_logger().info(f"[CMD] Received: {cmd}")
        # 可扩展: 模式切换、灯光控制等

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


# ======================== 入口 ========================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='config.yaml')
    args = parser.parse_args()

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    rclpy.init()
    node = MQTTBridgeNode(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
