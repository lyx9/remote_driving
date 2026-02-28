#!/usr/bin/env python3
"""
阿里云 IoT MQTT 认证信息生成器
运行后将输出填入 config.yaml

用法: python3 gen_auth.py
"""

import hmac
import hashlib
import time

def gen_aliyun_mqtt_auth(product_key: str, device_name: str, device_secret: str):
    timestamp = str(int(time.time() * 1000))
    client_id_raw = device_name  # 可自定义

    client_id = f"{client_id_raw}|securemode=3,signmethod=hmacsha256,timestamp={timestamp}|"
    username = f"{device_name}&{product_key}"

    # 签名内容
    sign_content = f"clientId{client_id_raw}deviceName{device_name}productKey{product_key}timestamp{timestamp}"
    password = hmac.new(
        device_secret.encode('utf-8'),
        sign_content.encode('utf-8'),
        hashlib.sha256
    ).hexdigest()

    print("=" * 60)
    print("阿里云 IoT MQTT 认证信息")
    print("=" * 60)
    print(f"host:      {product_key}.iot-as-mqtt.cn-shanghai.aliyuncs.com")
    print(f"port:      8883")
    print(f"client_id: {client_id}")
    print(f"username:  {username}")
    print(f"password:  {password}")
    print("=" * 60)
    print("将以上内容填入 config.yaml 的 aliyun 节")

if __name__ == '__main__':
    pk = input("ProductKey: ").strip()
    dn = input("DeviceName: ").strip()
    ds = input("DeviceSecret: ").strip()
    gen_aliyun_mqtt_auth(pk, dn, ds)
