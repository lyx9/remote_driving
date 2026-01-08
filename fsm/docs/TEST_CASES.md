# FSM-Pilot 测试用例文档

## 1. 单元测试

### 1.1 配置管理测试

```cpp
// test/test_config_manager.cpp

TEST(VehicleConfigManager, LoadFromFile) {
    config::VehicleConfigManager config;
    EXPECT_TRUE(config.loadFromFile("test_vehicle_config.yaml"));
    EXPECT_EQ(config.getVehicleId(), "FSM-TEST");
}

TEST(VehicleConfigManager, GetCameraConfig) {
    config::VehicleConfigManager config;
    config.loadFromFile("test_vehicle_config.yaml");

    auto cam = config.getCameraById("cam_front_center");
    EXPECT_TRUE(cam.has_value());
    EXPECT_EQ(cam->fps, 30);
}

TEST(CloudConfigManager, SchedulingWeights) {
    config::CloudConfigManager config;
    config.loadFromFile("test_cloud_config.yaml");

    auto& weights = config.getSchedulingConfig().weights;
    EXPECT_NEAR(weights.emergency + weights.latency + weights.distance +
                weights.battery + weights.task_priority, 1.0, 0.001);
}
```

### 1.2 工具函数测试

```cpp
// test/test_utils.cpp

TEST(MathUtils, DegToRad) {
    EXPECT_NEAR(utils::MathUtils::degToRad(180.0), M_PI, 1e-6);
    EXPECT_NEAR(utils::MathUtils::degToRad(90.0), M_PI / 2.0, 1e-6);
}

TEST(MathUtils, GpsDistance) {
    // 上海到北京约1000km
    double dist = utils::MathUtils::gpsDistance(
        31.2304, 121.4737,  // 上海
        39.9042, 116.4074   // 北京
    );
    EXPECT_NEAR(dist, 1068000, 10000);  // 允许10km误差
}

TEST(MathUtils, QuaternionToEuler) {
    double roll, pitch, yaw;
    // 单位四元数 (无旋转)
    utils::MathUtils::quaternionToEuler(0, 0, 0, 1, roll, pitch, yaw);
    EXPECT_NEAR(roll, 0.0, 1e-6);
    EXPECT_NEAR(pitch, 0.0, 1e-6);
    EXPECT_NEAR(yaw, 0.0, 1e-6);
}

TEST(MovingAverage, Basic) {
    utils::MovingAverage avg(3);
    avg.add(1.0);
    avg.add(2.0);
    avg.add(3.0);
    EXPECT_NEAR(avg.get(), 2.0, 1e-6);

    avg.add(4.0);  // 移除1.0
    EXPECT_NEAR(avg.get(), 3.0, 1e-6);
}
```

### 1.3 调度算法测试

```cpp
// test/test_scheduling.cpp

TEST(SchedulingService, PriorityCalculation) {
    config::CloudConfigManager config;
    config.loadFromFile("test_cloud_config.yaml");

    cloud::SchedulingService scheduler(config);

    cloud::VehicleSchedulingInfo v1;
    v1.vehicle_id = "FSM-01";
    v1.emergency_level = 4;  // 最高紧急
    v1.latency_ms = 100;
    v1.battery_level = 80;

    cloud::VehicleSchedulingInfo v2;
    v2.vehicle_id = "FSM-02";
    v2.emergency_level = 0;
    v2.latency_ms = 100;
    v2.battery_level = 80;

    scheduler.updateVehicleState(v1);
    scheduler.updateVehicleState(v2);

    auto queue = scheduler.getSchedulingQueue();
    EXPECT_EQ(queue[0].vehicle_id, "FSM-01");  // 紧急的排前面
}

TEST(SchedulingService, LatencyAlert) {
    // 测试高延迟告警
    config::CloudConfigManager config;
    cloud::SchedulingService scheduler(config);

    cloud::VehicleSchedulingInfo v;
    v.vehicle_id = "FSM-01";
    v.latency_ms = 600;  // 超过临界阈值

    // 期望生成告警
    scheduler.updateVehicleState(v);

    // 验证告警逻辑...
}
```

## 2. 集成测试

### 2.1 端到端延迟测试

```yaml
# test/integration/latency_test.yaml
name: "E2E Latency Test"
description: "测试从车端到操作端的端到端延迟"

setup:
  - start_vehicle_node: "FSM-TEST"
  - start_cloud_server: true
  - connect_operator: true

test_steps:
  - name: "Measure RTT"
    action: "send_ping"
    expect:
      rtt_ms: "<100"

  - name: "Measure Video Latency"
    action: "measure_video_latency"
    expect:
      latency_ms: "<200"

  - name: "Measure Control Latency"
    action: "send_control_command"
    expect:
      response_time_ms: "<150"

cleanup:
  - disconnect_all: true
  - stop_servers: true
```

### 2.2 多车辆连接测试

```yaml
# test/integration/multi_vehicle_test.yaml
name: "Multi-Vehicle Connection Test"
description: "测试多车辆同时连接的稳定性"

setup:
  - start_cloud_server: true
  - vehicles:
      - id: "FSM-01"
      - id: "FSM-02"
      - id: "FSM-03"

test_steps:
  - name: "Connect All Vehicles"
    action: "connect_vehicles"
    expect:
      connected_count: 3

  - name: "Verify Scheduling Queue"
    action: "get_scheduling_queue"
    expect:
      queue_length: 3

  - name: "Switch Active Vehicle"
    action: "switch_vehicle"
    target: "FSM-02"
    expect:
      active_vehicle: "FSM-02"

  - name: "Disconnect One Vehicle"
    action: "disconnect_vehicle"
    target: "FSM-01"
    expect:
      connected_count: 2

cleanup:
  - disconnect_all: true
```

### 2.3 网络中断恢复测试

```yaml
# test/integration/network_recovery_test.yaml
name: "Network Recovery Test"
description: "测试网络中断后的自动恢复能力"

test_steps:
  - name: "Establish Connection"
    action: "connect_vehicle"
    vehicle_id: "FSM-01"
    expect:
      state: "CONNECTED"

  - name: "Simulate Network Interruption"
    action: "block_network"
    duration_ms: 5000

  - name: "Verify Reconnection Attempt"
    action: "wait_for_state"
    expected_state: "RECONNECTING"
    timeout_ms: 3000

  - name: "Restore Network"
    action: "restore_network"

  - name: "Verify Reconnection Success"
    action: "wait_for_state"
    expected_state: "CONNECTED"
    timeout_ms: 10000

  - name: "Verify No Data Loss"
    action: "verify_telemetry_continuity"
```

### 2.4 紧急停车测试

```yaml
# test/integration/emergency_stop_test.yaml
name: "Emergency Stop Test"
description: "测试紧急停车功能的响应时间"

preconditions:
  - vehicle_connected: true
  - remote_control_active: true

test_steps:
  - name: "Start Moving"
    action: "send_control_command"
    command:
      speed: 10.0
      steering: 0.0

  - name: "Trigger Emergency Stop"
    action: "trigger_emergency"
    timestamp: "record"

  - name: "Measure Response Time"
    action: "wait_for_stop"
    expect:
      response_time_ms: "<500"
      final_speed: 0.0

  - name: "Verify Emergency State"
    action: "get_vehicle_state"
    expect:
      emergency_active: true

  - name: "Release Emergency"
    action: "release_emergency"
    expect:
      emergency_active: false

acceptance_criteria:
  - emergency_response_time: "<500ms"
  - complete_stop: true
```

## 3. 性能测试

### 3.1 视频流带宽测试

```yaml
# test/performance/bandwidth_test.yaml
name: "Video Bandwidth Test"
description: "测试不同分辨率下的带宽使用"

configurations:
  - name: "1080p@30fps"
    resolution: [1920, 1080]
    fps: 30
    bitrate: 4000000

  - name: "720p@30fps"
    resolution: [1280, 720]
    fps: 30
    bitrate: 2000000

  - name: "480p@30fps"
    resolution: [640, 480]
    fps: 30
    bitrate: 1000000

test_steps:
  - name: "Stream Video"
    duration_seconds: 60
    cameras: 5  # 5路摄像头

measurements:
  - total_bandwidth_mbps
  - cpu_usage_percent
  - frame_drop_rate
  - latency_ms

expected_results:
  1080p@30fps:
    bandwidth_mbps: "~20"
    cpu_usage: "<50%"
    frame_drop: "<1%"

  720p@30fps:
    bandwidth_mbps: "~10"
    cpu_usage: "<30%"
    frame_drop: "<0.5%"
```

### 3.2 最大并发连接测试

```yaml
# test/performance/concurrency_test.yaml
name: "Max Concurrency Test"
description: "测试云端服务器的最大并发车辆连接数"

test_steps:
  - name: "Ramp Up Connections"
    action: "connect_vehicles_gradually"
    start_count: 1
    end_count: 100
    step: 10
    step_duration_seconds: 30

  - name: "Measure Performance at Each Step"
    measurements:
      - response_time_ms
      - cpu_usage
      - memory_usage
      - connection_success_rate

  - name: "Find Breaking Point"
    criteria: "response_time > 1000ms OR success_rate < 95%"

expected_results:
  max_connections: ">50"
  avg_response_time: "<200ms at 50 vehicles"
```

## 4. 方向盘控制测试

### 4.1 罗技方向盘功能测试

```yaml
# test/wheel/logitech_test.yaml
name: "Logitech Wheel Function Test"
description: "测试罗技方向盘的基本功能"

supported_wheels:
  - "Logitech G29"
  - "Logitech G920"
  - "Logitech G27"

test_steps:
  - name: "Detect Wheel"
    action: "detect_wheel_devices"
    expect:
      devices_found: ">0"

  - name: "Connect Wheel"
    action: "open_wheel_device"
    expect:
      connected: true

  - name: "Test Steering"
    action: "rotate_wheel"
    positions: [-1.0, 0.0, 1.0]
    expect:
      input_received: true
      range: "full"

  - name: "Test Pedals"
    action: "press_pedals"
    pedals: ["throttle", "brake", "clutch"]
    expect:
      all_working: true

  - name: "Test Buttons"
    action: "press_buttons"
    buttons: ["emergency", "gear_up", "gear_down"]
    expect:
      all_working: true

  - name: "Test Force Feedback"
    action: "apply_force_feedback"
    effects: ["spring", "constant", "damper"]
    expect:
      feedback_felt: true
```

### 4.2 控制响应测试

```yaml
# test/wheel/control_response_test.yaml
name: "Control Response Test"
description: "测试方向盘输入到车辆响应的延迟"

test_steps:
  - name: "Setup Recording"
    action: "start_input_recording"

  - name: "Steering Test"
    action: "turn_wheel"
    direction: "left"
    angle: 90
    measure: "vehicle_steering_response_time"

  - name: "Throttle Test"
    action: "press_throttle"
    percentage: 50
    measure: "vehicle_acceleration_response_time"

  - name: "Emergency Stop Test"
    action: "press_emergency_button"
    measure: "emergency_stop_response_time"

acceptance_criteria:
  - steering_response: "<100ms"
  - throttle_response: "<100ms"
  - emergency_response: "<50ms"
```

## 5. 演示场景

### 5.1 单车远程控制演示

```yaml
# demo/single_vehicle_demo.yaml
name: "Single Vehicle Remote Control Demo"
description: "演示单车辆远程控制的完整流程"

duration: 10分钟

流程:
  1. 系统启动:
     - 启动Autoware仿真 (AWSIM)
     - 启动FSM车端节点
     - 启动云端服务
     - 打开前端界面

  2. 连接建立:
     - 观察WebRTC连接状态
     - 验证视频流显示
     - 验证遥测数据更新

  3. 远程控制:
     - 切换到远程控制模式
     - 使用方向盘控制车辆
     - 演示转向、加速、刹车
     - 演示转向灯操作

  4. 紧急场景:
     - 模拟网络抖动
     - 观察延迟显示变化
     - 演示紧急停车功能

  5. 系统监控:
     - 查看系统日志
     - 查看遥测数据
     - 查看CPU/内存使用

演示重点:
  - 视频流流畅度
  - 控制响应速度
  - 紧急停车可靠性
```

### 5.2 多车调度演示

```yaml
# demo/multi_vehicle_scheduling_demo.yaml
name: "Multi-Vehicle Scheduling Demo"
description: "演示多车辆调度系统"

duration: 15分钟

准备:
  - 3辆模拟车辆 (FSM-01, FSM-02, FSM-03)
  - 不同初始状态

流程:
  1. 车辆上线:
     - FSM-01: 正常状态, 近距离
     - FSM-02: 低电量, 中等距离
     - FSM-03: 紧急任务, 远距离

  2. 调度队列展示:
     - 观察初始排序
     - 解释优先级计算

  3. 状态变化:
     - FSM-01: 模拟网络延迟增加
     - 观察排序变化
     - FSM-03: 升级紧急级别
     - 观察排序变化

  4. 切换控制:
     - 从FSM-01切换到FSM-03
     - 演示快速切换流程
     - 验证上下文保持

  5. 算法切换:
     - 切换到"紧急优先"算法
     - 观察排序变化
     - 切换到"延迟优先"算法
     - 观察排序变化

演示重点:
  - 实时排序更新
  - 多车快速切换
  - 算法可配置性
```

## 6. 测试环境配置

### 6.1 仿真环境 (AWSIM)

```bash
# 启动AWSIM仿真器
cd ~/awsim
./awsim.x86_64

# 启动Autoware
cd ~/autoware
source install/setup.bash
ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:=~/maps/sample_map \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit
```

### 6.2 FSM车端节点

```bash
# 编译
cd ~/fsm/cpp
colcon build --packages-select fsm_pilot

# 运行
source install/setup.bash
ros2 run fsm_pilot fsm_vehicle_node config/vehicle_config.yaml
```

### 6.3 云端服务

```bash
# 运行云端服务
./fsm_cloud_server config/cloud_config.yaml
```

### 6.4 前端界面

```bash
# 开发模式
cd ~/fsm
npm run dev

# 生产模式
npm run build
npm run preview
```

## 7. 测试报告模板

```markdown
# FSM-Pilot 测试报告

## 测试信息
- 测试日期: YYYY-MM-DD
- 测试人员: [姓名]
- 版本号: v1.1.0
- 测试环境: [环境描述]

## 测试结果摘要

| 测试类别 | 通过 | 失败 | 跳过 | 通过率 |
|---------|------|------|------|--------|
| 单元测试 | XX   | XX   | XX   | XX%    |
| 集成测试 | XX   | XX   | XX   | XX%    |
| 性能测试 | XX   | XX   | XX   | XX%    |
| **总计** | XX   | XX   | XX   | XX%    |

## 关键指标

| 指标 | 目标值 | 实测值 | 状态 |
|-----|-------|-------|-----|
| 端到端延迟 | <200ms | XXms | ✓/✗ |
| 视频帧率 | 30fps | XXfps | ✓/✗ |
| 紧急停车响应 | <500ms | XXms | ✓/✗ |
| 最大并发 | >50车 | XX车 | ✓/✗ |

## 问题列表

### 严重问题
- [问题描述]

### 一般问题
- [问题描述]

### 建议改进
- [改进建议]

## 结论
[测试结论和建议]
```
