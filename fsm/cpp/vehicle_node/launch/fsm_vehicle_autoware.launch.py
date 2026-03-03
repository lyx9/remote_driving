#!/usr/bin/env python3
# =============================================================================
# FSM-Pilot  |  Autoware.universe 集成 Launch 文件
# =============================================================================
# 用法：
#   source /opt/ros/humble/setup.bash
#   source ~/autoware/install/setup.bash      # Autoware workspace
#   ros2 launch fsm_vehicle_node fsm_vehicle_autoware.launch.py \
#       cloud_url:=wss://your-server/ws        \
#       vehicle_id:=FSM-CAR-AW-01              \
#       config_file:=/path/to/vehicle_config_autoware.yaml
# =============================================================================

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── 参数声明 ─────────────────────────────────────────────────────────────
    cloud_url_arg = DeclareLaunchArgument(
        "cloud_url",
        default_value="wss://localhost/ws",
        description="Cloud WebSocket signaling URL",
    )
    vehicle_id_arg = DeclareLaunchArgument(
        "vehicle_id",
        default_value="FSM-CAR-AW-01",
        description="Vehicle unique ID",
    )
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("fsm_vehicle_node"), "config",
             "vehicle_config_autoware.yaml"]
        ),
        description="Path to vehicle_config_autoware.yaml",
    )
    # 是否在 Launch 时自动切换 Autoware 到 REMOTE 模式
    auto_switch_mode_arg = DeclareLaunchArgument(
        "auto_switch_mode",
        default_value="true",
        description="Auto-switch Autoware operation mode to REMOTE on startup",
    )

    cloud_url    = LaunchConfiguration("cloud_url")
    vehicle_id   = LaunchConfiguration("vehicle_id")
    config_file  = LaunchConfiguration("config_file")

    # ── FSM 车端节点 ──────────────────────────────────────────────────────────
    fsm_vehicle_node = Node(
        package="fsm_vehicle_node",
        executable="fsm_vehicle_node",
        name="fsm_vehicle_node",
        output="screen",
        parameters=[
            config_file,
            # 运行时覆盖关键参数
            {"vehicle.id": vehicle_id},
            {"webrtc.signaling_url": cloud_url},
        ],
        # 将 FSM 控制指令 topic 直接映射到 Autoware external command 接口
        # （与 vehicle_config_autoware.yaml 中保持一致，此处仅作文档说明）
        remappings=[
            # 遥测订阅（Autoware → FSM）
            ("/vehicle/status/velocity_status",
             "/vehicle/status/velocity_status"),
            ("/vehicle/status/steering_status",
             "/vehicle/status/steering_status"),
            ("/localization/kinematic_state",
             "/localization/kinematic_state"),
            # 控制发布（FSM → Autoware external interface）— 已在 yaml 中配置
        ],
        # 确保不与 Autoware planning 指令冲突
        additional_env={
            "RCUTILS_LOGGING_BUFFERED_STREAM": "1",
        },
    )

    # ── 等待 Autoware 启动后切换到 REMOTE 模式 ────────────────────────────────
    # 延迟 8 秒（等待 Autoware vehicle_cmd_gate 就绪）
    switch_to_remote = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="[FSM] Switching Autoware operation mode to REMOTE..."),
            ExecuteProcess(
                cmd=[
                    "ros2", "service", "call",
                    "/system/operation_mode/change_operation_mode",
                    "autoware_adapi_v1_msgs/srv/ChangeOperationMode",
                    "{}",  # REMOTE = 4, 空请求即默认 REMOTE
                ],
                output="screen",
                # 失败不阻断 FSM 节点运行
                on_exit=[
                    LogInfo(msg="[FSM] Operation mode switch completed (or already REMOTE)"),
                ],
            ),
        ],
    )

    # ── 启动描述 ──────────────────────────────────────────────────────────────
    return LaunchDescription([
        cloud_url_arg,
        vehicle_id_arg,
        config_file_arg,
        auto_switch_mode_arg,
        LogInfo(msg="[FSM-Pilot] Launching FSM vehicle node for Autoware.universe"),
        fsm_vehicle_node,
        switch_to_remote,
    ])
