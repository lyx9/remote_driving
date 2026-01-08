#!/bin/bash
# FSM-Pilot 停止所有服务脚本

echo "╔════════════════════════════════════════╗"
echo "║     FSM-Pilot 服务停止                 ║"
echo "╚════════════════════════════════════════╝"
echo ""

# 停止进程函数
stop_process() {
    local name=$1
    local pattern=$2

    if pgrep -f "$pattern" > /dev/null 2>&1; then
        echo "正在停止 $name..."
        pkill -f "$pattern" 2>/dev/null
        sleep 1
        if pgrep -f "$pattern" > /dev/null 2>&1; then
            echo "  强制停止 $name..."
            pkill -9 -f "$pattern" 2>/dev/null
        fi
        echo "  ✓ $name 已停止"
    else
        echo "  - $name 未运行"
    fi
}

# 停止各服务
stop_process "Mock 服务" "mock_server.js"
stop_process "前端开发服务器" "vite"
stop_process "云端服务" "fsm_cloud_server"
stop_process "车端节点" "fsm_vehicle_node"
stop_process "操作端" "fsm_operator_client"

echo ""
echo "✓ 所有 FSM-Pilot 服务已停止"
