#!/bin/bash
# FSM-Pilot 网络模拟脚本

ACTION=$1
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$ACTION" ]; then
    echo "FSM-Pilot 网络模拟工具"
    echo ""
    echo "用法: $0 <action>"
    echo ""
    echo "可用操作:"
    echo "  delay       模拟网络延迟 (200ms)"
    echo "  jitter      模拟网络抖动 (50-300ms)"
    echo "  disconnect  模拟网络断开 (5秒)"
    echo "  reset       恢复正常网络"
    echo ""
    echo "示例:"
    echo "  $0 delay"
    echo "  $0 reset"
    exit 0
fi

MOCK_URL="http://localhost:3001"

case $ACTION in
    "delay")
        echo "📡 模拟网络延迟 (200ms)..."
        curl -s -X POST "$MOCK_URL/mock/simulate" \
            -H "Content-Type: application/json" \
            -d '{"type": "delay", "value": 200}' | jq .
        echo "✓ 延迟模拟已启动"
        ;;
    "jitter")
        echo "📡 模拟网络抖动 (50-300ms)..."
        curl -s -X POST "$MOCK_URL/mock/simulate" \
            -H "Content-Type: application/json" \
            -d '{"type": "jitter", "min": 50, "max": 300}' | jq .
        echo "✓ 抖动模拟已启动"
        ;;
    "disconnect")
        echo "📡 模拟网络断开 (5秒)..."
        curl -s -X POST "$MOCK_URL/mock/simulate" \
            -H "Content-Type: application/json" \
            -d '{"type": "disconnect", "duration": 5000}' | jq .
        echo "✓ 网络将在5秒后自动恢复"
        ;;
    "reset")
        echo "📡 恢复正常网络..."
        curl -s -X POST "$MOCK_URL/mock/simulate" \
            -H "Content-Type: application/json" \
            -d '{"type": "reset"}' | jq .
        echo "✓ 网络已恢复正常"
        ;;
    *)
        echo "❌ 未知操作: $ACTION"
        echo "运行 '$0' 查看帮助"
        exit 1
        ;;
esac
