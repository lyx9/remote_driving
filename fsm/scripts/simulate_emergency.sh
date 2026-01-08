#!/bin/bash
# FSM-Pilot 紧急状态模拟脚本

VEHICLE_ID=${1:-"FSM-01"}
ACTION=${2:-"trigger"}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    echo "FSM-Pilot 紧急状态模拟工具"
    echo ""
    echo "用法: $0 <vehicle_id> <action>"
    echo ""
    echo "参数:"
    echo "  vehicle_id   车辆ID (默认: FSM-01)"
    echo "  action       操作: trigger 或 release (默认: trigger)"
    echo ""
    echo "示例:"
    echo "  $0 FSM-01 trigger    # 触发 FSM-01 紧急停车"
    echo "  $0 FSM-01 release    # 解除 FSM-01 紧急状态"
    echo "  $0 FSM-02 trigger    # 触发 FSM-02 紧急停车"
    exit 0
fi

MOCK_URL="http://localhost:3001"

case $ACTION in
    "trigger")
        echo "🚨 触发 $VEHICLE_ID 紧急状态..."
        curl -s -X POST "$MOCK_URL/mock/emergency" \
            -H "Content-Type: application/json" \
            -d "{\"vehicle_id\": \"$VEHICLE_ID\", \"action\": \"trigger\"}" | jq .
        echo ""
        echo "⚠️  紧急停车已触发!"
        echo "   车辆: $VEHICLE_ID"
        echo "   状态: 已停止"
        ;;
    "release")
        echo "✅ 解除 $VEHICLE_ID 紧急状态..."
        curl -s -X POST "$MOCK_URL/mock/emergency" \
            -H "Content-Type: application/json" \
            -d "{\"vehicle_id\": \"$VEHICLE_ID\", \"action\": \"release\"}" | jq .
        echo ""
        echo "✓ 紧急状态已解除"
        echo "   车辆: $VEHICLE_ID"
        echo "   状态: 正常"
        ;;
    *)
        echo "❌ 未知操作: $ACTION"
        echo "可用操作: trigger, release"
        exit 1
        ;;
esac
