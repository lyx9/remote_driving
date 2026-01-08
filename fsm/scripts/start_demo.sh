#!/bin/bash
# FSM-Pilot Demo 环境启动脚本

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "╔════════════════════════════════════════╗"
echo "║     FSM-Pilot Demo 环境启动            ║"
echo "╚════════════════════════════════════════╝"
echo ""

# 清理旧进程
cleanup() {
    echo ""
    echo "正在停止服务..."
    kill $MOCK_PID $FRONTEND_PID 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

# 检查依赖
echo "[1/4] 检查依赖..."
if ! command -v node &> /dev/null; then
    echo "❌ 需要安装 Node.js"
    exit 1
fi
echo "✓ Node.js: $(node --version)"

if ! command -v npm &> /dev/null; then
    echo "❌ 需要安装 npm"
    exit 1
fi
echo "✓ npm: $(npm --version)"

# 安装 Mock 服务依赖
echo ""
echo "[2/4] 安装 Mock 服务依赖..."
cd "$PROJECT_DIR/mock"
if [ ! -d "node_modules" ]; then
    npm install
fi
echo "✓ Mock 依赖已安装"

# 启动 Mock 服务
echo ""
echo "[3/4] 启动 Mock 数据服务..."
node mock_server.js &
MOCK_PID=$!
sleep 2

# 检查 Mock 服务是否启动成功
if ! kill -0 $MOCK_PID 2>/dev/null; then
    echo "❌ Mock 服务启动失败"
    exit 1
fi
echo "✓ Mock 服务已启动 (PID: $MOCK_PID)"

# 启动前端
echo ""
echo "[4/4] 启动前端界面..."
cd "$PROJECT_DIR"
npm run dev &
FRONTEND_PID=$!
sleep 3

# 检查前端是否启动成功
if ! kill -0 $FRONTEND_PID 2>/dev/null; then
    echo "❌ 前端启动失败"
    kill $MOCK_PID 2>/dev/null
    exit 1
fi
echo "✓ 前端已启动 (PID: $FRONTEND_PID)"

# 输出信息
echo ""
echo "╔════════════════════════════════════════════════════════╗"
echo "║                 Demo 环境已就绪                         ║"
echo "╠════════════════════════════════════════════════════════╣"
echo "║                                                        ║"
echo "║  🌐 前端界面: http://localhost:5173                    ║"
echo "║  📡 Mock API: http://localhost:3001                    ║"
echo "║  🔌 WebSocket: ws://localhost:3002                     ║"
echo "║                                                        ║"
echo "║  🚗 模拟车辆:                                           ║"
echo "║     • FSM-01 (ROBO-TAXI) - 正常状态                    ║"
echo "║     • FSM-02 (LOGISTICS) - 低电量                      ║"
echo "║     • FSM-03 (SECURITY) - 紧急任务                     ║"
echo "║                                                        ║"
echo "║  📋 可用模拟命令:                                       ║"
echo "║     • ./scripts/simulate_network.sh delay              ║"
echo "║     • ./scripts/simulate_network.sh disconnect         ║"
echo "║     • ./scripts/simulate_emergency.sh FSM-01 trigger   ║"
echo "║                                                        ║"
echo "║  按 Ctrl+C 停止所有服务                                 ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

# 等待退出
wait
