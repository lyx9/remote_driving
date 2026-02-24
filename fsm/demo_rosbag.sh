#!/bin/bash

################################################################################
# FSM-Pilot V2.0 - RosBag Streaming Demo Script
#
# 功能:
# - 启动 RosBag 流式服务器 (端口 8765)
# - 启动前端开发服务器 (端口 3000)
# - 自动打开浏览器并导航到 RosBag Replay 页面
# - 支持大文件 (>15GB) 流式播放
#
# @author Li Yixiang
# @institution City University of Hong Kong
# @version 2.0
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'
BOLD='\033[1m'

clear

echo -e "${CYAN}"
cat << 'EOF'
╔════════════════════════════════════════════════════════════════╗
║                                                                ║
║       FSM-Pilot V2.0 - RosBag Streaming Demo                  ║
║                                                                ║
║       Large File Support (>15GB)                              ║
║                                                                ║
╚════════════════════════════════════════════════════════════════╝
EOF
echo -e "${NC}"

echo ""
echo -e "${BOLD}演示内容:${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo -e "  ${GREEN}✓${NC} 大文件 RosBag 流式播放 (>15GB)"
echo -e "  ${GREEN}✓${NC} 多摄像头实时显示"
echo -e "  ${GREEN}✓${NC} 点云数据可视化"
echo -e "  ${GREEN}✓${NC} 车辆 CAN 数据展示"
echo -e "  ${GREEN}✓${NC} Topic 选择和过滤"
echo ""

echo -e "${YELLOW}[1/5]${NC} 检查依赖..."
echo ""

# Check Node.js
if ! command -v node &> /dev/null; then
    echo -e "${RED}错误: 未找到 Node.js${NC}"
    exit 1
fi
echo -e "  ${GREEN}✓${NC} Node.js: $(node --version)"

# Check npm
if ! command -v npm &> /dev/null; then
    echo -e "${RED}错误: 未找到 npm${NC}"
    exit 1
fi
echo -e "  ${GREEN}✓${NC} npm: v$(npm --version)"

echo ""
echo -e "${YELLOW}[2/5]${NC} 检查 RosBag 文件..."
echo ""

ROSBAG_DIR="/home/lyx/fsm/rosbag/1210"
if [ ! -d "$ROSBAG_DIR" ]; then
    echo -e "${RED}错误: RosBag 目录不存在: $ROSBAG_DIR${NC}"
    exit 1
fi

BAG_COUNT=$(find "$ROSBAG_DIR" -name "*.db3" | wc -l)
echo -e "  ${GREEN}✓${NC} 找到 $BAG_COUNT 个 RosBag 文件"

echo ""
echo -e "${YELLOW}[3/5]${NC} 安装服务器依赖..."
echo ""

cd /home/lyx/fsm/server
if [ ! -d "node_modules" ]; then
    npm install > /dev/null 2>&1
fi
echo -e "  ${GREEN}✓${NC} 服务器依赖已安装"

echo ""
echo -e "${YELLOW}[4/5]${NC} 启动 RosBag 流式服务器..."
echo ""

# Kill existing server
lsof -ti:8765 | xargs kill -9 2>/dev/null || true

# Start RosBag server in background
npm run rosbag > /tmp/rosbag_server.log 2>&1 &
ROSBAG_PID=$!

# Wait for server
sleep 3
if ps -p $ROSBAG_PID > /dev/null; then
    echo -e "  ${GREEN}✓${NC} RosBag 服务器启动成功 (PID: $ROSBAG_PID)"
else
    echo -e "${RED}错误: RosBag 服务器启动失败${NC}"
    exit 1
fi

echo ""
echo -e "${YELLOW}[5/5]${NC} 启动前端服务器..."
echo ""

cd /home/lyx/fsm

# Kill existing frontend server
lsof -ti:3000 | xargs kill -9 2>/dev/null || true

# Start frontend server in background
npm run dev > /tmp/frontend_server.log 2>&1 &
FRONTEND_PID=$!

# Wait for frontend server
for i in {1..30}; do
    if curl -s "http://localhost:3000" > /dev/null 2>&1; then
        echo -e "  ${GREEN}✓${NC} 前端服务器启动成功 (PID: $FRONTEND_PID)"
        break
    fi
    sleep 1
done

echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}${BOLD}  演示步骤${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${BOLD}1. 登录系统${NC}"
echo -e "   用户名: ${GREEN}cityu${NC}"
echo -e "   密码: ${GREEN}2026${NC}"
echo ""
echo -e "${BOLD}2. 进入 RosBag Replay 页面${NC}"
echo -e "   点击导航栏: ${GREEN}📼 RosBag Replay${NC}"
echo ""
echo -e "${BOLD}3. 选择 RosBag 文件${NC}"
echo -e "   点击: ${GREEN}📁 Select RosBag${NC}"
echo -e "   选择一个 RosBag 文件"
echo ""
echo -e "${BOLD}4. 选择 Topics${NC}"
echo -e "   在左侧选择要播放的 topics"
echo -e "   推荐: 摄像头和车辆数据"
echo ""
echo -e "${BOLD}5. 开始播放${NC}"
echo -e "   点击: ${GREEN}▶️ Play${NC}"
echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# Open browser
BROWSER=""
if command -v google-chrome &> /dev/null; then
    BROWSER="google-chrome"
elif command -v chromium-browser &> /dev/null; then
    BROWSER="chromium-browser"
elif command -v firefox &> /dev/null; then
    BROWSER="firefox"
fi

if [ -n "$BROWSER" ]; then
    $BROWSER "http://localhost:3000/rosbag-replay-pro" > /dev/null 2>&1 &
fi

echo -e "${GREEN}${BOLD}  ✓ 服务器运行中${NC}"
echo -e "  ${CYAN}RosBag 服务器: ws://localhost:8765${NC}"
echo -e "  ${CYAN}前端服务器: http://localhost:3000${NC}"
echo -e "  ${CYAN}RosBag Replay: http://localhost:3000/rosbag-replay-pro${NC}"
echo ""
echo -e "${YELLOW}  按 Ctrl+C 停止服务器${NC}"
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo -e "${YELLOW}正在停止服务器...${NC}"
    kill $ROSBAG_PID 2>/dev/null || true
    kill $FRONTEND_PID 2>/dev/null || true
    echo -e "${GREEN}服务器已停止${NC}"
    exit 0
}

trap cleanup EXIT INT TERM

# Keep script running
wait
