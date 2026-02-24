#!/bin/bash
# Guardian Mobility v0.0 - 一键启动脚本（简化版）
# 快速启动 RosBag Replay 演示

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}╔══════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║${NC}     Guardian Mobility v0.0 - RosBag Replay 一键启动           ${BLUE}║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# 1. 启动 RosBag 服务器
echo -e "${BLUE}▶${NC} 启动 RosBag 服务器..."
cd /home/lyx/fsm/python
./rosbag_server.sh start

echo ""

# 2. 检查前端服务器
echo -e "${BLUE}▶${NC} 检查前端服务器..."
if lsof -Pi :3000 -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo -e "${GREEN}✓${NC} 前端服务器已在运行"
else
    echo -e "${YELLOW}ℹ${NC} 前端服务器未运行，请手动启动: npm run dev"
fi

echo ""
echo -e "${BLUE}╔══════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║${NC}  系统已启动！                                                  ${BLUE}║${NC}"
echo -e "${BLUE}╚══════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo "访问地址:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "${GREEN}🎬 RosBag 回放:${NC}     http://localhost:3000/rosbag-replay-pro"
echo -e "${GREEN}🧪 WebSocket 测试:${NC}  http://localhost:3000/rosbag-test.html"
echo -e "${GREEN}📊 数据库:${NC}          http://localhost:3000/database"
echo ""
echo "管理命令:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  停止服务:  cd /home/lyx/fsm/python && ./rosbag_server.sh stop"
echo "  查看状态:  cd /home/lyx/fsm/python && ./rosbag_server.sh status"
echo "  查看日志:  cd /home/lyx/fsm/python && ./rosbag_server.sh logs"
echo ""
