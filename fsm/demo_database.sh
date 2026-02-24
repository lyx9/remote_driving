#!/bin/bash

################################################################################
# FSM-Pilot V2.0 - 完整一键演示脚本
#
# 功能:
# - 自动启动开发服务器 (端口 3000)
# - 打开浏览器并自动登录
# - 展示数据库可视化页面
# - 生成模拟数据
# - 支持导入导出功能演示
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
MAGENTA='\033[0;35m'
NC='\033[0m'
BOLD='\033[1m'

# 配置
DEMO_PORT=3000
DEMO_URL="http://localhost:${DEMO_PORT}"
LOGIN_USER="cityu"
LOGIN_PASS="2026"

clear

echo -e "${CYAN}"
cat << 'EOF'
╔════════════════════════════════════════════════════════════════╗
║                                                                ║
║          FSM-Pilot V2.0 - 数据库可视化一键演示                ║
║                                                                ║
║          Database Visualization One-Click Demo                ║
║                                                                ║
║          City University of Hong Kong                         ║
║                                                                ║
╚════════════════════════════════════════════════════════════════╝
EOF
echo -e "${NC}"

echo ""
echo -e "${BOLD}演示内容 / Demo Content:${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo -e "  ${GREEN}✓${NC} 自动驾驶场景数据库可视化"
echo -e "  ${GREEN}✓${NC} 车辆记录、接管事件、AI分析、操作员数据"
echo -e "  ${GREEN}✓${NC} 数据导入/导出功能 (JSON格式)"
echo -e "  ${GREEN}✓${NC} 模拟数据生成"
echo -e "  ${GREEN}✓${NC} 实时统计展示"
echo ""

echo -e "${YELLOW}[1/6]${NC} 检查系统环境..."
echo ""

# 检查 Node.js
if ! command -v node &> /dev/null; then
    echo -e "${RED}错误: 未找到 Node.js${NC}"
    echo "请安装 Node.js v18+ : https://nodejs.org"
    exit 1
fi

NODE_VERSION=$(node --version)
echo -e "  ${GREEN}✓${NC} Node.js: $NODE_VERSION"

# 检查 npm
if ! command -v npm &> /dev/null; then
    echo -e "${RED}错误: 未找到 npm${NC}"
    exit 1
fi

NPM_VERSION=$(npm --version)
echo -e "  ${GREEN}✓${NC} npm: v$NPM_VERSION"

echo ""
echo -e "${YELLOW}[2/6]${NC} 检查端口 $DEMO_PORT..."

# 检查端口
if lsof -Pi :$DEMO_PORT -sTCP:LISTEN -t >/dev/null 2>&1 ; then
    echo -e "${YELLOW}  端口 $DEMO_PORT 已被占用，正在释放...${NC}"
    lsof -ti:$DEMO_PORT | xargs kill -9 2>/dev/null || true
    sleep 2
    echo -e "  ${GREEN}✓${NC} 端口已释放"
else
    echo -e "  ${GREEN}✓${NC} 端口 $DEMO_PORT 可用"
fi

echo ""
echo -e "${YELLOW}[3/6]${NC} 检查项目依赖..."

if [ ! -d "node_modules" ]; then
    echo "  正在安装依赖..."
    npm install > /dev/null 2>&1
    echo -e "  ${GREEN}✓${NC} 依赖安装完成"
else
    echo -e "  ${GREEN}✓${NC} 依赖已安装"
fi

echo ""
echo -e "${YELLOW}[4/6]${NC} 启动开发服务器..."
echo ""

# 启动服务器（后台）
npm run dev > /tmp/fsm_server.log 2>&1 &
SERVER_PID=$!

# 等待服务器启动
echo -e "  等待服务器启动"
for i in {1..30}; do
    if curl -s "http://localhost:$DEMO_PORT" > /dev/null 2>&1; then
        echo -e "  ${GREEN}✓${NC} 服务器启动成功 (PID: $SERVER_PID)"
        break
    fi
    echo -ne "  ."
    sleep 1
done

if ! curl -s "http://localhost:$DEMO_PORT" > /dev/null 2>&1; then
    echo -e "\n${RED}错误: 服务器启动失败${NC}"
    kill $SERVER_PID 2>/dev/null || true
    exit 1
fi

echo ""
echo ""
echo -e "${YELLOW}[5/6]${NC} 打开浏览器..."
echo ""

# 检测浏览器
BROWSER=""
if command -v google-chrome &> /dev/null; then
    BROWSER="google-chrome"
elif command -v chromium-browser &> /dev/null; then
    BROWSER="chromium-browser"
elif command -v firefox &> /dev/null; then
    BROWSER="firefox"
elif command -v open &> /dev/null; then
    BROWSER="open"  # macOS
else
    echo -e "${YELLOW}  未检测到浏览器，请手动打开: $DEMO_URL${NC}"
fi

if [ -n "$BROWSER" ]; then
    echo -e "  使用浏览器: $BROWSER"
    $BROWSER "$DEMO_URL" > /dev/null 2>&1 &
    echo -e "  ${GREEN}✓${NC} 浏览器已打开"
fi

echo ""
echo -e "${YELLOW}[6/6]${NC} 准备演示..."
echo ""

# 演示说明
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}${BOLD}  演示步骤 / Demo Steps${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${BOLD}步骤 1: 登录系统${NC}"
echo -e "  • 访问地址: ${CYAN}$DEMO_URL${NC}"
echo -e "  • 用户名: ${GREEN}${BOLD}$LOGIN_USER${NC}"
echo -e "  • 密码: ${GREEN}${BOLD}$LOGIN_PASS${NC}"
echo ""

echo -e "${BOLD}步骤 2: 进入数据库可视化页面${NC}"
echo -e "  • 点击顶部导航栏的 ${GREEN}🗄️ Database${NC}"
echo -e "  • 或直接访问: ${CYAN}$DEMO_URL/database-visualization${NC}"
echo ""

echo -e "${BOLD}步骤 3: 生成模拟数据${NC}"
echo -e "  • 点击 ${GREEN}「生成模拟数据」${NC} 按钮"
echo -e "  • 系统将生成:"
echo -e "    - 10 条车辆记录"
echo -e "    - 5 个接管事件"
echo -e "    - 3 个AI分析"
echo -e "    - 3 个操作员"
echo ""

echo -e "${BOLD}步骤 4: 浏览数据${NC}"
echo -e "  • 切换标签页查看不同数据:"
echo -e "    - ${CYAN}🚗 车辆记录${NC} - 车辆状态、位置、风险评分"
echo -e "    - ${CYAN}🎮 接管事件${NC} - 接管记录、持续时间、结果"
echo -e "    - ${CYAN}🤖 AI分析${NC} - AI场景分析、风险因素、建议"
echo -e "    - ${CYAN}👥 操作员${NC} - 操作员状态、成功率、响应时间"
echo ""

echo -e "${BOLD}步骤 5: 导出数据${NC}"
echo -e "  • 点击 ${GREEN}「导出数据」${NC} 按钮"
echo -e "  • 下载 JSON 格式的数据文件"
echo -e "  • 文件包含所有数据库记录和统计信息"
echo ""

echo -e "${BOLD}步骤 6: 导入数据${NC}"
echo -e "  • 点击 ${GREEN}「导入数据」${NC} 按钮"
echo -e "  • 选择之前导出的 JSON 文件"
echo -e "  • 确认导入后数据将自动加载"
echo ""

echo -e "${BOLD}步骤 7: 查看统计卡片${NC}"
echo -e "  • 顶部显示实时统计:"
echo -e "    - 车辆记录总数"
echo -e "    - 接管事件总数"
echo -e "    - AI分析总数"
echo -e "    - 操作员总数"
echo ""

echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}${BOLD}  主要功能展示 / Key Features${NC}"
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "  ${MAGENTA}🗄️  IndexedDB 数据存储${NC}"
echo -e "      浏览器本地数据库，数据持久化保存"
echo ""
echo -e "  ${MAGENTA}📊  多维度数据展示${NC}"
echo -e "      车辆、接管、AI分析、操作员 四大数据表"
echo ""
echo -e "  ${MAGENTA}📤  数据导出功能${NC}"
echo -e "      一键导出为 JSON 格式，便于分享和备份"
echo ""
echo -e "  ${MAGENTA}📥  数据导入功能${NC}"
echo -e "      支持导入 JSON 数据文件，快速恢复数据"
echo ""
echo -e "  ${MAGENTA}🎲  模拟数据生成${NC}"
echo -e "      快速生成测试数据，演示系统功能"
echo ""
echo -e "  ${MAGENTA}🔍  实时搜索过滤${NC}"
echo -e "      表格数据支持实时搜索和筛选"
echo ""
echo -e "  ${MAGENTA}📈  统计数据可视化${NC}"
echo -e "      顶部卡片展示关键指标"
echo ""

echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

echo -e "${GREEN}${BOLD}  ✓ 服务器正在运行于: $DEMO_URL${NC}"
echo -e "${GREEN}${BOLD}  ✓ 数据库可视化: $DEMO_URL/database-visualization${NC}"
echo ""
echo -e "${YELLOW}  提示: 按 Ctrl+C 停止服务器${NC}"
echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# 清理函数
cleanup() {
    echo ""
    echo -e "${YELLOW}正在停止服务器...${NC}"
    kill $SERVER_PID 2>/dev/null || true
    echo -e "${GREEN}服务器已停止${NC}"
    echo ""
    echo -e "${CYAN}感谢使用 FSM-Pilot V2.0！${NC}"
    echo ""
    exit 0
}

trap cleanup EXIT INT TERM

# 保持脚本运行
wait $SERVER_PID
