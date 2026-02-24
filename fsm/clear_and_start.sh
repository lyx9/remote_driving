#!/bin/bash

################################################################################
# FSM-Pilot V2.0 - 清除缓存并重新启动
#
# 用于清除浏览器缓存、localStorage 和账户锁定状态
# 然后在 localhost:3000 启动完整演示
#
# @author Li Yixiang
# @institution City University of Hong Kong
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
║         FSM-Pilot V2.0 - 清除缓存并启动演示                   ║
║                                                                ║
║         Clear Cache & Start Demo                              ║
║                                                                ║
╚════════════════════════════════════════════════════════════════╝
EOF
echo -e "${NC}"

echo ""
echo -e "${BOLD}准备工作 / Preparation:${NC}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Step 1: 检查端口
echo -e "${YELLOW}[1/6]${NC} 检查端口 3000..."
if lsof -Pi :3000 -sTCP:LISTEN -t >/dev/null 2>&1 ; then
    echo -e "${YELLOW}      ⚠ 端口 3000 已被占用，正在释放...${NC}"
    lsof -ti:3000 | xargs kill -9 2>/dev/null || true
    sleep 2
    echo -e "${GREEN}      ✓ 端口已释放${NC}"
else
    echo -e "${GREEN}      ✓ 端口 3000 可用${NC}"
fi
echo ""

# Step 2: 清理旧的构建产物
echo -e "${YELLOW}[2/6]${NC} 清理旧的构建产物..."
if [ -d "dist" ]; then
    rm -rf dist
    echo -e "${GREEN}      ✓ dist 目录已清除${NC}"
else
    echo -e "${GREEN}      ✓ 无需清理${NC}"
fi
echo ""

# Step 3: 检查依赖
echo -e "${YELLOW}[3/6]${NC} 检查项目依赖..."
if [ ! -d "node_modules" ]; then
    echo -e "${YELLOW}      正在安装依赖...${NC}"
    npm install
    echo -e "${GREEN}      ✓ 依赖安装完成${NC}"
else
    echo -e "${GREEN}      ✓ 依赖已安装${NC}"
fi
echo ""

# Step 4: 重新构建项目
echo -e "${YELLOW}[4/6]${NC} 重新构建项目..."
npm run build > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo -e "${GREEN}      ✓ 构建成功${NC}"
else
    echo -e "${RED}      ✗ 构建失败${NC}"
    exit 1
fi
echo ""

# Step 5: 显示清除缓存说明
echo -e "${YELLOW}[5/6]${NC} 清除浏览器缓存说明..."
echo ""
echo -e "${CYAN}      【重要】请在浏览器中执行以下操作：${NC}"
echo ""
echo -e "      1. 打开浏览器开发者工具 (按 ${BOLD}F12${NC})"
echo ""
echo -e "      2. 进入 ${BOLD}Application${NC} 标签页 (Chrome/Edge)"
echo -e "         或 ${BOLD}Storage${NC} 标签页 (Firefox)"
echo ""
echo -e "      3. 左侧找到 ${BOLD}Local Storage${NC}"
echo ""
echo -e "      4. 点击 ${BOLD}http://localhost:3000${NC}"
echo ""
echo -e "      5. 删除以下项目（如果存在）："
echo -e "         • ${BOLD}fsm_pilot_login_attempts${NC}  (登录尝试记录)"
echo -e "         • ${BOLD}fsm_pilot_session${NC}         (会话信息)"
echo ""
echo -e "      6. 或者直接点击 ${BOLD}Clear All${NC} 清除所有存储"
echo ""
echo -e "      7. 刷新页面 (${BOLD}Ctrl + Shift + R${NC})"
echo ""

read -p "      按 Enter 继续启动服务器..."
echo ""

# Step 6: 启动开发服务器
echo -e "${YELLOW}[6/6]${NC} 启动开发服务器 (端口 3000)..."
echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}${BOLD}  服务器启动中...${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${CYAN}  访问地址: ${BOLD}http://localhost:3000${NC}"
echo ""
echo -e "${CYAN}  登录凭据:${NC}"
echo -e "    用户名: ${BOLD}cityu${NC}"
echo -e "    密码:   ${BOLD}2026${NC}"
echo ""
echo -e "${YELLOW}  提示: 按 Ctrl+C 停止服务器${NC}"
echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# 启动开发服务器
npm run dev

# 清理函数
cleanup() {
    echo ""
    echo -e "${YELLOW}正在停止服务器...${NC}"
    exit 0
}

trap cleanup EXIT INT TERM
