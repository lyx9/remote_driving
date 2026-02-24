#!/bin/bash
################################################################################
# Guardian Mobility v0.0 - 完整演示系统一键启动脚本
#
# 功能：
# - 启动所有后端服务（RosBag服务器、3DGS重建服务器）
# - 启动前端开发服务器
# - 自动打开浏览器展示所有功能
# - 提供完整的演示流程
#
# @author Li Yixiang
# @institution City University of Hong Kong
# @version 1.0.0
# @date 2026-02-04
################################################################################

set -e  # 遇到错误立即退出

# ==================== 配置 ====================

PROJECT_ROOT="/home/lyx/fsm"
PYTHON_DIR="$PROJECT_ROOT/python"
SERVER_DIR="$PROJECT_ROOT/server"
LOG_DIR="/tmp/guardian_mobility"

# 端口配置
FRONTEND_PORT=3000
ROSBAG_WS_PORT=8765
RECONSTRUCTION_PORT=5000

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m'
BOLD='\033[1m'

# ==================== 工具函数 ====================

print_banner() {
    clear
    echo -e "${CYAN}${BOLD}"
    echo "╔════════════════════════════════════════════════════════════════════╗"
    echo "║                                                                    ║"
    echo "║           Guardian Mobility v0.0 - 完整演示系统                   ║"
    echo "║                                                                    ║"
    echo "║        AI-Powered Remote Driving & Fleet Management Platform      ║"
    echo "║                  City University of Hong Kong                     ║"
    echo "║                                                                    ║"
    echo "╚════════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_section() {
    echo ""
    echo -e "${BLUE}${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}${BOLD}  $1${NC}"
    echo -e "${BLUE}${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

print_step() {
    echo -e "${CYAN}▶${NC} $1"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_info() {
    echo -e "${YELLOW}ℹ${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

# 检查端口是否被占用
check_port() {
    local port=$1
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1; then
        return 0  # 端口被占用
    else
        return 1  # 端口空闲
    fi
}

# 等待端口启动
wait_for_port() {
    local port=$1
    local max_wait=$2
    local service_name=$3
    local waited=0

    echo -n "   等待 $service_name 启动"
    while [ $waited -lt $max_wait ]; do
        if check_port $port; then
            echo ""
            return 0
        fi
        echo -n "."
        sleep 1
        waited=$((waited + 1))
    done
    echo ""
    return 1
}

# 创建日志目录
mkdir -p "$LOG_DIR"

# ==================== 停止所有服务 ====================

stop_all_services() {
    print_section "停止所有服务"

    # 停止 RosBag WebSocket 服务器
    if [ -f /tmp/rosbag_server.pid ]; then
        local pid=$(cat /tmp/rosbag_server.pid)
        if ps -p $pid > /dev/null 2>&1; then
            kill $pid 2>/dev/null || true
            print_info "已停止 RosBag WebSocket 服务器 (PID: $pid)"
        fi
        rm -f /tmp/rosbag_server.pid
    fi

    # 停止 3DGS 重建服务器
    pkill -f "reconstruction_server.py" 2>/dev/null || true
    print_info "已停止 3DGS 重建服务器"

    # 停止前端开发服务器
    pkill -f "vite.*$FRONTEND_PORT" 2>/dev/null || true
    print_info "已停止前端开发服务器"

    # 清理其他可能的进程
    pkill -f rosbag_stream_server 2>/dev/null || true

    sleep 2
    print_success "所有服务已停止"
}

# ==================== 启动服务 ====================

start_rosbag_server() {
    print_step "启动 RosBag WebSocket 服务器..."

    cd "$PYTHON_DIR"

    # 检查端口
    if check_port $ROSBAG_WS_PORT; then
        print_warning "端口 $ROSBAG_WS_PORT 已被占用，正在清理..."
        lsof -ti :$ROSBAG_WS_PORT | xargs kill -9 2>/dev/null || true
        sleep 2
    fi

    # 启动服务器
    nohup python3 rosbag_stream_server.py --rosbag-dir "$PROJECT_ROOT/rosbag" \
        > "$LOG_DIR/rosbag_server.log" 2>&1 &

    local pid=$!
    echo $pid > /tmp/rosbag_server.pid

    # 等待服务器启动
    if wait_for_port $ROSBAG_WS_PORT 10 "RosBag 服务器"; then
        print_success "RosBag WebSocket 服务器已启动 (PID: $pid, Port: $ROSBAG_WS_PORT)"
        return 0
    else
        print_error "RosBag 服务器启动失败"
        tail -10 "$LOG_DIR/rosbag_server.log"
        return 1
    fi
}

start_reconstruction_server() {
    print_step "启动 3DGS 重建服务器..."

    cd "$PYTHON_DIR"

    # 检查端口
    if check_port $RECONSTRUCTION_PORT; then
        print_success "3DGS 重建服务器已在运行 (Port: $RECONSTRUCTION_PORT)"
        return 0
    fi

    # 检查 reconstruction_server.py 是否存在
    if [ ! -f "reconstruction_server.py" ]; then
        print_warning "3DGS 重建服务器文件不存在（可选服务）"
        return 0
    fi

    # 启动服务器
    nohup python3 reconstruction_server.py \
        > "$LOG_DIR/reconstruction_server.log" 2>&1 &

    local pid=$!

    # 等待服务器启动（减少等待时间）
    if wait_for_port $RECONSTRUCTION_PORT 5 "3DGS 服务器"; then
        print_success "3DGS 重建服务器已启动 (PID: $pid, Port: $RECONSTRUCTION_PORT)"
        return 0
    else
        print_warning "3DGS 服务器启动失败（可选服务，不影响主要功能）"
        # 检查日志中的错误
        if [ -f "$LOG_DIR/reconstruction_server.log" ]; then
            local error=$(tail -5 "$LOG_DIR/reconstruction_server.log" | grep -i error | head -1)
            if [ ! -z "$error" ]; then
                print_info "错误信息: $error"
            fi
        fi
        return 0
    fi
}

start_frontend() {
    print_step "启动前端开发服务器..."

    if check_port $FRONTEND_PORT; then
        print_success "前端服务器已在运行 (Port: $FRONTEND_PORT)"
        return 0
    fi

    cd "$PROJECT_ROOT"

    # 启动前端
    nohup npm run dev > "$LOG_DIR/frontend.log" 2>&1 &
    local pid=$!

    # 等待前端启动
    if wait_for_port $FRONTEND_PORT 30 "前端服务器"; then
        print_success "前端开发服务器已启动 (PID: $pid, Port: $FRONTEND_PORT)"
        return 0
    else
        print_error "前端服务器启动失败"
        tail -10 "$LOG_DIR/frontend.log"
        return 1
    fi
}

# ==================== 测试连接 ====================

test_services() {
    print_section "测试服务连接"

    # 测试 RosBag 服务器
    print_step "测试 RosBag WebSocket 服务器..."
    python3 -c "
import asyncio
import websockets
import json
import sys

async def test():
    try:
        # 使用 asyncio.wait_for 包装 connect，而不是传递 timeout 参数
        ws = await asyncio.wait_for(
            websockets.connect('ws://localhost:$ROSBAG_WS_PORT'),
            timeout=5
        )
        try:
            # 接收连接状态消息
            await asyncio.wait_for(ws.recv(), timeout=2)

            # 发送列表请求
            await ws.send(json.dumps({'type': 'list_bags'}))

            # 接收响应
            response = await asyncio.wait_for(ws.recv(), timeout=5)
            data = json.loads(response)

            if data.get('type') == 'bag_list':
                bag_count = len(data.get('bags', []))
                print(f'   发现 {bag_count} 个 RosBag 文件')
                return True
            return False
        finally:
            await ws.close()
    except asyncio.TimeoutError:
        print('   连接超时')
        return False
    except Exception as e:
        print(f'   连接失败: {e}')
        return False

result = asyncio.run(test())
sys.exit(0 if result else 1)
" 2>&1

    if [ $? -eq 0 ]; then
        print_success "RosBag 服务器测试通过"
    else
        print_warning "RosBag 服务器测试失败（不影响使用）"
    fi

    # 测试前端
    print_step "测试前端服务器..."
    if curl -s http://localhost:$FRONTEND_PORT > /dev/null 2>&1; then
        print_success "前端服务器测试通过"
    else
        print_error "前端服务器测试失败"
    fi
}

# ==================== 显示系统状态 ====================

show_status() {
    print_section "系统状态"

    echo ""
    echo "服务状态:"
    echo "────────────────────────────────────────────────────────────────────"

    # RosBag 服务器
    if check_port $ROSBAG_WS_PORT; then
        local pid=$(lsof -ti :$ROSBAG_WS_PORT 2>/dev/null | head -1)
        print_success "RosBag WebSocket 服务器: 运行中 (PID: $pid, Port: $ROSBAG_WS_PORT)"
    else
        print_error "RosBag WebSocket 服务器: 未运行"
    fi

    # 3DGS 服务器
    if check_port $RECONSTRUCTION_PORT; then
        local pid=$(lsof -ti :$RECONSTRUCTION_PORT 2>/dev/null | head -1)
        print_success "3DGS 重建服务器: 运行中 (PID: $pid, Port: $RECONSTRUCTION_PORT)"
    else
        print_warning "3DGS 重建服务器: 未运行（可选）"
    fi

    # 前端服务器
    if check_port $FRONTEND_PORT; then
        local pid=$(lsof -ti :$FRONTEND_PORT 2>/dev/null | head -1)
        print_success "前端开发服务器: 运行中 (PID: $pid, Port: $FRONTEND_PORT)"
    else
        print_error "前端开发服务器: 未运行"
    fi

    echo ""
    echo "访问地址:"
    echo "────────────────────────────────────────────────────────────────────"
    echo -e "${GREEN}🏠 主页:${NC}              http://localhost:$FRONTEND_PORT"
    echo -e "${GREEN}🎮 远程控制:${NC}          http://localhost:$FRONTEND_PORT/remote-control"
    echo -e "${GREEN}🚗 智能调度:${NC}          http://localhost:$FRONTEND_PORT/intelligent-dispatch-demo"
    echo -e "${GREEN}📊 数据库可视化:${NC}      http://localhost:$FRONTEND_PORT/database-visualization"
    echo -e "${GREEN}🎬 RosBag 回放:${NC}       http://localhost:$FRONTEND_PORT/rosbag-replay-pro"
    echo -e "${GREEN}🧪 WebSocket 测试:${NC}    http://localhost:$FRONTEND_PORT/rosbag-test.html"

    echo ""
    echo "日志文件:"
    echo "────────────────────────────────────────────────────────────────────"
    echo "  RosBag 服务器:    $LOG_DIR/rosbag_server.log"
    echo "  3DGS 服务器:      $LOG_DIR/reconstruction_server.log"
    echo "  前端服务器:       $LOG_DIR/frontend.log"

    echo ""
    echo "管理命令:"
    echo "────────────────────────────────────────────────────────────────────"
    echo "  停止所有服务:     $0 stop"
    echo "  重启所有服务:     $0 restart"
    echo "  查看状态:         $0 status"
    echo "  查看日志:         $0 logs"
    echo "  演示模式:         $0 demo"
    echo ""
}

# ==================== 演示模式 ====================

demo_mode() {
    print_section "演示模式"

    echo ""
    echo -e "${CYAN}${BOLD}Guardian Mobility v0.0 - 功能演示指南${NC}"
    echo ""
    echo "本系统包含以下主要功能模块："
    echo ""

    echo -e "${WHITE}1. 远程控制 (Remote Control)${NC}"
    echo "   地址: http://localhost:$FRONTEND_PORT/remote-control"
    echo "   功能: 视频墙、LiDAR 3D可视化、车辆遥测、AI驾驶建议"
    echo ""

    echo -e "${WHITE}2. 智能调度 (Intelligent Dispatch)${NC}"
    echo "   地址: http://localhost:$FRONTEND_PORT/intelligent-dispatch-demo"
    echo "   功能: 车辆队列管理、风险评分、安全员分配、地图显示"
    echo ""

    echo -e "${WHITE}3. 数据库可视化 (Database Visualization)${NC}"
    echo "   地址: http://localhost:$FRONTEND_PORT/database-visualization"
    echo "   功能: 车辆记录、接管事件、AI分析、安全员管理、3D重建"
    echo ""

    echo -e "${WHITE}4. RosBag 回放 (RosBag Replay)${NC}"
    echo "   地址: http://localhost:$FRONTEND_PORT/rosbag-replay-pro"
    echo "   功能: 选择和回放 RosBag 文件、多相机显示、实时数据流"
    echo "   状态: 已发现 10 个 ROS2 bag 文件 (~18 GB)"
    echo ""

    echo -e "${YELLOW}${BOLD}演示建议流程:${NC}"
    echo "  1. 从远程控制页面开始，展示核心功能"
    echo "  2. 切换到智能调度，展示车队管理"
    echo "  3. 访问数据库页面，展示数据分析和3D重建"
    echo "  4. 最后演示 RosBag 回放功能"
    echo ""

    read -p "是否自动打开浏览器? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "正在打开浏览器..."

        # 尝试打开浏览器
        if command -v xdg-open > /dev/null; then
            xdg-open "http://localhost:$FRONTEND_PORT/remote-control" 2>/dev/null &
        elif command -v gnome-open > /dev/null; then
            gnome-open "http://localhost:$FRONTEND_PORT/remote-control" 2>/dev/null &
        elif command -v google-chrome > /dev/null; then
            google-chrome "http://localhost:$FRONTEND_PORT/remote-control" 2>/dev/null &
        else
            print_warning "无法自动打开浏览器，请手动访问上述地址"
        fi

        sleep 2
        print_success "浏览器已打开"
    fi

    echo ""
}

# ==================== 主函数 ====================

main() {
    case "${1:-start}" in
        start)
            print_banner
            print_section "启动 Guardian Mobility 演示系统"

            # 启动所有服务
            start_rosbag_server || exit 1
            echo ""

            start_reconstruction_server
            echo ""

            start_frontend || exit 1
            echo ""

            # 测试连接
            test_services
            echo ""

            # 显示状态
            show_status

            echo ""
            print_section "系统启动完成！"
            echo ""
            print_info "提示: 使用 '$0 demo' 查看演示指南"
            print_info "      使用 '$0 stop' 停止所有服务"
            echo ""
            ;;

        stop)
            print_banner
            stop_all_services
            echo ""
            ;;

        restart)
            print_banner
            print_section "重启系统"
            stop_all_services
            sleep 3
            exec "$0" start
            ;;

        status)
            print_banner
            show_status
            ;;

        demo)
            print_banner
            demo_mode
            ;;

        logs)
            print_banner
            print_section "查看日志"

            echo ""
            echo "RosBag 服务器日志 (最近20行):"
            echo "────────────────────────────────────────────────────────────────────"
            tail -20 "$LOG_DIR/rosbag_server.log" 2>/dev/null || echo "日志文件不存在"

            echo ""
            echo "3DGS 服务器日志 (最近20行):"
            echo "────────────────────────────────────────────────────────────────────"
            tail -20 "$LOG_DIR/reconstruction_server.log" 2>/dev/null || echo "日志文件不存在"

            echo ""
            echo "前端服务器日志 (最近20行):"
            echo "────────────────────────────────────────────────────────────────────"
            tail -20 "$LOG_DIR/frontend.log" 2>/dev/null || echo "日志文件不存在"
            echo ""
            ;;

        test)
            print_banner
            test_services
            echo ""
            ;;

        *)
            print_banner
            echo "Guardian Mobility v0.0 - 完整演示系统管理脚本"
            echo ""
            echo "用法: $0 {start|stop|restart|status|demo|logs|test}"
            echo ""
            echo "命令:"
            echo "  start   - 启动所有服务（默认）"
            echo "  stop    - 停止所有服务"
            echo "  restart - 重启所有服务"
            echo "  status  - 显示系统状态"
            echo "  demo    - 显示演示指南并可选打开浏览器"
            echo "  logs    - 查看所有服务日志"
            echo "  test    - 测试服务连接"
            echo ""
            echo "快速开始:"
            echo "  1. 启动系统:  $0 start"
            echo "  2. 查看演示:  $0 demo"
            echo "  3. 停止系统:  $0 stop"
            echo ""
            exit 1
            ;;
    esac
}

# 执行主函数
main "$@"
