#!/bin/bash

###############################################################################
# FSM-Pilot V2.0 - Complete System Demo Script
#
# @project     FSM-Pilot Remote Driving Platform
# @author      Li Yixiang
# @institution City University of Hong Kong
# @copyright   2025 City University of Hong Kong. All rights reserved.
#
# @description Complete demonstration of all system features:
#              - Frontend with orange-red tech theme
#              - Amap integration with vehicle tracking
#              - RosBag streaming and visualization
#              - Database management
#              - Remote control interface
###############################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
ORANGE='\033[0;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Project root
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_ROOT"

# Log file
LOG_FILE="/tmp/fsm_demo_$(date +%Y%m%d_%H%M%S).log"

# PID tracking
PIDS=()

###############################################################################
# Helper Functions
###############################################################################

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1" | tee -a "$LOG_FILE"
}

log_warn() {
    echo -e "${ORANGE}[WARN]${NC} $1" | tee -a "$LOG_FILE"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1" | tee -a "$LOG_FILE"
}

log_section() {
    echo -e "\n${PURPLE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${PURPLE}  $1${NC}"
    echo -e "${PURPLE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}\n"
}

check_command() {
    if ! command -v "$1" &> /dev/null; then
        log_error "$1 is not installed"
        return 1
    fi
    return 0
}

wait_for_port() {
    local port=$1
    local max_wait=30
    local count=0

    log_info "Waiting for port $port to be ready..."
    while ! nc -z localhost "$port" 2>/dev/null; do
        sleep 1
        count=$((count + 1))
        if [ $count -ge $max_wait ]; then
            log_error "Timeout waiting for port $port"
            return 1
        fi
    done
    log_info "Port $port is ready"
    return 0
}

cleanup() {
    log_section "Cleaning Up"

    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            log_info "Stopping process $pid"
            kill "$pid" 2>/dev/null || true
        fi
    done

    # Kill any remaining node/npm processes
    pkill -f "vite" 2>/dev/null || true
    pkill -f "rosbag-server" 2>/dev/null || true

    log_info "Cleanup complete"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

###############################################################################
# System Check
###############################################################################

log_section "FSM-Pilot V2.0 - Complete System Demo"

log_info "Project: FSM-Pilot Remote Driving Platform"
log_info "Author: Li Yixiang"
log_info "Institution: City University of Hong Kong"
log_info "Log file: $LOG_FILE"
echo ""

log_section "System Requirements Check"

# Check required commands
REQUIRED_CMDS=("node" "npm" "sqlite3" "nc")
for cmd in "${REQUIRED_CMDS[@]}"; do
    if check_command "$cmd"; then
        log_info "✓ $cmd is installed"
    else
        log_error "✗ $cmd is required but not installed"
        exit 1
    fi
done

# Check Node.js version
NODE_VERSION=$(node --version)
log_info "Node.js version: $NODE_VERSION"

# Check npm version
NPM_VERSION=$(npm --version)
log_info "npm version: $NPM_VERSION"

###############################################################################
# Check RosBag Files
###############################################################################

log_section "RosBag Files Check"

ROSBAG_FILE="/home/lyx/fsm/rosbag/rosbag2_2025_02_23-16_49_58/rosbag2_2025_02_23-16_49_58_0.db3"

if [ -f "$ROSBAG_FILE" ]; then
    ROSBAG_SIZE=$(du -h "$ROSBAG_FILE" | cut -f1)
    log_info "✓ RosBag file found: $ROSBAG_FILE"
    log_info "  Size: $ROSBAG_SIZE"

    # Check topics
    TOPIC_COUNT=$(sqlite3 "$ROSBAG_FILE" "SELECT COUNT(*) FROM topics;")
    log_info "  Topics: $TOPIC_COUNT"

    # List topics
    log_info "  Available topics:"
    sqlite3 "$ROSBAG_FILE" "SELECT '    - ' || name || ' (' || type || ')' FROM topics;" | head -10
else
    log_warn "RosBag file not found at $ROSBAG_FILE"
    log_warn "RosBag streaming demo will not be available"
fi

###############################################################################
# Install Dependencies
###############################################################################

log_section "Installing Dependencies"

# Frontend dependencies
if [ ! -d "node_modules" ]; then
    log_info "Installing frontend dependencies..."
    npm install >> "$LOG_FILE" 2>&1
    log_info "✓ Frontend dependencies installed"
else
    log_info "✓ Frontend dependencies already installed"
fi

# Server dependencies
cd server
if [ ! -d "node_modules" ]; then
    log_info "Installing server dependencies..."
    npm install >> "$LOG_FILE" 2>&1
    log_info "✓ Server dependencies installed"
else
    log_info "✓ Server dependencies already installed"
fi
cd ..

###############################################################################
# Start RosBag Streaming Server
###############################################################################

log_section "Starting RosBag Streaming Server"

if [ -f "$ROSBAG_FILE" ]; then
    log_info "Starting RosBag server on port 8765..."
    cd server
    ROSBAG_PATH="$ROSBAG_FILE" node rosbag-server.js >> "$LOG_FILE" 2>&1 &
    ROSBAG_PID=$!
    PIDS+=($ROSBAG_PID)
    cd ..

    if wait_for_port 8765; then
        log_info "✓ RosBag streaming server started (PID: $ROSBAG_PID)"
    else
        log_error "Failed to start RosBag streaming server"
    fi
else
    log_warn "Skipping RosBag server (no RosBag file)"
fi

###############################################################################
# Start Frontend Development Server
###############################################################################

log_section "Starting Frontend Development Server"

log_info "Starting Vite development server on port 3000..."
npm run dev >> "$LOG_FILE" 2>&1 &
FRONTEND_PID=$!
PIDS+=($FRONTEND_PID)

if wait_for_port 3000; then
    log_info "✓ Frontend server started (PID: $FRONTEND_PID)"
else
    log_error "Failed to start frontend server"
    exit 1
fi

###############################################################################
# Display System Information
###############################################################################

log_section "System Ready"

echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                                                                ║${NC}"
echo -e "${GREEN}║           FSM-Pilot V2.0 - System Demo Running                ║${NC}"
echo -e "${GREEN}║                                                                ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}🌐 Frontend Application:${NC}"
echo -e "   ${ORANGE}http://localhost:3000${NC}"
echo ""
echo -e "${CYAN}🔐 Login Credentials:${NC}"
echo -e "   Username: ${ORANGE}cityu${NC}"
echo -e "   Password: ${ORANGE}2026${NC}"
echo ""
echo -e "${CYAN}📊 Available Features:${NC}"
echo -e "   ${GREEN}✓${NC} Remote Control Interface (orange-red theme)"
echo -e "   ${GREEN}✓${NC} Amap Vehicle Location Tracking"
echo -e "   ${GREEN}✓${NC} Database Visualization & Management"
echo -e "   ${GREEN}✓${NC} Intelligent Dispatch Demo"

if [ -f "$ROSBAG_FILE" ]; then
    echo -e "   ${GREEN}✓${NC} RosBag Streaming & Replay"
    echo ""
    echo -e "${CYAN}🎬 RosBag Replay:${NC}"
    echo -e "   ${ORANGE}http://localhost:3000/rosbag-replay-pro${NC}"
    echo -e "   WebSocket: ${ORANGE}ws://localhost:8765${NC}"
fi

echo ""
echo -e "${CYAN}📝 Demo Highlights:${NC}"
echo -e "   1. ${ORANGE}New Orange-Red Tech Theme${NC} - Modern, tech-focused UI"
echo -e "   2. ${ORANGE}Amap Integration${NC} - Real-time vehicle tracking on map"
echo -e "   3. ${ORANGE}RosBag Streaming${NC} - Handle large files (>15GB) efficiently"
echo -e "   4. ${ORANGE}Multi-Camera Display${NC} - Synchronized camera feeds"
echo -e "   5. ${ORANGE}Database Management${NC} - Import/Export/Visualize data"
echo ""
echo -e "${CYAN}🎯 Recommended Demo Flow:${NC}"
echo -e "   ${PURPLE}Step 1:${NC} Login with cityu/2026"
echo -e "   ${PURPLE}Step 2:${NC} View Remote Control page (Amap + Video + Lidar)"
echo -e "   ${PURPLE}Step 3:${NC} Check Database Visualization"
echo -e "   ${PURPLE}Step 4:${NC} Try RosBag Replay (if available)"
echo -e "   ${PURPLE}Step 5:${NC} Explore Intelligent Dispatch Demo"
echo ""
echo -e "${CYAN}📋 Log File:${NC}"
echo -e "   ${ORANGE}$LOG_FILE${NC}"
echo ""
echo -e "${RED}Press Ctrl+C to stop all services${NC}"
echo ""

###############################################################################
# Open Browser
###############################################################################

sleep 2

if command -v xdg-open &> /dev/null; then
    log_info "Opening browser..."
    xdg-open "http://localhost:3000" &
elif command -v open &> /dev/null; then
    log_info "Opening browser..."
    open "http://localhost:3000" &
else
    log_warn "Could not detect browser command. Please open http://localhost:3000 manually"
fi

###############################################################################
# Keep Running
###############################################################################

log_info "System is running. Press Ctrl+C to stop."
echo ""

# Wait for user interrupt
while true; do
    sleep 1

    # Check if processes are still running
    for pid in "${PIDS[@]}"; do
        if ! kill -0 "$pid" 2>/dev/null; then
            log_error "Process $pid has stopped unexpectedly"
            cleanup
        fi
    done
done
