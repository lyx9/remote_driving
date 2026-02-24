#!/bin/bash

###############################################################################
# Guardian Mobility - Professional Demo Recording Script
#
# @product     Guardian Mobility - AI-Powered Remote Driving Platform
# @author      CEO / Founder
# @description Automated setup for professional 2-minute demo video recording
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
ORANGE='\033[0;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

# Project root
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_ROOT"

# RosBag file
ROSBAG_FILE="/home/lyx/fsm/rosbag/1210/rosbag2_2025_12_10-17_25_58/rosbag2_2025_12_10-17_25_58_0.db3"

# PID tracking
PIDS=()

###############################################################################
# Helper Functions
###############################################################################

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${ORANGE}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_section() {
    echo -e "\n${PURPLE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${PURPLE}  $1${NC}"
    echo -e "${PURPLE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}\n"
}

wait_for_port() {
    local port=$1
    local max_wait=30
    local count=0

    while ! nc -z localhost "$port" 2>/dev/null; do
        sleep 1
        count=$((count + 1))
        if [ $count -ge $max_wait ]; then
            log_error "Timeout waiting for port $port"
            return 1
        fi
    done
    return 0
}

cleanup() {
    log_section "Cleaning Up"

    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done

    pkill -f "vite" 2>/dev/null || true
    pkill -f "rosbag-server" 2>/dev/null || true

    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

###############################################################################
# Main Script
###############################################################################

log_section "Guardian Mobility - Professional Demo Setup"

echo -e "${CYAN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                                                                ║${NC}"
echo -e "${CYAN}║              GUARDIAN MOBILITY                                 ║${NC}"
echo -e "${CYAN}║         AI-Powered Remote Driving Platform                     ║${NC}"
echo -e "${CYAN}║                                                                ║${NC}"
echo -e "${CYAN}║         Professional 2-Minute Demo Recording Setup             ║${NC}"
echo -e "${CYAN}║                                                                ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

###############################################################################
# Pre-Recording Checklist
###############################################################################

log_section "Pre-Recording Checklist"

log_info "System Preparation:"
echo "  [ ] Close all unnecessary applications"
echo "  [ ] Enable Do Not Disturb mode"
echo "  [ ] Clear browser cache"
echo "  [ ] Set screen resolution to 1920x1080"
echo "  [ ] Test audio levels"
echo "  [ ] Verify recording software ready"
echo ""

log_info "Visual Preparation:"
echo "  [ ] Browser full-screen mode (F11)"
echo "  [ ] Zoom level at 100%"
echo "  [ ] Hide bookmarks bar"
echo "  [ ] Professional cursor"
echo ""

read -p "Press ENTER when ready to start demo servers..."

###############################################################################
# Start Services
###############################################################################

log_section "Starting Demo Services"

# Check RosBag file
if [ ! -f "$ROSBAG_FILE" ]; then
    log_error "RosBag file not found: $ROSBAG_FILE"
    exit 1
fi

log_info "RosBag file: $(du -h "$ROSBAG_FILE" | cut -f1)"

# Install dependencies if needed
if [ ! -d "node_modules" ]; then
    log_info "Installing dependencies..."
    npm install > /dev/null 2>&1
fi

cd server
if [ ! -d "node_modules" ]; then
    npm install > /dev/null 2>&1
fi
cd ..

# Start RosBag server
log_info "Starting RosBag streaming server..."
cd server
ROSBAG_PATH="$ROSBAG_FILE" node rosbag-server.js > /tmp/rosbag_server.log 2>&1 &
ROSBAG_PID=$!
PIDS+=($ROSBAG_PID)
cd ..

if wait_for_port 8765; then
    log_info "✓ RosBag server ready (PID: $ROSBAG_PID)"
else
    log_error "Failed to start RosBag server"
    exit 1
fi

# Start frontend
log_info "Starting frontend server..."
npm run dev > /tmp/frontend_server.log 2>&1 &
FRONTEND_PID=$!
PIDS+=($FRONTEND_PID)

if wait_for_port 3000; then
    log_info "✓ Frontend server ready (PID: $FRONTEND_PID)"
else
    log_error "Failed to start frontend server"
    exit 1
fi

sleep 3

###############################################################################
# Recording Instructions
###############################################################################

log_section "Recording Instructions"

echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                                                                ║${NC}"
echo -e "${GREEN}║              DEMO READY FOR RECORDING                          ║${NC}"
echo -e "${GREEN}║                                                                ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

echo -e "${CYAN}📹 Recording Setup:${NC}"
echo -e "   1. Open OBS Studio / Camtasia / ScreenFlow"
echo -e "   2. Set recording area to 1920x1080"
echo -e "   3. Configure audio (desktop + microphone)"
echo -e "   4. Test recording for 5 seconds"
echo ""

echo -e "${CYAN}🌐 Application URLs:${NC}"
echo -e "   Main:         ${ORANGE}http://localhost:3000${NC}"
echo -e "   RosBag:       ${ORANGE}http://localhost:3000/rosbag-replay-pro${NC}"
echo ""

echo -e "${CYAN}🔐 Login Credentials:${NC}"
echo -e "   Username:     ${ORANGE}cityu${NC}"
echo -e "   Password:     ${ORANGE}2026${NC}"
echo ""

echo -e "${CYAN}📊 Demo Data:${NC}"
echo -e "   RosBag Size:  ${ORANGE}3.7 GB${NC}"
echo -e "   Topics:       ${ORANGE}43${NC}"
echo -e "   Cameras:      ${ORANGE}6${NC}"
echo -e "   Duration:     ${ORANGE}~5 minutes${NC}"
echo ""

echo -e "${CYAN}🎬 Recording Timeline (2 minutes):${NC}"
echo -e "   ${PURPLE}0:00-0:10${NC}  Opening & Login"
echo -e "   ${PURPLE}0:10-0:40${NC}  Remote Control Interface"
echo -e "   ${PURPLE}0:40-1:30${NC}  RosBag Scenario Replay"
echo -e "   ${PURPLE}1:30-1:50${NC}  Key Differentiators"
echo -e "   ${PURPLE}1:50-2:00${NC}  Use Cases"
echo -e "   ${PURPLE}2:00-2:10${NC}  Closing"
echo ""

echo -e "${CYAN}📝 Key Features to Highlight:${NC}"
echo -e "   ${GREEN}✓${NC} 6-Camera Surround View (<100ms latency)"
echo -e "   ${GREEN}✓${NC} Real-time LiDAR 3D Visualization"
echo -e "   ${GREEN}✓${NC} Amap Vehicle Tracking"
echo -e "   ${GREEN}✓${NC} Comprehensive Telemetry"
echo -e "   ${GREEN}✓${NC} RosBag Streaming (>15GB support)"
echo -e "   ${GREEN}✓${NC} Multi-camera Synchronized Playback"
echo -e "   ${GREEN}✓${NC} AI-Powered Data Analysis"
echo ""

echo -e "${CYAN}🎯 Recording Tips:${NC}"
echo -e "   • Speak clearly and confidently"
echo -e "   • Move mouse smoothly"
echo -e "   • Pause briefly between sections"
echo -e "   • Highlight key features with cursor"
echo -e "   • Maintain professional pace"
echo ""

echo -e "${CYAN}📄 Full Script:${NC}"
echo -e "   ${ORANGE}VIDEO_SCRIPT_2MIN_PROFESSIONAL.md${NC}"
echo ""

###############################################################################
# Open Browser
###############################################################################

log_info "Opening browser..."
sleep 2

if command -v xdg-open &> /dev/null; then
    xdg-open "http://localhost:3000" &
elif command -v open &> /dev/null; then
    open "http://localhost:3000" &
fi

echo ""
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}  Ready to Record!${NC}"
echo -e "${GREEN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${ORANGE}When ready:${NC}"
echo -e "  1. Start your recording software"
echo -e "  2. Begin narration"
echo -e "  3. Follow the 2-minute script"
echo ""
echo -e "${RED}Press Ctrl+C to stop all services when done${NC}"
echo ""

###############################################################################
# Keep Running
###############################################################################

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
