#!/bin/bash

###############################################################################
# FSM-Pilot V2.0 - RosBag 1210 Demo Script
#
# @project     FSM-Pilot Remote Driving Platform
# @author      Li Yixiang
# @institution City University of Hong Kong
# @description Demo script using RosBag data from 1210 directory
#              - 6 cameras (camera0-5)
#              - LiDAR point cloud
#              - Chassis CAN data
#              - 4.5GB data file
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

    log_info "Waiting for port $port..."
    while ! nc -z localhost "$port" 2>/dev/null; do
        sleep 1
        count=$((count + 1))
        if [ $count -ge $max_wait ]; then
            log_error "Timeout waiting for port $port"
            return 1
        fi
    done
    log_info "✓ Port $port is ready"
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

    pkill -f "vite" 2>/dev/null || true
    pkill -f "rosbag-server" 2>/dev/null || true

    log_info "Cleanup complete"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

###############################################################################
# Main Script
###############################################################################

log_section "FSM-Pilot V2.0 - RosBag 1210 Demo"

log_info "Project: FSM-Pilot Remote Driving Platform"
log_info "Author: Li Yixiang"
log_info "Institution: City University of Hong Kong"
echo ""

###############################################################################
# Check RosBag File
###############################################################################

log_section "RosBag File Check"

if [ ! -f "$ROSBAG_FILE" ]; then
    log_error "RosBag file not found: $ROSBAG_FILE"
    exit 1
fi

ROSBAG_SIZE=$(du -h "$ROSBAG_FILE" | cut -f1)
log_info "✓ RosBag file found"
log_info "  Path: $ROSBAG_FILE"
log_info "  Size: $ROSBAG_SIZE"

# Check topics
log_info "\nAnalyzing RosBag contents..."
TOPIC_COUNT=$(sqlite3 "$ROSBAG_FILE" "SELECT COUNT(*) FROM topics;")
MESSAGE_COUNT=$(sqlite3 "$ROSBAG_FILE" "SELECT COUNT(*) FROM messages;")

log_info "  Total topics: $TOPIC_COUNT"
log_info "  Total messages: $MESSAGE_COUNT"

# List camera topics
log_info "\n📷 Camera Topics:"
sqlite3 "$ROSBAG_FILE" "SELECT '    - ' || name FROM topics WHERE name LIKE '%camera%/image_raw/compressed' ORDER BY name;"

# List LiDAR topics
log_info "\n🎯 LiDAR Topics:"
sqlite3 "$ROSBAG_FILE" "SELECT '    - ' || name FROM topics WHERE name LIKE '%pointcloud%';"

# List chassis topics
log_info "\n🚗 Chassis CAN Topics:"
sqlite3 "$ROSBAG_FILE" "SELECT '    - ' || name FROM topics WHERE name LIKE '%chassis_can%';"

###############################################################################
# Install Dependencies
###############################################################################

log_section "Installing Dependencies"

if [ ! -d "node_modules" ]; then
    log_info "Installing frontend dependencies..."
    npm install > /dev/null 2>&1
    log_info "✓ Frontend dependencies installed"
else
    log_info "✓ Frontend dependencies already installed"
fi

cd server
if [ ! -d "node_modules" ]; then
    log_info "Installing server dependencies..."
    npm install > /dev/null 2>&1
    log_info "✓ Server dependencies installed"
else
    log_info "✓ Server dependencies already installed"
fi
cd ..

###############################################################################
# Start RosBag Streaming Server
###############################################################################

log_section "Starting RosBag Streaming Server"

log_info "Starting RosBag server on port 8765..."
log_info "Using RosBag: $ROSBAG_FILE"

cd server
ROSBAG_PATH="$ROSBAG_FILE" node rosbag-server.js > /tmp/rosbag_server.log 2>&1 &
ROSBAG_PID=$!
PIDS+=($ROSBAG_PID)
cd ..

if wait_for_port 8765; then
    log_info "✓ RosBag streaming server started (PID: $ROSBAG_PID)"
else
    log_error "Failed to start RosBag streaming server"
    cat /tmp/rosbag_server.log
    exit 1
fi

###############################################################################
# Start Frontend Development Server
###############################################################################

log_section "Starting Frontend Development Server"

log_info "Starting Vite development server on port 3000..."
npm run dev > /tmp/frontend_server.log 2>&1 &
FRONTEND_PID=$!
PIDS+=($FRONTEND_PID)

if wait_for_port 3000; then
    log_info "✓ Frontend server started (PID: $FRONTEND_PID)"
else
    log_error "Failed to start frontend server"
    cat /tmp/frontend_server.log
    exit 1
fi

###############################################################################
# Display Demo Information
###############################################################################

log_section "Demo Ready"

echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║                                                                ║${NC}"
echo -e "${GREEN}║         FSM-Pilot V2.0 - RosBag 1210 Demo Running            ║${NC}"
echo -e "${GREEN}║                                                                ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}🌐 Application URLs:${NC}"
echo -e "   Frontend:     ${ORANGE}http://localhost:3000${NC}"
echo -e "   RosBag Replay: ${ORANGE}http://localhost:3000/rosbag-replay-pro${NC}"
echo -e "   WebSocket:    ${ORANGE}ws://localhost:8765${NC}"
echo ""
echo -e "${CYAN}🔐 Login Credentials:${NC}"
echo -e "   Username: ${ORANGE}cityu${NC}"
echo -e "   Password: ${ORANGE}2026${NC}"
echo ""
echo -e "${CYAN}📊 RosBag Data (1210):${NC}"
echo -e "   File Size:    ${ORANGE}$ROSBAG_SIZE${NC}"
echo -e "   Topics:       ${ORANGE}$TOPIC_COUNT${NC}"
echo -e "   Messages:     ${ORANGE}$MESSAGE_COUNT${NC}"
echo ""
echo -e "${CYAN}📷 Available Cameras:${NC}"
echo -e "   ${GREEN}✓${NC} camera0 - Front camera"
echo -e "   ${GREEN}✓${NC} camera1 - Front-left camera"
echo -e "   ${GREEN}✓${NC} camera2 - Left camera"
echo -e "   ${GREEN}✓${NC} camera3 - Rear camera"
echo -e "   ${GREEN}✓${NC} camera4 - Right camera"
echo -e "   ${GREEN}✓${NC} camera5 - Front-right camera"
echo ""
echo -e "${CYAN}🎯 Available Data:${NC}"
echo -e "   ${GREEN}✓${NC} LiDAR Point Cloud (/sensing/lidar/top/pointcloud_raw)"
echo -e "   ${GREEN}✓${NC} Chassis Speed (/Sensor_msgs/chassis_can/speed)"
echo -e "   ${GREEN}✓${NC} Steering Angle (/Sensor_msgs/chassis_can/steerAngle)"
echo -e "   ${GREEN}✓${NC} Brake Rate (/Sensor_msgs/chassis_can/brakeRate)"
echo -e "   ${GREEN}✓${NC} Battery SOC (/Sensor_msgs/chassis_can/SOC)"
echo ""
echo -e "${CYAN}🎬 Demo Steps:${NC}"
echo -e "   ${PURPLE}1.${NC} Open ${ORANGE}http://localhost:3000${NC} in browser"
echo -e "   ${PURPLE}2.${NC} Login with ${ORANGE}cityu/2026${NC}"
echo -e "   ${PURPLE}3.${NC} Navigate to ${ORANGE}RosBag Replay Pro${NC}"
echo -e "   ${PURPLE}4.${NC} Select the 1210 RosBag file"
echo -e "   ${PURPLE}5.${NC} Choose topics to visualize:"
echo -e "      - Camera topics for video playback"
echo -e "      - LiDAR topic for point cloud"
echo -e "      - Chassis topics for vehicle data"
echo -e "   ${PURPLE}6.${NC} Click ${ORANGE}Start Streaming${NC}"
echo -e "   ${PURPLE}7.${NC} Use playback controls:"
echo -e "      - Play/Pause"
echo -e "      - Speed adjustment (0.5x - 2.0x)"
echo -e "      - Timeline scrubbing"
echo ""
echo -e "${CYAN}💡 Tips:${NC}"
echo -e "   • Select compressed image topics for better performance"
echo -e "   • Use topic filtering to reduce bandwidth"
echo -e "   • Adjust playback speed for detailed analysis"
echo -e "   • Check the multi-camera grid view"
echo ""
echo -e "${CYAN}📝 Logs:${NC}"
echo -e "   RosBag Server: ${ORANGE}/tmp/rosbag_server.log${NC}"
echo -e "   Frontend:      ${ORANGE}/tmp/frontend_server.log${NC}"
echo ""
echo -e "${RED}Press Ctrl+C to stop all services${NC}"
echo ""

###############################################################################
# Open Browser
###############################################################################

sleep 2

if command -v xdg-open &> /dev/null; then
    log_info "Opening browser..."
    xdg-open "http://localhost:3000/rosbag-replay-pro" &
elif command -v open &> /dev/null; then
    log_info "Opening browser..."
    open "http://localhost:3000/rosbag-replay-pro" &
fi

###############################################################################
# Keep Running
###############################################################################

log_info "System is running. Press Ctrl+C to stop."
echo ""

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
