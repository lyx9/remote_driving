#!/bin/bash
# RosBag Stream Server Startup Script
# Guardian Mobility v0.0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_FILE="/tmp/rosbag_server.log"
PID_FILE="/tmp/rosbag_server.pid"
ROSBAG_DIR="/home/lyx/fsm/rosbag"

case "$1" in
    start)
        if [ -f "$PID_FILE" ]; then
            PID=$(cat "$PID_FILE")
            if ps -p $PID > /dev/null 2>&1; then
                echo "Server is already running (PID: $PID)"
                exit 1
            fi
        fi

        echo "Starting RosBag Stream Server..."
        nohup python3 "$SCRIPT_DIR/rosbag_stream_server.py" --rosbag-dir "$ROSBAG_DIR" > "$LOG_FILE" 2>&1 &
        echo $! > "$PID_FILE"
        sleep 2

        if ps -p $(cat "$PID_FILE") > /dev/null 2>&1; then
            echo "✓ Server started successfully (PID: $(cat "$PID_FILE"))"
            echo "  Log file: $LOG_FILE"
            echo "  WebSocket: ws://localhost:8765"
        else
            echo "✗ Failed to start server. Check log: $LOG_FILE"
            exit 1
        fi
        ;;

    stop)
        if [ ! -f "$PID_FILE" ]; then
            echo "Server is not running"
            exit 1
        fi

        PID=$(cat "$PID_FILE")
        echo "Stopping RosBag Stream Server (PID: $PID)..."
        kill $PID 2>/dev/null

        sleep 1
        if ps -p $PID > /dev/null 2>&1; then
            echo "Force killing..."
            kill -9 $PID 2>/dev/null
        fi

        rm -f "$PID_FILE"
        echo "✓ Server stopped"
        ;;

    restart)
        $0 stop
        sleep 2
        $0 start
        ;;

    status)
        if [ -f "$PID_FILE" ]; then
            PID=$(cat "$PID_FILE")
            if ps -p $PID > /dev/null 2>&1; then
                echo "✓ Server is running (PID: $PID)"
                echo "  WebSocket: ws://localhost:8765"
                lsof -i :8765 2>/dev/null
                exit 0
            else
                echo "✗ Server is not running (stale PID file)"
                rm -f "$PID_FILE"
                exit 1
            fi
        else
            echo "✗ Server is not running"
            exit 1
        fi
        ;;

    logs)
        if [ -f "$LOG_FILE" ]; then
            tail -f "$LOG_FILE"
        else
            echo "No log file found"
            exit 1
        fi
        ;;

    *)
        echo "Usage: $0 {start|stop|restart|status|logs}"
        echo ""
        echo "Commands:"
        echo "  start   - Start the RosBag stream server"
        echo "  stop    - Stop the RosBag stream server"
        echo "  restart - Restart the RosBag stream server"
        echo "  status  - Check server status"
        echo "  logs    - View server logs (tail -f)"
        exit 1
        ;;
esac

exit 0
