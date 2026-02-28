#!/usr/bin/env bash
# ===========================================================================
# FSM-Pilot Local Self-Test
#
# Starts the cloud server, runs mock_vehicle.py + mock_operator.py,
# and verifies bidirectional communication over the relay channel.
#
# Prerequisites:
#   1. Cloud server binary is built:
#        cmake -B build && cmake --build build --target fsm_cloud_server
#   2. Python3 with websockets package:
#        pip3 install websockets
#   3. websocket-client for REST verification (optional):
#        pip3 install requests
#
# Usage:
#   cd /path/to/fsm/cpp
#   bash test/run_self_test.sh [cloud_binary] [config_file]
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CLOUD_BIN="${1:-${SCRIPT_DIR}/../build/cloud_server/fsm_cloud_server}"
CONFIG="${2:-${SCRIPT_DIR}/../cloud_server/config/cloud_config.yaml}"
SIGNALING_URL="ws://localhost:8080"
VEHICLE_ID="FSM-SELF-TEST-01"
LOG_DIR="${SCRIPT_DIR}/logs"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
pass() { echo -e "${GREEN}[PASS]${NC} $*"; }
fail() { echo -e "${RED}[FAIL]${NC} $*"; FAILURES=$((FAILURES + 1)); }
info() { echo -e "${YELLOW}[INFO]${NC} $*"; }

FAILURES=0
CLOUD_PID=""
VEHICLE_PID=""
OPERATOR_PID=""

cleanup() {
  info "Cleaning up..."
  [[ -n "$OPERATOR_PID" ]] && kill "$OPERATOR_PID" 2>/dev/null || true
  [[ -n "$VEHICLE_PID"  ]] && kill "$VEHICLE_PID"  2>/dev/null || true
  [[ -n "$CLOUD_PID"    ]] && kill "$CLOUD_PID"    2>/dev/null || true
  sleep 1
  if [[ $FAILURES -eq 0 ]]; then
    echo -e "${GREEN}══════════════════════════════${NC}"
    echo -e "${GREEN}  ALL TESTS PASSED${NC}"
    echo -e "${GREEN}══════════════════════════════${NC}"
    exit 0
  else
    echo -e "${RED}══════════════════════════════${NC}"
    echo -e "${RED}  $FAILURES TEST(S) FAILED${NC}"
    echo -e "${RED}══════════════════════════════${NC}"
    exit 1
  fi
}
trap cleanup EXIT INT TERM

mkdir -p "$LOG_DIR"

# ---------------------------------------------------------------------------
# Step 1: Verify prerequisites
# ---------------------------------------------------------------------------
info "=== Step 1: Verifying prerequisites ==="

if [[ ! -f "$CLOUD_BIN" ]]; then
  fail "Cloud server binary not found: $CLOUD_BIN"
  fail "Build it with: cd cpp && cmake -B build && cmake --build build --target fsm_cloud_server"
  exit 1
fi
pass "Cloud server binary found: $CLOUD_BIN"

if ! command -v python3 &>/dev/null; then
  fail "python3 not found"
  exit 1
fi
pass "python3 found"

if ! python3 -c "import websockets" 2>/dev/null; then
  fail "websockets package not installed. Run: pip3 install websockets"
  exit 1
fi
pass "websockets package found"

# ---------------------------------------------------------------------------
# Step 2: Start cloud server
# ---------------------------------------------------------------------------
info "=== Step 2: Starting cloud server ==="
"$CLOUD_BIN" "$CONFIG" > "$LOG_DIR/cloud_server.log" 2>&1 &
CLOUD_PID=$!
info "Cloud server PID: $CLOUD_PID"

# Wait for it to listen on port 8080
for i in {1..20}; do
  if ss -tlnp 2>/dev/null | grep -q ':8080 ' || \
     netstat -tlnp 2>/dev/null | grep -q ':8080 '; then
    break
  fi
  sleep 0.5
done

if ! kill -0 "$CLOUD_PID" 2>/dev/null; then
  fail "Cloud server exited immediately. Check $LOG_DIR/cloud_server.log"
  exit 1
fi
pass "Cloud server is running (PID $CLOUD_PID)"

# ---------------------------------------------------------------------------
# Step 3: Start mock vehicle
# ---------------------------------------------------------------------------
info "=== Step 3: Starting mock vehicle ==="
python3 "$SCRIPT_DIR/mock_vehicle.py" "$SIGNALING_URL" "$VEHICLE_ID" \
    > "$LOG_DIR/mock_vehicle.log" 2>&1 &
VEHICLE_PID=$!
sleep 1.5

if ! kill -0 "$VEHICLE_PID" 2>/dev/null; then
  fail "Mock vehicle exited. Check $LOG_DIR/mock_vehicle.log"
  exit 1
fi
pass "Mock vehicle running (PID $VEHICLE_PID)"

# ---------------------------------------------------------------------------
# Step 4: Start mock operator
# ---------------------------------------------------------------------------
info "=== Step 4: Starting mock operator ==="
python3 "$SCRIPT_DIR/mock_operator.py" "$SIGNALING_URL" "$VEHICLE_ID" \
    > "$LOG_DIR/mock_operator.log" 2>&1 &
OPERATOR_PID=$!
sleep 1.5

if ! kill -0 "$OPERATOR_PID" 2>/dev/null; then
  fail "Mock operator exited. Check $LOG_DIR/mock_operator.log"
  exit 1
fi
pass "Mock operator running (PID $OPERATOR_PID)"

# ---------------------------------------------------------------------------
# Step 5: Wait for relay communication to establish (5 seconds)
# ---------------------------------------------------------------------------
info "=== Step 5: Waiting for relay communication (5s) ==="
sleep 5

# ---------------------------------------------------------------------------
# Step 6: Verify vehicle log shows telemetry sent + control received
# ---------------------------------------------------------------------------
info "=== Step 6: Verifying vehicle output ==="
VEHICLE_LOG="$LOG_DIR/mock_vehicle.log"

if grep -q "Registered" "$VEHICLE_LOG"; then
  pass "Vehicle registered with signaling server"
else
  fail "Vehicle did not register (check $VEHICLE_LOG)"
fi

if grep -q "Operator connected" "$VEHICLE_LOG"; then
  pass "Vehicle detected operator connection"
else
  fail "Vehicle did not detect operator (check $VEHICLE_LOG)"
fi

TELEM_COUNT=$(grep -c "Sent.*telemetry frames" "$VEHICLE_LOG" 2>/dev/null || echo 0)
if [[ "$TELEM_COUNT" -gt 0 ]]; then
  pass "Vehicle is sending telemetry"
else
  fail "No telemetry log entries found in $VEHICLE_LOG"
fi

if grep -q "Control #" "$VEHICLE_LOG"; then
  pass "Vehicle received control commands from operator"
else
  fail "Vehicle did not receive control commands (check $VEHICLE_LOG)"
fi

# ---------------------------------------------------------------------------
# Step 7: Verify operator log shows telemetry received + vehicle ready
# ---------------------------------------------------------------------------
info "=== Step 7: Verifying operator output ==="
OPERATOR_LOG="$LOG_DIR/mock_operator.log"

if grep -q "Registered" "$OPERATOR_LOG"; then
  pass "Operator registered with signaling server"
else
  fail "Operator did not register (check $OPERATOR_LOG)"
fi

if grep -q "relay ready" "$OPERATOR_LOG"; then
  pass "Operator received vehicle_relay_ready"
else
  fail "Operator did not receive relay_ready (check $OPERATOR_LOG)"
fi

if grep -q "Telemetry #" "$OPERATOR_LOG"; then
  pass "Operator is receiving telemetry from vehicle"
else
  fail "Operator did not receive telemetry (check $OPERATOR_LOG)"
fi

# ---------------------------------------------------------------------------
# Step 8: Verify REST API health endpoint
# ---------------------------------------------------------------------------
info "=== Step 8: Testing REST API ==="
if command -v curl &>/dev/null; then
  HTTP_STATUS=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:8081/api/v1/health 2>/dev/null || echo "000")
  if [[ "$HTTP_STATUS" == "200" ]]; then
    pass "REST API health endpoint returned 200"
    HEALTH=$(curl -s http://localhost:8081/api/v1/health)
    info "Health: $HEALTH"
  else
    fail "REST API returned HTTP $HTTP_STATUS (expected 200)"
  fi

  VEHICLES=$(curl -s http://localhost:8081/api/v1/vehicles)
  if echo "$VEHICLES" | grep -q "$VEHICLE_ID"; then
    pass "REST API /api/v1/vehicles shows connected vehicle"
  else
    fail "REST API /api/v1/vehicles did not show $VEHICLE_ID"
    info "Vehicles response: $VEHICLES"
  fi
else
  info "curl not found — skipping REST API test"
fi

# ---------------------------------------------------------------------------
# Step 9: Cloud server health check
# ---------------------------------------------------------------------------
info "=== Step 9: Cloud server final check ==="
if ! kill -0 "$CLOUD_PID" 2>/dev/null; then
  fail "Cloud server crashed during test"
else
  pass "Cloud server still running"
fi

if grep -q "Status:" "$LOG_DIR/cloud_server.log" 2>/dev/null; then
  LAST_STATUS=$(grep "Status:" "$LOG_DIR/cloud_server.log" | tail -1)
  info "Last status: $LAST_STATUS"
fi

info "Logs saved in: $LOG_DIR/"
info "  cloud_server.log, mock_vehicle.log, mock_operator.log"
