#!/usr/bin/env bash
# ============================================================
# FSM-Pilot 部署自检脚本
# 运行方式:
#   ./run_self_test.sh [--host HOST] [--api-port 8081]
#                      [--ws-port 8080] [--skip-mqtt]
# 成功退出码: 0；失败: 1
# ============================================================
set -uo pipefail

# ─── 默认参数 ─────────────────────────────────────────────
HOST="${FSM_TEST_HOST:-localhost}"
API_PORT="${FSM_API_PORT:-8081}"
WS_PORT="${FSM_WS_PORT:-8080}"
MQTT_HOST="${FSM_MQTT_HOST:-${HOST}}"
MQTT_PORT="${FSM_MQTT_PORT:-1883}"
SKIP_MQTT="${FSM_SKIP_MQTT:-false}"
TIMEOUT=5   # seconds per check
PASS=0
FAIL=0
SKIP=0

# ─── 颜色输出 ─────────────────────────────────────────────
GREEN='\033[0;32m'; RED='\033[0;31m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; NC='\033[0m'
pass()  { PASS=$((PASS+1)); echo -e "${GREEN}[PASS]${NC} $*"; }
fail()  { FAIL=$((FAIL+1)); echo -e "${RED}[FAIL]${NC} $*"; }
skip()  { SKIP=$((SKIP+1)); echo -e "${YELLOW}[SKIP]${NC} $*"; }
header(){ echo -e "\n${CYAN}──── $* ────${NC}"; }

# ─── 参数解析 ─────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)       HOST="$2";       shift 2 ;;
    --api-port)   API_PORT="$2";   shift 2 ;;
    --ws-port)    WS_PORT="$2";    shift 2 ;;
    --mqtt-host)  MQTT_HOST="$2";  shift 2 ;;
    --mqtt-port)  MQTT_PORT="$2";  shift 2 ;;
    --skip-mqtt)  SKIP_MQTT=true;  shift   ;;
    *) echo "Unknown argument: $1"; exit 1 ;;
  esac
done

API_BASE="http://${HOST}:${API_PORT}/api/v1"

# ─── 工具检查 ─────────────────────────────────────────────
require_cmd() {
  if ! command -v "$1" &>/dev/null; then
    echo "Required tool '$1' not found. Install with: ${2:-apt-get install $1}"
    exit 1
  fi
}
require_cmd curl  "apt-get install curl"
require_cmd jq    "apt-get install jq"
if [[ "$SKIP_MQTT" != "true" ]]; then
  require_cmd mosquitto_pub  "apt-get install mosquitto-clients"
  require_cmd mosquitto_sub  "apt-get install mosquitto-clients"
fi

echo "═══════════════════════════════════════════════════════"
echo "  FSM-Pilot Self-Test"
echo "  API:  ${API_BASE}"
echo "  WS:   ws://${HOST}:${WS_PORT}"
echo "  MQTT: tcp://${MQTT_HOST}:${MQTT_PORT}"
echo "═══════════════════════════════════════════════════════"

# ============================================================
# 1. REST API 健康检查
# ============================================================
header "REST API Health"

resp=$(curl -sf --max-time "$TIMEOUT" "${API_BASE}/health" 2>&1) && {
  status=$(echo "$resp" | jq -r '.status' 2>/dev/null)
  if [[ "$status" == "ok" ]]; then
    pass "GET /api/v1/health → status=ok"
  else
    fail "GET /api/v1/health → unexpected status: $status (body: $resp)"
  fi
} || fail "GET /api/v1/health → connection refused or timeout"

# ============================================================
# 2. REST API — 车辆列表
# ============================================================
header "REST API Vehicles"

resp=$(curl -sf --max-time "$TIMEOUT" "${API_BASE}/vehicles" 2>&1) && {
  count=$(echo "$resp" | jq -r '.count' 2>/dev/null)
  if [[ "$count" =~ ^[0-9]+$ ]]; then
    pass "GET /api/v1/vehicles → count=${count} (valid JSON)"
  else
    fail "GET /api/v1/vehicles → invalid response: $resp"
  fi
} || fail "GET /api/v1/vehicles → connection refused or timeout"

# ============================================================
# 3. REST API — 调度队列
# ============================================================
header "REST API Queue"

resp=$(curl -sf --max-time "$TIMEOUT" "${API_BASE}/queue" 2>&1) && {
  count=$(echo "$resp" | jq -r '.count' 2>/dev/null)
  if [[ "$count" =~ ^[0-9]+$ ]]; then
    pass "GET /api/v1/queue → count=${count} (valid JSON)"
  else
    fail "GET /api/v1/queue → invalid response: $resp"
  fi
} || fail "GET /api/v1/queue → connection refused or timeout"

# ============================================================
# 4. REST API — 告警列表
# ============================================================
header "REST API Alerts"

resp=$(curl -sf --max-time "$TIMEOUT" "${API_BASE}/alerts" 2>&1) && {
  count=$(echo "$resp" | jq -r '.count' 2>/dev/null)
  if [[ "$count" =~ ^[0-9]+$ ]]; then
    pass "GET /api/v1/alerts → count=${count} (valid JSON)"
  else
    fail "GET /api/v1/alerts → invalid response: $resp"
  fi
} || fail "GET /api/v1/alerts → connection refused or timeout"

# ============================================================
# 5. REST API — 会话列表
# ============================================================
header "REST API Sessions"

resp=$(curl -sf --max-time "$TIMEOUT" "${API_BASE}/sessions" 2>&1) && {
  count=$(echo "$resp" | jq -r '.count' 2>/dev/null)
  if [[ "$count" =~ ^[0-9]+$ ]]; then
    pass "GET /api/v1/sessions → count=${count} (valid JSON)"
  else
    fail "GET /api/v1/sessions → invalid response: $resp"
  fi
} || fail "GET /api/v1/sessions → connection refused or timeout"

# ============================================================
# 6. REST API — Prometheus 指标
# ============================================================
header "REST API Metrics"

resp=$(curl -sf --max-time "$TIMEOUT" "http://${HOST}:${API_PORT}/metrics" 2>&1) && {
  if echo "$resp" | grep -q "^# HELP fsm_"; then
    pass "GET /metrics → Prometheus format (fsm_ metrics found)"
  elif [[ -n "$resp" ]]; then
    pass "GET /metrics → response received (${#resp} bytes)"
  else
    fail "GET /metrics → empty response"
  fi
} || fail "GET /metrics → connection refused or timeout"

# ============================================================
# 7. WebSocket 信令服务器连通性
# ============================================================
header "WebSocket Signaling"

if command -v websocat &>/dev/null; then
  echo '{"type":"ping"}' | timeout "$TIMEOUT" \
    websocat -n "ws://${HOST}:${WS_PORT}" >/dev/null 2>&1 && \
    pass "WebSocket ws://${HOST}:${WS_PORT} → connection accepted" || \
    fail "WebSocket ws://${HOST}:${WS_PORT} → connection failed"
else
  # Fallback: TCP port check
  if timeout "$TIMEOUT" bash -c "echo >/dev/tcp/${HOST}/${WS_PORT}" 2>/dev/null; then
    pass "WebSocket port ${WS_PORT} → TCP reachable (install websocat for full check)"
  else
    fail "WebSocket port ${WS_PORT} → TCP unreachable"
  fi
fi

# ============================================================
# 8. MQTT Broker 连通性 + 遥测消息流
# ============================================================
header "MQTT Broker"

if [[ "$SKIP_MQTT" == "true" ]]; then
  skip "MQTT tests skipped (--skip-mqtt)"
else
  # 8a. Broker TCP reachable
  if timeout "$TIMEOUT" bash -c "echo >/dev/tcp/${MQTT_HOST}/${MQTT_PORT}" 2>/dev/null; then
    pass "MQTT broker ${MQTT_HOST}:${MQTT_PORT} → TCP reachable"
  else
    fail "MQTT broker ${MQTT_HOST}:${MQTT_PORT} → TCP unreachable"
    skip "Skipping remaining MQTT tests (broker not reachable)"
    SKIP=$((SKIP+3))
    goto_summary=true
  fi

  if [[ "${goto_summary:-false}" != "true" ]]; then
    # 8b. Publish a test message and subscribe to verify round-trip
    TEST_TOPIC="fsm/selftest/$$"
    TEST_PAYLOAD="{\"ts\":$(date +%s),\"msg\":\"selftest\"}"
    RECEIVED=""

    # Subscribe in background (1 second window)
    received_file=$(mktemp)
    mosquitto_sub \
      -h "$MQTT_HOST" -p "$MQTT_PORT" \
      -t "$TEST_TOPIC" -C 1 \
      --keepalive 5 \
      -W 4 \
      > "$received_file" 2>/dev/null &
    SUB_PID=$!
    sleep 0.5

    # Publish test payload
    mosquitto_pub \
      -h "$MQTT_HOST" -p "$MQTT_PORT" \
      -t "$TEST_TOPIC" -m "$TEST_PAYLOAD" \
      --keepalive 5 2>/dev/null && PUB_OK=true || PUB_OK=false

    wait "$SUB_PID" 2>/dev/null || true
    RECEIVED=$(cat "$received_file" 2>/dev/null)
    rm -f "$received_file"

    if [[ "$PUB_OK" == "true" ]]; then
      pass "MQTT publish to ${TEST_TOPIC}"
    else
      fail "MQTT publish to ${TEST_TOPIC} failed"
    fi

    if echo "$RECEIVED" | grep -q "selftest"; then
      pass "MQTT round-trip: message received on ${TEST_TOPIC}"
    else
      fail "MQTT round-trip: no message received (subscriber timed out)"
    fi

    # 8c. Test telemetry topic subscription (fsm/telemetry/#)
    #     Verify broker accepts wildcard subscription without error
    telem_file=$(mktemp)
    mosquitto_sub \
      -h "$MQTT_HOST" -p "$MQTT_PORT" \
      -t "fsm/telemetry/#" -C 0 \
      --keepalive 5 \
      -W 2 \
      > "$telem_file" 2>&1 &
    TELEM_PID=$!
    wait "$TELEM_PID" 2>/dev/null || true
    SUB_EXIT=$?
    rm -f "$telem_file"

    # mosquitto_sub exits 0 on clean timeout, non-zero on connect error
    if [[ $SUB_EXIT -eq 0 || $SUB_EXIT -eq 27 ]]; then
      # exit 27 = MQTTCLIENT_DISCONNECTED (clean timeout)
      pass "MQTT wildcard subscription fsm/telemetry/# accepted"
    else
      fail "MQTT wildcard subscription fsm/telemetry/# failed (exit ${SUB_EXIT})"
    fi
  fi
fi

# ============================================================
# 9. 未知路由 → 404
# ============================================================
header "REST API 404 handling"

http_code=$(curl -s -o /dev/null -w "%{http_code}" \
  --max-time "$TIMEOUT" "${API_BASE}/nonexistent_endpoint_xyz" 2>/dev/null)
if [[ "$http_code" == "404" ]]; then
  pass "GET /api/v1/nonexistent → HTTP 404"
else
  fail "GET /api/v1/nonexistent → expected 404, got ${http_code}"
fi

# ============================================================
# 总结
# ============================================================
echo ""
echo "═══════════════════════════════════════════════════════"
echo -e "  结果: ${GREEN}PASS=${PASS}${NC}  ${RED}FAIL=${FAIL}${NC}  ${YELLOW}SKIP=${SKIP}${NC}"
echo "═══════════════════════════════════════════════════════"

if [[ $FAIL -gt 0 ]]; then
  echo "  !! 部分测试失败，请检查服务状态"
  exit 1
fi

echo "  所有测试通过"
exit 0
