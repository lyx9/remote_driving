#!/bin/bash

###############################################################################
# Guardian Mobility - System Verification Script
###############################################################################

set -e

GREEN='\033[0;32m'
RED='\033[0;31m'
ORANGE='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                                                                ║${NC}"
echo -e "${CYAN}║         Guardian Mobility - System Verification                ║${NC}"
echo -e "${CYAN}║                                                                ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""

PASS=0
FAIL=0

check() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC} $1"
        PASS=$((PASS + 1))
    else
        echo -e "${RED}✗${NC} $1"
        FAIL=$((FAIL + 1))
    fi
}

echo -e "${CYAN}[1/6] Checking Services${NC}"
echo ""

# Check RosBag server
curl -s http://localhost:8765/health > /dev/null 2>&1
check "RosBag server running (port 8765)"

# Check frontend
curl -s http://localhost:3000 > /dev/null 2>&1
check "Frontend server running (port 3000)"

echo ""
echo -e "${CYAN}[2/6] Checking RosBag Files${NC}"
echo ""

# Check RosBag file exists
test -f "/home/lyx/fsm/rosbag/1210/rosbag2_2025_12_10-17_25_58/rosbag2_2025_12_10-17_25_58_0.db3"
check "RosBag database file exists"

# Check metadata
test -f "/home/lyx/fsm/rosbag/1210/rosbag2_2025_12_10-17_25_58/metadata.yaml"
check "RosBag metadata file exists"

echo ""
echo -e "${CYAN}[3/6] Checking API Endpoints${NC}"
echo ""

# Check bags API
BAGS=$(curl -s http://localhost:8765/api/bags | jq -r '.bags | length')
test "$BAGS" -gt 0
check "RosBag API returns bags ($BAGS found)"

echo ""
echo -e "${CYAN}[4/6] Checking CDR Decoder${NC}"
echo ""

# Check decoder file
test -f "/home/lyx/fsm/server/cdr-decoder.js"
check "CDR decoder file exists"

# Check decoder is imported
grep -q "cdr-decoder" /home/lyx/fsm/server/rosbag-server.js
check "CDR decoder imported in server"

echo ""
echo -e "${CYAN}[5/6] Testing Image Decoding${NC}"
echo ""

# Run image decode test
cd /home/lyx/fsm
timeout 30 node test_image_decode.cjs > /tmp/decode_test.log 2>&1
check "Image decoding test passed"

# Check test results
IMAGES=$(grep -o "Image messages: [0-9]*" /tmp/decode_test.log | grep -o "[0-9]*")
test "$IMAGES" -ge 50
check "Decoded $IMAGES images (expected ≥50)"

echo ""
echo -e "${CYAN}[6/6] Checking Frontend Components${NC}"
echo ""

# Check RosBagReplayPro component
test -f "/home/lyx/fsm/src/components/RosBagReplayPro.vue"
check "RosBagReplayPro component exists"

# Check message handlers updated
grep -q "messageType === 'image'" /home/lyx/fsm/src/components/RosBagReplayPro.vue
check "Frontend handles decoded images"

echo ""
echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}╔════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║                                                                ║${NC}"
    echo -e "${GREEN}║              ✅ ALL TESTS PASSED ($PASS/$((PASS + FAIL)))                          ║${NC}"
    echo -e "${GREEN}║                                                                ║${NC}"
    echo -e "${GREEN}║         RosBag Image Decoding: OPERATIONAL                     ║${NC}"
    echo -e "${GREEN}║                                                                ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${CYAN}🌐 Access the application:${NC}"
    echo -e "   Frontend:     ${ORANGE}http://localhost:3000${NC}"
    echo -e "   RosBag Replay: ${ORANGE}http://localhost:3000/rosbag-replay-pro${NC}"
    echo ""
    echo -e "${CYAN}🔐 Login:${NC}"
    echo -e "   Username: ${ORANGE}cityu${NC}"
    echo -e "   Password: ${ORANGE}2026${NC}"
    echo ""
    exit 0
else
    echo -e "${RED}╔════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${RED}║                                                                ║${NC}"
    echo -e "${RED}║              ❌ SOME TESTS FAILED                              ║${NC}"
    echo -e "${RED}║                                                                ║${NC}"
    echo -e "${RED}║         Passed: $PASS | Failed: $FAIL                                    ║${NC}"
    echo -e "${RED}║                                                                ║${NC}"
    echo -e "${RED}╚════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    exit 1
fi
