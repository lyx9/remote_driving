#!/usr/bin/env bash
# =============================================================================
# FSM-Pilot  |  Autoware.universe 环境下车端部署脚本
# =============================================================================
# 前提：车辆机载电脑上已安装 Autoware.universe（含 autoware_auto_*_msgs）
#
# 用法：
#   export FSM_CLOUD_URL=wss://your-server/ws
#   export FSM_VEHICLE_ID=FSM-CAR-AW-01
#   export AUTOWARE_WS=~/autoware           # Autoware workspace 路径
#   bash vehicle_deploy_autoware.sh
# =============================================================================

set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ─── 参数 ─────────────────────────────────────────────────────────────────────
CLOUD_URL="${FSM_CLOUD_URL:-}"
VEHICLE_ID="${FSM_VEHICLE_ID:-FSM-CAR-AW-01}"
AUTOWARE_WS="${AUTOWARE_WS:-$HOME/autoware}"
FSM_SRC_DIR="${FSM_SRC_DIR:-}"
INSTALL_DIR="${FSM_VEHICLE_DIR:-$HOME/fsm_vehicle}"

[[ -n "$CLOUD_URL" ]] || error "请设置 FSM_CLOUD_URL：export FSM_CLOUD_URL=wss://your-server/ws"

info "============================================================"
info " FSM-Pilot × Autoware.universe 集成部署"
info " 云端地址      : $CLOUD_URL"
info " 车辆 ID      : $VEHICLE_ID"
info " Autoware WS  : $AUTOWARE_WS"
info "============================================================"

# ─── Step 1: 检查 Autoware workspace ─────────────────────────────────────────
check_autoware() {
  local setup_bash="$AUTOWARE_WS/install/setup.bash"
  if [[ ! -f "$setup_bash" ]]; then
    error "未找到 Autoware workspace：$setup_bash\n请设置 AUTOWARE_WS=/path/to/autoware 或先完成 Autoware 安装"
  fi
  # shellcheck disable=SC1090
  source "$setup_bash"
  info "Autoware workspace 已加载：$AUTOWARE_WS"

  # 验证关键消息包
  if ros2 pkg list 2>/dev/null | grep -q "autoware_auto_vehicle_msgs"; then
    info "✓ autoware_auto_vehicle_msgs 可用（HAVE_AUTOWARE_MSGS 将自动启用）"
  else
    warn "未检测到 autoware_auto_vehicle_msgs，将使用通用消息（无 Gear/TurnSignal）"
  fi
}
check_autoware

# ─── Step 2: 找到 FSM 源码 ────────────────────────────────────────────────────
find_fsm_src() {
  if [[ -n "$FSM_SRC_DIR" && -f "$FSM_SRC_DIR/cpp/CMakeLists.txt" ]]; then
    echo "$FSM_SRC_DIR"; return 0
  fi
  for d in "$(cd "$(dirname "$0")/../.." 2>/dev/null && pwd)" \
            "$HOME/fsm" "$HOME/fsm-src" "/opt/fsm-src"; do
    if [[ -f "$d/cpp/CMakeLists.txt" ]]; then echo "$d"; return 0; fi
  done
  return 1
}
FSM_SRC="$(find_fsm_src)" || error "未找到 FSM 源码。请设置 FSM_SRC_DIR=/path/to/fsm"
info "FSM 源码目录：$FSM_SRC"

# ─── Step 3: 将 FSM 包链接/复制进 Autoware colcon workspace ──────────────────
# 方式 A（推荐）：symlink 到 Autoware src/
# 方式 B：standalone colcon build（需手动 source Autoware）
setup_colcon_workspace() {
  local aw_src="$AUTOWARE_WS/src"

  if [[ -d "$aw_src" ]]; then
    # 方式 A：在 Autoware workspace 中以软链接接入 FSM
    local link_target="$aw_src/fsm_pilot"
    if [[ ! -e "$link_target" ]]; then
      ln -sf "$FSM_SRC/cpp" "$link_target"
      info "已创建软链接：$link_target → $FSM_SRC/cpp"
    else
      info "软链接已存在：$link_target"
    fi
    BUILD_IN_AW=true
    BUILD_DIR="$AUTOWARE_WS"
  else
    # 方式 B：standalone build，手动 source Autoware
    warn "Autoware workspace src/ 不存在，使用独立 colcon build"
    BUILD_IN_AW=false
    BUILD_DIR="$HOME/fsm_colcon_ws"
    mkdir -p "$BUILD_DIR/src"
    [[ -e "$BUILD_DIR/src/fsm_pilot" ]] || \
      ln -sf "$FSM_SRC/cpp" "$BUILD_DIR/src/fsm_pilot"
  fi
}
setup_colcon_workspace

# ─── Step 4: 安装额外系统依赖 ────────────────────────────────────────────────
install_extra_deps() {
  info "安装额外依赖（Autoware 通常已含大部分依赖）..."
  sudo apt-get install -y \
    libmosquitto-dev \
    libssl-dev       \
    libyaml-cpp-dev  \
    libspdlog-dev    \
    libx264-dev      \
    2>/dev/null || warn "部分依赖安装失败，继续"
}
install_extra_deps

# ─── Step 5: colcon build ────────────────────────────────────────────────────
build_fsm() {
  info "开始编译 FSM vehicle_node（在 Autoware 环境中）..."
  cd "$BUILD_DIR"

  # 只构建 fsm 相关包，不重建整个 Autoware
  colcon build \
    --packages-select \
      fsm_pilot \
      fsm_vehicle_node \
    --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DFSM_DEMO_MODE=OFF \
      -Wno-dev \
    --symlink-install \
    2>&1 | tail -20

  info "编译完成"
  local setup_file="$BUILD_DIR/install/setup.bash"
  [[ -f "$setup_file" ]] || error "编译后未找到 $setup_file，请检查 colcon 输出"
}
build_fsm

# ─── Step 6: 生成 Autoware 专用配置 ──────────────────────────────────────────
generate_autoware_config() {
  mkdir -p "$INSTALL_DIR/config" "$INSTALL_DIR/logs"
  local cfg="$INSTALL_DIR/config/vehicle_config_autoware.yaml"

  # 将源码中的模板复制过来并替换关键字段
  if [[ -f "$FSM_SRC/cpp/vehicle_node/config/vehicle_config_autoware.yaml" ]]; then
    cp "$FSM_SRC/cpp/vehicle_node/config/vehicle_config_autoware.yaml" "$cfg"
    # 替换 cloud URL 和 vehicle ID
    sed -i "s|wss://YOUR_SERVER/ws|${CLOUD_URL}|g" "$cfg"
    sed -i "s|FSM-CAR-AW-01|${VEHICLE_ID}|g" "$cfg"
    # 替换 STUN/TURN（从 cloud URL 提取主机）
    local cloud_host
    cloud_host=$(echo "$CLOUD_URL" | sed -E 's|wss?://([^/:]+).*|\1|')
    sed -i "s|stun:YOUR_SERVER:3478|stun:${cloud_host}:3478|g" "$cfg"
    sed -i "s|turn:YOUR_SERVER:3478|turn:${cloud_host}:3478|g" "$cfg"
    info "配置文件已生成：$cfg"
  else
    warn "未找到模板配置，请手动配置 $cfg"
    echo "webrtc:" > "$cfg"
    echo "  signaling_url: \"${CLOUD_URL}\"" >> "$cfg"
  fi
}
generate_autoware_config

# ─── Step 7: 安装 systemd 服务 ───────────────────────────────────────────────
install_systemd_service() {
  local setup_aw="$AUTOWARE_WS/install/setup.bash"
  local setup_fsm="$BUILD_DIR/install/setup.bash"
  local cfg="$INSTALL_DIR/config/vehicle_config_autoware.yaml"
  local launch_pkg="fsm_vehicle_node"
  local launch_file="fsm_vehicle_autoware.launch.py"

  sudo tee /etc/systemd/system/fsm_vehicle.service > /dev/null <<SERVICE
[Unit]
Description=FSM-Pilot Vehicle Node (${VEHICLE_ID}) — Autoware.universe mode
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${USER}
WorkingDirectory=${INSTALL_DIR}
ExecStart=/bin/bash -c 'source ${setup_aw} && source ${setup_fsm} && \\
  ros2 launch ${launch_pkg} ${launch_file} \\
    cloud_url:=${CLOUD_URL} \\
    vehicle_id:=${VEHICLE_ID} \\
    config_file:=${cfg}'
Restart=on-failure
RestartSec=10s
StandardOutput=append:${INSTALL_DIR}/logs/fsm_vehicle.log
StandardError=append:${INSTALL_DIR}/logs/fsm_vehicle_err.log
NoNewPrivileges=true

[Install]
WantedBy=multi-user.target
SERVICE

  sudo systemctl daemon-reload
  sudo systemctl enable fsm_vehicle
  info "systemd 服务已安装（fsm_vehicle.service）"
}
install_systemd_service

# ─── Step 8: 验证 Autoware 话题可用性 ────────────────────────────────────────
verify_autoware_topics() {
  info "检查 Autoware 关键话题（需要 Autoware 正在运行）..."
  local topics=(
    "/vehicle/status/velocity_status"
    "/vehicle/status/steering_status"
    "/localization/kinematic_state"
  )
  local ok=true
  for t in "${topics[@]}"; do
    if timeout 2 ros2 topic info "$t" &>/dev/null 2>&1; then
      info "✓ $t"
    else
      warn "✗ $t （Autoware 未运行或话题不存在，部署后启动 Autoware 再验证）"
      ok=false
    fi
  done
  $ok && info "所有话题验证通过" || warn "部分话题不可用，请启动 Autoware 后再验证"
}
verify_autoware_topics

# ─── 完成摘要 ─────────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}  FSM-Pilot × Autoware.universe 部署完成！${NC}"
echo -e "${GREEN}============================================================${NC}"
echo ""
echo "  车辆 ID       : $VEHICLE_ID"
echo "  云端地址      : $CLOUD_URL"
echo "  配置文件      : $INSTALL_DIR/config/vehicle_config_autoware.yaml"
echo "  日志目录      : $INSTALL_DIR/logs/"
echo ""
echo "  手动启动（推荐调试时使用）："
echo "    source $AUTOWARE_WS/install/setup.bash"
echo "    source $BUILD_DIR/install/setup.bash"
echo "    ros2 launch fsm_vehicle_node fsm_vehicle_autoware.launch.py \\"
echo "      cloud_url:=$CLOUD_URL \\"
echo "      vehicle_id:=$VEHICLE_ID"
echo ""
echo "  systemd 服务："
echo "    sudo systemctl start  fsm_vehicle"
echo "    sudo systemctl status fsm_vehicle"
echo "    journalctl -u fsm_vehicle -f"
echo ""
echo "  操作模式手动切换（Autoware 运行时）："
echo "    # 切换到 REMOTE（FSM 控制）："
echo "    ros2 service call /system/operation_mode/change_operation_mode \\"
echo "      autoware_adapi_v1_msgs/srv/ChangeOperationMode '{}'"
echo ""
echo "    # 切换回自动驾驶："
echo "    ros2 service call /system/operation_mode/change_operation_mode \\"
echo "      autoware_adapi_v1_msgs/srv/ChangeOperationMode '{mode: 2}'"
echo ""
echo "  话题验证（Autoware 运行中）："
echo "    ros2 topic echo /vehicle/status/velocity_status"
echo "    ros2 topic echo /external/selected/control_cmd"
echo ""
