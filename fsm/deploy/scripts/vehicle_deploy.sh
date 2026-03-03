#!/usr/bin/env bash
# =============================================================================
# FSM-Pilot  |  车端一键部署脚本
# =============================================================================
# 用法：
#   export FSM_CLOUD_URL=wss://fsm.yourdomain.com/ws   # 云端 WebSocket 地址
#   export FSM_VEHICLE_ID=FSM-CAR-01                   # 车辆唯一 ID
#   export FSM_DEMO_MODE=ON                            # Demo 模式（无真实硬件）
#   bash vehicle_deploy.sh
#
# 系统要求：Ubuntu 22.04 + ROS2 Humble
# =============================================================================

set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ─── 参数 ─────────────────────────────────────────────────────────────────────
CLOUD_URL="${FSM_CLOUD_URL:-}"
VEHICLE_ID="${FSM_VEHICLE_ID:-FSM-CAR-01}"
DEMO_MODE="${FSM_DEMO_MODE:-ON}"           # ON = 仿真模式, OFF = 真实硬件
DEMO_SCENARIO="${FSM_DEMO_SCENARIO:-idle}" # straight_line / round_about / slalom / idle
INSTALL_DIR="${FSM_VEHICLE_DIR:-$HOME/fsm_vehicle}"
SRC_DIR="${FSM_SRC_DIR:-}"                 # 源码目录；留空则自动查找

[[ -n "$CLOUD_URL" ]] || error "请设置 FSM_CLOUD_URL，例如：export FSM_CLOUD_URL=wss://your-server/ws"

info "============================================================"
info " FSM-Pilot 车端部署"
info " 云端地址   : $CLOUD_URL"
info " 车辆 ID   : $VEHICLE_ID"
info " 模式      : ${DEMO_MODE}"
info " 安装目录  : $INSTALL_DIR"
info "============================================================"

# ─── Step 1: 查找源码目录 ─────────────────────────────────────────────────────
find_src_dir() {
  if [[ -n "$SRC_DIR" && -f "$SRC_DIR/cpp/CMakeLists.txt" ]]; then
    echo "$SRC_DIR"; return 0
  fi
  for d in \
    "$(cd "$(dirname "$0")/../.." 2>/dev/null && pwd)" \
    "$HOME/fsm-src" \
    "$HOME/fsm" \
    "/opt/fsm-src"
  do
    if [[ -f "$d/cpp/CMakeLists.txt" ]]; then
      echo "$d"; return 0
    fi
  done
  return 1
}

SRC_DIR="$(find_src_dir)" || error "未找到 FSM-Pilot 源码目录。请设置 FSM_SRC_DIR=/path/to/fsm 或将源码放置于 ~/fsm"
info "源码目录：$SRC_DIR"

# ─── Step 2: 安装系统依赖 ─────────────────────────────────────────────────────
install_deps() {
  info "安装系统依赖..."
  sudo apt-get update -qq

  # 基础编译工具
  sudo apt-get install -y \
    build-essential cmake git curl wget \
    libboost-system-dev libssl-dev \
    libyaml-cpp-dev libspdlog-dev \
    libprotobuf-dev protobuf-compiler \
    nlohmann-json3-dev \
    libopencv-dev \
    libmosquitto-dev \
    2>/dev/null || warn "部分依赖安装失败，将在编译时报错"
  info "基础依赖安装完成"
}
install_deps

# ─── Step 3: 安装 ROS2 Humble（如未安装）─────────────────────────────────────
install_ros2() {
  if command -v ros2 &>/dev/null; then
    info "ROS2 已安装：$(ros2 --version 2>&1 | head -1)"
    return 0
  fi

  info "安装 ROS2 Humble..."
  sudo apt-get install -y software-properties-common
  sudo add-apt-repository universe -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list
  sudo apt-get update -qq
  sudo apt-get install -y ros-humble-ros-base python3-colcon-common-extensions
  # 安装常用消息包
  sudo apt-get install -y \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    2>/dev/null || true

  # 添加 source 到 bashrc
  if ! grep -q "ros-humble" "$HOME/.bashrc" 2>/dev/null; then
    echo "source /opt/ros/humble/setup.bash" >> "$HOME/.bashrc"
  fi

  info "ROS2 Humble 安装完成"
}
install_ros2

# source ROS2
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null || warn "ROS2 环境未加载，尝试继续"

# ─── Step 4: 安装 rosdep 依赖 ─────────────────────────────────────────────────
install_rosdep() {
  if ! command -v rosdep &>/dev/null; then
    sudo apt-get install -y python3-rosdep 2>/dev/null || \
      pip3 install rosdep 2>/dev/null || warn "rosdep 安装失败，跳过"
    return
  fi
  sudo rosdep init 2>/dev/null || true
  rosdep update 2>/dev/null || warn "rosdep update 失败，继续"
  rosdep install --from-paths "$SRC_DIR/cpp" --ignore-src -r -y 2>/dev/null \
    || warn "部分 rosdep 依赖未安装，继续编译"
}
install_rosdep

# ─── Step 5: 编译 ─────────────────────────────────────────────────────────────
build_vehicle_node() {
  local build_type="Release"
  local cmake_flags="-DCMAKE_BUILD_TYPE=${build_type}"

  if [[ "$DEMO_MODE" == "ON" ]]; then
    cmake_flags="$cmake_flags -DFSM_DEMO_MODE=ON"
    info "编译模式：DEMO（仿真车辆，无真实硬件）"
  else
    info "编译模式：PRODUCTION（真实硬件）"
  fi

  local build_dir="$SRC_DIR/cpp/build_vehicle"
  mkdir -p "$build_dir"

  info "CMake 配置..."
  cmake -S "$SRC_DIR/cpp" -B "$build_dir" $cmake_flags -Wno-dev 2>&1 | tail -8

  info "开始编译（使用 $(nproc) 个核心）..."
  cmake --build "$build_dir" --target fsm_vehicle_node -j"$(nproc)" 2>&1 | tail -15

  # 检查产物
  local bin_path="$build_dir/vehicle_node/fsm_vehicle_node"
  [[ -f "$bin_path" ]] || error "编译失败：未找到 $bin_path"
  info "编译成功：$bin_path"

  # 安装到 INSTALL_DIR
  mkdir -p "$INSTALL_DIR/bin"
  cp "$bin_path" "$INSTALL_DIR/bin/"
  info "已安装：$INSTALL_DIR/bin/fsm_vehicle_node"
}
build_vehicle_node

# ─── Step 6: 生成车端配置 ─────────────────────────────────────────────────────
generate_config() {
  mkdir -p "$INSTALL_DIR/config"
  local cfg="$INSTALL_DIR/config/vehicle_config.yaml"

  info "生成车端配置：$cfg"
  cat > "$cfg" <<YAML
vehicle:
  id: "${VEHICLE_ID}"
  type: "robotaxi"

vehicle_params:
  max_speed_kph: 30.0
  max_steering_angle_deg: 35.0
  max_acceleration_mps2: 3.0
  max_deceleration_mps2: 6.0
  emergency_decel_mps2: 8.0
  max_steering_rate_dps: 60.0

webrtc:
  signaling_url: "${CLOUD_URL}"
  stun_servers: ["${CLOUD_URL/wss/stun}"]
  turn_servers: ["${CLOUD_URL/wss:\/\//turn:}"]

cameras:
  - id: "front_center"
    topic: "/sensing/camera/front/image_raw"
    fps: 15
    bitrate_kbps: 2000
    encoder: software_x264

mqtt:
  enabled: false   # 可选辅助通道；需云端 EMQX 正常运行

safety:
  watchdog_timeout_ms: 500
  command_publish_rate_hz: 20
  soft_limits_enabled: true

logging:
  level: info
  file: ${INSTALL_DIR}/logs/vehicle.log
YAML

  # Demo 专用配置
  if [[ "$DEMO_MODE" == "ON" ]]; then
    cat >> "$cfg" <<YAML

demo:
  scenario: "${DEMO_SCENARIO}"
  start_lat: 31.2304
  start_lon: 121.4737
  cruise_speed_mps: 5.0
  loop_radius_m: 20.0
  battery_pct: 85.0
  update_rate_hz: 20.0
YAML
  fi

  info "配置文件已生成：$cfg"
}
generate_config

mkdir -p "$INSTALL_DIR/logs"

# ─── Step 7: 创建 systemd 服务 ────────────────────────────────────────────────
install_systemd_service() {
  local service_file="/etc/systemd/system/fsm_vehicle.service"
  info "安装 systemd 服务：$service_file"

  local ros2_setup="/opt/ros/humble/setup.bash"
  local bin="$INSTALL_DIR/bin/fsm_vehicle_node"
  local cfg="$INSTALL_DIR/config/vehicle_config.yaml"

  sudo tee "$service_file" > /dev/null <<SERVICE
[Unit]
Description=FSM-Pilot Vehicle Node (${VEHICLE_ID})
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${USER}
WorkingDirectory=${INSTALL_DIR}
EnvironmentFile=-${INSTALL_DIR}/config/env
ExecStartPre=/bin/bash -c 'source ${ros2_setup}'
ExecStart=/bin/bash -c 'source ${ros2_setup} && ${bin} ${cfg}'
Restart=on-failure
RestartSec=5s
StandardOutput=append:${INSTALL_DIR}/logs/vehicle.log
StandardError=append:${INSTALL_DIR}/logs/vehicle_err.log

# 安全加固
NoNewPrivileges=true
PrivateTmp=true

[Install]
WantedBy=multi-user.target
SERVICE

  sudo systemctl daemon-reload
  sudo systemctl enable fsm_vehicle
  info "systemd 服务已安装"
}
install_systemd_service

# ─── Step 8: 自测（连接验证） ─────────────────────────────────────────────────
self_test() {
  info "执行自测..."

  # 检查二进制
  local bin="$INSTALL_DIR/bin/fsm_vehicle_node"
  [[ -f "$bin" && -x "$bin" ]] || { warn "二进制不存在或无执行权限：$bin"; return 1; }
  info "✓ 二进制文件正常"

  # 在 Demo 模式下验证 build_variant
  if [[ "$DEMO_MODE" == "ON" ]]; then
    if strings "$bin" 2>/dev/null | grep -q "build_variant=DEMO"; then
      info "✓ build_variant=DEMO 已确认"
    else
      warn "未找到 build_variant=DEMO 标记（可能是 strip 后的二进制）"
    fi
  fi

  # 网络连通性：尝试 WebSocket（需要 wscat 或 curl）
  local ws_host
  ws_host=$(echo "$CLOUD_URL" | sed -E 's|wss?://([^/:]+).*|\1|')
  local ws_port
  ws_port=$(echo "$CLOUD_URL" | sed -E 's|.*:([0-9]+).*|\1|; t; s|.*|443|')

  if command -v nc &>/dev/null; then
    if nc -z -w5 "$ws_host" "$ws_port" 2>/dev/null; then
      info "✓ TCP 连通 $ws_host:$ws_port"
    else
      warn "✗ 无法连接 $ws_host:$ws_port（确认云端已启动 + 安全组已开放）"
    fi
  else
    warn "nc 未安装，跳过网络连通性测试"
  fi

  info "自测完成"
}
self_test

# ─── 完成摘要 ─────────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}  FSM-Pilot 车端部署完成！${NC}"
echo -e "${GREEN}============================================================${NC}"
echo ""
echo "  车辆 ID      : $VEHICLE_ID"
echo "  云端地址     : $CLOUD_URL"
echo "  模式         : $DEMO_MODE"
echo "  安装目录     : $INSTALL_DIR"
echo "  配置文件     : $INSTALL_DIR/config/vehicle_config.yaml"
echo "  日志目录     : $INSTALL_DIR/logs/"
echo ""
echo "  启动命令（手动）："
echo "    source /opt/ros/humble/setup.bash"
echo "    $INSTALL_DIR/bin/fsm_vehicle_node $INSTALL_DIR/config/vehicle_config.yaml"
echo ""
echo "  systemd 服务："
echo "    sudo systemctl start  fsm_vehicle   # 启动"
echo "    sudo systemctl status fsm_vehicle   # 状态"
echo "    sudo systemctl stop   fsm_vehicle   # 停止"
echo "    journalctl -u fsm_vehicle -f        # 实时日志"
echo ""
if [[ "$DEMO_MODE" == "ON" ]]; then
  echo -e "  ${YELLOW}[Demo 模式] 使用仿真车辆，场景：${DEMO_SCENARIO}${NC}"
  echo -e "  ${YELLOW}  可选场景：straight_line / round_about / slalom / emergency_brake / idle${NC}"
  echo -e "  ${YELLOW}  修改场景：编辑 $INSTALL_DIR/config/vehicle_config.yaml 中的 demo.scenario${NC}"
fi
echo ""
