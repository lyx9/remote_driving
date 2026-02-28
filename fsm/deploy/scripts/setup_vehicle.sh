#!/usr/bin/env bash
# ============================================================
# FSM-Pilot 车端 Ubuntu 安装脚本
# 在车载工控机（Ubuntu 22.04 + ROS2 Humble）上运行
# 使用方式: sudo ./setup_vehicle.sh <cloud_domain> <vehicle_id>
# 示例:     sudo ./setup_vehicle.sh your.domain.com FSM-01
# ============================================================
set -euo pipefail

CLOUD_DOMAIN="${1:-your.domain.com}"
VEHICLE_ID="${2:-FSM-01}"
INSTALL_DIR="/opt/fsm_vehicle"
SERVICE_FILE="/etc/systemd/system/fsm-vehicle.service"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

[[ $EUID -eq 0 ]] || error "请以 root 权限运行"

check_ros2() {
  source /opt/ros/humble/setup.bash 2>/dev/null || \
    error "未找到 ROS2 Humble，请先安装: https://docs.ros.org/en/humble/Installation.html"
  info "ROS2 Humble 检测正常"
}

install_cpp_deps() {
  info "安装 C++ 依赖..."
  apt-get update -qq
  apt-get install -y --no-install-recommends \
    libboost-all-dev \
    libssl-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libprotobuf-dev \
    protobuf-compiler \
    nlohmann-json3-dev \
    libopencv-dev \
    libx264-dev \
    libmosquitto-dev \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs
}

build_vehicle_node() {
  info "编译 vehicle_node..."
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  SRC_DIR="${SCRIPT_DIR}/../../cpp"

  mkdir -p "$INSTALL_DIR"
  source /opt/ros/humble/setup.bash

  cmake -S "$SRC_DIR" -B /tmp/fsm_build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
    -DBUILD_TESTING=OFF \
    -DUSE_GRPC=OFF

  cmake --build /tmp/fsm_build --target fsm_vehicle_node -j$(nproc)
  cmake --install /tmp/fsm_build --component Runtime 2>/dev/null || \
    cp /tmp/fsm_build/vehicle_node/fsm_vehicle_node "$INSTALL_DIR/bin/"

  info "编译完成: $INSTALL_DIR/bin/fsm_vehicle_node"
}

generate_vehicle_config() {
  info "生成车端配置..."
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  CONFIG_TEMPLATE="${SCRIPT_DIR}/../configs/vehicle_config.template.yaml"
  CONFIG_DST="${INSTALL_DIR}/config/vehicle_config.yaml"

  mkdir -p "${INSTALL_DIR}/config"

  sed -e "s/FSM-01/${VEHICLE_ID}/g" \
      -e "s/YOUR_DOMAIN/${CLOUD_DOMAIN}/g" \
      "$CONFIG_TEMPLATE" > "$CONFIG_DST"

  info "配置已生成: $CONFIG_DST"
  warn "请手动编辑 $CONFIG_DST，填写以下占位符:"
  warn "  YOUR_TURN_SECRET — 与云端 deploy.sh 输出的 turn_secret 一致"
  warn "  mqtt.username / mqtt.password — 与 EMQX MQTT 用户信息一致"
}

install_systemd_service() {
  info "安装 systemd 服务..."
  cat > "$SERVICE_FILE" << EOF
[Unit]
Description=FSM-Pilot Vehicle Node
After=network-online.target ros2.service
Wants=network-online.target

[Service]
Type=simple
User=root
Environment="ROS_DOMAIN_ID=42"
EnvironmentFile=/opt/ros/humble/setup.sh
ExecStart=${INSTALL_DIR}/bin/fsm_vehicle_node ${INSTALL_DIR}/config/vehicle_config.yaml
Restart=on-failure
RestartSec=5s
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

  systemctl daemon-reload
  systemctl enable fsm-vehicle
  info "systemd 服务已安装并启用"
  info "启动服务: systemctl start fsm-vehicle"
  info "查看日志: journalctl -u fsm-vehicle -f"
}

main() {
  info "FSM-Pilot 车端安装开始"
  info "云端域名: $CLOUD_DOMAIN  车辆ID: $VEHICLE_ID"
  check_ros2
  install_cpp_deps
  build_vehicle_node
  generate_vehicle_config
  install_systemd_service
  info "车端安装完成！"
}

main "$@"
