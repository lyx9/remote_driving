#!/usr/bin/env bash
# ============================================================
# FSM-Pilot 阿里云一键部署脚本
# 使用方式：
#   1. 在 ECS 实例上运行（Ubuntu 22.04 LTS）
#   2. chmod +x deploy.sh && sudo ./deploy.sh
# ============================================================
set -euo pipefail

# ─── 可配置参数（部署前修改）──────────────────────────────
DOMAIN="${FSM_DOMAIN:-your.domain.com}"
EMAIL="${FSM_EMAIL:-admin@your.domain.com}"      # Let's Encrypt 通知邮箱
TURN_SECRET="${FSM_TURN_SECRET:-$(openssl rand -hex 32)}"
PUBLIC_IP="${FSM_PUBLIC_IP:-$(curl -s ifconfig.me)}"
DEPLOY_DIR="/opt/fsm"
LOG_DIR="/var/log/fsm"
# ──────────────────────────────────────────────────────────

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

check_root() {
  [[ $EUID -eq 0 ]] || error "请以 root 权限运行：sudo $0"
}

install_deps() {
  info "安装系统依赖..."
  apt-get update -qq
  apt-get install -y --no-install-recommends \
    curl wget git ca-certificates gnupg lsb-release \
    ufw openssl cron

  # Docker
  if ! command -v docker &>/dev/null; then
    info "安装 Docker..."
    curl -fsSL https://get.docker.com | sh
    systemctl enable --now docker
  fi

  # Docker Compose plugin
  if ! docker compose version &>/dev/null; then
    info "安装 Docker Compose..."
    apt-get install -y docker-compose-plugin
  fi

  # certbot (Let's Encrypt)
  if ! command -v certbot &>/dev/null; then
    info "安装 certbot..."
    snap install --classic certbot 2>/dev/null || \
      apt-get install -y certbot
    ln -sf /snap/bin/certbot /usr/bin/certbot 2>/dev/null || true
  fi
}

configure_firewall() {
  info "配置防火墙 (ufw)..."
  ufw --force reset
  ufw default deny incoming
  ufw default allow outgoing
  ufw allow 22/tcp          comment "SSH"
  ufw allow 80/tcp          comment "HTTP"
  ufw allow 443/tcp         comment "HTTPS/WSS"
  ufw allow 3478/tcp        comment "TURN/STUN TCP"
  ufw allow 3478/udp        comment "TURN/STUN UDP"
  ufw allow 3479/udp        comment "TURN alt UDP"
  ufw allow 5349/tcp        comment "TURN TLS"
  ufw allow 49152:65535/udp comment "TURN relay UDP"
  # EMQX MQTT broker
  ufw allow 1883/tcp        comment "MQTT TCP"
  ufw allow 8883/tcp        comment "MQTT TLS"
  ufw allow 8083/tcp        comment "MQTT WebSocket"
  ufw allow 8084/tcp        comment "MQTT WebSocket TLS"
  # EMQX dashboard (restrict to localhost in production)
  ufw allow from 127.0.0.1 to any port 18083 comment "EMQX Dashboard (local only)"
  ufw --force enable
  info "防火墙规则已应用"
}

setup_dirs() {
  info "创建目录结构..."
  mkdir -p "$DEPLOY_DIR"
  mkdir -p "$LOG_DIR"
  mkdir -p /var/log/coturn
  chmod 755 "$LOG_DIR"
}

obtain_tls_cert() {
  info "申请 TLS 证书 (Let's Encrypt)..."
  if [[ -f "/etc/letsencrypt/live/${DOMAIN}/fullchain.pem" ]]; then
    warn "证书已存在，跳过申请"
    return
  fi

  # certbot standalone 模式：自己在 80 端口监听完成 ACME 验证
  certbot certonly \
    --standalone \
    --non-interactive \
    --agree-tos \
    --email "$EMAIL" \
    -d "$DOMAIN" || \
    error "TLS 证书申请失败，请确认域名 $DOMAIN 已正确解析到本机 IP ($PUBLIC_IP)"

  # 自动续期（每天凌晨 3 点检查，成功后重载 Nginx）
  (crontab -l 2>/dev/null; echo "0 3 * * * certbot renew --quiet && docker compose -f ${DEPLOY_DIR}/docker-compose.yml exec nginx nginx -s reload") \
    | sort -u | crontab -
  info "TLS 证书申请成功，已设置自动续期"
}

generate_configs() {
  info "生成配置文件..."

  # 从模板生成生产配置
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  CONFIG_SRC="${SCRIPT_DIR}/../configs"
  CONFIG_DST="${DEPLOY_DIR}/configs"
  mkdir -p "$CONFIG_DST"

  # Generate JWT secret if not provided
  JWT_SECRET="${FSM_JWT_SECRET:-$(openssl rand -hex 32)}"

  sed -e "s/YOUR_DOMAIN/${DOMAIN}/g" \
      -e "s/YOUR_TURN_SECRET/${TURN_SECRET}/g" \
      -e "s/YOUR_PUBLIC_IP/${PUBLIC_IP}/g" \
      -e "s/YOUR_JWT_SECRET/${JWT_SECRET}/g" \
      "${CONFIG_SRC}/cloud_config.prod.yaml" > "${CONFIG_DST}/cloud_config.yaml"

  sed -e "s/YOUR_DOMAIN/${DOMAIN}/g" \
      -e "s/YOUR_TURN_SECRET/${TURN_SECRET}/g" \
      "${SCRIPT_DIR}/../nginx/nginx.conf" > "${CONFIG_DST}/nginx.conf"

  sed -e "s/YOUR_DOMAIN/${DOMAIN}/g" \
      -e "s/YOUR_TURN_SECRET/${TURN_SECRET}/g" \
      -e "s/YOUR_PUBLIC_IP/${PUBLIC_IP}/g" \
      "${SCRIPT_DIR}/../coturn/turnserver.conf" > "${CONFIG_DST}/turnserver.conf"

  # 覆盖 docker-compose 中的挂载路径
  cp "${SCRIPT_DIR}/../docker-compose.yml" "${DEPLOY_DIR}/docker-compose.yml"

  info "配置文件已生成至 $CONFIG_DST"
  info "TURN_SECRET: $TURN_SECRET  ← 请妥善保存此密钥"
  info "JWT_SECRET:  $JWT_SECRET   ← 请妥善保存此密钥"
}

provision_emqx() {
  info "配置 EMQX MQTT Broker..."

  # EMQX 5.x 使用 REST API 进行用户管理
  local api="http://localhost:18083/api/v5"
  local retries=30

  # 等待 EMQX REST API 就绪
  info "等待 EMQX 启动..."
  while ! curl -sf "${api}/status" -u "admin:public" >/dev/null 2>&1; do
    retries=$((retries - 1))
    if [[ $retries -le 0 ]]; then
      warn "EMQX 未能在超时时间内就绪，请手动登录 Dashboard 完成配置"
      return
    fi
    sleep 2
  done

  # 修改 Dashboard 默认 admin 密码
  EMQX_ADMIN_PASS="${FSM_EMQX_ADMIN_PASS:-$(openssl rand -hex 16)}"
  if curl -sf -X PUT "${api}/users/admin" \
    -H "Content-Type: application/json" \
    -u "admin:public" \
    -d "{\"password\": \"${EMQX_ADMIN_PASS}\"}" >/dev/null 2>&1; then
    info "EMQX Dashboard admin 密码已更新"
  else
    warn "EMQX admin 密码更新失败（可能已被更改），继续使用现有密码"
    EMQX_ADMIN_PASS="public"
  fi

  # 启用内置数据库认证 (Password-Based)
  curl -sf -X POST "${api}/authentication" \
    -H "Content-Type: application/json" \
    -u "admin:${EMQX_ADMIN_PASS}" \
    -d '{"mechanism":"password_based","backend":"built_in_database","password_hash_algorithm":{"name":"sha256","salt_position":"suffix"},"enable":true}' \
    >/dev/null 2>&1 || true

  # 禁用匿名访问
  curl -sf -X PUT "${api}/configs/global_zone" \
    -H "Content-Type: application/json" \
    -u "admin:${EMQX_ADMIN_PASS}" \
    -d '{"mqtt":{"allow_anonymous":false}}' \
    >/dev/null 2>&1 || true

  # 创建 FSM MQTT 应用用户
  EMQX_USER="${FSM_MQTT_USER:-fsm_app}"
  EMQX_PASS="${FSM_MQTT_PASS:-$(openssl rand -hex 16)}"
  if curl -sf -X POST \
    "${api}/authentication/password_based:built_in_database/users" \
    -H "Content-Type: application/json" \
    -u "admin:${EMQX_ADMIN_PASS}" \
    -d "{\"user_id\":\"${EMQX_USER}\",\"password\":\"${EMQX_PASS}\",\"is_superuser\":true}" \
    >/dev/null 2>&1; then
    info "EMQX MQTT 用户创建成功"
  else
    warn "EMQX MQTT 用户创建失败，请手动在 Dashboard 中创建"
  fi

  echo ""
  echo "  ┌─ EMQX 配置信息 ─────────────────────────────────┐"
  echo "  │  Dashboard:   http://${PUBLIC_IP}:18083          │"
  echo "  │  Admin 账号:  admin / ${EMQX_ADMIN_PASS}         │"
  echo "  │  MQTT 用户:   ${EMQX_USER} / ${EMQX_PASS}        │"
  echo "  │  (请将以上信息填入 vehicle_config.yaml 的 mqtt 节) │"
  echo "  └──────────────────────────────────────────────────┘"
  echo ""
  echo "  车端 vehicle_config.yaml MQTT 配置:"
  echo "    mqtt:"
  echo "      enabled: true"
  echo "      broker_url: \"tcp://${DOMAIN}:1883\""
  echo "      username: \"${EMQX_USER}\""
  echo "      password: \"${EMQX_PASS}\""
}

build_and_start() {
  info "构建 Docker 镜像 (fsm_cloud)..."
  cd "$DEPLOY_DIR"
  docker compose build --no-cache fsm_cloud

  info "启动所有服务..."
  docker compose up -d

  info "等待服务健康检查 (最多 60 秒)..."
  local waited=0
  while [[ $waited -lt 60 ]]; do
    sleep 5; waited=$((waited + 5))
    local healthy
    healthy=$(docker compose ps --format json 2>/dev/null | \
      python3 -c "import sys,json; d=json.load(sys.stdin); print(sum(1 for s in (d if isinstance(d,list) else [d]) if s.get('Health','')=='healthy'))" 2>/dev/null || echo "?")
    info "健康服务数: ${healthy}/4"
    if [[ "$healthy" == "4" ]]; then break; fi
  done

  docker compose ps
}

print_summary() {
  echo ""
  echo "═══════════════════════════════════════════════════════"
  echo "  FSM-Pilot 部署完成！"
  echo "═══════════════════════════════════════════════════════"
  echo "  信令服务 (WSS):    wss://${DOMAIN}/ws"
  echo "  API 接口:          https://${DOMAIN}/api"
  echo "  TURN 服务:         turn:${DOMAIN}:3478"
  echo "  STUN 服务:         stun:${DOMAIN}:3478"
  echo "  MQTT TCP:          mqtt://${DOMAIN}:1883"
  echo "  MQTT TLS:          mqtts://${DOMAIN}:8883"
  echo "  MQTT WebSocket:    ws://${DOMAIN}:8083/mqtt"
  echo "  EMQX Dashboard:    http://${PUBLIC_IP}:18083"
  echo ""
  echo "  车端 vehicle_config.yaml 填写:"
  echo "    signaling_url: \"wss://${DOMAIN}/ws\""
  echo "    turn_url:      \"turn:${DOMAIN}:3478\""
  echo "    turn_secret:   \"${TURN_SECRET}\""
  echo "    mqtt.broker_url: \"tcp://${DOMAIN}:1883\""
  echo ""
  echo "  查看日志:"
  echo "    docker compose logs -f fsm_cloud"
  echo "    docker compose logs -f coturn"
  echo "    docker compose logs -f emqx"
  echo "═══════════════════════════════════════════════════════"
}

main() {
  info "FSM-Pilot 阿里云一键部署开始"
  info "目标域名: $DOMAIN  公网IP: $PUBLIC_IP"

  check_root
  install_deps
  configure_firewall
  setup_dirs
  obtain_tls_cert
  generate_configs
  build_and_start
  provision_emqx
  print_summary
}

main "$@"
