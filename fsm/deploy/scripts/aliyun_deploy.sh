#!/usr/bin/env bash
# =============================================================================
# FSM-Pilot  |  阿里云 ECS 一键部署脚本
# =============================================================================
# 用法：
#   export FSM_DOMAIN=fsm.yourdomain.com   # 有域名时填写；无域名则留空，用自签证书
#   bash aliyun_deploy.sh
#
# 依赖：Docker, Docker Compose (脚本自动安装)
# 兼容系统：Ubuntu 22.04 LTS (推荐)
# =============================================================================

set -euo pipefail

# ─── 颜色输出 ─────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# ─── 参数与路径 ───────────────────────────────────────────────────────────────
INSTALL_DIR="${FSM_INSTALL_DIR:-/opt/fsm}"
DOMAIN="${FSM_DOMAIN:-}"        # 可选；空字符串 = 用自签证书 + 公网IP
EMAIL="${FSM_EMAIL:-}"          # Let's Encrypt 注册邮箱；有域名时必填
TURN_PASSWD="${FSM_TURN_PASSWD:-$(openssl rand -hex 12)}"
JWT_SECRET="${FSM_JWT_SECRET:-$(openssl rand -hex 32)}"

# ─── 检测公网 IP ──────────────────────────────────────────────────────────────
detect_public_ip() {
  for url in "https://ifconfig.me" "https://api.ipify.org" "https://ipecho.net/plain"; do
    ip=$(curl -sfL --max-time 5 "$url" 2>/dev/null || true)
    if [[ -n "$ip" && "$ip" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
      echo "$ip"; return 0
    fi
  done
  error "无法检测公网 IP，请手动设置 FSM_PUBLIC_IP 环境变量"
}

PUBLIC_IP="${FSM_PUBLIC_IP:-$(detect_public_ip)}"
SERVER_HOST="${DOMAIN:-$PUBLIC_IP}"

info "============================================================"
info " FSM-Pilot 一键部署"
info " 公网 IP : $PUBLIC_IP"
info " 服务地址 : $SERVER_HOST"
info " 安装目录 : $INSTALL_DIR"
info "============================================================"

# ─── Step 0: 安装 Docker ──────────────────────────────────────────────────────
install_docker() {
  if command -v docker &>/dev/null && docker compose version &>/dev/null 2>&1; then
    info "Docker 已安装: $(docker --version)"
    return 0
  fi
  info "安装 Docker..."
  # 清理冲突包（如 podman-docker）
  apt-get remove -y docker docker-engine docker.io containerd runc 2>/dev/null || true
  apt-get update -qq
  apt-get install -y ca-certificates curl gnupg lsb-release
  install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
    | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  chmod a+r /etc/apt/keyrings/docker.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
    https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" \
    > /etc/apt/sources.list.d/docker.list
  apt-get update -qq
  apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
  systemctl enable docker --now
  info "Docker 安装完成: $(docker --version)"
}

# 需要 root
[[ "$(id -u)" -eq 0 ]] || error "请以 root 身份运行（sudo bash $0）"
install_docker

# ─── Step 1: 创建目录结构 ─────────────────────────────────────────────────────
info "创建目录结构..."
mkdir -p "$INSTALL_DIR"/{configs,certs,logs,data/emqx}

# ─── Step 2: 生成/复制 TLS 证书 ──────────────────────────────────────────────
setup_tls() {
  local cert_dir="$INSTALL_DIR/certs"

  if [[ -n "$DOMAIN" ]]; then
    # ── 有域名：尝试 Let's Encrypt ──────────────────────────────────────────
    info "申请 Let's Encrypt 证书 (域名=$DOMAIN)..."
    if ! command -v certbot &>/dev/null; then
      apt-get install -y certbot
    fi
    # 验证域名解析
    resolved_ip=$(dig +short "$DOMAIN" | tail -1)
    if [[ "$resolved_ip" != "$PUBLIC_IP" ]]; then
      warn "域名 $DOMAIN 当前解析到 $resolved_ip，期望 $PUBLIC_IP"
      warn "DNS 未生效，切换为自签证书模式；证书申请成功后可重新运行脚本"
      DOMAIN=""  # 降级
    else
      [[ -n "$EMAIL" ]] || error "使用域名申请证书需设置 FSM_EMAIL"
      certbot certonly --standalone --non-interactive --agree-tos \
        -m "$EMAIL" -d "$DOMAIN" \
        --cert-path "$cert_dir/cert.pem" \
        --key-path  "$cert_dir/key.pem"  || {
        warn "Let's Encrypt 申请失败，降级为自签证书"
        DOMAIN=""
      }
    fi
  fi

  if [[ -z "$DOMAIN" ]]; then
    # ── 无域名或申请失败：生成自签证书 ──────────────────────────────────────
    if [[ ! -f "$cert_dir/cert.pem" ]]; then
      info "生成自签 TLS 证书（CN=$PUBLIC_IP）..."
      openssl req -x509 -newkey rsa:2048 -days 3650 -nodes \
        -keyout "$cert_dir/key.pem" \
        -out    "$cert_dir/cert.pem" \
        -subj   "/CN=$PUBLIC_IP" \
        -addext "subjectAltName=IP:$PUBLIC_IP" 2>/dev/null
    else
      info "已存在证书，跳过生成"
    fi
    SERVER_HOST="$PUBLIC_IP"
  fi

  chmod 600 "$cert_dir"/*.pem
  info "TLS 证书就绪：$cert_dir"
}
setup_tls

# ─── Step 3: 写入配置文件 ─────────────────────────────────────────────────────
info "写入云端配置..."
cat > "$INSTALL_DIR/configs/cloud_config.yaml" <<YAML
server:
  signaling_port: 8080
  api_port: 8081

api:
  auth:
    jwt_secret: "${JWT_SECRET}"
  session_timeout_s: 300

scheduling:
  algorithm: weighted_priority
  weights:
    emergency: 0.35
    latency:   0.25
    battery:   0.10

mqtt:
  enabled: true
  broker_host: "emqx"
  broker_port: 1883

logging:
  level: info
  file: /var/log/fsm/cloud.log
YAML

# ─── Step 4: 写入 nginx 配置 ──────────────────────────────────────────────────
info "写入 nginx 配置..."
mkdir -p "$INSTALL_DIR/nginx"
cat > "$INSTALL_DIR/nginx/nginx.conf" <<NGINX
events { worker_connections 1024; }

http {
    # 强制 HTTP → HTTPS
    server {
        listen 80;
        server_name _;
        return 301 https://\$host\$request_uri;
    }

    server {
        listen 443 ssl;
        server_name ${SERVER_HOST};

        ssl_certificate     /etc/nginx/certs/cert.pem;
        ssl_certificate_key /etc/nginx/certs/key.pem;
        ssl_protocols       TLSv1.2 TLSv1.3;
        ssl_ciphers         HIGH:!aNULL:!MD5;

        # 健康检查（无需 TLS 客户端验证）
        location /api/v1/health {
            proxy_pass http://fsm_cloud:8081;
        }

        # Prometheus 指标
        location /metrics {
            proxy_pass http://fsm_cloud:8081;
        }

        # WebSocket 信令
        location /ws {
            proxy_pass         http://fsm_cloud:8080;
            proxy_http_version 1.1;
            proxy_set_header   Upgrade    \$http_upgrade;
            proxy_set_header   Connection "upgrade";
            proxy_read_timeout 3600s;
        }

        # REST API
        location /api/ {
            proxy_pass http://fsm_cloud:8081;
            proxy_set_header Host \$host;
            proxy_set_header X-Real-IP \$remote_addr;
        }
    }
}
NGINX

# ─── Step 5: 写入 coturn 配置 ─────────────────────────────────────────────────
info "写入 coturn 配置..."
mkdir -p "$INSTALL_DIR/coturn"
cat > "$INSTALL_DIR/coturn/turnserver.conf" <<TURN
listening-port=3478
tls-listening-port=5349
external-ip=${PUBLIC_IP}
fingerprint
use-auth-secret
static-auth-secret=${TURN_PASSWD}
realm=${SERVER_HOST}
log-file=/var/log/coturn/coturn.log
simple-log
no-multicast-peers
denied-peer-ip=10.0.0.0-10.255.255.255
denied-peer-ip=172.16.0.0-172.31.255.255
denied-peer-ip=192.168.0.0-192.168.255.255
TURN

# ─── Step 6: 写入 docker-compose.yml ─────────────────────────────────────────
info "写入 docker-compose.yml..."
cat > "$INSTALL_DIR/docker-compose.yml" <<COMPOSE
version: "3.9"

services:
  # ── FSM 云端服务器 ───────────────────────────────────────────────────────────
  fsm_cloud:
    image: ubuntu:22.04
    container_name: fsm_cloud
    restart: unless-stopped
    network_mode: host          # 使用宿主网络，简化端口映射
    volumes:
      - ${INSTALL_DIR}/configs:/opt/fsm/configs:ro
      - ${INSTALL_DIR}/logs:/var/log/fsm
      - /opt/fsm-bin:/opt/fsm-bin:ro   # 编译好的二进制
    environment:
      - FSM_CONFIG=/opt/fsm/configs/cloud_config.yaml
    command: >
      bash -c "
        mkdir -p /var/log/fsm &&
        /opt/fsm-bin/fsm_cloud_server /opt/fsm/configs/cloud_config.yaml
      "
    healthcheck:
      test: ["CMD", "curl", "-sf", "http://localhost:8081/api/v1/health"]
      interval: 15s
      timeout: 5s
      retries: 3

  # ── nginx TLS 终止 ──────────────────────────────────────────────────────────
  nginx:
    image: nginx:1.25-alpine
    container_name: fsm_nginx
    restart: unless-stopped
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ${INSTALL_DIR}/nginx/nginx.conf:/etc/nginx/nginx.conf:ro
      - ${INSTALL_DIR}/certs:/etc/nginx/certs:ro
    depends_on:
      - fsm_cloud

  # ── coturn STUN/TURN ────────────────────────────────────────────────────────
  coturn:
    image: coturn/coturn:4.6
    container_name: fsm_coturn
    restart: unless-stopped
    network_mode: host
    volumes:
      - ${INSTALL_DIR}/coturn/turnserver.conf:/etc/coturn/turnserver.conf:ro
    command: ["--config", "/etc/coturn/turnserver.conf"]

  # ── EMQX MQTT Broker ────────────────────────────────────────────────────────
  emqx:
    image: emqx/emqx:5.6.0
    container_name: fsm_emqx
    restart: unless-stopped
    ports:
      - "1883:1883"    # MQTT
      - "8083:8083"    # MQTT over WebSocket
    volumes:
      - ${INSTALL_DIR}/data/emqx:/opt/emqx/data
    environment:
      - EMQX_ALLOW_ANONYMOUS=true   # Demo 模式；生产请改为 false 并配置 ACL
      - EMQX_LOG__CONSOLE_HANDLER__LEVEL=warning
COMPOSE

# ─── Step 7: 构建云端二进制（如有源码） ───────────────────────────────────────
build_cloud_server() {
  local src_dir
  # 查找源码目录
  for d in /opt/fsm-src ~/fsm-src "$(dirname "$(dirname "$0")")/../.."; do
    if [[ -f "$d/cpp/CMakeLists.txt" ]]; then
      src_dir="$(realpath "$d")"
      break
    fi
  done

  if [[ -z "${src_dir:-}" ]]; then
    warn "未找到源码目录，跳过编译（假设已有预编译二进制）"
    warn "若需编译，请将源码放置于 /opt/fsm-src 并重新运行脚本"
    return 0
  fi

  info "检测到源码目录：$src_dir，开始编译云端服务器..."

  # 安装编译依赖（最小集合）
  apt-get install -y \
    build-essential cmake git \
    libboost-system-dev libssl-dev \
    libyaml-cpp-dev libspdlog-dev \
    libprotobuf-dev protobuf-compiler \
    nlohmann-json3-dev \
    2>/dev/null || warn "部分依赖包安装失败，继续尝试编译"

  mkdir -p "$src_dir/cpp/build_cloud"
  cmake -S "$src_dir/cpp" -B "$src_dir/cpp/build_cloud" \
    -DCMAKE_BUILD_TYPE=Release \
    -DFSM_DEMO_MODE=OFF \
    -Wno-dev 2>&1 | tail -5
  cmake --build "$src_dir/cpp/build_cloud" \
    --target fsm_cloud_server -j"$(nproc)" 2>&1 | tail -10

  mkdir -p /opt/fsm-bin
  cp "$src_dir/cpp/build_cloud/cloud_server/fsm_cloud_server" /opt/fsm-bin/
  info "编译完成：/opt/fsm-bin/fsm_cloud_server"
}

build_cloud_server

# ─── Step 8: 启动服务 ─────────────────────────────────────────────────────────
info "启动 Docker 服务..."
cd "$INSTALL_DIR"
docker compose pull --quiet 2>/dev/null || true
docker compose up -d

# 等待健康检查
info "等待服务就绪（最多 60s）..."
for i in $(seq 1 12); do
  if curl -sf "http://localhost:8081/api/v1/health" &>/dev/null; then
    info "云端服务器已就绪"
    break
  fi
  sleep 5
  [[ "$i" -eq 12 ]] && warn "健康检查超时，请查看日志：docker compose -f $INSTALL_DIR/docker-compose.yml logs fsm_cloud"
done

# ─── Step 9: 设置防火墙 ───────────────────────────────────────────────────────
setup_firewall() {
  if command -v ufw &>/dev/null; then
    ufw allow 22/tcp   comment "SSH"    2>/dev/null || true
    ufw allow 80/tcp   comment "HTTP"   2>/dev/null || true
    ufw allow 443/tcp  comment "HTTPS"  2>/dev/null || true
    ufw allow 3478/udp comment "TURN"   2>/dev/null || true
    ufw allow 1883/tcp comment "MQTT"   2>/dev/null || true
    ufw --force enable 2>/dev/null || true
    info "UFW 防火墙已配置"
  fi
  # 提示阿里云安全组
  warn "请确认阿里云安全组已开放端口：22 80 443 3478(UDP) 1883"
}
setup_firewall

# ─── 完成摘要 ─────────────────────────────────────────────────────────────────
WSS_URL="wss://${SERVER_HOST}/ws"
STUN_URL="stun:${SERVER_HOST}:3478"
TURN_URL="turn:${SERVER_HOST}:3478"
API_URL="https://${SERVER_HOST}/api/v1"

echo ""
echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN}  FSM-Pilot 部署完成！${NC}"
echo -e "${GREEN}============================================================${NC}"
echo ""
echo "  WebSocket 信令 : $WSS_URL"
echo "  REST API       : $API_URL"
echo "  STUN           : $STUN_URL"
echo "  TURN           : $TURN_URL"
echo "  JWT Secret     : $JWT_SECRET"
echo "  TURN Secret    : $TURN_PASSWD"
echo ""
echo "  验证健康："
echo "    curl -k $API_URL/health"
echo ""
echo "  车端启动命令（复制后在车辆上执行）："
echo "    export FSM_CLOUD_URL=$WSS_URL"
echo "    export FSM_VEHICLE_ID=FSM-CAR-01"
echo "    bash vehicle_deploy.sh"
echo ""
echo "  操作员端启动："
echo "    ./fsm_operator $WSS_URL FSM-CAR-01"
echo ""
if [[ -z "$DOMAIN" ]]; then
  echo -e "  ${YELLOW}[提示] 使用自签证书，客户端需加 -k / --insecure 或信任证书${NC}"
  echo -e "  ${YELLOW}       证书文件: $INSTALL_DIR/certs/cert.pem${NC}"
fi
echo ""
echo "  配置文件 : $INSTALL_DIR/configs/cloud_config.yaml"
echo "  日志目录 : $INSTALL_DIR/logs/"
echo "  管理命令 : docker compose -f $INSTALL_DIR/docker-compose.yml [logs|restart|down]"
echo ""
