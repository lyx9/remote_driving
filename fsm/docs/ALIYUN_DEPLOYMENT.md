# FSM-Pilot V2.0 阿里云部署指南

## 1. 架构概述

```
┌─────────────────────────────────────────────────────────────────────┐
│                        阿里云部署架构                                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐          │
│  │   Client    │     │   CDN       │     │   WAF       │          │
│  │  (Browser)  │────▶│  (阿里云)   │────▶│ (Web应用)   │          │
│  └─────────────┘     └─────────────┘     └─────────────┘          │
│                                                │                   │
│                                                ▼                   │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │                        负载均衡 (SLB)                         │ │
│  └──────────────────────────────────────────────────────────────┘ │
│          │                    │                    │              │
│          ▼                    ▼                    ▼              │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐        │
│  │  Web Server   │  │  API Server   │  │  Signaling    │        │
│  │  (Nginx/OSS)  │  │  (ECS/K8s)    │  │  (WebSocket)  │        │
│  └───────────────┘  └───────────────┘  └───────────────┘        │
│                             │                    │               │
│                             ▼                    ▼               │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐        │
│  │   RDS/Redis   │  │   TURN/STUN   │  │   日志服务    │        │
│  │   (数据库)    │  │   (RTC)       │  │   (SLS)       │        │
│  └───────────────┘  └───────────────┘  └───────────────┘        │
│                                                                  │
│                              │                                   │
│                              ▼                                   │
│  ┌──────────────────────────────────────────────────────────────┐│
│  │                    专线/VPN (私有网络)                        ││
│  └──────────────────────────────────────────────────────────────┘│
│                              │                                   │
│                              ▼                                   │
│  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐        │
│  │   Vehicle 1   │  │   Vehicle 2   │  │   Vehicle N   │        │
│  │  (Autoware)   │  │  (Autoware)   │  │  (Autoware)   │        │
│  └───────────────┘  └───────────────┘  └───────────────┘        │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

## 2. 资源清单

### 2.1 必需资源

| 资源类型 | 规格 | 数量 | 用途 |
|---------|------|------|------|
| ECS | ecs.g6.2xlarge (8vCPU, 32GB) | 2+ | API/信令服务器 |
| SLB | 标准型 | 1 | 负载均衡 |
| RDS MySQL | mysql.n2.medium.1 | 1 | 业务数据库 |
| Redis | 4GB | 1 | 会话/缓存 |
| OSS | 标准存储 | 1 | 静态资源/录像 |
| CDN | 流量包 | 1 | 静态资源加速 |
| 私有网络 VPC | - | 1 | 网络隔离 |
| NAT网关 | 小型 | 1 | 出网流量 |

### 2.2 可选资源

| 资源类型 | 规格 | 用途 |
|---------|------|------|
| TURN服务器 | ecs.c6.xlarge | WebRTC中继 |
| SLS日志服务 | - | 日志分析 |
| ARMS应用监控 | - | APM监控 |
| 专线/VPN | - | 车端连接 |

## 3. 部署步骤

### 3.1 准备工作

```bash
# 1. 安装阿里云 CLI
curl -fsSL https://aliyuncli.alicdn.com/aliyun-cli-linux-latest-amd64.tgz | tar -xz
sudo mv aliyun /usr/local/bin/

# 2. 配置访问凭证
aliyun configure
# Access Key ID: [Your Access Key ID]
# Access Key Secret: [Your Access Key Secret]
# Region Id: cn-hongkong (或其他区域)

# 3. 安装 Terraform (可选，用于 IaC)
curl -fsSL https://releases.hashicorp.com/terraform/1.6.0/terraform_1.6.0_linux_amd64.zip -o terraform.zip
unzip terraform.zip && sudo mv terraform /usr/local/bin/
```

### 3.2 创建 VPC 和网络

```bash
# 创建 VPC
aliyun vpc CreateVpc \
  --RegionId cn-hongkong \
  --VpcName fsm-pilot-vpc \
  --CidrBlock 172.16.0.0/12

# 创建交换机 (可用区 A)
aliyun vpc CreateVSwitch \
  --RegionId cn-hongkong \
  --ZoneId cn-hongkong-a \
  --VpcId vpc-xxxxxx \
  --VSwitchName fsm-pilot-vsw-a \
  --CidrBlock 172.16.0.0/24

# 创建安全组
aliyun ecs CreateSecurityGroup \
  --RegionId cn-hongkong \
  --VpcId vpc-xxxxxx \
  --SecurityGroupName fsm-pilot-sg \
  --Description "FSM-Pilot Security Group"

# 添加安全组规则
aliyun ecs AuthorizeSecurityGroup \
  --RegionId cn-hongkong \
  --SecurityGroupId sg-xxxxxx \
  --IpProtocol tcp \
  --PortRange 80/80 \
  --SourceCidrIp 0.0.0.0/0

aliyun ecs AuthorizeSecurityGroup \
  --RegionId cn-hongkong \
  --SecurityGroupId sg-xxxxxx \
  --IpProtocol tcp \
  --PortRange 443/443 \
  --SourceCidrIp 0.0.0.0/0

aliyun ecs AuthorizeSecurityGroup \
  --RegionId cn-hongkong \
  --SecurityGroupId sg-xxxxxx \
  --IpProtocol tcp \
  --PortRange 8080/8080 \
  --SourceCidrIp 0.0.0.0/0

aliyun ecs AuthorizeSecurityGroup \
  --RegionId cn-hongkong \
  --SecurityGroupId sg-xxxxxx \
  --IpProtocol udp \
  --PortRange 3478/3478 \
  --SourceCidrIp 0.0.0.0/0
```

### 3.3 创建 ECS 实例

```bash
# 创建 API 服务器
aliyun ecs CreateInstance \
  --RegionId cn-hongkong \
  --ImageId ubuntu_22_04_x64_20G_alibase_20231221.vhd \
  --InstanceType ecs.g6.2xlarge \
  --SecurityGroupId sg-xxxxxx \
  --VSwitchId vsw-xxxxxx \
  --InstanceName fsm-pilot-api-1 \
  --HostName fsm-pilot-api-1 \
  --SystemDisk.Category cloud_essd \
  --SystemDisk.Size 100 \
  --InternetMaxBandwidthOut 100 \
  --Password 'YourSecurePassword123!'
```

### 3.4 配置 RDS 数据库

```bash
# 创建 RDS MySQL 实例
aliyun rds CreateDBInstance \
  --RegionId cn-hongkong \
  --Engine MySQL \
  --EngineVersion 8.0 \
  --DBInstanceClass mysql.n2.medium.1 \
  --DBInstanceStorage 50 \
  --DBInstanceNetType Intranet \
  --VPCId vpc-xxxxxx \
  --VSwitchId vsw-xxxxxx \
  --PayType Postpaid \
  --DBInstanceDescription fsm-pilot-db

# 创建数据库账号
aliyun rds CreateAccount \
  --DBInstanceId rm-xxxxxx \
  --AccountName fsmpilot \
  --AccountPassword 'YourDBPassword123!' \
  --AccountType Super

# 创建数据库
aliyun rds CreateDatabase \
  --DBInstanceId rm-xxxxxx \
  --DBName fsmpilot \
  --CharacterSetName utf8mb4
```

### 3.5 配置 Redis

```bash
# 创建 Redis 实例
aliyun r-kvstore CreateInstance \
  --RegionId cn-hongkong \
  --InstanceClass redis.master.small.default \
  --InstanceName fsm-pilot-redis \
  --VpcId vpc-xxxxxx \
  --VSwitchId vsw-xxxxxx \
  --Password 'YourRedisPassword123!'
```

### 3.6 配置 OSS 和 CDN

```bash
# 创建 OSS Bucket
aliyun oss mb oss://fsm-pilot-static --region cn-hongkong

# 配置 Bucket 为公共读
aliyun oss bucket-acl oss://fsm-pilot-static --acl public-read

# 上传静态文件
aliyun oss cp ./dist oss://fsm-pilot-static/web/ -r

# 创建 CDN 加速域名
aliyun cdn AddCdnDomain \
  --DomainName static.fsm-pilot.com \
  --CdnType web \
  --Sources '[{"content":"fsm-pilot-static.oss-cn-hongkong.aliyuncs.com","type":"oss","priority":"20"}]'
```

### 3.7 配置负载均衡

```bash
# 创建 SLB 实例
aliyun slb CreateLoadBalancer \
  --RegionId cn-hongkong \
  --LoadBalancerName fsm-pilot-slb \
  --VpcId vpc-xxxxxx \
  --VSwitchId vsw-xxxxxx \
  --AddressType intranet \
  --LoadBalancerSpec slb.s2.medium

# 创建后端服务器组
aliyun slb CreateVServerGroup \
  --RegionId cn-hongkong \
  --LoadBalancerId lb-xxxxxx \
  --VServerGroupName fsm-pilot-api

# 添加后端服务器
aliyun slb AddVServerGroupBackendServers \
  --RegionId cn-hongkong \
  --VServerGroupId rsp-xxxxxx \
  --BackendServers '[{"ServerId":"i-xxxxxx","Port":"3000","Weight":"100"}]'

# 创建监听
aliyun slb CreateLoadBalancerHTTPListener \
  --RegionId cn-hongkong \
  --LoadBalancerId lb-xxxxxx \
  --ListenerPort 80 \
  --BackendServerPort 3000 \
  --VServerGroupId rsp-xxxxxx \
  --HealthCheck on \
  --HealthCheckURI /api/health
```

## 4. 应用部署

### 4.1 构建前端

```bash
# 在本地构建
cd /home/lyx/fsm
npm run build

# 上传到 OSS
aliyun oss cp ./dist oss://fsm-pilot-static/web/ -r --update
```

### 4.2 部署后端服务

```bash
# SSH 到 ECS 实例
ssh root@<ECS_PUBLIC_IP>

# 安装 Docker
curl -fsSL https://get.docker.com | sh
systemctl enable docker && systemctl start docker

# 克隆代码
git clone https://github.com/your-org/fsm-pilot.git
cd fsm-pilot

# 配置环境变量
cat > .env.production << 'EOF'
NODE_ENV=production
PORT=3000
DATABASE_URL=mysql://fsmpilot:YourDBPassword123!@rm-xxxxxx.mysql.rds.aliyuncs.com:3306/fsmpilot
REDIS_URL=redis://:YourRedisPassword123!@r-xxxxxx.redis.rds.aliyuncs.com:6379
JWT_SECRET=your-jwt-secret-here
ALIYUN_ACCESS_KEY_ID=your-access-key
ALIYUN_ACCESS_KEY_SECRET=your-access-secret
TURN_SERVER_URL=turn:turn.fsm-pilot.com:3478
TURN_USERNAME=turnuser
TURN_CREDENTIAL=turnpassword
EOF

# 构建并运行
docker build -t fsm-pilot-api .
docker run -d \
  --name fsm-pilot-api \
  --restart always \
  -p 3000:3000 \
  --env-file .env.production \
  fsm-pilot-api
```

### 4.3 部署信令服务器

```bash
# 创建信令服务 Dockerfile
cat > Dockerfile.signaling << 'EOF'
FROM node:20-alpine
WORKDIR /app
COPY package*.json ./
RUN npm ci --only=production
COPY ./signaling-server ./
EXPOSE 8080
CMD ["node", "index.js"]
EOF

# 构建并运行
docker build -t fsm-pilot-signaling -f Dockerfile.signaling .
docker run -d \
  --name fsm-pilot-signaling \
  --restart always \
  -p 8080:8080 \
  --env-file .env.production \
  fsm-pilot-signaling
```

### 4.4 部署 TURN 服务器

```bash
# 安装 coturn
apt update && apt install -y coturn

# 配置 coturn
cat > /etc/turnserver.conf << 'EOF'
# 监听端口
listening-port=3478
tls-listening-port=5349

# 外部 IP (ECS 公网 IP)
external-ip=YOUR_ECS_PUBLIC_IP

# 认证
lt-cred-mech
user=turnuser:turnpassword

# 安全
fingerprint
no-stun

# 日志
log-file=/var/log/turn.log
verbose

# Realm
realm=fsm-pilot.com
EOF

# 启动服务
systemctl enable coturn
systemctl start coturn
```

## 5. Nginx 配置

```nginx
# /etc/nginx/sites-available/fsm-pilot
upstream api_servers {
    server 127.0.0.1:3000;
}

upstream signaling_servers {
    server 127.0.0.1:8080;
}

server {
    listen 80;
    server_name fsm-pilot.com;
    return 301 https://$server_name$request_uri;
}

server {
    listen 443 ssl http2;
    server_name fsm-pilot.com;

    ssl_certificate /etc/letsencrypt/live/fsm-pilot.com/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/fsm-pilot.com/privkey.pem;

    # 静态文件 (前端)
    location / {
        root /var/www/fsm-pilot/dist;
        try_files $uri $uri/ /index.html;
        expires 1d;
    }

    # API 代理
    location /api/ {
        proxy_pass http://api_servers;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection 'upgrade';
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        proxy_cache_bypass $http_upgrade;
    }

    # WebSocket 信令
    location /signaling {
        proxy_pass http://signaling_servers;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_read_timeout 86400;
    }
}
```

## 6. 监控和日志

### 6.1 配置 SLS 日志

```bash
# 安装 Logtail
wget http://logtail-release-cn-hongkong.oss-cn-hongkong.aliyuncs.com/linux64/logtail.sh
chmod +x logtail.sh
./logtail.sh install cn-hongkong

# 配置应用日志采集
cat > /etc/ilogtail/user_log_config.json << 'EOF'
{
    "fsm-pilot-api": {
        "log_path": "/var/log/fsm-pilot",
        "file_pattern": "*.log",
        "log_type": "json"
    }
}
EOF
```

### 6.2 配置 ARMS 监控

```bash
# 在应用中集成 ARMS SDK
npm install @aliyun/ahas-arms-sdk

# 初始化代码
import { AliMonitor } from '@aliyun/ahas-arms-sdk';
AliMonitor.init({
    pid: 'YOUR_PID',
    region: 'cn-hongkong'
});
```

## 7. 安全配置

### 7.1 配置 WAF

```bash
# 创建 WAF 域名配置
aliyun waf-openapi CreateDomain \
  --Domain fsm-pilot.com \
  --SourceIps '["SLB_IP"]' \
  --IsAccessProduct false \
  --HttpPort '[80]' \
  --HttpsPort '[443]'

# 启用防护规则
aliyun waf-openapi ModifyProtectionModuleMode \
  --Domain fsm-pilot.com \
  --DefenseType waf \
  --Mode 1
```

### 7.2 配置 SSL 证书

```bash
# 使用 certbot 申请证书
apt install certbot python3-certbot-nginx
certbot --nginx -d fsm-pilot.com -d www.fsm-pilot.com

# 或使用阿里云 SSL 证书服务
aliyun cas CreateCertificateRequest \
  --DomainList '["fsm-pilot.com","*.fsm-pilot.com"]' \
  --ProductCode digicert
```

## 8. 自动化部署脚本

```bash
#!/bin/bash
# deploy.sh - FSM-Pilot 阿里云部署脚本

set -e

# 配置
REGION="cn-hongkong"
PROJECT_NAME="fsm-pilot"
OSS_BUCKET="fsm-pilot-static"
ECS_HOST="your-ecs-ip"

echo "=== FSM-Pilot 部署脚本 ==="

# 1. 构建前端
echo ">>> 构建前端..."
npm run build

# 2. 上传到 OSS
echo ">>> 上传静态文件到 OSS..."
aliyun oss sync ./dist oss://${OSS_BUCKET}/web/ --delete --region ${REGION}

# 3. 刷新 CDN 缓存
echo ">>> 刷新 CDN..."
aliyun cdn RefreshObjectCaches \
  --ObjectPath "https://static.fsm-pilot.com/web/"

# 4. 部署后端
echo ">>> 部署后端服务..."
ssh root@${ECS_HOST} << 'REMOTE_SCRIPT'
cd /opt/fsm-pilot
git pull origin main
docker-compose down
docker-compose up -d --build
docker-compose logs -f --tail=100
REMOTE_SCRIPT

echo "=== 部署完成 ==="
```

## 9. 成本估算

| 资源 | 规格 | 月成本 (人民币) |
|------|------|----------------|
| ECS x 2 | g6.2xlarge | ~2,400 |
| RDS MySQL | n2.medium | ~600 |
| Redis | 4GB | ~400 |
| SLB | 标准型 | ~200 |
| OSS | 100GB | ~20 |
| CDN | 500GB流量 | ~200 |
| NAT网关 | 小型 | ~300 |
| 带宽 | 100Mbps | ~1,500 |
| **总计** | | **~5,620** |

## 10. 故障排查

### 常见问题

1. **WebSocket 连接失败**
   - 检查安全组是否开放 8080 端口
   - 检查 Nginx WebSocket 配置
   - 检查 SLB 健康检查配置

2. **视频延迟高**
   - 检查 TURN 服务器是否正常
   - 检查网络带宽
   - 考虑使用边缘节点

3. **数据库连接超时**
   - 检查 RDS 白名单配置
   - 检查 VPC 网络配置
   - 增加连接池大小
