# FSM-Pilot 视频压缩技术文档

## 概述

本文档描述 FSM-Pilot 远程驾驶平台的视频压缩系统，用于降低带宽消耗，提高网络传输效率。

## 1. 压缩策略

### 1.1 双重压缩机制

系统采用**帧率压缩**和**视频流压缩**的双重策略：

#### **策略 1: 帧率压缩 (Frame Rate Reduction)**
- **原理**: 降低视频帧率，减少每秒传输的帧数
- **实现**: 通过丢帧策略，从原始帧率中选择性传输帧
- **典型配置**:
  - 正常模式: 30 FPS
  - 节省模式: 15 FPS (带宽减少 ~50%)
  - 极限模式: 10 FPS (带宽减少 ~67%)

#### **策略 2: 视频编码压缩 (Video Codec Compression)**
- **原理**: 使用 H.264 编码器压缩视频数据
- **实现**: 调整编码器 bitrate、质量参数、preset
- **典型配置**:
  - 高质量模式: 2000 kbps
  - 平衡模式: 1000 kbps (带宽减少 ~50%)
  - 低带宽模式: 500 kbps (带宽减少 ~75%)

### 1.2 组合压缩效果

| 压缩模式 | 帧率 | Bitrate | 总带宽节省 | 适用场景 |
|---------|------|---------|-----------|---------|
| 无压缩   | 30 FPS | 2000 kbps | 0% | 理想网络环境 |
| 轻度压缩 | 30 FPS | 1000 kbps | ~50% | 良好网络环境 |
| 中度压缩 | 15 FPS | 1000 kbps | ~75% | 一般网络环境 |
| 重度压缩 | 10 FPS | 500 kbps | ~91.7% | 弱网环境 |

## 2. 技术实现

### 2.1 后端架构 (C++ ROS2)

#### 视频编码器 (x264)

**文件位置**: `cpp/vehicle_node/src/video_encoder.cpp`

**核心参数**:
```cpp
// 编码器配置
struct EncoderConfig {
    int width = 1920;           // 分辨率宽度
    int height = 1080;          // 分辨率高度
    int fps = 30;               // 帧率
    int bitrate_kbps = 2000;    // 码率 (kbps)
    int keyframe_interval = 30; // 关键帧间隔
    std::string preset = "ultrafast";  // 编码速度预设
    std::string tune = "zerolatency";  // 优化目标
};
```

**压缩参数调整**:
```cpp
// x264 参数配置
param->rc.i_rc_method = X264_RC_ABR;           // 平均码率控制
param->rc.i_bitrate = config.bitrate_kbps;     // 目标码率
param->rc.i_vbv_max_bitrate = bitrate * 1.2;   // 最大码率
param->rc.i_vbv_buffer_size = bitrate;         // 缓冲区大小
param->i_slice_max_size = 1500;                // MTU 分片大小
param->i_bframe = 0;                           // 无 B 帧 (低延迟)
```

**帧率控制实现**:
```cpp
// 帧率丢帧策略
class FrameRateController {
    double target_fps_;
    double source_fps_;
    int frame_counter_ = 0;

    bool shouldSendFrame() {
        frame_counter_++;
        int skip_interval = static_cast<int>(source_fps_ / target_fps_);
        return (frame_counter_ % skip_interval) == 0;
    }
};
```

### 2.2 前端架构 (TypeScript/Vue)

#### 压缩控制服务

**文件位置**: `src/services/videoCompressionService.ts`

**核心接口**:
```typescript
interface CompressionConfig {
    enabled: boolean
    mode: 'none' | 'light' | 'medium' | 'heavy'
    targetFps: number
    targetBitrate: number
}

interface CompressionStats {
    originalBandwidth: number  // Mbps
    compressedBandwidth: number
    savingsPercent: number
    currentFps: number
    droppedFrames: number
}
```

**压缩模式配置**:
```typescript
const COMPRESSION_MODES = {
    none: { fps: 30, bitrate: 2000 },
    light: { fps: 30, bitrate: 1000 },
    medium: { fps: 15, bitrate: 1000 },
    heavy: { fps: 10, bitrate: 500 }
}
```

### 2.3 WebRTC 数据通道集成

视频压缩配置通过 WebRTC DataChannel 实时传递到车端：

```typescript
// 前端发送压缩配置
function sendCompressionConfig(config: CompressionConfig) {
    const message = {
        type: 'compression_config',
        payload: config
    }
    dataChannel.send(JSON.stringify(message))
}

// 车端接收并应用配置
void onDataChannelMessage(const std::vector<uint8_t>& data) {
    auto json = nlohmann::json::parse(data);
    if (json["type"] == "compression_config") {
        updateEncoderConfig(json["payload"]);
    }
}
```

## 3. 压缩算法原理

### 3.1 H.264 编码原理

#### 帧内压缩 (Intra-frame)
- **DCT 变换**: 将图像从空间域转换到频域
- **量化**: 丢弃高频信息（人眼不敏感）
- **熵编码**: 使用 CABAC/CAVLC 进一步压缩

#### 帧间压缩 (Inter-frame)
- **运动估计**: 检测帧间运动向量
- **运动补偿**: 只传输差异信息
- **参考帧**: P 帧参考前一帧，压缩率高

**压缩比计算**:
```
原始数据 = width × height × 3 (RGB) × fps
         = 1920 × 1080 × 3 × 30
         = 186,624,000 bytes/s
         ≈ 1,493 Mbps

H.264 压缩后 (2000 kbps) = 2 Mbps
压缩比 = 1,493 / 2 ≈ 746:1
```

### 3.2 帧率自适应算法

```typescript
class AdaptiveFrameRate {
    private measureBandwidth(): number {
        // 测量当前网络带宽
        return this.sentBytes / this.duration * 8 / 1000000 // Mbps
    }

    private adjustFrameRate(bandwidth: number) {
        if (bandwidth < 1) {
            this.targetFps = 10  // 弱网
        } else if (bandwidth < 3) {
            this.targetFps = 15  // 一般
        } else {
            this.targetFps = 30  // 良好
        }
    }
}
```

## 4. 性能指标

### 4.1 延迟分析

| 环节 | 无压缩 | 重度压缩 | 说明 |
|-----|-------|---------|------|
| 编码延迟 | ~10ms | ~8ms | 压缩降低编码复杂度 |
| 传输延迟 | ~50ms (10Mbps) | ~15ms (1Mbps) | 带宽节省显著 |
| 解码延迟 | ~10ms | ~12ms | 压缩略增解码复杂度 |
| **总延迟** | **~70ms** | **~35ms** | **整体延迟降低** |

### 4.2 质量评估

**PSNR (峰值信噪比)**:
- 无压缩: ∞ (原始画质)
- 轻度压缩 (2000 kbps): 40-45 dB (优秀)
- 中度压缩 (1000 kbps): 35-40 dB (良好)
- 重度压缩 (500 kbps): 30-35 dB (可接受)

**主观质量**:
- 2000 kbps: 肉眼几乎无法分辨差异
- 1000 kbps: 轻微模糊，细节保留良好
- 500 kbps: 明显压缩痕迹，但车道线/红绿灯清晰可见

## 5. 使用场景

### 5.1 城市环境 (良好网络)
- **推荐配置**: 无压缩 / 轻度压缩
- **帧率**: 30 FPS
- **码率**: 2000 kbps
- **带宽**: ~2 Mbps/camera

### 5.2 郊区环境 (一般网络)
- **推荐配置**: 中度压缩
- **帧率**: 15 FPS
- **码率**: 1000 kbps
- **带宽**: ~0.5 Mbps/camera

### 5.3 偏远地区 (弱网)
- **推荐配置**: 重度压缩
- **帧率**: 10 FPS
- **码率**: 500 kbps
- **带宽**: ~0.25 Mbps/camera

### 5.4 多摄像头场景
对于 4 摄像头系统：
- 无压缩: ~8 Mbps 总带宽
- 轻度压缩: ~4 Mbps 总带宽
- 中度压缩: ~2 Mbps 总带宽
- 重度压缩: ~1 Mbps 总带宽

## 6. 带宽计算公式

### 6.1 理论带宽需求
```
带宽 (Mbps) = (码率 kbps × 帧率实际 / 帧率配置) / 1000

示例:
- 配置: 30 FPS, 2000 kbps
  实际带宽 = 2000 / 1000 = 2 Mbps

- 配置: 15 FPS, 1000 kbps
  实际带宽 = 1000 / 1000 = 1 Mbps
```

### 6.2 实际开销
```
实际带宽 = 理论带宽 × (1 + 协议开销)

协议开销:
- WebRTC/RTP: ~10-15%
- 重传开销: ~5-10% (丢包环境)

总计: ~1.2x 理论带宽
```

## 7. 实现建议

### 7.1 自动压缩策略
```typescript
function autoSelectCompressionMode(
    bandwidth: number,    // 当前带宽 Mbps
    latency: number,      // 当前延迟 ms
    cameraCount: number   // 摄像头数量
): CompressionMode {
    const requiredBandwidth = cameraCount * 2 // 理想带宽

    if (bandwidth > requiredBandwidth && latency < 100) {
        return 'none'
    } else if (bandwidth > requiredBandwidth * 0.5) {
        return 'light'
    } else if (bandwidth > requiredBandwidth * 0.3) {
        return 'medium'
    } else {
        return 'heavy'
    }
}
```

### 7.2 质量监控
```typescript
interface QualityMetrics {
    psnr: number           // 画质指标
    fps: number            // 实际帧率
    bitrate: number        // 实际码率
    packetLoss: number     // 丢包率
    jitter: number         // 抖动
}

function shouldAdjustCompression(metrics: QualityMetrics): boolean {
    return metrics.packetLoss > 5 ||  // 丢包率过高
           metrics.fps < targetFps * 0.8 ||  // 帧率不达标
           metrics.jitter > 50  // 抖动过大
}
```

## 8. 参考资料

- **H.264 标准**: ITU-T H.264 | ISO/IEC 14496-10
- **WebRTC 视频**: https://webrtc.org/getting-started/media-devices
- **x264 文档**: https://www.videolan.org/developers/x264.html
- **带宽计算工具**: https://toolstud.io/video/bitrate.php

## 9. 总结

FSM-Pilot 视频压缩系统通过帧率控制和 H.264 编码的双重策略，可在不同网络环境下灵活调整，最高可节省 **91.7%** 的带宽消耗，同时保持可接受的视频质量和低延迟特性，满足远程驾驶的实时性要求。
