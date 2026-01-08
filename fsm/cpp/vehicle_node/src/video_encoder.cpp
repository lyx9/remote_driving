/**
 * FSM-Pilot 视频编码器实现
 * 支持 H.264 硬件加速编码
 */

#include "fsm/vehicle/video_encoder.hpp"
#include "fsm/logger.hpp"

namespace fsm {
namespace vehicle {

VideoEncoder::VideoEncoder(const config::CameraConfig& config)
    : config_(config)
    , initialized_(false)
    , frame_count_(0)
    , codec_ctx_(nullptr)
    , hw_device_ctx_(nullptr)
{
}

VideoEncoder::~VideoEncoder() {
    shutdown();
}

bool VideoEncoder::initialize() {
    if (initialized_) {
        return true;
    }

    LOG_INFO("[Encoder] Initializing encoder for camera: " + config_.id);

#ifdef USE_FFMPEG
    // 注册所有编解码器
    // avcodec_register_all(); // FFmpeg 4.0+ 不需要

    // 查找 H.264 编码器
    const AVCodec* codec = nullptr;

    // 优先使用硬件编码器
    if (config_.encoding == "h264") {
        // 尝试 NVIDIA NVENC
        codec = avcodec_find_encoder_by_name("h264_nvenc");
        if (codec) {
            LOG_INFO("[Encoder] Using NVIDIA NVENC hardware encoder");
        }

        // 尝试 VA-API (Intel/AMD)
        if (!codec) {
            codec = avcodec_find_encoder_by_name("h264_vaapi");
            if (codec) {
                LOG_INFO("[Encoder] Using VA-API hardware encoder");
            }
        }

        // 回退到软件编码
        if (!codec) {
            codec = avcodec_find_encoder(AV_CODEC_ID_H264);
            if (codec) {
                LOG_INFO("[Encoder] Using libx264 software encoder");
            }
        }
    } else if (config_.encoding == "vp8") {
        codec = avcodec_find_encoder(AV_CODEC_ID_VP8);
    } else if (config_.encoding == "vp9") {
        codec = avcodec_find_encoder(AV_CODEC_ID_VP9);
    }

    if (!codec) {
        LOG_ERROR("[Encoder] Failed to find encoder for: " + config_.encoding);
        return false;
    }

    // 分配编码器上下文
    codec_ctx_ = avcodec_alloc_context3(codec);
    if (!codec_ctx_) {
        LOG_ERROR("[Encoder] Failed to allocate encoder context");
        return false;
    }

    // 设置编码参数
    codec_ctx_->width = config_.width;
    codec_ctx_->height = config_.height;
    codec_ctx_->time_base = {1, config_.fps};
    codec_ctx_->framerate = {config_.fps, 1};
    codec_ctx_->gop_size = config_.fps;  // 1秒一个关键帧
    codec_ctx_->max_b_frames = 0;        // 禁用 B 帧以降低延迟
    codec_ctx_->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_ctx_->bit_rate = config_.bitrate;

    // 低延迟设置
    codec_ctx_->flags |= AV_CODEC_FLAG_LOW_DELAY;
    codec_ctx_->flags2 |= AV_CODEC_FLAG2_FAST;

    // 编码器特定选项
    AVDictionary* opts = nullptr;

    if (strstr(codec->name, "nvenc")) {
        // NVENC 低延迟设置
        av_dict_set(&opts, "preset", "ll", 0);       // 低延迟
        av_dict_set(&opts, "tune", "ull", 0);        // 超低延迟
        av_dict_set(&opts, "zerolatency", "1", 0);
        av_dict_set(&opts, "rc", "cbr", 0);          // CBR 模式
    } else if (strstr(codec->name, "vaapi")) {
        // VA-API 设置
        av_dict_set(&opts, "rc_mode", "CBR", 0);
    } else {
        // x264 软件编码设置
        av_dict_set(&opts, "preset", "ultrafast", 0);
        av_dict_set(&opts, "tune", "zerolatency", 0);
        av_dict_set(&opts, "profile", "baseline", 0);
    }

    // 打开编码器
    int ret = avcodec_open2(codec_ctx_, codec, &opts);
    av_dict_free(&opts);

    if (ret < 0) {
        char err_buf[256];
        av_strerror(ret, err_buf, sizeof(err_buf));
        LOG_ERROR("[Encoder] Failed to open encoder: " + std::string(err_buf));
        avcodec_free_context(&codec_ctx_);
        return false;
    }

    // 分配帧和包
    frame_ = av_frame_alloc();
    frame_->format = codec_ctx_->pix_fmt;
    frame_->width = codec_ctx_->width;
    frame_->height = codec_ctx_->height;

    ret = av_frame_get_buffer(frame_, 32);
    if (ret < 0) {
        LOG_ERROR("[Encoder] Failed to allocate frame buffer");
        return false;
    }

    packet_ = av_packet_alloc();

    // 创建颜色转换上下文 (BGR -> YUV420P)
    sws_ctx_ = sws_getContext(
        config_.width, config_.height, AV_PIX_FMT_BGR24,
        config_.width, config_.height, AV_PIX_FMT_YUV420P,
        SWS_FAST_BILINEAR, nullptr, nullptr, nullptr
    );

    if (!sws_ctx_) {
        LOG_ERROR("[Encoder] Failed to create SWS context");
        return false;
    }

    initialized_ = true;
    LOG_INFO("[Encoder] Encoder initialized successfully");
    return true;

#else
    LOG_ERROR("[Encoder] FFmpeg not available, encoder disabled");
    return false;
#endif
}

void VideoEncoder::shutdown() {
    if (!initialized_) {
        return;
    }

#ifdef USE_FFMPEG
    if (sws_ctx_) {
        sws_freeContext(sws_ctx_);
        sws_ctx_ = nullptr;
    }

    if (packet_) {
        av_packet_free(&packet_);
    }

    if (frame_) {
        av_frame_free(&frame_);
    }

    if (codec_ctx_) {
        avcodec_free_context(&codec_ctx_);
    }
#endif

    initialized_ = false;
    LOG_INFO("[Encoder] Encoder shutdown");
}

EncodedFrame VideoEncoder::encode(const uint8_t* rgb_data,
                                   int width,
                                   int height,
                                   int64_t timestamp) {
    EncodedFrame result;
    result.timestamp = timestamp;
    result.is_keyframe = false;

    if (!initialized_) {
        LOG_ERROR("[Encoder] Encoder not initialized");
        return result;
    }

#ifdef USE_FFMPEG
    // 确保帧可写
    int ret = av_frame_make_writable(frame_);
    if (ret < 0) {
        LOG_ERROR("[Encoder] Frame not writable");
        return result;
    }

    // BGR -> YUV420P 颜色转换
    const uint8_t* src_data[1] = { rgb_data };
    int src_linesize[1] = { width * 3 };

    sws_scale(sws_ctx_,
              src_data, src_linesize,
              0, height,
              frame_->data, frame_->linesize);

    // 设置时间戳
    frame_->pts = frame_count_++;

    // 发送帧到编码器
    ret = avcodec_send_frame(codec_ctx_, frame_);
    if (ret < 0) {
        char err_buf[256];
        av_strerror(ret, err_buf, sizeof(err_buf));
        LOG_ERROR("[Encoder] Error sending frame: " + std::string(err_buf));
        return result;
    }

    // 接收编码后的数据
    ret = avcodec_receive_packet(codec_ctx_, packet_);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        // 需要更多输入或编码结束
        return result;
    } else if (ret < 0) {
        char err_buf[256];
        av_strerror(ret, err_buf, sizeof(err_buf));
        LOG_ERROR("[Encoder] Error receiving packet: " + std::string(err_buf));
        return result;
    }

    // 复制编码数据
    result.data.assign(packet_->data, packet_->data + packet_->size);
    result.is_keyframe = (packet_->flags & AV_PKT_FLAG_KEY) != 0;
    result.pts = packet_->pts;
    result.dts = packet_->dts;

    av_packet_unref(packet_);

#endif
    return result;
}

void VideoEncoder::setBitrate(int bitrate) {
#ifdef USE_FFMPEG
    if (codec_ctx_) {
        codec_ctx_->bit_rate = bitrate;
        // 注意: 运行时更改比特率可能需要重新配置编码器
    }
#endif
}

void VideoEncoder::requestKeyframe() {
#ifdef USE_FFMPEG
    if (frame_) {
        frame_->pict_type = AV_PICTURE_TYPE_I;
    }
#endif
}

EncoderStats VideoEncoder::getStats() const {
    EncoderStats stats;
    stats.frames_encoded = frame_count_;
    stats.bitrate = config_.bitrate;
    stats.width = config_.width;
    stats.height = config_.height;
    stats.fps = config_.fps;

#ifdef USE_FFMPEG
    if (codec_ctx_) {
        // 可以添加更多统计信息
    }
#endif

    return stats;
}

}  // namespace vehicle
}  // namespace fsm
