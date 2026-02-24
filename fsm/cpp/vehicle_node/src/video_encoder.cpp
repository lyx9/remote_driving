/**
 * FSM-Pilot Video Encoder Implementation
 * 视频编码器实现
 */

#include "fsm/vehicle/video_encoder.hpp"
#include "fsm/logger.hpp"
#include <cstring>
#include <sstream>

// For x264 software encoding
extern "C" {
#include <x264.h>
}

namespace fsm {
namespace vehicle {

// ============================================================================
// SoftwareEncoder Implementation (x264)
// ============================================================================

struct SoftwareEncoder::Impl {
    x264_t* encoder = nullptr;
    x264_param_t param;
    x264_picture_t pic_in;
    x264_picture_t pic_out;

    EncoderConfig config;
    bool initialized = false;
    uint64_t frame_count = 0;
    bool keyframe_requested = false;

    ~Impl() {
        if (encoder) {
            x264_encoder_close(encoder);
            encoder = nullptr;
        }
        x264_picture_clean(&pic_in);
    }
};

SoftwareEncoder::SoftwareEncoder()
    : impl_(std::make_unique<Impl>()) {
}

SoftwareEncoder::~SoftwareEncoder() {
    shutdown();
}

bool SoftwareEncoder::initialize(const EncoderConfig& config) {
    if (impl_->initialized) {
        FSM_LOG_WARN("Encoder already initialized");
        return true;
    }

    impl_->config = config;

    // Initialize x264 parameters
    x264_param_t* param = &impl_->param;

    // Use preset and tune
    if (x264_param_default_preset(param,
                                   config.preset.c_str(),
                                   config.tune.c_str()) < 0) {
        FSM_LOG_ERROR("Failed to set x264 preset/tune");
        return false;
    }

    // Configure video parameters
    param->i_width = config.width;
    param->i_height = config.height;
    param->i_fps_num = config.fps;
    param->i_fps_den = 1;
    param->i_keyint_max = config.keyframe_interval;
    param->b_repeat_headers = 1;  // Put SPS/PPS before each keyframe
    param->b_annexb = 1;           // Use Annex-B format

    // Rate control
    param->rc.i_rc_method = X264_RC_ABR;
    param->rc.i_bitrate = config.bitrate_kbps;
    param->rc.i_vbv_max_bitrate = config.bitrate_kbps * 1.2;
    param->rc.i_vbv_buffer_size = config.bitrate_kbps;

    // Low latency settings
    param->i_slice_max_size = 1500; // MTU size
    param->b_intra_refresh = 1;
    param->i_bframe = 0; // No B-frames for low latency

    // Threading
    param->i_threads = 4;

    // Apply profile
    if (x264_param_apply_profile(param, "baseline") < 0) {
        FSM_LOG_ERROR("Failed to apply x264 profile");
        return false;
    }

    // Create encoder
    impl_->encoder = x264_encoder_open(param);
    if (!impl_->encoder) {
        FSM_LOG_ERROR("Failed to open x264 encoder");
        return false;
    }

    // Allocate input picture
    if (x264_picture_alloc(&impl_->pic_in, X264_CSP_I420,
                           config.width, config.height) < 0) {
        FSM_LOG_ERROR("Failed to allocate x264 picture");
        x264_encoder_close(impl_->encoder);
        impl_->encoder = nullptr;
        return false;
    }

    impl_->initialized = true;
    FSM_LOG_INFO("Software encoder initialized: {}x{}@{}fps",
                 config.width, config.height, config.fps);

    return true;
}

bool SoftwareEncoder::encode(const cv::Mat& frame, EncodedFrame& output) {
    if (!impl_->initialized) {
        FSM_LOG_ERROR("Encoder not initialized");
        return false;
    }

    // Convert BGR to YUV420
    cv::Mat yuv;
    cv::cvtColor(frame, yuv, cv::COLOR_BGR2YUV_I420);

    // Copy to x264 picture
    int y_size = impl_->config.width * impl_->config.height;
    int uv_size = y_size / 4;

    std::memcpy(impl_->pic_in.img.plane[0], yuv.data, y_size);
    std::memcpy(impl_->pic_in.img.plane[1], yuv.data + y_size, uv_size);
    std::memcpy(impl_->pic_in.img.plane[2], yuv.data + y_size + uv_size, uv_size);

    impl_->pic_in.i_pts = impl_->frame_count++;

    // Force keyframe if requested
    if (impl_->keyframe_requested) {
        impl_->pic_in.i_type = X264_TYPE_IDR;
        impl_->keyframe_requested = false;
    } else {
        impl_->pic_in.i_type = X264_TYPE_AUTO;
    }

    // Encode
    x264_nal_t* nals = nullptr;
    int i_nals = 0;
    int frame_size = x264_encoder_encode(impl_->encoder, &nals, &i_nals,
                                         &impl_->pic_in, &impl_->pic_out);

    if (frame_size < 0) {
        FSM_LOG_ERROR("x264_encoder_encode failed");
        return false;
    }

    if (frame_size == 0) {
        // No output yet (delayed frames)
        return false;
    }

    // Copy encoded data
    output.data.clear();
    for (int i = 0; i < i_nals; i++) {
        output.data.insert(output.data.end(),
                          nals[i].p_payload,
                          nals[i].p_payload + nals[i].i_payload);
    }

    output.is_keyframe = (impl_->pic_out.b_keyframe != 0);
    output.timestamp = impl_->pic_out.i_pts;
    output.pts = impl_->pic_out.i_pts;
    output.dts = impl_->pic_out.i_dts;

    return true;
}

void SoftwareEncoder::requestKeyframe() {
    impl_->keyframe_requested = true;
}

std::string SoftwareEncoder::getEncoderInfo() const {
    std::ostringstream oss;
    oss << "x264 Software Encoder - "
        << impl_->config.width << "x" << impl_->config.height
        << "@" << impl_->config.fps << "fps, "
        << impl_->config.bitrate_kbps << "kbps";
    return oss.str();
}

void SoftwareEncoder::shutdown() {
    if (!impl_->initialized) {
        return;
    }

    // Flush delayed frames
    if (impl_->encoder) {
        while (true) {
            x264_nal_t* nals = nullptr;
            int i_nals = 0;
            int frame_size = x264_encoder_encode(impl_->encoder, &nals, &i_nals,
                                                 nullptr, &impl_->pic_out);
            if (frame_size <= 0) {
                break;
            }
        }
    }

    impl_->initialized = false;
    FSM_LOG_INFO("Encoder shutdown");
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<VideoEncoder> createEncoder(EncoderType type) {
    switch (type) {
        case EncoderType::SOFTWARE_X264:
            return std::make_unique<SoftwareEncoder>();

        case EncoderType::NVENC:
            FSM_LOG_WARN("NVENC not implemented, falling back to software encoder");
            return std::make_unique<SoftwareEncoder>();

        case EncoderType::VAAPI:
            FSM_LOG_WARN("VAAPI not implemented, falling back to software encoder");
            return std::make_unique<SoftwareEncoder>();

        case EncoderType::AUTO:
            // Try hardware encoders first, then fallback
            if (isNvencAvailable()) {
                FSM_LOG_INFO("NVENC available but not implemented, using software");
            }
            if (isVaapiAvailable()) {
                FSM_LOG_INFO("VAAPI available but not implemented, using software");
            }
            return std::make_unique<SoftwareEncoder>();

        default:
            return std::make_unique<SoftwareEncoder>();
    }
}

bool isNvencAvailable() {
    // TODO: Implement NVENC detection
    return false;
}

bool isVaapiAvailable() {
    // TODO: Implement VAAPI detection
    return false;
}

} // namespace vehicle
} // namespace fsm
