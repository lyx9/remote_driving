#include "fsm/vehicle/video_encoder.hpp"
#include "fsm/logger.hpp"
#include <cstring>
#include <sstream>

extern "C" {
#include <x264.h>
}

namespace fsm {
namespace vehicle {

// ---------------------------------------------------------------------------
// SoftwareEncoder
// ---------------------------------------------------------------------------

struct SoftwareEncoder::Impl {
  x264_t*       encoder = nullptr;
  x264_param_t  param;
  x264_picture_t pic_in;
  x264_picture_t pic_out;

  EncoderConfig config;
  bool          initialized       = false;
  uint64_t      frame_count       = 0;
  bool          keyframe_requested = false;

  ~Impl() {
    if (encoder) {
      x264_encoder_close(encoder);
      encoder = nullptr;
    }
    x264_picture_clean(&pic_in);
  }
};

SoftwareEncoder::SoftwareEncoder()
    : impl_(std::make_unique<Impl>()) {}

SoftwareEncoder::~SoftwareEncoder() {
  Shutdown();
}

bool SoftwareEncoder::Initialize(const EncoderConfig& config) {
  if (impl_->initialized) {
    FSM_LOG_WARN("Encoder already initialized");
    return true;
  }
  impl_->config = config;

  x264_param_t* p = &impl_->param;
  if (x264_param_default_preset(p, config.preset.c_str(), config.tune.c_str()) < 0) {
    FSM_LOG_ERROR("Failed to set x264 preset/tune");
    return false;
  }

  p->i_width        = config.width;
  p->i_height       = config.height;
  p->i_fps_num      = config.fps;
  p->i_fps_den      = 1;
  p->i_keyint_max   = config.keyframe_interval;
  p->b_repeat_headers = 1;
  p->b_annexb       = 1;

  p->rc.i_rc_method      = X264_RC_ABR;
  p->rc.i_bitrate        = config.bitrate_kbps;
  p->rc.i_vbv_max_bitrate = config.bitrate_kbps * 1.2;
  p->rc.i_vbv_buffer_size = config.bitrate_kbps;

  p->i_slice_max_size = 1500;
  p->b_intra_refresh  = 1;
  p->i_bframe         = 0;
  p->i_threads        = 4;

  if (x264_param_apply_profile(p, "baseline") < 0) {
    FSM_LOG_ERROR("Failed to apply x264 profile");
    return false;
  }

  impl_->encoder = x264_encoder_open(p);
  if (!impl_->encoder) {
    FSM_LOG_ERROR("Failed to open x264 encoder");
    return false;
  }

  if (x264_picture_alloc(&impl_->pic_in, X264_CSP_I420,
                         config.width, config.height) < 0) {
    FSM_LOG_ERROR("Failed to allocate x264 picture");
    x264_encoder_close(impl_->encoder);
    impl_->encoder = nullptr;
    return false;
  }

  impl_->initialized = true;
  FSM_LOG_INFO("SoftwareEncoder: {}x{}@{}fps {}kbps",
               config.width, config.height, config.fps, config.bitrate_kbps);
  return true;
}

bool SoftwareEncoder::Encode(const cv::Mat& frame, EncodedFrame& output) {
  if (!impl_->initialized) {
    FSM_LOG_ERROR("Encoder not initialized");
    return false;
  }

  cv::Mat yuv;
  cv::cvtColor(frame, yuv, cv::COLOR_BGR2YUV_I420);

  const int y_size  = impl_->config.width * impl_->config.height;
  const int uv_size = y_size / 4;
  std::memcpy(impl_->pic_in.img.plane[0], yuv.data,              y_size);
  std::memcpy(impl_->pic_in.img.plane[1], yuv.data + y_size,     uv_size);
  std::memcpy(impl_->pic_in.img.plane[2], yuv.data + y_size + uv_size, uv_size);

  impl_->pic_in.i_pts = impl_->frame_count++;
  impl_->pic_in.i_type = impl_->keyframe_requested ? X264_TYPE_IDR : X264_TYPE_AUTO;
  impl_->keyframe_requested = false;

  x264_nal_t* nals  = nullptr;
  int         i_nals = 0;
  const int frame_size = x264_encoder_encode(
      impl_->encoder, &nals, &i_nals, &impl_->pic_in, &impl_->pic_out);

  if (frame_size < 0) {
    FSM_LOG_ERROR("x264_encoder_encode failed");
    return false;
  }
  if (frame_size == 0) return false;

  output.data.clear();
  for (int i = 0; i < i_nals; ++i) {
    output.data.insert(output.data.end(),
                       nals[i].p_payload,
                       nals[i].p_payload + nals[i].i_payload);
  }
  output.is_keyframe = (impl_->pic_out.b_keyframe != 0);
  output.timestamp   = impl_->pic_out.i_pts;
  output.pts         = impl_->pic_out.i_pts;
  output.dts         = impl_->pic_out.i_dts;
  return true;
}

void SoftwareEncoder::RequestKeyframe() {
  impl_->keyframe_requested = true;
}

std::string SoftwareEncoder::EncoderInfo() const {
  std::ostringstream oss;
  oss << "x264 " << impl_->config.width << "x" << impl_->config.height
      << "@" << impl_->config.fps << "fps " << impl_->config.bitrate_kbps << "kbps";
  return oss.str();
}

void SoftwareEncoder::Shutdown() {
  if (!impl_->initialized) return;

  if (impl_->encoder) {
    x264_nal_t* nals  = nullptr;
    int         i_nals = 0;
    while (x264_encoder_encode(impl_->encoder, &nals, &i_nals,
                               nullptr, &impl_->pic_out) > 0) {}
  }
  impl_->initialized = false;
  FSM_LOG_INFO("SoftwareEncoder shutdown");
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

std::unique_ptr<VideoEncoder> CreateEncoder(EncoderType type) {
  switch (type) {
    case EncoderType::kNvenc:
      FSM_LOG_WARN("NVENC not implemented, falling back to software");
      return std::make_unique<SoftwareEncoder>();
    case EncoderType::kVaapi:
      FSM_LOG_WARN("VAAPI not implemented, falling back to software");
      return std::make_unique<SoftwareEncoder>();
    case EncoderType::kAuto:
      if (IsNvencAvailable()) FSM_LOG_INFO("NVENC available but not implemented");
      if (IsVaapiAvailable()) FSM_LOG_INFO("VAAPI available but not implemented");
      return std::make_unique<SoftwareEncoder>();
    default:
      return std::make_unique<SoftwareEncoder>();
  }
}

bool IsNvencAvailable() { return false; }
bool IsVaapiAvailable() { return false; }

}  // namespace vehicle
}  // namespace fsm
