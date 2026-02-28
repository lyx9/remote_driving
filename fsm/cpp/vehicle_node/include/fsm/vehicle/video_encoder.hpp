#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

namespace fsm {
namespace vehicle {

enum class EncoderType {
  kSoftwareX264,
  kNvenc,
  kVaapi,
  kAuto,
};

struct EncoderConfig {
  int width = 1920;
  int height = 1080;
  int fps = 30;
  int bitrate_kbps = 4000;
  int keyframe_interval = 30;
  EncoderType encoder_type = EncoderType::kAuto;
  std::string preset = "ultrafast";
  std::string tune = "zerolatency";
};

struct EncodedFrame {
  std::vector<uint8_t> data;
  bool is_keyframe = false;
  uint64_t timestamp = 0;
  uint64_t pts = 0;
  uint64_t dts = 0;
};

class VideoEncoder {
 public:
  virtual ~VideoEncoder() = default;

  [[nodiscard]] virtual bool Initialize(const EncoderConfig& config) = 0;
  [[nodiscard]] virtual bool Encode(const cv::Mat& frame, EncodedFrame& output) = 0;
  virtual void RequestKeyframe() = 0;
  virtual std::string EncoderInfo() const = 0;
  virtual void Shutdown() = 0;
};

class SoftwareEncoder : public VideoEncoder {
 public:
  SoftwareEncoder();
  ~SoftwareEncoder() override;

  [[nodiscard]] bool Initialize(const EncoderConfig& config) override;
  [[nodiscard]] bool Encode(const cv::Mat& frame, EncodedFrame& output) override;
  void RequestKeyframe() override;
  std::string EncoderInfo() const override;
  void Shutdown() override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Factory. Returns nullptr if the requested type is unavailable.
std::unique_ptr<VideoEncoder> CreateEncoder(EncoderType type);

bool IsNvencAvailable();
bool IsVaapiAvailable();

}  // namespace vehicle
}  // namespace fsm
