/**
 * FSM-Pilot Video Encoder
 * 视频编码器头文件
 */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <opencv2/opencv.hpp>

namespace fsm {
namespace vehicle {

/**
 * 编码器类型
 */
enum class EncoderType {
    SOFTWARE_X264,    // 软件 x264
    NVENC,            // NVIDIA 硬件编码
    VAAPI,            // Intel/AMD 硬件编码
    AUTO              // 自动选择
};

/**
 * 编码配置
 */
struct EncoderConfig {
    int width = 1920;
    int height = 1080;
    int fps = 30;
    int bitrate_kbps = 4000;
    int keyframe_interval = 30;
    EncoderType encoder_type = EncoderType::AUTO;
    std::string preset = "ultrafast";
    std::string tune = "zerolatency";
};

/**
 * 编码后的帧数据
 */
struct EncodedFrame {
    std::vector<uint8_t> data;
    bool is_keyframe;
    uint64_t timestamp;
    uint64_t pts;
    uint64_t dts;
};

/**
 * 编码帧回调
 */
using EncodedFrameCallback = std::function<void(const EncodedFrame&)>;

/**
 * 视频编码器接口
 */
class VideoEncoder {
public:
    virtual ~VideoEncoder() = default;

    /**
     * 初始化编码器
     */
    virtual bool initialize(const EncoderConfig& config) = 0;

    /**
     * 编码一帧
     */
    virtual bool encode(const cv::Mat& frame, EncodedFrame& output) = 0;

    /**
     * 请求关键帧
     */
    virtual void requestKeyframe() = 0;

    /**
     * 获取编码器信息
     */
    virtual std::string getEncoderInfo() const = 0;

    /**
     * 关闭编码器
     */
    virtual void shutdown() = 0;
};

/**
 * 软件编码器实现 (x264)
 */
class SoftwareEncoder : public VideoEncoder {
public:
    SoftwareEncoder();
    ~SoftwareEncoder() override;

    bool initialize(const EncoderConfig& config) override;
    bool encode(const cv::Mat& frame, EncodedFrame& output) override;
    void requestKeyframe() override;
    std::string getEncoderInfo() const override;
    void shutdown() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * 创建编码器
 */
std::unique_ptr<VideoEncoder> createEncoder(EncoderType type);

/**
 * 检查硬件编码器可用性
 */
bool isNvencAvailable();
bool isVaapiAvailable();

} // namespace vehicle
} // namespace fsm
