#pragma once

#include <depthai-shared/common/optional.hpp>

#include "depthai-shared/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for VideoEncoder such as profile, bitrate, ...
 */
struct VideoEncoderProperties : PropertiesSerializable<Properties, VideoEncoderProperties> {
    /**
     * Rate control mode specifies if constant or variable bitrate should be used (H264 / H265)
     */
    enum class RateControlMode : int { CBR, VBR };

    /**
     * Encoding profile, H264 (AVC), H265 (HEVC) or MJPEG
     */
    enum class Profile : int { H264_BASELINE, H264_HIGH, H264_MAIN, H265_MAIN, MJPEG };
    /**
     * Specifies preferred bitrate (in bit/s) of compressed output bitstream in CBR mode
     *
     * "0" for automatic computation, based on input resolution and FPS:
     * 720p30: 4Mbps, 1080p30: 8.5Mbps, 1440p30: 14Mbps, 2160p30: 20Mbps
     */
    std::int32_t bitrate = 0;
    /**
     * Every x number of frames a keyframe will be inserted
     */
    std::int32_t keyframeFrequency = 30;
    /**
     * Specifies maximum bitrate (in bit/s) of compressed output bitstream in CBR mode
     *
     * "0" to follow `bitrate` setting
     */
    std::int32_t maxBitrate = 0;
    /**
     * Specifies number of B frames to be inserted
     */
    std::int32_t numBFrames = 0;
    /**
     * This options specifies how many frames are available in this node's pool.
     * Helps when receiver is slow at consuming.
     *
     * Value "0" indicates automatic number of frames assignment
     */
    std::uint32_t numFramesPool = 0;
    /**
     * Specifies max output frame size in pool.
     * Value "0" indicates auto
     */
    std::int32_t outputFrameSize = 0;
    /**
     * Encoding profile, H264, H265 or MJPEG
     */
    Profile profile = Profile::H264_BASELINE;
    /**
     * Value between 0-100% (approximates quality)
     */
    std::int32_t quality = 80;
    /**
     * Lossless mode ([M]JPEG only)
     */
    bool lossless = false;
    /**
     * Rate control mode specifies if constant or variable bitrate should be used (H264 / H265)
     */
    RateControlMode rateCtrlMode = RateControlMode::CBR;
    /**
     * Frame rate
     */
    float frameRate = 30.0f;
};

DEPTHAI_SERIALIZE_EXT(VideoEncoderProperties,
                      bitrate,
                      keyframeFrequency,
                      maxBitrate,
                      numBFrames,
                      numFramesPool,
                      outputFrameSize,
                      profile,
                      quality,
                      lossless,
                      rateCtrlMode,
                      frameRate);

}  // namespace dai
