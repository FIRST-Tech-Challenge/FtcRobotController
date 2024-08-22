#pragma once

#include <depthai/pipeline/Node.hpp>

// shared
#include <depthai-shared/properties/VideoEncoderProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief VideoEncoder node. Encodes frames into MJPEG, H264 or H265.
 */
class VideoEncoder : public NodeCRTP<Node, VideoEncoder, VideoEncoderProperties> {
   public:
    constexpr static const char* NAME = "VideoEncoder";

    VideoEncoder(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    VideoEncoder(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Input for NV12 ImgFrame to be encoded
     * Default queue is blocking with size set by 'setNumFramesPool' (4).
     */
    Input input{*this, "in", Input::Type::SReceiver, true, 4, true, {{DatatypeEnum::ImgFrame, true}}};

    /**
     * Outputs ImgFrame message that carries BITSTREAM encoded (MJPEG, H264 or H265) frame data.
     */
    Output bitstream{*this, "bitstream", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    // Sets default options for a specified size and profile
    /**
     * Sets a default preset based on specified frame rate and profile
     * @param fps Frame rate in frames per second
     * @param profile Encoding profile
     */
    void setDefaultProfilePreset(float fps, Properties::Profile profile);

    /**
     * Sets a default preset based on specified input size, frame rate and profile
     * @param width Input frame width
     * @param height Input frame height
     * @param fps Frame rate in frames per second
     * @param profile Encoding profile
     */
    [[deprecated("Input width/height no longer needed, automatically determined from first frame")]] void setDefaultProfilePreset(int width,
                                                                                                                                  int height,
                                                                                                                                  float fps,
                                                                                                                                  Properties::Profile profile);

    /**
     * Sets a default preset based on specified input size, frame rate and profile
     * @param size Input frame size
     * @param fps Frame rate in frames per second
     * @param profile Encoding profile
     */
    [[deprecated("Input size no longer needed, automatically determined from first frame")]] void setDefaultProfilePreset(std::tuple<int, int> size,
                                                                                                                          float fps,
                                                                                                                          Properties::Profile profile);

    // node properties
    /**
     * Set number of frames in pool
     * @param frames Number of pool frames
     */
    void setNumFramesPool(int frames);

    /**
     * Get number of frames in pool
     * @returns Number of pool frames
     */
    int getNumFramesPool() const;

    // encoder properties
    /// Set rate control mode
    void setRateControlMode(Properties::RateControlMode mode);
    /// Set encoding profile
    void setProfile(Properties::Profile profile);
    /// Set encoding profile
    [[deprecated("Input size no longer needed, automatically determined from first frame")]] void setProfile(std::tuple<int, int> size,
                                                                                                             Properties::Profile profile);
    /// Set encoding profile
    [[deprecated("Input width/height no longer needed, automatically determined from first frame")]] void setProfile(int width,
                                                                                                                     int height,
                                                                                                                     Properties::Profile profile);
    /// Set output bitrate in bps, for CBR rate control mode. 0 for auto (based on frame size and FPS)
    void setBitrate(int bitrate);
    /// Set output bitrate in kbps, for CBR rate control mode. 0 for auto (based on frame size and FPS)
    void setBitrateKbps(int bitrateKbps);

    /**
     * Set keyframe frequency. Every Nth frame a keyframe is inserted.
     *
     * Applicable only to H264 and H265 profiles
     *
     * Examples:
     *
     *  - 30 FPS video, keyframe frequency: 30. Every 1s a keyframe will be inserted
     *
     *  - 60 FPS video, keyframe frequency: 180. Every 3s a keyframe will be inserted
     *
     */
    void setKeyframeFrequency(int freq);

    /// Set number of B frames to be inserted
    void setNumBFrames(int numBFrames);

    /**
     * Set quality
     * @param quality Value between 0-100%. Approximates quality
     */
    void setQuality(int quality);

    /**
     * Set lossless mode. Applies only to [M]JPEG profile
     * @param lossless True to enable lossless jpeg encoding, false otherwise
     */
    void setLossless(bool lossless);

    /**
     * Sets expected frame rate
     * @param frameRate Frame rate in frames per second
     */
    void setFrameRate(float frameRate);

    /**
     * Specifies maximum output encoded frame size
     */
    void setMaxOutputFrameSize(int maxFrameSize);

    /// Get rate control mode
    Properties::RateControlMode getRateControlMode() const;
    /// Get profile
    Properties::Profile getProfile() const;
    /// Get bitrate in bps
    int getBitrate() const;
    /// Get bitrate in kbps
    int getBitrateKbps() const;
    /// Get keyframe frequency
    int getKeyframeFrequency() const;
    // int getMaxBitrate() const;
    /// Get number of B frames
    int getNumBFrames() const;
    /// Get quality
    int getQuality() const;
    /// Get input size
    [[deprecated("Input size no longer available, it's determined when first frame arrives")]] std::tuple<int, int> getSize() const;
    /// Get input width
    [[deprecated("Input size no longer available, it's determined when first frame arrives")]] int getWidth() const;
    /// Get input height
    [[deprecated("Input size no longer available, it's determined when first frame arrives")]] int getHeight() const;
    /// Get frame rate
    float getFrameRate() const;
    /// Get lossless mode. Applies only when using [M]JPEG profile.
    bool getLossless() const;
    int getMaxOutputFrameSize() const;
};

}  // namespace node
}  // namespace dai
