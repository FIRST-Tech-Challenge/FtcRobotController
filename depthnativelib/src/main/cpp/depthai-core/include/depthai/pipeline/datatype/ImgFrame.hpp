#pragma once

#include <chrono>
#include <unordered_map>
#include <vector>

// project
#include "depthai/build/config.hpp"
#include "depthai/common/CameraExposureOffset.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

// shared
#include "depthai-shared/datatype/RawImgFrame.hpp"

// optional
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/opencv.hpp>
#endif

namespace dai {

/**
 * ImgFrame message. Carries image data and metadata.
 */
class ImgFrame : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawImgFrame& img;

   public:
    // Raw* mirror
    using Type = RawImgFrame::Type;
    using Specs = RawImgFrame::Specs;
    using CameraSettings = RawImgFrame::CameraSettings;
    using Buffer::getTimestamp;
    using Buffer::getTimestampDevice;

    /**
     * Construct ImgFrame message.
     * Timestamp is set to now
     */
    ImgFrame();
    explicit ImgFrame(std::shared_ptr<RawImgFrame> ptr);
    virtual ~ImgFrame() = default;

    // getters
    /**
     * Retrieves image timestamp (at the specified offset of exposure) related to dai::Clock::now()
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestamp(CameraExposureOffset offset) const;

    /**
     * Retrieves image timestamp (at the specified offset of exposure) directly captured from device's monotonic clock,
     * not synchronized to host time. Used when monotonicity is required.
     */
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> getTimestampDevice(CameraExposureOffset offset) const;

    /**
     * Retrieves instance number
     */
    unsigned int getInstanceNum() const;

    /**
     * Retrieves image category
     */
    unsigned int getCategory() const;

    /**
     * Retrieves image width in pixels
     */
    unsigned int getWidth() const;

    /**
     * Retrieves image height in pixels
     */
    unsigned int getHeight() const;

    /**
     * Retrieves image type
     */
    Type getType() const;

    /**
     * Retrieves exposure time
     */
    std::chrono::microseconds getExposureTime() const;

    /**
     * Retrieves sensitivity, as an ISO value
     */
    int getSensitivity() const;

    /**
     * Retrieves white-balance color temperature of the light source, in kelvins
     */
    int getColorTemperature() const;

    /**
     * Retrieves lens position, range 0..255. Returns -1 if not available
     */
    int getLensPosition() const;

    // setters
    /**
     * Retrieves image timestamp related to dai::Clock::now()
     */
    ImgFrame& setTimestamp(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Sets image timestamp related to dai::Clock::now()
     */
    ImgFrame& setTimestampDevice(std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> timestamp);

    /**
     * Instance number relates to the origin of the frame (which camera)
     *
     * @param instance Instance number
     */
    ImgFrame& setInstanceNum(unsigned int instance);

    /**
     * @param category Image category
     */
    ImgFrame& setCategory(unsigned int category);

    /**
     * Specifies sequence number
     *
     * @param seq Sequence number
     */
    ImgFrame& setSequenceNum(int64_t seq);

    /**
     * Specifies frame width
     *
     * @param width frame width
     */
    ImgFrame& setWidth(unsigned int width);

    /**
     * Specifies frame height
     *
     * @param height frame height
     */
    ImgFrame& setHeight(unsigned int height);

    /**
     * Specifies frame size
     *
     * @param height frame height
     * @param width frame width
     */
    ImgFrame& setSize(unsigned int width, unsigned int height);

    /**
     * Specifies frame size
     *
     * @param size frame size
     */
    ImgFrame& setSize(std::tuple<unsigned int, unsigned int> size);

    /**
     * Specifies frame type, RGB, BGR, ...
     *
     * @param type Type of image
     */
    ImgFrame& setType(Type type);

// Optional - OpenCV support
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    /**
     * @note This API only available if OpenCV support is enabled
     *
     * Copies cv::Mat data to ImgFrame buffer
     *
     * @param frame Input cv::Mat frame from which to copy the data
     */
    ImgFrame& setFrame(cv::Mat frame);

    /**
     * @note This API only available if OpenCV support is enabled
     *
     * Retrieves data as cv::Mat with specified width, height and type
     *
     * @param copy If false only a reference to data is made, otherwise a copy
     * @returns cv::Mat with corresponding to ImgFrame parameters
     */
    cv::Mat getFrame(bool copy = false);

    /**
     * @note This API only available if OpenCV support is enabled
     *
     * Retrieves cv::Mat suitable for use in common opencv functions.
     * ImgFrame is converted to color BGR interleaved or grayscale depending on type.
     *
     * A copy is always made
     *
     * @returns cv::Mat for use in opencv functions
     */
    cv::Mat getCvFrame();

#else

    template <typename... T>
    struct dependent_false {
        static constexpr bool value = false;
    };
    template <typename... T>
    ImgFrame& setFrame(T...) {
        static_assert(dependent_false<T...>::value, "Library not configured with OpenCV support");
        return *this;
    }
    template <typename... T>
    void getFrame(T...) {
        static_assert(dependent_false<T...>::value, "Library not configured with OpenCV support");
    }
    template <typename... T>
    void getCvFrame(T...) {
        static_assert(dependent_false<T...>::value, "Library not configured with OpenCV support");
    }

#endif
};

}  // namespace dai
