#pragma once

#include <depthai/pipeline/datatype/CameraControl.hpp>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/properties/MonoCameraProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief MonoCamera node. For use with grayscale sensors.
 */
class MonoCamera : public NodeCRTP<Node, MonoCamera, MonoCameraProperties> {
   public:
    constexpr static const char* NAME = "MonoCamera";

   private:
    std::shared_ptr<RawCameraControl> rawControl;

   protected:
    Properties& getProperties();

   public:
    MonoCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    MonoCamera(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Initial control options to apply to sensor
     */
    CameraControl initialControl;

    /**
     * Input for CameraControl message, which can modify camera parameters in runtime
     * Default queue is blocking with size 8
     */
    Input inputControl{*this, "inputControl", Input::Type::SReceiver, true, 8, {{DatatypeEnum::CameraControl, false}}};

    /**
     * Outputs ImgFrame message that carries RAW8 encoded (grayscale) frame data.
     *
     * Suitable for use StereoDepth node. Processed by ISP
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs ImgFrame message that carries RAW10-packed (MIPI CSI-2 format) frame data.
     *
     * Captured directly from the camera sensor
     */
    Output raw{*this, "raw", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Outputs metadata-only ImgFrame message as an early indicator of an incoming frame.
     *
     * It's sent on the MIPI SoF (start-of-frame) event, just after the exposure of the current frame
     * has finished and before the exposure for next frame starts.
     * Could be used to synchronize various processes with camera capture.
     * Fields populated: camera id, sequence number, timestamp
     */
    Output frameEvent{*this, "frameEvent", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

    /**
     * Specify which board socket to use
     * @param boardSocket Board socket to use
     */
    void setBoardSocket(CameraBoardSocket boardSocket);

    /**
     * Retrieves which board socket to use
     * @returns Board socket to use
     */
    CameraBoardSocket getBoardSocket() const;

    /**
     * Specify which camera to use by name
     * @param name Name of the camera to use
     */
    void setCamera(std::string name);

    /**
     * Retrieves which camera to use by name
     * @returns Name of the camera to use
     */
    std::string getCamera() const;

    // Set which mono camera to use
    [[deprecated("Use 'setBoardSocket()' instead.")]] void setCamId(int64_t id);

    // Get which mono camera to use
    [[deprecated("Use 'getBoardSocket()' instead.")]] int64_t getCamId() const;

    /// Set camera image orientation
    void setImageOrientation(CameraImageOrientation imageOrientation);

    /// Get camera image orientation
    CameraImageOrientation getImageOrientation() const;

    /// Set sensor resolution
    void setResolution(Properties::SensorResolution resolution);

    /// Get sensor resolution
    Properties::SensorResolution getResolution() const;

    // Set events on which frames will be received
    void setFrameEventFilter(const std::vector<dai::FrameEvent>& events);

    // Get events on which frames will be received
    std::vector<dai::FrameEvent> getFrameEventFilter() const;

    /**
     * Set rate at which camera should produce frames
     * @param fps Rate in frames per second
     */
    void setFps(float fps);

    /**
     * Isp 3A rate (auto focus, auto exposure, auto white balance, camera controls etc.).
     * Default (0) matches the camera FPS, meaning that 3A is running on each frame.
     * Reducing the rate of 3A reduces the CPU usage on CSS, but also increases the convergence rate of 3A.
     * Note that camera controls will be processed at this rate. E.g. if camera is running at 30 fps, and camera control is sent at every frame,
     * but 3A fps is set to 15, the camera control messages will be processed at 15 fps rate, which will lead to queueing.

     */
    void setIsp3aFps(int isp3aFps);

    /**
     * Get rate at which camera should produce frames
     * @returns Rate in frames per second
     */
    float getFps() const;

    /// Get sensor resolution as size
    std::tuple<int, int> getResolutionSize() const;
    /// Get sensor resolution width
    int getResolutionWidth() const;
    /// Get sensor resolution height
    int getResolutionHeight() const;

    /// Set number of frames in main (ISP output) pool
    void setNumFramesPool(int num);
    /// Set number of frames in raw pool
    void setRawNumFramesPool(int num);

    /// Get number of frames in main (ISP output) pool
    int getNumFramesPool() const;
    /// Get number of frames in raw pool
    int getRawNumFramesPool() const;

    /**
     * Configures whether the camera `raw` frames are saved as MIPI-packed to memory.
     * The packed format is more efficient, consuming less memory on device, and less data
     * to send to host: RAW10: 4 pixels saved on 5 bytes, RAW12: 2 pixels saved on 3 bytes.
     * When packing is disabled (`false`), data is saved lsb-aligned, e.g. a RAW10 pixel
     * will be stored as uint16, on bits 9..0: 0b0000'00pp'pppp'pppp.
     * Default is auto: enabled for standard color/monochrome cameras where ISP can work
     * with both packed/unpacked, but disabled for other cameras like ToF.
     */
    void setRawOutputPacked(bool packed);
};

}  // namespace node
}  // namespace dai
