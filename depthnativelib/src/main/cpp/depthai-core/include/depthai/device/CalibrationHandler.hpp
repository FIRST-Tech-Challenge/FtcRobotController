#pragma once
#include <string>
#include <tuple>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/EepromData.hpp"
#include "depthai-shared/common/Point2f.hpp"
#include "depthai-shared/common/Size2f.hpp"
#include "depthai/utility/Path.hpp"

namespace dai {
/**
 * CalibrationHandler is an interface to read/load/write structured calibration and device data.
 * The following fields are protected and aren't allowed to be overridden by default:
 *  - boardName
 *  - boardRev
 *  - boardConf
 *  - hardwareConf
 *  - batchName
 *  - batchTime
 *  - boardOptions
 *  - productName
 */
class CalibrationHandler {
   public:
    CalibrationHandler() = default;

    /**
     * Construct a new Calibration Handler object using the
     * eeprom json file created from calibration procedure.
     *
     * @param eepromDataPath takes the full path to the json file containing the calibration and device info.
     */
    explicit CalibrationHandler(dai::Path eepromDataPath);

    /**
     * Construct a new Calibration Handler object using the board
     * config json file and .calib binary files created using gen1 calibration.
     *
     * @param calibrationDataPath Full Path to the .calib binary file from the gen1 calibration. (Supports only Version 5)
     * @param boardConfigPath Full Path to the board config json file containing device information.
     */
    CalibrationHandler(dai::Path calibrationDataPath, dai::Path boardConfigPath);

    /**
     * Construct a new Calibration Handler object from EepromData object.
     *
     * @param eepromData EepromData data structure containing the calibration data.
     */
    explicit CalibrationHandler(EepromData eepromData);

    /**
     * Construct a new Calibration Handler object from JSON EepromData.
     *
     * @param eepromDataJson EepromData as JSON
     */
    static CalibrationHandler fromJson(nlohmann::json eepromDataJson);

    /**
     * Get the Eeprom Data object
     *
     * @return EepromData object which contains the raw calibration data
     */
    dai::EepromData getEepromData() const;

    /**
     * Get the Camera Intrinsics object
     *
     * @param cameraId Uses the cameraId to identify which camera intrinsics to return
     * @param resizewidth resized width of the image for which intrinsics is requested.  resizewidth = -1 represents width is same as default intrinsics
     * @param resizeHeight resized height of the image for which intrinsics is requested.  resizeHeight = -1 represents height is same as default intrinsics
     * @param topLeftPixelId (x, y) point represents the top left corner coordinates of the cropped image which is used to modify the intrinsics for the
     * respective cropped image
     * @param bottomRightPixelId (x, y) point represents the bottom right corner coordinates of the cropped image which is used to modify the intrinsics for
     * the respective cropped image
     * @param keepAspectRatio Enabling this will scale on width or height depending on which provides the max resolution and crops the remaining part of the
     * other side
     * @return Represents the 3x3 intrinsics matrix of the respective camera at the requested size and crop dimensions.
     *
     * Matrix representation of intrinsic matrix
     * \f[ \text{Intrinsic Matrix} = \left [ \begin{matrix}
     *                                        f_x & 0 & c_x \\
     *                                        0 & f_y & c_y \\
     *                                        0 &  0  & 1
     *                                      \end{matrix} \right ] \f]
     *
     */
    std::vector<std::vector<float>> getCameraIntrinsics(CameraBoardSocket cameraId,
                                                        int resizeWidth = -1,
                                                        int resizeHeight = -1,
                                                        Point2f topLeftPixelId = Point2f(),
                                                        Point2f bottomRightPixelId = Point2f(),
                                                        bool keepAspectRatio = true) const;

    /**
     * Get the Camera Intrinsics object
     *
     * @param cameraId Uses the cameraId to identify which camera intrinsics to return
     * @param destShape resized width and height of the image for which intrinsics is requested.
     * @param topLeftPixelId (x, y) point represents the top left corner coordinates of the cropped image which is used to modify the intrinsics for the
     * respective cropped image
     * @param bottomRightPixelId (x, y) point represents the bottom right corner coordinates of the cropped image which is used to modify the intrinsics for
     * the respective cropped image
     * @param keepAspectRatio Enabling this will scale on width or height depending on which provides the max resolution and crops the remaining part of the
     * other side
     * @return Represents the 3x3 intrinsics matrix of the respective camera at the requested size and crop dimensions.
     *
     * Matrix representation of intrinsic matrix
     * \f[ \text{Intrinsic Matrix} = \left [ \begin{matrix}
     *                                        f_x & 0 & c_x \\
     *                                        0 & f_y & c_y \\
     *                                        0 &  0  & 1
     *                                      \end{matrix} \right ] \f]
     *
     */
    std::vector<std::vector<float>> getCameraIntrinsics(CameraBoardSocket cameraId,
                                                        Size2f destShape,
                                                        Point2f topLeftPixelId = Point2f(),
                                                        Point2f bottomRightPixelId = Point2f(),
                                                        bool keepAspectRatio = true) const;

    /**
     * Get the Camera Intrinsics object
     *
     * @param cameraId Uses the cameraId to identify which camera intrinsics to return
     * @param destShape resized width and height of the image for which intrinsics is requested.
     * @param topLeftPixelId (x, y) point represents the top left corner coordinates of the cropped image which is used to modify the intrinsics for the
     * respective cropped image
     * @param bottomRightPixelId (x, y) point represents the bottom right corner coordinates of the cropped image which is used to modify the intrinsics for
     * the respective cropped image
     * @param keepAspectRatio Enabling this will scale on width or height depending on which provides the max resolution and crops the remaining part of the
     * other side
     * @return Represents the 3x3 intrinsics matrix of the respective camera at the requested size and crop dimensions.
     *
     * Matrix representation of intrinsic matrix
     * \f[ \text{Intrinsic Matrix} = \left [ \begin{matrix}
     *                                        f_x & 0 & c_x \\
     *                                        0 & f_y & c_y \\
     *                                        0 &  0  & 1
     *                                      \end{matrix} \right ] \f]
     *
     */
    std::vector<std::vector<float>> getCameraIntrinsics(CameraBoardSocket cameraId,
                                                        std::tuple<int, int> destShape,
                                                        Point2f topLeftPixelId = Point2f(),
                                                        Point2f bottomRightPixelId = Point2f(),
                                                        bool keepAspectRatio = true) const;

    /**
     * Get the Default Intrinsics object
     *
     * @param cameraId Uses the cameraId to identify which camera intrinsics to return
     * @return Represents the 3x3 intrinsics matrix of the respective camera along with width and height at which it was calibrated.
     *
     * Matrix representation of intrinsic matrix
     * \f[ \text{Intrinsic Matrix} = \left [ \begin{matrix}
     *                                        f_x & 0 & c_x \\
     *                                        0 & f_y & c_y \\
     *                                        0 &  0  & 1
     *                                      \end{matrix} \right ] \f]
     *
     */
    std::tuple<std::vector<std::vector<float>>, int, int> getDefaultIntrinsics(CameraBoardSocket cameraId) const;

    /**
     * Get the Distortion Coefficients object
     *
     * @param cameraId Uses the cameraId to identify which distortion Coefficients to return.
     * @return the distortion coefficients of the requested camera in this order: [k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4,τx,τy]
     */
    std::vector<float> getDistortionCoefficients(CameraBoardSocket cameraId) const;

    /**
     *  Get the Fov of the camera
     *
     * @param cameraId of the camera of which we are fetching fov.
     * @param useSpec Disabling this bool will calculate the fov based on intrinsics (focal length, image width), instead of getting it from the camera specs
     * @return field of view of the camera with given cameraId.
     */
    float getFov(CameraBoardSocket cameraId, bool useSpec = true) const;

    /**
     *  Get the lens position of the given camera
     *
     * @param cameraId of the camera with lens position is requested.
     * @return lens position of the camera with given cameraId at which it was calibrated.
     */
    uint8_t getLensPosition(CameraBoardSocket cameraId) const;

    /**
     *  Get the distortion model of the given camera
     *
     * @param cameraId of the camera with lens position is requested.
     * @return lens position of the camera with given cameraId at which it was calibrated.
     */
    CameraModel getDistortionModel(CameraBoardSocket cameraId) const;

    /**
     * Get the Camera Extrinsics object between two cameras from the calibration data if there is a linked connection
     *  between any two cameras then the relative rotation and translation (in centimeters) is returned by this function.
     *
     * @param srcCamera Camera Id of the camera which will be considered as origin.
     * @param dstCamera  Camera Id of the destination camera to which we are fetching the rotation and translation from the SrcCamera
     * @param useSpecTranslation Enabling this bool uses the translation information from the board design data
     * @return a transformationMatrix which is 4x4 in homogeneous coordinate system
     *
     * Matrix representation of transformation matrix
     * \f[ \text{Transformation Matrix} = \left [ \begin{matrix}
     *                                             r_{00} & r_{01} & r_{02} & T_x \\
     *                                             r_{10} & r_{11} & r_{12} & T_y \\
     *                                             r_{20} & r_{21} & r_{22} & T_z \\
     *                                               0    &   0    &   0    & 1
     *                                            \end{matrix} \right ] \f]
     *
     */
    std::vector<std::vector<float>> getCameraExtrinsics(CameraBoardSocket srcCamera, CameraBoardSocket dstCamera, bool useSpecTranslation = false) const;

    /**
     * Get the Camera translation vector between two cameras from the calibration data.
     *
     * @param srcCamera Camera Id of the camera which will be considered as origin.
     * @param dstCamera  Camera Id of the destination camera to which we are fetching the translation vector from the SrcCamera
     * @param useSpecTranslation Disabling this bool uses the translation information from the calibration data (not the board design data)
     * @return a translation vector like [x, y, z] in centimeters
     */
    std::vector<float> getCameraTranslationVector(CameraBoardSocket srcCamera, CameraBoardSocket dstCamera, bool useSpecTranslation = true) const;

    /**
     * Get the baseline distance between two specified cameras. By default it will get the baseline between CameraBoardSocket.CAM_C
     * and CameraBoardSocket.CAM_B.
     *
     * @param cam1 First camera
     * @param cam2 Second camera
     * @param useSpecTranslation Enabling this bool uses the translation information from the board design data (not the calibration data)
     * @return baseline distance in centimeters
     */
    float getBaselineDistance(CameraBoardSocket cam1 = CameraBoardSocket::CAM_C,
                              CameraBoardSocket cam2 = CameraBoardSocket::CAM_B,
                              bool useSpecTranslation = true) const;

    /**
     * Get the Camera To Imu Extrinsics object
     * From the data loaded if there is a linked connection between IMU and the given camera then there relative rotation and translation from the camera to IMU
     * is returned.
     *
     * @param cameraId Camera Id of the camera which will be considered as origin. from which Transformation matrix to the IMU will be found
     * @param useSpecTranslation Enabling this bool uses the translation information from the board design data
     * @return Returns a transformationMatrix which is 4x4 in homogeneous coordinate system
     *
     * Matrix representation of transformation matrix
     * \f[ \text{Transformation Matrix} = \left [ \begin{matrix}
     *                                             r_{00} & r_{01} & r_{02} & T_x \\
     *                                             r_{10} & r_{11} & r_{12} & T_y \\
     *                                             r_{20} & r_{21} & r_{22} & T_z \\
     *                                               0    &   0    &   0    & 1
     *                                            \end{matrix} \right ] \f]
     *
     */
    std::vector<std::vector<float>> getCameraToImuExtrinsics(CameraBoardSocket cameraId, bool useSpecTranslation = false) const;

    /**
     * Get the Imu To Camera Extrinsics object from the data loaded if there is a linked connection
     * between IMU and the given camera then there relative rotation and translation from the IMU to Camera
     * is returned.
     *
     * @param cameraId Camera Id of the camera which will be considered as destination. To which Transformation matrix from the IMU will be found.
     * @param useSpecTranslation Enabling this bool uses the translation information from the board design data
     * @return Returns a transformationMatrix which is 4x4 in homogeneous coordinate system
     *
     * Matrix representation of transformation matrix
     * \f[ \text{Transformation Matrix} = \left [ \begin{matrix}
     *                                             r_{00} & r_{01} & r_{02} & T_x \\
     *                                             r_{10} & r_{11} & r_{12} & T_y \\
     *                                             r_{20} & r_{21} & r_{22} & T_z \\
     *                                               0    &   0    &   0    & 1
     *                                            \end{matrix} \right ] \f]
     *
     */
    std::vector<std::vector<float>> getImuToCameraExtrinsics(CameraBoardSocket cameraId, bool useSpecTranslation = false) const;

    /**
     *
     * Get the Stereo Right Rectification Rotation object
     *
     * @return returns a 3x3 rectification rotation matrix
     */
    std::vector<std::vector<float>> getStereoRightRectificationRotation() const;

    /**
     * Get the Stereo Left Rectification Rotation object
     *
     * @return returns a 3x3 rectification rotation matrix
     */
    std::vector<std::vector<float>> getStereoLeftRectificationRotation() const;

    /**
     * Get the camera id of the camera which is used as left camera of the stereo setup
     *
     * @return cameraID of the camera used as left camera
     */
    dai::CameraBoardSocket getStereoLeftCameraId() const;

    /**
     * Get the camera id of the camera which is used as right camera of the stereo setup
     *
     * @return cameraID of the camera used as right camera
     */
    dai::CameraBoardSocket getStereoRightCameraId() const;

    /**
     * Write raw calibration/board data to json file.
     *
     * @param destPath  Full path to the json file in which raw calibration data will be stored
     * @return True on success, false otherwise
     */
    bool eepromToJsonFile(dai::Path destPath) const;

    /**
     * Get JSON representation of calibration data
     *
     * @return JSON structure
     */
    nlohmann::json eepromToJson() const;

    /**
     * Set the Board Info object
     *
     * @param version Sets the version of the Calibration data(Current version is 6)
     * @param boardName Sets your board name.
     * @param boardRev set your board revision id.
     */
    void setBoardInfo(std::string boardName, std::string boardRev);

    /**
     * Set the Board Info object. Creates version 7 EEPROM data
     *
     * @param productName Sets product name (alias).
     * @param boardName Sets board name.
     * @param boardRev Sets board revision id.
     * @param boardConf Sets board configuration id.
     * @param hardwareConf Sets hardware configuration id.
     * @param batchName Sets batch name.
     * @param batchTime Sets batch time (unix timestamp).
     * @param boardCustom Sets a custom board (Default empty string).
     */
    void setBoardInfo(std::string productName,
                      std::string boardName,
                      std::string boardRev,
                      std::string boardConf,
                      std::string hardwareConf,
                      std::string batchName,
                      uint64_t batchTime,
                      uint32_t boardOptions,
                      std::string boardCustom = "");

    /**
     * Set the Board Info object. Creates version 7 EEPROM data
     *
     * @param deviceName Sets device name.
     * @param productName Sets product name (alias).
     * @param boardName Sets board name.
     * @param boardRev Sets board revision id.
     * @param boardConf Sets board configuration id.
     * @param hardwareConf Sets hardware configuration id.
     * @param batchName Sets batch name. Not supported anymore
     * @param batchTime Sets batch time (unix timestamp).
     * @param boardCustom Sets a custom board (Default empty string).
     */
    void setBoardInfo(std::string deviceName,
                      std::string productName,
                      std::string boardName,
                      std::string boardRev,
                      std::string boardConf,
                      std::string hardwareConf,
                      std::string batchName,
                      uint64_t batchTime,
                      uint32_t boardOptions,
                      std::string boardCustom = "");

    /**
     * Set the deviceName which responses to getDeviceName of Device
     *
     * @param deviceName Sets device name.
     */
    void setDeviceName(std::string deviceName);

    /**
     * Set the productName which acts as alisas for users to identify the device
     *
     * @param productName Sets product name (alias).
     */

    void setProductName(std::string productName);

    /**
     * Set the Camera Intrinsics object
     *
     * @param cameraId CameraId of the camera for which Camera intrinsics are being loaded
     * @param intrinsics 3x3 intrinsics matrix
     * @param frameSize Represents the width and height of the image at which intrinsics are calculated.
     *
     * Matrix representation of intrinsic matrix
     * \f[ \text{Intrinsic Matrix} = \left [ \begin{matrix}
     *                                        f_x & 0 & c_x \\
     *                                        0 & f_y & c_y \\
     *                                        0 &  0  & 1
     *                                      \end{matrix} \right ] \f]
     *
     */
    void setCameraIntrinsics(CameraBoardSocket cameraId, std::vector<std::vector<float>> intrinsics, Size2f frameSize);

    /**
     * Set the Camera Intrinsics object
     *
     * @param cameraId CameraId of the camera for which Camera intrinsics are being loaded
     * @param intrinsics 3x3 intrinsics matrix
     * @param width Represents the width of the image at which intrinsics are calculated.
     * @param height Represents the height of the image at which intrinsics are calculated.
     *
     * Matrix representation of intrinsic matrix
     * \f[ \text{Intrinsic Matrix} = \left [ \begin{matrix}
     *                                        f_x & 0 & c_x \\
     *                                        0 & f_y & c_y \\
     *                                        0 &  0  & 1
     *                                      \end{matrix} \right ] \f]
     *
     */
    void setCameraIntrinsics(CameraBoardSocket cameraId, std::vector<std::vector<float>> intrinsics, int width, int height);

    /**
     * Set the Camera Intrinsics object
     *
     * @param cameraId CameraId of the camera for which Camera intrinsics are being loaded
     * @param intrinsics 3x3 intrinsics matrix
     * @param frameSize Represents the width and height of the image at which intrinsics are calculated.
     *
     * Matrix representation of intrinsic matrix
     * \f[ \text{Intrinsic Matrix} = \left [ \begin{matrix}
     *                                        f_x & 0 & c_x \\
     *                                        0 & f_y & c_y \\
     *                                        0 &  0  & 1
     *                                      \end{matrix} \right ] \f]
     *
     */
    void setCameraIntrinsics(CameraBoardSocket cameraId, std::vector<std::vector<float>> intrinsics, std::tuple<int, int> frameSize);

    /**
     * Sets the distortion Coefficients obtained from camera calibration
     *
     * @param cameraId Camera Id of the camera for which distortion coefficients are computed
     * @param distortionCoefficients Distortion Coefficients of the respective Camera.
     */
    void setDistortionCoefficients(CameraBoardSocket cameraId, std::vector<float> distortionCoefficients);

    /**
     * Set the Fov of the Camera
     *
     * @param cameraId Camera Id of the camera
     * @param hfov Horizontal fov of the camera from Camera Datasheet
     */
    void setFov(CameraBoardSocket cameraId, float hfov);

    /**
     * Sets the distortion Coefficients obtained from camera calibration
     *
     * @param cameraId Camera Id of the camera
     * @param lensPosition lens posiotion value of the camera at the time of calibration
     */
    void setLensPosition(CameraBoardSocket cameraId, uint8_t lensPosition);

    /**
     * Set the Camera Type object
     *
     * @param cameraId CameraId of the camera for which cameraModel Type is being updated.
     * @param cameraModel Type of the model the camera represents
     */
    void setCameraType(CameraBoardSocket cameraId, CameraModel cameraModel);

    /**
     * Set the Camera Extrinsics object
     *
     * @param srcCameraId Camera Id of the camera which will be considered as relative origin.
     * @param destCameraId Camera Id of the camera which will be considered as destination from srcCameraId.
     * @param rotationMatrix Rotation between srcCameraId and destCameraId origins.
     * @param translation Translation between srcCameraId and destCameraId origins.
     * @param specTranslation Translation between srcCameraId and destCameraId origins from the design.
     */
    void setCameraExtrinsics(CameraBoardSocket srcCameraId,
                             CameraBoardSocket destCameraId,
                             std::vector<std::vector<float>> rotationMatrix,
                             std::vector<float> translation,
                             std::vector<float> specTranslation = {0, 0, 0});

    /**
     * Set the Imu to Camera Extrinsics object
     *
     * @param destCameraId Camera Id of the camera which will be considered as destination from IMU.
     * @param rotationMatrix Rotation between srcCameraId and destCameraId origins.
     * @param translation Translation between IMU and destCameraId origins.
     * @param specTranslation Translation between IMU and destCameraId origins from the design.
     */
    void setImuExtrinsics(CameraBoardSocket destCameraId,
                          std::vector<std::vector<float>> rotationMatrix,
                          std::vector<float> translation,
                          std::vector<float> specTranslation = {0, 0, 0});

    /**
     * Set the Stereo Left Rectification object
     *
     * @param cameraId CameraId of the camera which will be used as left Camera of stereo Setup
     * @param rectifiedRotation Rectification rotation of the left camera required for feature matching
     *
     * Homography of the Left Rectification = Intrinsics_right * rectifiedRotation * inv(Intrinsics_left)
     */
    void setStereoLeft(CameraBoardSocket cameraId, std::vector<std::vector<float>> rectifiedRotation);

    /**
     * Set the Stereo Right Rectification object
     *
     * @param cameraId CameraId of the camera which will be used as left Camera of stereo Setup
     * @param rectifiedRotation Rectification rotation of the left camera required for feature matching
     *
     * Homography of the Right Rectification = Intrinsics_right * rectifiedRotation * inv(Intrinsics_right)
     */
    void setStereoRight(CameraBoardSocket cameraId, std::vector<std::vector<float>> rectifiedRotation);

    /**
     * Using left camera as the head it iterates over the camera extrinsics connection
     * to check if all the camera extrinsics are connected and no loop exists.
     *
     * @return true on proper connection with no loops.
     */
    bool validateCameraArray() const;

   private:
    /** when the user is writing extrinsics do we validate if
     * the connection between all the cameras exists ?
     * Some users might not need that connection so they might ignore adding
     * that in that case if the user calls the extrinsics betwwn those cameras it
     * fails We can provide an appropriate error that connection doesn't exist between the requested camera id's..
     * And other option is making sure the connection exists all the time by validating the links.
     */
    // bool isCameraArrayConnected;
    dai::EepromData eepromData;
    std::vector<std::vector<float>> computeExtrinsicMatrix(CameraBoardSocket srcCamera, CameraBoardSocket dstCamera, bool useSpecTranslation = false) const;
    bool checkExtrinsicsLink(CameraBoardSocket srcCamera, CameraBoardSocket dstCamera) const;
    bool checkSrcLinks(CameraBoardSocket headSocket) const;
};

}  // namespace dai