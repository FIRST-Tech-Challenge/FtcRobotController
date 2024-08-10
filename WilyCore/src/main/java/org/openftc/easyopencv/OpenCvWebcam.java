/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.openftc.easyopencv;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity;

public interface OpenCvWebcam extends OpenCvCamera
{
    /**
     * Set how long to wait for permission to open the
     * camera before giving up
     * @param ms milliseconds to wait for
     */
    void setMillisecondsPermissionTimeout(int ms);

    enum StreamFormat
    {
        // The only format that was supported historically; it is uncompressed but
        // chroma subsampled and uses lots of bandwidth - this limits frame rate
        // at higher resolutions and also limits the ability to use two cameras
        // on the same bus to lower resolutions
        YUY2,

        // Compressed motion JPEG stream format; allows for higher resolutions at
        // full frame rate, and better ability to use two cameras on the same bus.
        // Requires extra CPU time to run decompression routine.
        MJPEG;
    }

    /**
     * Same as {@link #startStreaming(int, int, OpenCvCameraRotation)} except for
     * @param streamFormat indicates the desired stream format.
     */
    void startStreaming(int width, int height, OpenCvCameraRotation rotation, StreamFormat streamFormat);

    /***
     * Gets the {@link ExposureControl} for this webcam.
     * Please see that interface's javadoc for how to use
     * it. It is an interface provided directly by the SDK
     * UVC driver, not EasyOpenCV.
     *
     * @return the ExposureControl for this webcam
     */
    ExposureControl getExposureControl();

    /***
     * Gets the {@link FocusControl} for this webcam.
     * Please see that interface's javadoc for how to use
     * it. It is an interface provided directly by the SDK
     * UVC driver, not EasyOpenCV.
     *
     * @return the FocusControl for this webcam
     */
    FocusControl getFocusControl();

    /***
     * Gets the {@link PtzControl} for this webcam.
     * Please see that interface's javadoc for how to use
     * it. It is an interface provided directly by the SDK
     * UVC driver, not EasyOpenCV.
     *
     * @return the PtzControl for this webcam
     */
    PtzControl getPtzControl();

    /***
     * Gets the {@link GainControl} for this webcam.
     * Please see that interface's javadoc for how to use
     * it. It is an interface provided directly by the SDK
     * UVC driver, not EasyOpenCV.
     *
     * @return the GainControl for this webcam
     */
    GainControl getGainControl();

    /***
     * Gets the {@link WhiteBalanceControl} for this webcam.
     * Please see that interface's javadoc for how to use
     * it. It is an interface provided directly by the SDK
     * UVC driver, not EasyOpenCV.
     *
     * @return the WhiteBalanceControl for this webcam
     */
    WhiteBalanceControl getWhiteBalanceControl();

    /**
     * Gets a control for this webcam
     * @param controlType the class of the control to get
     * @return the requested control
     */
    <T extends CameraControl> T getControl(Class<T> controlType);

    /**
     * Gets the {@link CameraCalibrationIdentity} for this webcam.
     * Note: the camera must be opened before calling this!
     * @return the CameraCalibrationIdentity for this webcam
     */
    CameraCalibrationIdentity getCalibrationIdentity();
}
