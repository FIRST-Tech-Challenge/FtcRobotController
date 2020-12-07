/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
*/

package com.hfrobots.tnt.corelib.vision;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import lombok.AllArgsConstructor;
import lombok.Builder;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

@Builder
@AllArgsConstructor
public class EasyOpenCvPipelineAndCamera {
    private final OpenCvPipeline openCvPipeline;

    private final HardwareMap hardwareMap;

    private final Telemetry telemetry;

    private OpenCvCamera cvCamera;

    public void createAndRunPipeline() {
        try {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            // Phone Camera
            //cvCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

            // Webcam...
            cvCamera = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

            // OR...  Do Not Activate the Camera Monitor View
            //phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

            /*
             * Open the connection to the camera device
             */
            cvCamera.openCameraDevice();

            /*
             * Specify the image processing pipeline we wish to invoke upon receipt
             * of a frame from the camera. Note that switching pipelines on-the-fly
             * (while a streaming session is in flight) *IS* supported.
             */
            cvCamera.setPipeline(openCvPipeline);

            cvCamera.showFpsMeterOnViewport(false);

            /*
             * Tell the camera to start streaming images to us! Note that you must make sure
             * the resolution you specify is supported by the camera. If it is not, an exception
             * will be thrown.
             *
             * Also, we specify the rotation that the camera is used in. This is so that the image
             * from the camera sensor can be rotated such that it is always displayed with the image upright.
             * For a front facing camera, rotation is defined assuming the user is looking at the screen.
             * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
             * away from the user.
             */
            // 752x416
            cvCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        } catch (Throwable t) {
            Log.e(LOG_TAG, "Failed to start OpenCV Pipeline", t);
        }
    }

    public void pauseViewport() {
        if (cvCamera != null) {
            cvCamera.pauseViewport();
        }
    }

    public void resumeViewport() {
        if (cvCamera != null) {
            cvCamera.resumeViewport();
        }
    }
}
