//package org.firstinspires.ftc.teamcode.vision;
//
//public class AprilTagDetector {
//}

/*
 * Copyright (c) 2021 OpenFTC Team
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

package org.firstinspires.ftc.teamcode.vision.April;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

enum DetectorState {
    NOT_CONFIGURED,
    INITIALIZING,
    RUNNING,
    INIT_FAILURE_NOT_RUNNING;
}

public class AprilTagDetector extends OpenCvPipeline {

    private OpenCvCamera camera;
    private String cameraName;
    private AprilTagPipeline aprilTagPipeline;
    private final HardwareMap hardwareMap;

    private final int WIDTH;
    private final int HEIGHT;

    private DetectorState detectorState = DetectorState.NOT_CONFIGURED;
    private final Object sync = new Object();

    // Metres
    private double tagSize;
    // Pixels
    private double fx, fy, cx, cy;

    public AprilTagDetector(HardwareMap hardwareMap, String cameraName, int width, int height, double tagSize, double fx, double fy, double cx, double cy) {
        this.hardwareMap = hardwareMap;
        this.cameraName = cameraName;
        WIDTH = width;
        HEIGHT = height;

        this.tagSize = tagSize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
    }

    public DetectorState getDetectorState() {
        synchronized (sync) {
            return detectorState;
        }
    }

    public void init() {
        synchronized (sync) {
            if (detectorState == DetectorState.NOT_CONFIGURED) {
                int cameraMonitorViewId = hardwareMap
                        .appContext.getResources()
                        .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

                camera = OpenCvCameraFactory.getInstance()
                        .createWebcam(hardwareMap.get(WebcamName.class, cameraName), cameraMonitorViewId);

                camera.setPipeline(aprilTagPipeline = new AprilTagPipeline(tagSize, fx, fy, cx, cy));

                detectorState = DetectorState.INITIALIZING;

                camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPRIGHT);

                        synchronized (sync) {
                            detectorState = DetectorState.RUNNING;
                        }
                    }

                    @Override
                    public void onError(int errorCode) {
                        synchronized (sync) {
                            detectorState = DetectorState.INIT_FAILURE_NOT_RUNNING;
                        }

                        RobotLog.addGlobalWarningMessage("Warning: Camera device failed to open with EasyOpenCv error: " +
                                ((errorCode == -1) ? "CAMERA_OPEN_ERROR_FAILURE_TO_OPEN_CAMERA_DEVICE" : "CAMERA_OPEN_ERROR_POSTMORTEM_OPMODE")
                        ); //Warn the user about the issue
                    }
                });
            }
        }
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    private int getPlacementId() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagPipeline.getLatestDetections();
        AprilTagDetection tagOfInterest = null;
        boolean tagFound = true;

        if (currentDetections.size() != 0) {
            tagFound = false;
        }

        // TODO: Change April Tag Ids based on if u want to change them
        for (AprilTagDetection tag : currentDetections) {
            if (tag.id >= 1 && tag.id <= 6) {
                tagOfInterest = tag;
                break;
            }
        }

        if (tagOfInterest == null) {
            tagFound = false;
        }

        if (tagFound) {
            return tagOfInterest.id;
        } else {
            return 1;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        return null;
    }

    public enum Placement {
        BlueLeft,
        BlueCenter,
        BlueRight,
        RedLeft,
        RedCenter,
        RedRight
    }

    // TODO: You can change the IDs to different April Tags in the 36h11 family
    public Placement getPlacement() {
        switch (getPlacementId()) {
            case 1:
                return Placement.BlueLeft;
            case 2:
                return Placement.BlueCenter;
            case 3:
                return Placement.BlueRight;
            case 4:
                return Placement.RedLeft;
            case 5:
                return Placement.RedCenter;
            default:
                return Placement.RedRight;
        }
    }

}