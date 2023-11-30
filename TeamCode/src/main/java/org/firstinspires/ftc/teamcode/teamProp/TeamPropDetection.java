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

package org.firstinspires.ftc.teamcode.teamProp;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Point;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Objects;

public class TeamPropDetection {
    OpenCvCamera camera;
    TeamPropDetectionPipeline teamPropDetectionPipeline;

    static final int CAMERA_WIDTH = 800;
    static final int CAMERA_HEIGHT = 600;
    static final double CAMERA_OFFSET = 00000.0 / 12; // in feet

    static final double FEET_PER_METER = 3.28084;

    static final Point ORIGIN = new Point(0, 0);

    // These correspond to where the team prop is to be in either left, center or right position (in pixels)
    static final double LEFT_BOUNDING_BOX_X = 250;
    static final double RIGHT_BOUNDING_BOX_X = 500;



    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    private Telemetry telemetry;

    private enum propLocation {
        LEFT,
        RIGHT,
        CENTER,
        NULL
    }



    public propLocation GetPropLocation() {
        telemetry.addLine("X Position is:" + teamPropDetectionPipeline.getLatestPosition().x + "Y Position is:" + teamPropDetectionPipeline.getLatestPosition().y);

        if (Objects.equals(teamPropDetectionPipeline.getLatestPosition(), ORIGIN)){
            return propLocation.NULL;
        }
        if (teamPropDetectionPipeline.getLatestPosition().x < LEFT_BOUNDING_BOX_X){
                return propLocation.LEFT;
            }
        if (teamPropDetectionPipeline.getLatestPosition().x > RIGHT_BOUNDING_BOX_X){
                return propLocation.RIGHT;
            }
        else {
                return propLocation.CENTER;
            }
    }

    public void Setup(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        teamPropDetectionPipeline = new TeamPropDetectionPipeline(fx, fy, cx, cy, telemetry);

        camera.setPipeline(teamPropDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

}