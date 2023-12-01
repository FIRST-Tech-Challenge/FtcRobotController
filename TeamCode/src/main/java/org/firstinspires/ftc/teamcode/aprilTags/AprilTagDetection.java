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

package org.firstinspires.ftc.teamcode.aprilTags;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class AprilTagDetection extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final int CAMERA_WIDTH = 640;
    static final int CAMERA_HEIGHT = 480;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C270 webcam at 640 x 480.
    // You will need to do your own calibration for other configurations!
    // Calibration data is here: https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/TeamCode/src/main/res/xml/teamwebcamcalibrations.xml
    double fx = 822.317;
    double fy = 822.317;
    double cx = 319.495;
    double cy = 242.502;


    double tagsize_meters = 0.051; // This is the Centerstage 2023-2024 standard april tag size on the backdrop
    private double z = 0.0;
    int numberOfZeroValuesSinceNonZero = 0;
    private Telemetry telemetry;

    public double GetDistanceAwayFromTheBackdrop() {
        ArrayList<org.openftc.apriltag.AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        // Default is 0 if robot is too far to see
        double zNew = 0.0;
        numberOfZeroValuesSinceNonZero += 1;

        // Calcaulate average distance to AprilTag
        if (!(currentDetections.isEmpty())) {
            numberOfZeroValuesSinceNonZero = 0;
            for (org.openftc.apriltag.AprilTagDetection aprilTag : currentDetections) {
                zNew += zPos(aprilTag);
            }
        zNew /= currentDetections.size();
        }
        z = zNew!=0? zNew : (z > 1.0? 0 : z);
        //if (numberOfZeroValuesSinceNonZero > 30){
        //    z = 0;
        //}

        return z;
    }

    public void Setup(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //telemetry.addData("cameraMonitorViewId", cameraMonitorViewId);
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        //camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam B"), cameraMonitorViewId); // Webcam Back
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize_meters, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
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

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(org.openftc.apriltag.AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));

        telemetry.addLine(String.format("Position x", detection.pose.x));
        telemetry.addLine(String.format("Position y:", detection.pose.y));
        telemetry.addLine(String.format("Position z: ", detection.pose.z));

        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }

    public double xPos(org.openftc.apriltag.AprilTagDetection detection) { return detection.pose.x ; }
    public double yPos(org.openftc.apriltag.AprilTagDetection detection) { return detection.pose.y ; }
    public double zPos(org.openftc.apriltag.AprilTagDetection detection) { return detection.pose.z ; }

    Orientation rot;

    public double yaw(org.openftc.apriltag.AprilTagDetection detection) { return rot.firstAngle ; }
    public double pitch(org.openftc.apriltag.AprilTagDetection detection) { return rot.secondAngle ; }
    public double roll(org.openftc.apriltag.AprilTagDetection detection) { return rot.thirdAngle ; }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}

