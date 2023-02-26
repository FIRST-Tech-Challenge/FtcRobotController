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
package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
        (name = "LeftBlue", group = "Auto")
public class RightAuto2 extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag IDs of sleeve
    int Left = 7;
    int Middle = 9;
    int Right = 11;
    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;


    DcMotor Slide;
    ColorSensor color;
    TouchSensor limitSwitch;

    //Claw Mechanism
    Servo ClawX;
    Servo ClawY;


    @Override
    public void runOpMode() {


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Slide = hardwareMap.dcMotor.get("Slide");
        Slide.setDirection(DcMotor.Direction.REVERSE);
        ClawX = hardwareMap.servo.get("ClawX");
        ClawY = hardwareMap.servo.get("ClawY");
        color = hardwareMap.get(ColorSensor.class, "color");
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");


        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        ReadyClaw();


        Pose2d startPose = new Pose2d(0, 8, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence FirstCone = drive.trajectorySequenceBuilder(startPose)

                .addTemporalMarker(() -> SetSlideConditions ())


                .lineToLinearHeading(new Pose2d(59,0, Math.toRadians(0)))

                .waitSeconds(0.2)

                .back(7)
                .addTemporalMarker(() -> SlideTopPole ())


                .lineToLinearHeading(new Pose2d(59,15, Math.toRadians(0)))

                .forward(2.5)


                .waitSeconds(0.1)

                .addTemporalMarker(() -> ClawDrop())
                .waitSeconds(0.5)


                .back(3)



                .build();
        TrajectorySequence ConePickup = drive.trajectorySequenceBuilder(FirstCone.end())
                .addTemporalMarker(() -> FirstCycle())

                .lineToLinearHeading(new Pose2d(51.5,-6,Math.toRadians(-84)))
                .waitSeconds(0.1)
                .forward(20)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> ReadyClaw())
                .waitSeconds(0.2)
                .addTemporalMarker(() -> UpCone())
                .back(2)

                .build();


        TrajectorySequence  Cycle = drive.trajectorySequenceBuilder(ConePickup.end())


                .lineToLinearHeading(new Pose2d(52,0,Math.toRadians(0)))
                .waitSeconds(0.1)
                .lineToLinearHeading(new Pose2d(52,15.5,Math.toRadians(0)))
                .addTemporalMarker(() -> SlideTopPole())
                .forward(2.5)
                .waitSeconds(0.75)
                .addTemporalMarker(() -> ClawDrop())



                .build();
        TrajectorySequence Cyclep2 = drive.trajectorySequenceBuilder(Cycle.end())
                .back(4.75)
                .addTemporalMarker(() -> SecondCycle())

                .lineToLinearHeading(new Pose2d(58,-22 ,Math.toRadians(-86)))

                .addTemporalMarker(() -> ReadyClaw())
                .waitSeconds(0.2)
                .back(2)
                .addTemporalMarker(() -> UpCone())
                .waitSeconds(0.5)
                .back(27)
                .addTemporalMarker(() -> ResetSlide())
                .build();





        TrajectorySequence Zone2 = drive.trajectorySequenceBuilder(Cycle.end())
                .back(4.75)
                .addTemporalMarker(() -> SecondCycle())

                .lineToLinearHeading(new Pose2d(61, 24, Math.toRadians(86)))

                .addTemporalMarker(() -> ReadyClaw())
                .waitSeconds(0.2)
                .back(2)
                .addTemporalMarker(() -> UpCone())
                .waitSeconds(0.5)
                .back(27)
                .addTemporalMarker(() -> ResetSlide())
                .build();

        TrajectorySequence Zone1 = drive.trajectorySequenceBuilder(Cycle.end())
                .back(4.75)
                .addTemporalMarker(() -> SecondCycle())

                .lineToLinearHeading(new Pose2d(59, 24, Math.toRadians(86)))

                .addTemporalMarker(() -> ReadyClaw())
                .waitSeconds(0.2)
                .back(2)
                .addTemporalMarker(() -> UpCone())
                .waitSeconds(0.5)
                .back(5)
                .addTemporalMarker(() -> ResetSlide())
                .build();

        TrajectorySequence Zone3 = drive.trajectorySequenceBuilder(Cycle.end())
                .back(4.75)
                .addTemporalMarker(() -> ResetSlide())
                .strafeRight(10)
                .build();


        telemetry.addData("Hello User", "GIannIs Antelope, Online.");

        telemetry.update();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == Left || tag.id == Middle || tag.id == Right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == Left) {

            // SlideTopPole();
            telemetry.addData("Slide", Slide.getCurrentPosition());
            telemetry.update();




            drive.followTrajectorySequence(FirstCone);
            telemetry.addData("Giant Anteloupe: ", "Finished Cone One");
            telemetry.update();

            drive.followTrajectorySequence(ConePickup);

            drive.followTrajectorySequence(Cycle);
            telemetry.addData("Giant Anteloupe: ", "Finished Cone Two");
            telemetry.update();

            //drive.followTrajectorySequence(Zone1);
            telemetry.addData("Giant Anteloupe: ", "Parked");
            telemetry.update();

            //left code
        } else if (tagOfInterest == null || tagOfInterest.id == Middle) {

            // SlideTopPole();
            telemetry.addData("Slide", Slide.getCurrentPosition());
            telemetry.update();




            drive.followTrajectorySequence(FirstCone);
            telemetry.addData("Giant Anteloupe: ", "Finished Cone One");
            telemetry.update();

            drive.followTrajectorySequence(ConePickup);

            drive.followTrajectorySequence(Cycle);
            telemetry.addData("Giant Anteloupe: ", "Finished Cone Two");
            telemetry.update();

            //drive.followTrajectorySequence(Zone2);
            telemetry.addData("Giant Anteloupe: ", "Parked");
            telemetry.update();


        } else if (tagOfInterest.id == Right) {
            // SlideTopPole();
            telemetry.addData("Slide", Slide.getCurrentPosition());
            telemetry.update();




            drive.followTrajectorySequence(FirstCone);
            telemetry.addData("Giant Anteloupe: ", "Finished Cone One");
            telemetry.update();

            drive.followTrajectorySequence(ConePickup);

            drive.followTrajectorySequence(Cycle);
            telemetry.addData("Giant Anteloupe: ", "Finished Cone Two");
            telemetry.update();

           // drive.followTrajectorySequence(Zone3);
            telemetry.addData("Giant Anteloupe: ", "Parked");
            telemetry.update();

        }


    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    private void SlideTopPole(){

        Slide.setTargetPosition(-3558);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(Slide.isBusy()){
            telemetry.addData("Don't let me down","Im talking to you Dhruv" );

        }
    }

    private void ResetSlide(){

        Slide.setTargetPosition(-10);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Slide.isBusy()){
            telemetry.addData("Don't let me down","Im talking to you Dhruv" );

        }
    }
    private void FirstCycle() {
        Slide.setTargetPosition(-550);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (Slide.isBusy()) {
            telemetry.addData("Don't let me down", "Im talking to you Dhruv");

        }

    } private void SecondCycle(){
    Slide.setTargetPosition(-450);
    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    while(Slide.isBusy()){
        telemetry.addData("Don't let me down","Im talking to you Dhruv" );

    }
}
    private void UpCone(){
        Slide.setTargetPosition(-1400);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(Slide.isBusy())
            telemetry.addData("Don't let me down","Im talking to you Dhruv" );

    }


    private void ReadyClaw(){
        ClawX.setPosition(0.8);
        ClawY.setPosition(0.73);
    }
    private void ClawDrop(){
        ClawX.setPosition(0.62);
    }

    private void SetSlideConditions(){
        Slide.setTargetPosition(-1000);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(-0.75);
        telemetry.addData("Slide", Slide.getCurrentPosition());
        telemetry.update();



    }
}