package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team417_PowerPlay.AprilTagDetectionPipeline;
import org.firstinspires.ftc.team417_PowerPlay.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Autonomous (name="LEFT SIDE")
public class AutoBlueLeftSide extends LinearOpMode {
    OpenCvCamera camera; // calls camera
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    static final int CAMERA_WIDTH = 800;
    static final int CAMERA_HEIGHT = 448;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagSize = 0.166;

    // Tag ID 1, 2, 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;
    DcMotor motorArm;

    @Override
    public void runOpMode() {
        // ARM STUFF
        motorArm = hardwareMap.dcMotor.get("motorArm");
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorArm.setPower(0);

        Servo grabberServo = hardwareMap.servo.get("grabberServo");
        grabberServo.setPosition(0.8);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Trajectory traject2 = drive.trajectoryBuilder(startPose, false)
                .forward(48)
                .build();
        Trajectory traject3 = drive.trajectoryBuilder(traject2.end(), false)
                .back(21)
                .build();
        Trajectory traject4 = drive.trajectoryBuilder(traject3.end(), false)
                .strafeLeft(18.5)
                .build();
        Trajectory right = drive.trajectoryBuilder(traject4.end(), false)
                .strafeRight(45)
                .build();
        Trajectory middle = drive.trajectoryBuilder(traject4.end(), false)
                .strafeRight(21)
                .build();
        Trajectory left = drive.trajectoryBuilder(traject4.end(), false)
                .strafeLeft(12)
                .build();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) { }
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
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
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
        grabberServo.setPosition(0.4);
        while (Math.abs(motorArm.getCurrentPosition() - -200) > 10) {
            motorArm.setPower((-200 - motorArm.getCurrentPosition()) / 400.0);
        }
        motorArm.setPower(-0.01);

        drive.followTrajectory(traject2);
        drive.followTrajectory(traject3);

        motorArm.setPower(0);
        while (Math.abs(motorArm.getCurrentPosition() - -900) > 10) {
            motorArm.setPower((-900 - motorArm.getCurrentPosition()) / 1000.0);
        }
        motorArm.setPower(-0.005);
        drive.followTrajectory(traject4);
        motorArm.setPower(0);
        sleep(20);
        grabberServo.setPosition(0.8);


        /* Actually do something useful */
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            drive.followTrajectory(left);
        } else if (tagOfInterest.id == MIDDLE) {
            drive.followTrajectory(middle);
        } else {
            drive.followTrajectory(right);
        }

        // You wouldn't have this in your autonomous, this is just to prevent the sample from ending
        while (opModeIsActive()) {sleep(20);}
        // When we implement the rest of autonomous we can delete line the while loop
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}