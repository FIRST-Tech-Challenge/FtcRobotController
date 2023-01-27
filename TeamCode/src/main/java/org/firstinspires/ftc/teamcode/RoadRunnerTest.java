package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Locale;

@Autonomous(name = "RoadRunnerTest", group = "")
public class RoadRunnerTest extends LinearOpMode {
    //test1

    private SampleMecanumDrive drive;
    private Servo gripper = null; //Located on Expansion Hub- Servo port 0
    private DcMotor arm = null;

    static final float MAX_SPEED = 1.0f;
    static final float MIN_SPEED = 0.4f;
    static final int ACCEL = 75;  // Scaling factor used in accel / decel code.  Was 100!
    public double desiredHeading;

    Orientation angles;
    Acceleration gravity;
    private OpenCvCamera webCam;
    private boolean isCameraStreaming = false;
    Pipeline modifyPipeline = new Pipeline();

    private int resultROI = 2;

    private boolean done = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        arm = hardwareMap.get(DcMotor.class, "arm");
        gripper = hardwareMap.get(Servo.class, "gripper");
        Pose2d startPose = new Pose2d(-63, 40.5, 0);
        drive.setPoseEstimate(startPose);
        //TrajectorySequence aSeq = autoSeq(startPose);

        //Reverse the arm direction so it moves in the proper direction
        arm.setDirection(DcMotor.Direction.REVERSE);

        desiredHeading = getHeading();

        actuatorUtils.initializeActuator(arm, gripper);

        Long startTime = System.currentTimeMillis();
        Long currTime = startTime;

        initOpenCV();

        actuatorUtils.gripperClose(false);

        waitForStart();
        currTime = System.currentTimeMillis();
        startTime = currTime;
        if (resultROI == 2) {

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            done = false;
            while (!done && opModeIsActive()) {
                //if (currTime - startTime < 500) {
                //    telemetry.addData("Camera: ", "Waiting to make sure valid data is incoming");
                //} else {
                    telemetry.addData("Time Delta: ", (currTime - startTime));
                    resultROI = modifyPipeline.getResultROI();
                    if (resultROI == 1) {
                        telemetry.addData("Resulting ROI: ", "Red");
                        done = true;
                    } else if (resultROI == 2) {
                        telemetry.addData("Resulting ROI: ", "Green");
                        done = true;
                    } else if (resultROI == 3) {
                        telemetry.addData("Resulting ROI: ", "Blue");
                        done = true;
                    } else {
                        telemetry.addData("Resulting ROI: ", "Something went wrong.");
                    }
                //}
                telemetry.update();
                currTime = System.currentTimeMillis();

            }

        }
        telemetry.update();
        done = false;

        //lift arm up
        actuatorUtils.armPole(4, false);
        while (((currTime - startTime) < 30000) && !done && opModeIsActive()) {
            autoSeq();
            telemetry.addData("IM at ", getHeading());
            telemetry.update();
//            sleep(5000);
            parkSeq(resultROI);
            while (arm.getCurrentPosition() > -20) {
                telemetry.addData("ARM Position = ", arm.getCurrentPosition());
                telemetry.update();
                arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm.setPower(-0.7);
            }
            arm.setPower(0);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("ARM Position = ", arm.getCurrentPosition());
            telemetry.update();
            currTime = System.currentTimeMillis();
            done = true;
        }
    }

    private void autoSeq() {
        TrajectorySequence seq1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-60, 13, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-35,13,Math.toRadians(-45)))
                .build();
        try {
            actuatorUtils.armPole(3,false);
        } catch (InterruptedException e) {
            telemetry.addData(e.getMessage(), "");
            telemetry.update();
        }
        drive.followTrajectorySequence(seq1);
        //sleep(1000);
        TrajectorySequence seq2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-27.5, 5.5, Math.toRadians(-45)))
                .build();
        drive.followTrajectorySequence(seq2);
        TrajectorySequence seq3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-35,13, Math.toRadians(0)))
                .build();
        try {
            actuatorUtils.gripperOpen(false);
        } catch (InterruptedException e) {
            telemetry.addData(e.getMessage(), "");
            telemetry.update();
        }
        drive.followTrajectorySequence(seq3);
        try {
            actuatorUtils.armPole(4,false);
        } catch (InterruptedException e) {
            telemetry.addData(e.getMessage(), "");
            telemetry.update();
        }
        TrajectorySequence seq4 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-13,13, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-13, 65, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(seq4);
        for (int i = 0; i < 2 ; i++) {
            try {
                actuatorUtils.gripperClose(true);
                actuatorUtils.armPole(1, false);
            } catch (InterruptedException e) {
                telemetry.addData(e.getMessage(), "");
                telemetry.update();
            }
            TrajectorySequence seq5 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-13, 52, Math.toRadians(90)))
                    .lineToLinearHeading(new Pose2d(-13, 49, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-17, 49, Math.toRadians(180)))
                    //.lineToLinearHeading(new Pose2d(-18, 50, Math.toRadians(180)))
                    //.lineToLinearHeading(new Pose2d(-12, 64.5, Math.toRadians(90)))
                    .build();
            drive.followTrajectorySequence(seq5);
            try {
                actuatorUtils.gripperOpen(false);
                actuatorUtils.armPole(4, false);
            } catch (InterruptedException e) {
                telemetry.addData(e.getMessage(), "");
                telemetry.update();
            }
            TrajectorySequence seq6 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-13, 49, Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(-13, 65, Math.toRadians(90)))
                    //.turn(Math.toRadians(-90))
                    .build();
            drive.followTrajectorySequence(seq6);
        }
    }
    private void parkSeq(int park) {
        Pose2d pose = drive.getPoseEstimate();
        if (park == 1) {
            return;
        } else if (park == 2) {
            pose = new Pose2d(-12, 36, Math.toRadians(90));
        } else {
            pose = new Pose2d(-12, 12, Math.toRadians(90));
        }
        TrajectorySequence seq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(pose)
                    .build();
        drive.followTrajectorySequence(seq);
    }
    private void initOpenCV() {
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        // For a webcam (uncomment below)
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId2);
        // For a phone camera (uncomment below)
        // webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId2);
        webCam.setPipeline(modifyPipeline);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Pipeline: ", "Initialized");
                telemetry.update();
                isCameraStreaming = true;
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", "Something went wrong :(");
                telemetry.update();
            }
        });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getHeading() {
        double angle = drive.getRawExternalHeading();
        return angle;
    }
    private float convertRad(int input) {
        float x;
        x=input/180f;
        x*=Math.PI;
        x*=(1);
        return x;
    }
}


