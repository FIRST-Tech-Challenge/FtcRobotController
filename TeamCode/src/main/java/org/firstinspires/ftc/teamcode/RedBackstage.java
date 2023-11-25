package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Vision.OpenCVRed;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.OpenCVBlue;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "RedBackstage")
public class RedBackstage extends LinearOpMode {

    OpenCvWebcam webcam;

    OpenCVRed pipeline = new OpenCVRed(telemetry);

    float test = 1;

    private DcMotor Arm, Extend = null;

    private double ArmPower = 0.5, SlidePower = 0.4;

    @Override
    public void runOpMode() {
        Arm = hardwareMap.get(DcMotor.class, "AE");
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        Extend = hardwareMap.get(DcMotor.class, "SE");
        Extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        telemetry.addLine("Wait until debug");
        telemetry.update();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        //Left
        TrajectorySequence Right = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(25, -12, Math.toRadians(0)))
                .back(8)
                .turn(Math.toRadians(-90))
                .forward(25)
                .addTemporalMarker(() -> {
                    // arm up
                    Arm.setTargetPosition(700);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // slides out
                    Extend.setTargetPosition(1094);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // arm down
                    Arm.setTargetPosition(590);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(2)

                .back(8)

                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // slides in
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // arm down
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(2)
                .build();


        //middle
        TrajectorySequence Middle = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(31, 0, Math.toRadians(0)))
                .back(8)
                .turn(Math.toRadians(-90))
                .forward(38)
                .addTemporalMarker(() -> {
                    // arm up
                    Arm.setTargetPosition(700);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // slides out
                    Extend.setTargetPosition(1094);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // arm down
                    Arm.setTargetPosition(590);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(2)

                .back(8)

                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // slides in
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // arm down
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(2)
                .build();


        //right
        TrajectorySequence Left = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(24, 0, Math.toRadians(0)))
                .back(4)
                .strafeLeft(14)
                .turn(Math.toRadians(-90))
                .strafeRight(6)
                .forward(18)

                .turn(Math.toRadians(190))
                .forward(34)
                .strafeLeft(6)
                .addTemporalMarker(() -> {
                    // arm up
                    Arm.setTargetPosition(700);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // slides out
                    Extend.setTargetPosition(1094);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // arm down
                    Arm.setTargetPosition(590);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(2)

                .back(8)

                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // slides in
                    Extend.setTargetPosition(0);
                    Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Extend.setPower(SlidePower);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // arm down
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(ArmPower);
                })
                .waitSeconds(2)
                .build();



        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        String result = pipeline.getResult();
        webcam.stopStreaming();
        webcam.closeCameraDevice();

        if (result == "LEFT") {
            drive.followTrajectorySequence(Left);
        }
        if (result == "MIDDLE") {
            drive.followTrajectorySequence(Middle);
        }
        if (result == "RIGHT") {
            drive.followTrajectorySequence(Right);
        }


        while (opModeIsActive()) {

            telemetry.update();
            sleep(100);
        }
    }
}
