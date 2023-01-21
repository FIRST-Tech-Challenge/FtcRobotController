package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class AutonomousRight extends LinearOpMode {
    // the camera
    private OpenCvWebcam webcam;


    // we save the finishing angle for the field oriented after this op mode
    public static double lastAngle;
    // when turning off the op mode the imu turns off and his last value is 0, therefor we need to save the value before that (create a delay)
    public static double delayMaker;

    SampleMecanumDrive drive;
    @Override
    public void runOpMode() {

        // initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);
        webcam.setPipeline(new PipeLine(true));

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        // initialize the driveController (we do that after initializing the camera in order to enable "camera stream" in the drive controller)
        RobotController robotController = new RobotController(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-30.7, 61.4, Math.toRadians(-90));
        Pose2d scoringPose = new Pose2d(-37, 2.25, Math.toRadians(-15));

        Pose2d parking1 = new Pose2d(-61  , 33, Math.toRadians(-90));
        Pose2d parking2_1 = new Pose2d(-36, 20, Math.toRadians(-90));
        Pose2d parking2_2 = new Pose2d(-36, 33, Math.toRadians(-90));
        Pose2d parking3 = new Pose2d(-13.8, 33, Math.toRadians(-90));

        TrajectorySequence startToScore = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-35, 58), Math.toRadians(-95))
                .splineToSplineHeading(new Pose2d(-37.5, 20, Math.toRadians(-75)), Math.toRadians(-90))
                .splineToSplineHeading(scoringPose, Math.toRadians(-90)).build();

        TrajectorySequence scoringToParking1 = drive.trajectorySequenceBuilder(parking2_2).lineToLinearHeading(parking1).build();
        TrajectorySequence scoringToParking2 = drive.trajectorySequenceBuilder(scoringPose).setTangent(Math.toRadians(90)).splineToSplineHeading(parking2_1, Math.toRadians(90)).splineToSplineHeading(parking2_2, Math.toRadians(90)).build();
        TrajectorySequence scoringToParking3 = drive.trajectorySequenceBuilder(parking2_2).lineToLinearHeading(parking3).build();

        drive.setPoseEstimate(startPose);

        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();
        try{
            new Thread(() -> {
                while (opModeIsActive()){
                    gamepad.a = gamepad1.a;
                    gamepad.b = gamepad1.b;
                    gamepad.y = gamepad1.y;
                    gamepad.x = gamepad1.x;

                    gamepad.left_trigger = gamepad1.left_trigger;
                    gamepad.right_trigger = gamepad1.right_trigger;

                    gamepad.left_bumper = gamepad1.left_bumper;
                    gamepad.right_bumper = gamepad1.right_bumper;

                    gamepad.left_stick_y = gamepad1.left_stick_y;
                    gamepad.left_stick_x = gamepad1.left_stick_x;
                    gamepad.right_stick_x = gamepad1.right_stick_x;

                    lastAngle = delayMaker;
                    delayMaker = robotController.getRobotAngle();

                }

                robotController.terminate();
                telemetry.clearAll();
                telemetry.addLine("stop");
                telemetry.update();
            });

            webcam.closeCameraDevice();

            follow(startToScore);
            robotController.elevatorController.start();
            robotController.autoCycle.start();

            while(robotController.autoCycle.isAlive()){
                if (!opModeIsActive()) throw new InterruptedException("stop requested");
            }


            robotController.safeSleep(500);

            robotController.elevatorController.interrupt();

            switch (PipeLine.parkingPosition) {
                case 0: {
                    follow(scoringToParking2);
                    follow(scoringToParking3);
                    break;
                }
                case 1: {
                    follow(scoringToParking2);
                    break;
                }
                case 2: {
                    follow(scoringToParking2);
                    follow(scoringToParking1);
                    break;
                }
            }
            telemetry.addData("time", getRuntime());
            telemetry.update();

            while (opModeIsActive()){}
        } catch (InterruptedException e){}

        robotController.terminate();
        telemetry.clearAll();
        telemetry.addLine("stop");
        telemetry.update();
    }

    private void follow(TrajectorySequence ts) throws InterruptedException{
        drive.followTrajectorySequenceAsync(ts);
        while (drive.isBusy()){
            drive.update();
            if (!opModeIsActive()) throw new InterruptedException("stop requested");
        }
    }
}