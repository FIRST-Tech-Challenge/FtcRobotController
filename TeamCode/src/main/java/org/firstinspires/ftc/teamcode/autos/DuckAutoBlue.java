package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.Constants.LEVEL_1;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_2;
import static org.firstinspires.ftc.teamcode.Constants.LEVEL_3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.mechanism.Color;
import org.firstinspires.ftc.teamcode.mechanism.Hopper;
import org.firstinspires.ftc.teamcode.mechanism.Intake;
import org.firstinspires.ftc.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.opencv.DuckFinder;
import org.firstinspires.ftc.teamcode.opencv.ShippingElementRecognizer;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="RoadRunner Carousel Auto Blue", group="Autonomous")
public class DuckAutoBlue extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    SampleMecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Carousel carousel = new Carousel(Color.BLUE);
        Lift lift = new Lift();
        Hopper hopper = new Hopper();
        Intake intake = new Intake();
        OpenCvWebcam webcam;
        OpenCvWebcam frontWebcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(
                        cameraMonitorViewId, //The container we're splitting
                        2, //The number of sub-containers to create
                        OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY); //Whether to split the container vertically or horizontally
        // Setup first camera
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), viewportContainerIds[0]);
        ShippingElementRecognizer pipeline = new ShippingElementRecognizer();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });

        // Second camera
        frontWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Front Webcam"), viewportContainerIds[1]);
        DuckFinder pipeline2 = new DuckFinder(78);
        frontWebcam.setPipeline(pipeline2);
        frontWebcam.setMillisecondsPermissionTimeout(2500);
        frontWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                frontWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });
        carousel.init(hardwareMap);
        lift.init(hardwareMap);
        hopper.init(hardwareMap);
        intake.init(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-36, 64, Math.toRadians(90)));

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(-36, 64, Math.toRadians(90)))
                .strafeRight(1.5)
                .back(12)
                .turn(Math.toRadians(40))
                .back(19)
                .build();
        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .forward(28)
                .turn(Math.toRadians(-130))
                .back(18)
                .build();
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory2.end())
                .forward(5)
                .strafeLeft(5.5)
                .forward(100)
                .build();

        delay(500);
        int level = 3;


        while (!isStarted() && !isStopRequested()){
            level = pipeline.getShippingHubLevel();
        }
        waitForStart();

        drive.followTrajectorySequence(trajectory1);
        if (level == 1) {
            lift.goTo(LEVEL_1, 0.8);
        } else if (level == 2) {
            lift.goTo(LEVEL_2, 0.8);
        } else if (level == 3) {
            lift.goTo(LEVEL_3, 0.8);
        } else {
            throw new IllegalStateException("Invalid shipping hub level: " + level);
        }
        delay(750);
        hopper.hopper.setPosition(0.33);
        delay(1200);
        hopper.hopper.setPosition(0);
        lift.goTo(0,0.8);
        drive.followTrajectorySequence(trajectory2);
        carousel.turnCarousel();
        delay(2500);
        drive.followTrajectorySequence(trajectory3);
        drive.turn(pipeline2.calculateYaw(0));
        intake.intakeMotor.setPower(.9);


//                        .setReversed(true)
//                .splineTo(new Vector2d(-24, 37), Math.toRadians(-45))
//                .waitSeconds(1)
//                .forward(8)
//                .turn(Math.toRadians(-140))
//                .splineTo(new Vector2d(-55, 55), Math.toRadians(135))
//                .setReversed(false)
//                .waitSeconds(1)
//                .forward(5)
//                .turn(Math.toRadians(135))
//                .forward(8)
//                .setReversed(true)
//                .splineTo(new Vector2d(-24, 37), Math.toRadians(-45))
//                .waitSeconds(1)
//                .setReversed(false)
//                .splineTo(new Vector2d(44, 64), Math.toRadians(0))
//                .build());
        //                        .waitSeconds(2)
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-55, 55), Math.toRadians(135))
//                        .waitSeconds(1)
//                        .forward(8)
//                        .turn(Math.toRadians(180))
//                        .splineTo(new Vector2d(-24, 37), Math.toRadians(-45))
//                        .setReversed(false)
//                        .waitSeconds(1)
//                        .splineTo(new Vector2d(44, 64), Math.toRadians(0))
    }
    public void delay(int time) {
        double startTime = runtime.milliseconds();
        while (runtime.milliseconds() - startTime < time && !isStopRequested()) {
            drive.update();
        }
    }
}
