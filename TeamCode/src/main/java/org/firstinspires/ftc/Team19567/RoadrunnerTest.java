package org.firstinspires.ftc.Team19567;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Team19567.drive.SampleMecanumDrive;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.Team19567.tsePipeline.LOCATION;

@Autonomous(name="Roadrunner Test", group="Dababy")

public class RoadrunnerTest extends LinearOpMode {

    private ElapsedTime timeout = new ElapsedTime();
    private tsePipeline pipeline = new tsePipeline(telemetry); //Team shipping element OpenCV Pipeline
    private DcMotor armDC = null;
    private DcMotor carouselLeft = null;
    private DcMotor carouselRight = null;
    private DcMotor intakeDC = null;
    private Servo releaseServo = null;
    private Servo balanceServo = null;
    private LOCATION location = LOCATION.ALLIANCE_THIRD;
    private Mechanisms mechanisms = null;

    @Override
    public void runOpMode() throws InterruptedException {
        armDC = hardwareMap.get(DcMotor.class, "armDC");
        carouselLeft = hardwareMap.get(DcMotor.class, "carouselLeft");
        carouselRight = hardwareMap.get(DcMotor.class,"carouselRight");
        intakeDC = hardwareMap.get(DcMotor.class,"intakeDC");
        releaseServo = hardwareMap.get(Servo.class, "releaseServo");
        balanceServo = hardwareMap.get(Servo.class, "balanceServo");

        mechanisms = new Mechanisms(hardwareMap,telemetry);

        SampleMecanumDrive chassis = new SampleMecanumDrive(hardwareMap);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline);
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("OpenCV","OpenCV actually connected wow");
                telemetry.update();
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("OpenCV","OpenCV failed to load :( Error Code: " + errorCode);
                telemetry.update();
            }
        });

        TrajectorySequence mainSequence = chassis.trajectorySequenceBuilder(new Pose2d(6, -63, Math.toRadians(270)))
                .addTemporalMarker(8000,() -> { mechanisms.moveIntake(1.0); })
                .lineTo(new Vector2d(6, 12)).turn(Math.toRadians(90))
                //.lineToSplineHeading(new Pose2d(6,-24,0))
                .addDisplacementMarker(() -> {
                    //timeout.reset();
                    mechanisms.rotateArm(600,0.2);
                    while(mechanisms.armDC.getCurrentPosition() <= 600 && opModeIsActive()/*|| timeout.milliseconds() <= 3000 */) {
                        mechanisms.maintainBalance();
                    }
                }).waitSeconds(1.5)
                .addDisplacementMarker(() -> { mechanisms.releaseServoMove(0.6); }).waitSeconds(1)
                .addDisplacementMarker(() -> { mechanisms.reset(); }).splineTo(new Vector2d(10, -55), Math.toRadians(270))
                .splineTo(new Vector2d(36, -64),0).strafeTo(new Vector2d(47, -64)).waitSeconds(3)
                .addDisplacementMarker(() -> { mechanisms.reset(); }).strafeTo(new Vector2d(15,-64))
                .lineToSplineHeading(new Pose2d(-11.5,-41,Math.toRadians(-90))).waitSeconds(3)
                .splineTo(new Vector2d(45,-64),0).waitSeconds(5)
                .build();

        armDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDC.setDirection(DcMotorSimple.Direction.REVERSE);
        balanceServo.setDirection(Servo.Direction.REVERSE);

        while(!opModeIsActive()) {
            location = pipeline.getLocation();
        }

        waitForStart();

        if(!opModeIsActive()) return;

        switch(location) {
            case ALLIANCE_FIRST: {
                location = LOCATION.ALLIANCE_FIRST;
                telemetry.addData("OpenCV","First Level Detected");
                telemetry.update();
                break;
            }
            case ALLIANCE_SECOND: {
                location = LOCATION.ALLIANCE_SECOND;
                telemetry.addData("OpenCV","Second Level Detected");
                telemetry.update();
                break;
            }
            case ALLIANCE_THIRD: {
                location = LOCATION.ALLIANCE_THIRD;
                telemetry.addData("OpenCV","Third Level Detected");
                telemetry.update();
                break;
            }
            default: {
                location = LOCATION.ALLIANCE_THIRD;
            }
        }

        chassis.followTrajectorySequence(mainSequence);

    }
}