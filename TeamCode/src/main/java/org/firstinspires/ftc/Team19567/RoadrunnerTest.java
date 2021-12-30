package org.firstinspires.ftc.Team19567;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous(name="Roadrunner Test", group="Linear Opmode")

public class RoadrunnerTest extends LinearOpMode {

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
        armDC = hardwareMap.get(DcMotor.class, "linearSlideDC");
        carouselLeft = hardwareMap.get(DcMotor.class, "carouselLeft");
        carouselRight = hardwareMap.get(DcMotor.class,"carouselRight");
        intakeDC = hardwareMap.get(DcMotor.class,"intakeDC");
        releaseServo = hardwareMap.get(Servo.class, "releaseServo");
        balanceServo = hardwareMap.get(Servo.class, "intakeServo");

        mechanisms = new Mechanisms(armDC,carouselLeft,carouselRight,intakeDC,balanceServo,releaseServo);

        SampleMecanumDrive chassis = new SampleMecanumDrive(hardwareMap);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline);
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("OpenCV","OpenCV actually connected wow");
                telemetry.update();

                waitForStart();

            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("OpenCV","OpenCV failed to load :( Error Code: " + errorCode);
                telemetry.update();
            }
        });

        TrajectorySequence testTrajectory = chassis.trajectorySequenceBuilder(new Pose2d(6, -63, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(7,-24,0))
                .lineTo(new Vector2d(12,-64)).strafeTo(new Vector2d(47,-64))
                .strafeTo(new Vector2d(15,-64)).lineToSplineHeading(new Pose2d(-11.5,-41,Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(12,-64,0)).strafeTo(new Vector2d(45,-64))
                .build();

        while(!opModeIsActive()) {
            location = pipeline.getLocation();
        }

        waitForStart();

        if(!opModeIsActive()) return;

        switch(pipeline.getLocation()) {
            case ALLIANCE_FIRST: {
                location = LOCATION.ALLIANCE_FIRST;
                telemetry.addData("OpenCV","First Level Detected");
                telemetry.update();
            }
            case ALLIANCE_SECOND: {
                location = LOCATION.ALLIANCE_SECOND;
                telemetry.addData("OpenCV","Second Level Detected");
                telemetry.update();
            }
            case ALLIANCE_THIRD: {
                location = LOCATION.ALLIANCE_THIRD;
                telemetry.addData("OpenCV","Third Level Detected");
                telemetry.update();
            }
            default: {
                location = LOCATION.ALLIANCE_THIRD;
            }
        }

        chassis.followTrajectorySequence(testTrajectory);

    }
}