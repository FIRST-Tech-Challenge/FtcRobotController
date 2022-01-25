package org.firstinspires.ftc.Team19567;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Team19567.drive.SampleMecanumDrive;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Blue Warehouse", group="Dababy")

public class blueWarehouse extends LinearOpMode {

    private ElapsedTime timeout = new ElapsedTime();
    private tsePipeline pipeline = new tsePipeline(telemetry); //Team shipping element OpenCV Pipeline
    private DcMotor armDC = null;
    private DcMotor carouselLeft = null;
    private DcMotor carouselRight = null;
    private DcMotor intakeDC = null;
    private Servo releaseServo = null;
    private Servo balanceServo = null;
    private tsePipeline.LOCATION location = tsePipeline.LOCATION.ALLIANCE_THIRD;
    private TrajectorySequence chosenTrajectorySequence;
    private int chosenArmPos = 600;
    private double chosenArmSpeed = 0.3;
    private double chosenTrajectoryX = -1;
    private double chosenTrajectoryY = 39;
    private Mechanisms mechanisms = null;

    @Override
    public void runOpMode() {
        //Get the motors from the robot's configuration

        armDC = hardwareMap.get(DcMotor.class, "armDC");
        carouselLeft = hardwareMap.get(DcMotor.class, "carouselLeft");
        carouselRight = hardwareMap.get(DcMotor.class, "carouselRight");
        releaseServo = hardwareMap.get(Servo.class, "releaseServo");
        balanceServo = hardwareMap.get(Servo.class, "balanceServo");
        mechanisms = new Mechanisms(armDC,carouselLeft,carouselRight,intakeDC,balanceServo,releaseServo,telemetry);

        armDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDC.setDirection(DcMotor.Direction.REVERSE);
        balanceServo.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        SampleMecanumDrive chassis = new SampleMecanumDrive(hardwareMap);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline);
                camera.startStreaming(544,288, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("OpenCV","OpenCV actually connected wow");
                telemetry.update();
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("OpenCV","OpenCV failed to load :( Error Code: " + errorCode);
                telemetry.update();
            }
        });

        TrajectorySequence firstLevelSequence = chassis.trajectorySequenceBuilder(new Pose2d(6, 63,Math.toRadians(270)))
                .strafeTo(new Vector2d(2,50)).turn(Math.toRadians(60))
                .build();
        TrajectorySequence secondLevelSequence = chassis.trajectorySequenceBuilder(new Pose2d(6,63, Math.toRadians(270)))
                .strafeTo(new Vector2d(0,45)).turn(Math.toRadians(60))
                .build();
        TrajectorySequence thirdLevelSequence = chassis.trajectorySequenceBuilder(new Pose2d(6, 63, Math.toRadians(270)))
                .strafeTo(new Vector2d(2,50)).turn(Math.toRadians(60)).strafeTo(new Vector2d(-1,39)).build();

        while(!opModeIsActive()) {
            location = pipeline.getLocation();
            telemetry.addData("location",location);
            telemetry.update();
        }

        waitForStart();

        if(!opModeIsActive()) return;

        switch(location) {
            case ALLIANCE_FIRST: {
                chosenTrajectorySequence = thirdLevelSequence;
                chosenTrajectoryX = -1;
                chosenTrajectoryY = -39;
                chosenArmPos = 600;
                chosenArmSpeed = 0.3;
                telemetry.addData("OpenCV","First Level Detected");
                telemetry.update();
                break;
            }
            case ALLIANCE_SECOND: {
                chosenTrajectorySequence = secondLevelSequence;
                chosenTrajectoryX = 0;
                chosenTrajectoryY = -45;
                chosenArmPos = 600;
                chosenArmSpeed = 0.3;
                telemetry.addData("OpenCV","Second Level Detected");
                telemetry.update();
                break;
            }
            case ALLIANCE_THIRD: {
                chosenTrajectorySequence = firstLevelSequence;
                chosenTrajectoryX = 2;
                chosenTrajectoryY = -50;
                chosenArmPos = 750;
                chosenArmSpeed = 0.2;
                telemetry.addData("OpenCV","Third Level Detected");
                telemetry.update();
                break;
            }
            default: {
                location = tsePipeline.LOCATION.ALLIANCE_THIRD;
            }
        }

        telemetry.addData("Freight Level", "3");
        telemetry.update();

        TrajectorySequence secondTrajectory = chassis.trajectorySequenceBuilder(new Pose2d(chosenTrajectoryX, chosenTrajectoryY, Math.toRadians(330)))
                .strafeTo(new Vector2d(6, -27))
                .strafeTo(new Vector2d(-40,-27)).build();

        mechanisms.rotateArm(0);
        mechanisms.releaseServoMove(1.0);
        chassis.followTrajectorySequence(chosenTrajectorySequence);
        mechanisms.rotateArm(chosenArmPos,chosenArmSpeed);
        while(armDC.getCurrentPosition() <= chosenArmPos && opModeIsActive()) {
            mechanisms.maintainBalance();
        }
        sleep(500);
        mechanisms.releaseServoMove(0.25);
        sleep(1500);
        mechanisms.rotateArm(0,0.1);
        mechanisms.releaseServoMove(1.0);
        chassis.followTrajectorySequence(secondTrajectory);

        telemetry.addData("Status", "Path Complete");
        telemetry.update();
    }
}