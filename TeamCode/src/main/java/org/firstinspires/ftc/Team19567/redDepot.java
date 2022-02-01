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

@Autonomous(name="Red Depot", group="Dababy")

public class redDepot extends LinearOpMode {

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
    private TrajectorySequence chosenTrajectorySequence;
    private int chosenArmPos = 650;
    private double chosenArmSpeed = 0.3;
    private double chosenTrajectoryX = -20;

    @Override
    public void runOpMode() {
        //Get the motors from the robot's configuration
        //

        armDC = hardwareMap.get(DcMotor.class, "armDC");
        carouselLeft = hardwareMap.get(DcMotor.class, "carouselLeft");
        carouselRight = hardwareMap.get(DcMotor.class, "carouselRight");
        intakeDC = hardwareMap.get(DcMotor.class,"intakeDC");
        releaseServo = hardwareMap.get(Servo.class, "releaseServo");
        balanceServo = hardwareMap.get(Servo.class, "balanceServo");
        mechanisms = new Mechanisms(hardwareMap,telemetry);

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

        TrajectorySequence firstLevelSequence = chassis.trajectorySequenceBuilder(new Pose2d(-20, -63, Math.toRadians(270)))
                .strafeTo(new Vector2d(-32,-23)).turn(Math.toRadians(-96)).build();

        TrajectorySequence secondLevelSequence = chassis.trajectorySequenceBuilder(new Pose2d(-20, -63, Math.toRadians(270)))
                .strafeTo(new Vector2d(-30,-23)).turn(Math.toRadians(-96)).build();

        TrajectorySequence thirdLevelSequence = chassis.trajectorySequenceBuilder(new Pose2d(-20, -63, Math.toRadians(270)))
                .strafeTo(new Vector2d(-20,-23)).turn(Math.toRadians(-96)).build();

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
                chosenArmPos = 600;
                chosenArmSpeed = 0.25;
                chosenTrajectoryX = -20;
                telemetry.addData("OpenCV","First Level Detected");
                telemetry.update();
                break;
            }
            case NO_ALLIANCE: {
                chosenTrajectorySequence = thirdLevelSequence;
                chosenArmPos = 650;
                chosenArmSpeed = 0.25;
                chosenTrajectoryX = -20;
                telemetry.addData("OpenCV","Basically Third Level");
                telemetry.update();
                break;
            }
            case ALLIANCE_SECOND: {
                chosenTrajectorySequence = secondLevelSequence;
                chosenArmPos = 770;
                chosenArmSpeed = 0.1;
                chosenTrajectoryX = -30;
                telemetry.addData("OpenCV","Second Level Detected");
                telemetry.update();
                break;
            }
            case ALLIANCE_THIRD: {
                chosenTrajectorySequence = firstLevelSequence;
                chosenArmPos = 880;
                chosenArmSpeed = 0.1;
                chosenTrajectoryX = -32;
                telemetry.addData("OpenCV","Third Level Detected");
                telemetry.update();
                break;
            }
            default: {
                location = LOCATION.ALLIANCE_THIRD;
            }
        }

        TrajectorySequence secondTrajectory = chassis.trajectorySequenceBuilder(new Pose2d(chosenTrajectoryX-15.2, -23, Math.toRadians(174)))
                .strafeTo(new Vector2d(-15+(34+chosenTrajectoryX),0)).turn(Math.toRadians(180)).strafeTo(new Vector2d(-8+(34+chosenTrajectoryX),16)).build();
        //TrajectorySequence secondTrajectory = chassis.trajectorySequenceBuilder(new Pose2d(chosenTrajectoryX-15.2, -23, Math.toRadians(174)))
          //  .strafeTo(new Vector2d(-5,0)).turn(Math.toRadians(180)).strafeTo(new Vector2d(-15,19)).build();
        TrajectorySequence thirdTrajectory = chassis.trajectorySequenceBuilder(new Pose2d(-8, 16, Math.toRadians(354)))
                .strafeTo(new Vector2d(-3,-12)).build();

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
        mechanisms.balanceServoMove(0.0);
        chassis.followTrajectorySequence(secondTrajectory);
        mechanisms.rotateCarousel(0.5);
        sleep(3000);
        mechanisms.rotateCarousel(0.0);
        chassis.followTrajectorySequence(thirdTrajectory);

        telemetry.addData("Status", "Path Complete");
        telemetry.update();
    }
}