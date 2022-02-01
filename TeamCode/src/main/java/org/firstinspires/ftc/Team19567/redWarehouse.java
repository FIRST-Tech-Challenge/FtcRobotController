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

@Autonomous(name="Red Warehouse", group="Dababy")

public class redWarehouse extends LinearOpMode {

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
    private double chosenTrajectoryX = -1;
    private double chosenTrajectoryY = -39;
    private int chosenArmPos = 650;
    private double chosenArmSpeed = 0.3;

    @Override
    public void runOpMode() {
        //Get the motors from the robot's configuration
        //

        armDC = hardwareMap.get(DcMotor.class, "armDC");
        carouselLeft = hardwareMap.get(DcMotor.class, "carouselLeft");
        carouselRight = hardwareMap.get(DcMotor.class, "carouselRight");
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

        TrajectorySequence firstLevelSequence = chassis.trajectorySequenceBuilder(new Pose2d(6, -63,Math.toRadians(270)))
                .strafeTo(new Vector2d(0,-43)).turn(Math.toRadians(135))
                .build();
        TrajectorySequence secondLevelSequence = chassis.trajectorySequenceBuilder(new Pose2d(6,-63, Math.toRadians(270)))
                .strafeTo(new Vector2d(-1,-39)).turn(Math.toRadians(135))
                .build();
        TrajectorySequence thirdLevelSequence = chassis.trajectorySequenceBuilder(new Pose2d(6, -63, Math.toRadians(270)))
                .strafeTo(new Vector2d(-5,-35)).turn(Math.toRadians(135)).build();

        while(!opModeIsActive()) {
            location = pipeline.getLocation();
            telemetry.addData("location",location);
            telemetry.update();
        }

        waitForStart();

        if(!opModeIsActive()) return;

        switch(location) {
            case ALLIANCE_FIRST: {
                chosenTrajectorySequence = secondLevelSequence;
                chosenArmPos = 770;
                chosenArmSpeed = 0.15;
                chosenTrajectoryX = -1;
                chosenTrajectoryY = -39;
                telemetry.addData("OpenCV","First Level Detected");
                telemetry.update();
                break;
            }
            case ALLIANCE_SECOND: {
                chosenTrajectorySequence = firstLevelSequence;
                chosenTrajectoryX = 0;
                chosenTrajectoryY = -39;
                chosenArmPos = 880;
                chosenArmSpeed = 0.1;
                telemetry.addData("OpenCV","Second Level Detected");
                telemetry.update();
                break;
            }
            case NO_ALLIANCE: {
                chosenTrajectorySequence = thirdLevelSequence;
                chosenTrajectoryX = -5;
                chosenTrajectoryY = -43;
                chosenArmPos = 650;
                chosenArmSpeed = 0.3;
                telemetry.addData("OpenCV","Third Level Detected");
                telemetry.update();
                break;
            }
            default: {
                location = LOCATION.ALLIANCE_THIRD;
            }
        }

        TrajectorySequence secondTrajectory = chassis.trajectorySequenceBuilder(new Pose2d(chosenTrajectoryX, chosenTrajectoryY, Math.toRadians(45)))
                .turn(Math.toRadians(-50))
                .strafeTo(new Vector2d(chosenTrajectoryX, -5))
                .strafeTo(new Vector2d(-40+(chosenTrajectoryX),-5)).build();

        mechanisms.rotateArm(0);
        mechanisms.balanceServoMove(0.0);
        mechanisms.releaseServoMove(1.0);
        chassis.followTrajectorySequence(chosenTrajectorySequence);
        mechanisms.rotateArm(chosenArmPos,chosenArmSpeed);
        while(armDC.getCurrentPosition() <= chosenArmPos && opModeIsActive()) {
            mechanisms.maintainBalance();
        }
        sleep(1000);
        mechanisms.releaseServoMove(0.2);
        sleep(1500);
        mechanisms.balanceServoMove(0.0);
        mechanisms.rotateArm(0,0.1);
        mechanisms.releaseServoMove(1.0);
        chassis.followTrajectorySequence(secondTrajectory);

        telemetry.addData("Status", "Path Complete");
        telemetry.update();
    }
}