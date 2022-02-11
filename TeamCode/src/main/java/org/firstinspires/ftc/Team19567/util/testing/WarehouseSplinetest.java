package org.firstinspires.ftc.Team19567.util.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Team19567.drive.SampleMecanumDrive;
import org.firstinspires.ftc.Team19567.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.Team19567.pipeline.greenPipeline;
import org.firstinspires.ftc.Team19567.pipeline.LOCATION;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.Team19567.util.AUTO_STATE;
import org.firstinspires.ftc.Team19567.util.Mechanisms;
import org.firstinspires.ftc.Team19567.util.Utility_Constants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Warehouse Spline Test", group="Testing")

public class WarehouseSplinetest extends LinearOpMode {

    private ElapsedTime timeout = new ElapsedTime();
    private greenPipeline pipeline = new greenPipeline(telemetry); //Team shipping element OpenCV Pipeline
    private DcMotor armDC = null;
    private DcMotor carouselLeft = null;
    private DcMotor carouselRight = null;
    private DcMotor intakeDC = null;
    private Servo releaseServo = null;
    private Servo balanceServo = null;
    private DistanceSensor distanceSensor = null;
    private LOCATION location = LOCATION.ALLIANCE_THIRD;
    private AUTO_STATE currentState = AUTO_STATE.DETECTING_OPENCV;
    private TrajectorySequence chosenTrajectorySequence;
    private int chosenArmPos = 600;
    private double chosenArmSpeed = 0.3;
    private double chosenTrajectoryX = -1;
    private double chosenTrajectoryY = 39;
    private int delay = 0;
    private Mechanisms mechanisms = null;

    @Override
    public void runOpMode() {
        //Get the motors from the robot's configuration

        mechanisms = new Mechanisms(hardwareMap,telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        SampleMecanumDriveCancelable chassis = new SampleMecanumDriveCancelable(hardwareMap);
        mechanisms = new Mechanisms(hardwareMap,telemetry);
        mechanisms.setModes();

        TrajectorySequence SplineSequence = chassis.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(0)))
                .addSpatialMarker(new Vector2d(10,-63), () -> {
                    mechanisms.rotateArm(Utility_Constants.THIRD_LEVEL_POS,0.7);
                })
                .lineToSplineHeading(new Pose2d(37,-57, Math.toRadians(225)))
                //.lineToSplineHeading(new Pose2d(0,0,0))
                /* .setReversed(true).splineTo(new Vector2d(10, -60),Math.toRadians(-20))
                .splineTo(new Vector2d(50,-64),Math.toRadians(0))
                .splineTo(new Vector2d(-11.5,-41),Math.toRadians(90))
                .setReversed(false).waitSeconds(0.5).splineTo(new Vector2d(50,-64),Math.toRadians(0)) */
                .build();

        TrajectorySequence returnSplineSequence = chassis.trajectorySequenceBuilder(SplineSequence.end()).lineToSplineHeading(SplineSequence.start()).build();

        waitForStart();
        if(isStopRequested()) return;

        currentState = AUTO_STATE.MOVING_TO_HUB;

        chassis.followTrajectorySequenceAsync(SplineSequence);

        master:while(opModeIsActive()) {
            switch(currentState) {
                case MOVING_TO_HUB: {
                    if(!chassis.isBusy()) {
                        timeout.reset();
                        telemetry.addData("State Machine","Moved to DELIVERING_FREIGHT");
                        telemetry.update();
                        currentState = AUTO_STATE.DELIVERING_FREIGHT;
                    }
                    break;
                }
                case DELIVERING_FREIGHT: {
                    mechanisms.releaseServoMove(0.3);
                    if(timeout.milliseconds() >= Utility_Constants.FLICKER_TIME) {
                        telemetry.addData("State Machine","Moved to MOVING_TO_WAREHOUSE");
                        telemetry.update();
                        currentState = AUTO_STATE.MOVING_TO_WAREHOUSE;
                        chassis.followTrajectorySequenceAsync(returnSplineSequence);
                    }
                    break;
                }
                case MOVING_TO_WAREHOUSE: {
                    mechanisms.rotateArm(0,0.475);
                    mechanisms.releaseServoMove(1.0);
                    if(!chassis.isBusy()) {
                        telemetry.addData("State Machine","Moved to PATH_FINISHED");
                        telemetry.update();
                        currentState = AUTO_STATE.PATH_FINISHED;
                    }
                    break;
                }
                case PATH_FINISHED: {
                    break;
                }
                default: {
                    currentState = AUTO_STATE.MOVING_TO_HUB;
                }
            }
            mechanisms.maintainBalance();
            chassis.update();
            telemetry.addData("State",currentState);
            telemetry.update();
        }

        telemetry.addData("Status", "Path Complete");
        telemetry.update();
    }
}