package org.firstinspires.ftc.Team19567.opmode;

import android.text.method.Touch;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Team19567.drive.SampleMecanumDrive;
import org.firstinspires.ftc.Team19567.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.Team19567.pipeline.greenPipeline;
import org.firstinspires.ftc.Team19567.pipeline.LOCATION;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.Team19567.util.AUTO_STATE;
import org.firstinspires.ftc.Team19567.util.Mechanisms;
import org.firstinspires.ftc.Team19567.util.Utility_Constants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Red Warehouse FSM", group="Testing")

public class RedWarehouseFSM extends LinearOpMode {

    private ElapsedTime timeout = new ElapsedTime();
    private greenPipeline pipeline = new greenPipeline(telemetry); //Team shipping element OpenCV Pipeline
    private DcMotor armDC = null;
    private DcMotor carouselLeft = null;
    private DcMotor carouselRight = null;
    private DcMotor intakeDC = null;
    private Servo releaseServo = null;
    private Servo balanceServo = null;
    private DistanceSensor distanceSensor = null;
    private TouchSensor limitSwitch = null;
    private LOCATION location = LOCATION.ALLIANCE_THIRD;
    private AUTO_STATE currentState = AUTO_STATE.DETECTING_OPENCV;
    private TrajectorySequence chosenTrajectorySequence;
    private int chosenArmPos = 600;
    private double chosenArmSpeed = 0.3;
    private double chosenTrajectoryX = -1;
    private double chosenTrajectoryY = 39;
    private int delay = 0;
    private int freightCount = 0;
    private Mechanisms mechanisms = null;

    @Override
    public void runOpMode() {
        //Get the motors from the robot's configuration

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        SampleMecanumDriveCancelable chassis = new SampleMecanumDriveCancelable(hardwareMap);

        mechanisms = new Mechanisms(hardwareMap,telemetry);
        mechanisms.setModes();

        limitSwitch = hardwareMap.get(TouchSensor.class,"limitSwitch");

        chassis.setPoseEstimate(new Pose2d(10, -63, Math.toRadians(90)));

        TrajectorySequence SplineSequence = chassis.trajectorySequenceBuilder(new Pose2d(10,-63,Math.toRadians(90)))
                .addSpatialMarker(new Vector2d(5,-50), () -> {
                    mechanisms.rotateArm(Utility_Constants.THIRD_LEVEL_POS,Utility_Constants.THIRD_LEVEL_POWER);
                }).lineToSplineHeading(new Pose2d(0,-35,Math.toRadians(-45)))
                .build();

        TrajectorySequence returnSplineSequence = chassis.trajectorySequenceBuilder(SplineSequence.end()).addSpatialMarker(new Vector2d(30,-64), () -> {
            mechanisms.moveIntake(1.0);
        }).addSpatialMarker(new Vector2d(10, -50),() -> {mechanisms.releaseServoMove(1.0);}).lineToSplineHeading(new Pose2d(12,-66.5,0)).strafeTo(new Vector2d(54, -66.5)).build();

        TrajectorySequence intakingSequence = chassis.trajectorySequenceBuilder(returnSplineSequence.end()).back(10).forward(10).build();

        TrajectorySequence faiyoiSequence = chassis.trajectorySequenceBuilder(intakingSequence.end()).addSpatialMarker(new Vector2d(20,-50), () -> {
            mechanisms.rotateArm(Utility_Constants.THIRD_LEVEL_POS,Utility_Constants.THIRD_LEVEL_POWER);
        }).strafeTo(new Vector2d(12,-66.5)).lineToSplineHeading(new Pose2d(10,-63,Math.toRadians(90))).build();

        waitForStart();
        if(isStopRequested()) return;

        currentState = AUTO_STATE.MOVING_TO_HUB;

        chassis.followTrajectorySequenceAsync(SplineSequence);

        master:while(opModeIsActive()) {
            Pose2d poseEstimate = chassis.getPoseEstimate();
            telemetry.addData("Pose X",poseEstimate.getX());
            telemetry.addData("Pose Y",poseEstimate.getY());
            telemetry.addData("Pose Heading",poseEstimate.getHeading());

            switch(currentState) {
                case MOVING_TO_HUB: {
                    mechanisms.moveIntake(0);
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
                    mechanisms.rotateArm(0,0.5);
                    if(!chassis.isBusy()) {
                        telemetry.addData("State Machine","Moved to PATH_FINISHED");
                        telemetry.update();
                        timeout.reset();
                        chassis.followTrajectorySequenceAsync(intakingSequence);
                        currentState = AUTO_STATE.INTAKING_FREIGHT;
                    }
                    break;
                }
                case INTAKING_FREIGHT: {
                    if(freightCount >= 2) {
                        currentState = AUTO_STATE.PATH_FINISHED;
                    }
                    if(!chassis.isBusy()) {
                        if(timeout.milliseconds() <= 200) {
                            chassis.followTrajectorySequenceAsync(intakingSequence);
                        }
                        else {
                            chassis.followTrajectorySequenceAsync(faiyoiSequence);
                            currentState = AUTO_STATE.RETURNING_TO_HUB;
                        }
                    }
                    else if(timeout.milliseconds() >= 200) {
                        chassis.breakFollowing();
                        TrajectorySequence substituteReturnSplineSequence = chassis.trajectorySequenceBuilder(poseEstimate).addSpatialMarker(new Vector2d(20,-50), () -> {
                            mechanisms.rotateArm(Utility_Constants.THIRD_LEVEL_POS,0.65);
                        }).strafeTo(new Vector2d(12,-66.5)).lineToSplineHeading(new Pose2d(0,-35,Math.toRadians(-45))).build();
                        chassis.followTrajectorySequenceAsync(substituteReturnSplineSequence);
                        currentState = AUTO_STATE.MOVING_TO_HUB;
                        freightCount++;
                    }
                    break;
                }
                case RETURNING_TO_HUB: {
                    mechanisms.moveIntake(0);
                    if(!chassis.isBusy()) {
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