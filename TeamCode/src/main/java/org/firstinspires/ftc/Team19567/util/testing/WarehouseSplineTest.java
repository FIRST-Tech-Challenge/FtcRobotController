package org.firstinspires.ftc.Team19567.util.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Team19567.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.Team19567.pipeline.greenPipeline;
import org.firstinspires.ftc.Team19567.pipeline.LOCATION;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.Team19567.util.AUTO_STATE;
import org.firstinspires.ftc.Team19567.util.Mechanisms;
import org.firstinspires.ftc.Team19567.util.Utility_Constants;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Warehouse Spline Test", group="Testing")

public class WarehouseSplineTest extends LinearOpMode {

    private ElapsedTime timeout = new ElapsedTime();
    private greenPipeline pipeline = new greenPipeline(telemetry); //Team shipping element OpenCV Pipeline
    private AnalogInput forceSensor = null;
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
        forceSensor = hardwareMap.get(AnalogInput.class,"forceSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");

        chassis.setPoseEstimate(new Pose2d(10, -63, Math.toRadians(90)));

        TrajectorySequence SplineSequence = chassis.trajectorySequenceBuilder(new Pose2d(10,-63,Math.toRadians(90)))
                .addSpatialMarker(new Vector2d(8,-50), () -> {
                    mechanisms.rotateArm(Utility_Constants.THIRD_LEVEL_POS,Utility_Constants.THIRD_LEVEL_POWER);
                }).addSpatialMarker(new Vector2d(-1.5,-41), () -> {
                    mechanisms.releaseServoMove(0.3);
                }).lineToSplineHeading(new Pose2d(-1.5,-40.5,Math.toRadians(-45)))
                .build();
        TrajectorySequence firstReturnSplineSequence = chassis.trajectorySequenceBuilder(SplineSequence.end()).addSpatialMarker(new Vector2d(30,-64), () -> {
            mechanisms.moveIntake(1.0);
        }).addSpatialMarker(new Vector2d(10, -50),() -> {mechanisms.releaseServoMove(Utility_Constants.RELEASE_SERVO_DEFAULT);})
                .splineTo(new Vector2d(15, -68),Math.toRadians(-10))
                .splineTo(new Vector2d(69,-69),Math.toRadians(0))
                //.lineToSplineHeading(new Pose2d(12,-66.5,0)).strafeTo(new Vector2d(54, -66.5))
                .build();
        TrajectorySequence firstHubSplineSequence = chassis.trajectorySequenceBuilder(firstReturnSplineSequence.end()).addSpatialMarker(new Vector2d(20,-50), () -> {
            mechanisms.moveIntake(0.4);
            mechanisms.rotateArm(Utility_Constants.THIRD_LEVEL_POS, Utility_Constants.THIRD_LEVEL_POWER);
        }).addSpatialMarker(new Vector2d(8.5,-39.5),() -> {
            mechanisms.releaseServoMove(0.3);
            mechanisms.moveIntake(0.0);
        }).setReversed(true).splineTo(new Vector2d(15, -68),Math.toRadians(170)).splineTo(new Vector2d(8,-39),Math.toRadians(135))
                .setReversed(false).build();

        telemetry.addData("Status","Finished loading Roadrunner splines");
        telemetry.update();

        waitForStart();
        if(isStopRequested()) return;

        currentState = AUTO_STATE.MOVING_TO_HUB;

        mechanisms.releaseServoMove(Utility_Constants.RELEASE_SERVO_DEFAULT);
        chassis.followTrajectorySequenceAsync(SplineSequence);

        master:while(opModeIsActive()) {
            Pose2d poseEstimate = chassis.getPoseEstimate();
            telemetry.addData("Pose X",poseEstimate.getX());
            telemetry.addData("Pose Y",poseEstimate.getY());
            telemetry.addData("Pose Heading",poseEstimate.getHeading());

            switch(currentState) {
                case MOVING_TO_HUB: {
                    if(!chassis.isBusy()) {
                        timeout.reset();
                        telemetry.addData("State Machine","Moved to DELIVERING_FREIGHT");
                        telemetry.update();
                        chassis.followTrajectorySequenceAsync(firstReturnSplineSequence);
                        currentState = AUTO_STATE.MOVING_TO_WAREHOUSE;
                        freightCount++;
                    }
                    break;
                }

                case DELIVERING_FREIGHT: {
                    if(timeout.milliseconds() >= Utility_Constants.FLICKER_TIME) {
                        telemetry.addData("State Machine","Moved to MOVING_TO_WAREHOUSE");
                        telemetry.update();
                        currentState = AUTO_STATE.MOVING_TO_WAREHOUSE;
                    }
                    break;
                }

                case MOVING_TO_WAREHOUSE: {
                    mechanisms.rotateArm(0,Utility_Constants.GOING_DOWN_POWER);
                    if(!chassis.isBusy()) {
                        telemetry.addData("State Machine","Moved to PATH_FINISHED");
                        telemetry.update();
                        timeout.reset();
                        currentState = AUTO_STATE.INTAKING_FREIGHT;
                    }
                    break;
                }
                case INTAKING_FREIGHT: {
                    if(distanceSensor.getDistance(DistanceUnit.MM) <= Utility_Constants.DISTANCE_SENSOR_THRESHOLD || forceSensor.getVoltage() >= Utility_Constants.FORCE_SENSOR_THRESHOLD || timeout.milliseconds() >= 200) {
                            mechanisms.moveIntake(-1.0);
                            currentState = AUTO_STATE.MOVING_TO_HUB;
                            chassis.followTrajectorySequenceAsync(firstHubSplineSequence);
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
            telemetry.addData("Timeout",timeout.milliseconds());
            telemetry.addData("Force Sensor",forceSensor.getVoltage());
            telemetry.addData("Distance Sensor",distanceSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }

        telemetry.addData("Status", "Path Complete");
        telemetry.update();
    }
}