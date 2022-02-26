package org.firstinspires.ftc.Team19567.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Team19567.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.Team19567.drive.SlowSampleMecanumDriveCancelable;
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

@Autonomous(name="Red Warehouse FSM", group="Dababy")

public class RedWarehouseFSM extends LinearOpMode {

    private ElapsedTime timeout = new ElapsedTime();
    private ElapsedTime intakeTimeout = new ElapsedTime();
    private greenPipeline pipeline = new greenPipeline(telemetry); //Team shipping element OpenCV Pipeline
    private AnalogInput forceSensor = null;
    private DistanceSensor distanceSensor = null;
    private TouchSensor limitSwitch = null;
    private LOCATION location = LOCATION.ALLIANCE_THIRD;
    private RevBlinkinLedDriver blinkin = null;
    private AUTO_STATE currentState = AUTO_STATE.DETECTING_OPENCV;
    private int chosenArmPos = Utility_Constants.THIRD_LEVEL_POS;
    private double chosenArmSpeed = Utility_Constants.THIRD_LEVEL_POWER;
    private double chosenTrajectoryX = -1.5;
    private double chosenTrajectoryY = -40.5;
    private int freightCount = 0;
    private Mechanisms mechanisms = null;

    @Override
    public void runOpMode() {
        //Get the motors from the robot's configuration

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        SlowSampleMecanumDriveCancelable chassis = new SlowSampleMecanumDriveCancelable(hardwareMap);

        mechanisms = new Mechanisms(hardwareMap,telemetry);
        mechanisms.setModes();

        forceSensor = hardwareMap.get(AnalogInput.class,"forceSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class,"distanceSensor");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class,"blinkin");
        limitSwitch = hardwareMap.get(TouchSensor.class,"limitSwitch");

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

        chassis.setPoseEstimate(new Pose2d(10, -63, Math.toRadians(90)));

        TrajectorySequence firstHubSplineSequence = chassis.trajectorySequenceBuilder(new Pose2d(59.5, -68,0)).addTemporalMarker(Utility_Constants.INTAKE_TIME,() -> {
            mechanisms.moveIntake(0);
        }).addSpatialMarker(new Vector2d(15.5,-60), () -> {
            mechanisms.moveIntake(0.4);
            mechanisms.rotateArm(Utility_Constants.THIRD_LEVEL_POS, Utility_Constants.THIRD_LEVEL_POWER);
        }).addSpatialMarker(new Vector2d(2.5,-40.5),() -> {
            mechanisms.releaseServoMove(0.3);
            mechanisms.moveIntake(0.0);
        }).strafeTo(new Vector2d(12,-68.5)).lineToSplineHeading(new Pose2d(2,-40,Math.toRadians(-45))).build();
                /*
                .setReversed(true).splineTo(new Vector2d(16, -61),Math.toRadians(150)).splineTo(new Vector2d(11.5,-40.5),Math.toRadians(135))
                .setReversed(false).build();
                 */
        TrajectorySequence secondReturnSplineSequence = chassis.trajectorySequenceBuilder(firstHubSplineSequence.end()).addSpatialMarker(new Vector2d(20,-64), () -> {
            mechanisms.moveIntake(1.0);
        }).addSpatialMarker(new Vector2d(10, -50),() -> {mechanisms.releaseServoMove(Utility_Constants.RELEASE_SERVO_DEFAULT);})
                /* .splineTo(new Vector2d(14, -62),Math.toRadians(-30))
                .splineTo(new Vector2d(72,-63),Math.toRadians(0)) */
                .lineToSplineHeading(new Pose2d(12,-69.5,0)).strafeTo(new Vector2d(60, -68.5))
                .build();

        telemetry.addData("Status","Finished loading Roadrunner splines");
        telemetry.update();

        while(!opModeIsActive()) {
            location = pipeline.getLocation();
            telemetry.addData("Location",location);
            telemetry.update();
        }

        waitForStart();
        if(isStopRequested()) return;

        switch(location) {
            case ALLIANCE_FIRST: {
                chosenArmPos = Utility_Constants.SECOND_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.SECOND_LEVEL_POWER;
                chosenTrajectoryX = 2;
                chosenTrajectoryY = -42;
                telemetry.addData("OpenCV","Second Level Detected");
                telemetry.update();
                break;
            }
            case ALLIANCE_SECOND: {
                chosenArmPos = Utility_Constants.FIRST_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.FIRST_LEVEL_POWER;
                chosenTrajectoryX = 3;
                chosenTrajectoryY = -45;
                telemetry.addData("OpenCV","First Level Detected");
                telemetry.update();
                break;
            }
            case ALLIANCE_THIRD:
            case NO_ALLIANCE: {
                chosenArmPos = Utility_Constants.THIRD_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.THIRD_LEVEL_POWER;
                chosenTrajectoryX = -1.5;
                chosenTrajectoryY = -40.5;
                telemetry.addData("OpenCV","Third Level Detected");
                telemetry.update();
                break;
            }
            default: {
                chosenArmPos = Utility_Constants.THIRD_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.THIRD_LEVEL_POWER;
                chosenTrajectoryX = -1.5;
                chosenTrajectoryY = -40.5;
                telemetry.addData("OpenCV","Third Level Detected");
                telemetry.update();
            }
        }

        TrajectorySequence SplineSequence = chassis.trajectorySequenceBuilder(new Pose2d(10,-63,Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    mechanisms.moveIntake(0.4);
                    mechanisms.rotateArm(chosenArmPos,chosenArmSpeed);
                }).addSpatialMarker(new Vector2d(chosenTrajectoryX, chosenTrajectoryY-0.4), () -> {
                    mechanisms.releaseServoMove(0.3);
                    mechanisms.moveIntake(0);
                }).lineToSplineHeading(new Pose2d(chosenTrajectoryX,chosenTrajectoryY,Math.toRadians(-45)))
                .build();
        TrajectorySequence firstReturnSplineSequence = chassis.trajectorySequenceBuilder(SplineSequence.end()).addSpatialMarker(new Vector2d(20,-64), () -> {
            mechanisms.moveIntake(1.0);
        }).addSpatialMarker(new Vector2d(10, -50),() -> {mechanisms.releaseServoMove(Utility_Constants.RELEASE_SERVO_DEFAULT);})
                .lineToSplineHeading(new Pose2d(12,-69,0)).strafeTo(new Vector2d(59.5, -68))
                /*
                .splineTo(new Vector2d(14, -62),Math.toRadians(-30))
                .splineTo(new Vector2d(67,-62.5),Math.toRadians(0))
                 */
                .build();

        currentState = AUTO_STATE.SETTING_INTAKE;

        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        mechanisms.releaseServoMove(Utility_Constants.RELEASE_SERVO_DEFAULT);
        timeout.reset();

        master:while(opModeIsActive()) {
            Pose2d poseEstimate = chassis.getPoseEstimate();
            telemetry.addData("Pose X",poseEstimate.getX());
            telemetry.addData("Pose Y",poseEstimate.getY());
            telemetry.addData("Pose Heading",poseEstimate.getHeading());

            switch(currentState) {
                case SETTING_INTAKE: {
                    if(limitSwitch.isPressed()) mechanisms.moveIntake(0.0);
                    else mechanisms.moveIntake(0.1);
                    if(timeout.milliseconds() >= Utility_Constants.INTAKE_RESET_TIME) {
                        currentState = AUTO_STATE.MOVING_TO_HUB;
                        mechanisms.moveIntake(0);
                        chassis.followTrajectorySequenceAsync(SplineSequence);
                    }
                }
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
                    if(timeout.milliseconds() >= Utility_Constants.FLICKER_TIME+100) {
                        telemetry.addData("State Machine","Moved to MOVING_TO_WAREHOUSE");
                        telemetry.update();
                        currentState = AUTO_STATE.MOVING_TO_WAREHOUSE;
                    }
                    break;
                }
                case MOVING_TO_WAREHOUSE: {
                    mechanisms.rotateArm(0,Utility_Constants.GOING_DOWN_POWER);
                    if(freightCount >= 2) {
                        telemetry.addData("State Machine","Moved to PATH_FINISHED");
                        telemetry.update();
                        currentState = AUTO_STATE.PATH_FINISHED;
                    }
                    if(!chassis.isBusy()) {
                        telemetry.addData("State Machine","Moved to INTAKING_FREIGHT");
                        telemetry.update();
                        timeout.reset();
                        currentState = AUTO_STATE.INTAKING_FREIGHT;
                    }
                    break;
                }
                case INTAKING_FREIGHT: {
                    if(distanceSensor.getDistance(DistanceUnit.MM) <= Utility_Constants.DISTANCE_SENSOR_THRESHOLD || forceSensor.getVoltage() >= Utility_Constants.FORCE_SENSOR_THRESHOLD || timeout.milliseconds() >= 300) {
                        intakeTimeout.reset();
                        mechanisms.moveIntake(-0.35);
                        currentState = AUTO_STATE.RETURNING_TO_HUB;
                        chassis.followTrajectorySequenceAsync(firstHubSplineSequence);
                    }
                    break;
                }
                case RETURNING_TO_HUB: {
                    if(!chassis.isBusy()) {
                        timeout.reset();
                        telemetry.addData("State Machine","Moved to DELIVERING_FREIGHT");
                        telemetry.update();
                        chassis.followTrajectorySequenceAsync(secondReturnSplineSequence);
                        currentState = AUTO_STATE.MOVING_TO_WAREHOUSE;
                        freightCount++;
                    }
                    break;
                }
                case PATH_FINISHED: {
                    mechanisms.reset();
                    mechanisms.moveIntake(0);
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