package org.firstinspires.ftc.Team19567.util.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Team19567.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.Team19567.pipeline.greenPipeline;
import org.firstinspires.ftc.Team19567.pipeline.LOCATION;
import org.firstinspires.ftc.Team19567.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.Team19567.util.AUTO_STATE;
import org.firstinspires.ftc.Team19567.util.Mechanisms;
import org.firstinspires.ftc.Team19567.util.Utility_Constants;

@Autonomous(name="Depot Spline Test", group="Testing")
public class DepotSplineTest extends LinearOpMode {

    private ElapsedTime timeout = new ElapsedTime();
    private ElapsedTime carouselTimeout = new ElapsedTime();
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

        chassis.setPoseEstimate(new Pose2d(-34, -63, Math.toRadians(90)));

        TrajectorySequence PreloadSequence = chassis.trajectorySequenceBuilder(new Pose2d(-34, -63, Math.toRadians(90)))
                .addSpatialMarker(new Vector2d(-33,-40),() -> {
                    mechanisms.rotateArm(Utility_Constants.THIRD_LEVEL_POS,Utility_Constants.THIRD_LEVEL_POWER);
                }).lineToSplineHeading(new Pose2d(-32.5,-24,Math.toRadians(0)))
                .build();

        TrajectorySequence moveToCarouselSequence = chassis.trajectorySequenceBuilder(PreloadSequence.end())
                .lineToSplineHeading(new Pose2d(-62,-55,Math.toRadians(180))).build();

        TrajectorySequence warehouseSequence = chassis.trajectorySequenceBuilder(moveToCarouselSequence.end()).setReversed(true)
                .splineToConstantHeading(new Vector2d(10,-64),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50,-64),Math.toRadians(0)).build();

        waitForStart();
        if(isStopRequested()) return;

        currentState = AUTO_STATE.MOVING_TO_HUB;

        chassis.followTrajectorySequenceAsync(PreloadSequence);

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
                        telemetry.addData("State Machine","Moved to MOVING_TO_CAROUSEL");
                        telemetry.update();
                        currentState = AUTO_STATE.MOVING_TO_CAROUSEL;
                        chassis.followTrajectorySequenceAsync(moveToCarouselSequence);
                    }
                    break;
                }
                case MOVING_TO_CAROUSEL: {
                    mechanisms.reset();
                    if(!chassis.isBusy()) {
                        telemetry.addData("State Machine","Moved to ROTATING_CAROUSEL");
                        telemetry.update();
                        carouselTimeout.reset();
                        currentState = AUTO_STATE.ROTATING_CAROUSEL;
                    }
                    break;
                }
                case ROTATING_CAROUSEL: {
                    mechanisms.rotateCarousel(Utility_Constants.INIT_POWER);
                    if(carouselTimeout.milliseconds() >= Utility_Constants.MILLI_END) {
                        telemetry.addData("State Machine","Moved to ROTATING_CAROUSEL");
                        telemetry.update();
                        chassis.followTrajectorySequenceAsync(warehouseSequence);
                        currentState = AUTO_STATE.MOVING_TO_WAREHOUSE;
                    }
                    break;
                }
                case MOVING_TO_WAREHOUSE: {
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
            telemetry.addData("State", currentState);
            telemetry.update();
        }

        telemetry.addData("Status", "Path Complete");
        telemetry.update();
    }
}