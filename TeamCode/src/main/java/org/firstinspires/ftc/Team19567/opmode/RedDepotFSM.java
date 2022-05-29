package org.firstinspires.ftc.Team19567.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Team19567.drive.SlowMecanumDriveCancelable;
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

/**
 * This is the class for the red depot side. <br>
 * It utilizes a finite state machine (FSM), just like the other opmodes. <br>
 * Alignment: <a href="https://discord.com/channels/873012716757995540/912908534071558144/946196798375084072">Discord</a>
 */

//TODO: Create a base autonomous class and have all specific autonomous classes extend it
@Autonomous(name="Red Depot FSM", group="Dababy")
public class RedDepotFSM extends LinearOpMode {

    //Declare opmode members
    //TODO: In the future, these variables should all be situated in a base autonomous class

    //TIMEOUTS
    private ElapsedTime timeout = new ElapsedTime(); //Create new ElapsedTime() representing time for flicker to release freight
    private ElapsedTime carouselTimeout = new ElapsedTime(); //Create new ElapsedTime() representing time for carousel to spin the duck off

    //CUSTOM
    private greenPipeline pipeline = new greenPipeline(telemetry); //Team shipping element OpenCV Pipeline
    private LOCATION location = LOCATION.ALLIANCE_THIRD; //Variable representing location of TSE
    private AUTO_STATE currentState = AUTO_STATE.DETECTING_OPENCV; //State for the statemachine to begin with
    private Mechanisms mechanisms = null; //New mechanisms that will be properly initialized later

    //HARDWARE
    private RevBlinkinLedDriver blinkin = null; //Blinkin

    //PRELOAD
    private int chosenArmPos = 600; //Variable representing position to set the arm to (for preload)
    private double chosenArmSpeed = 0.3; //Variable representing speed to set the arm at (for preload)
    private double chosenTrajectoryX = -22; //Variable representing x coordinate for the robot to drive to (for the preload)
    private double chosenTrajectoryY = -40; //Variable representing the y coordinate for the robot to drive to (for the preload)

    @Override
    public void runOpMode() {
        //Get the motors from the robot's configuration

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        SlowMecanumDriveCancelable chassis = new SlowMecanumDriveCancelable(hardwareMap);

        mechanisms = new Mechanisms(hardwareMap,telemetry);
        mechanisms.setModes();

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class,"blinkin");

        chassis.setPoseEstimate(new Pose2d(-34, -63, Math.toRadians(90)));

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

        while(!opModeIsActive()) {
            location = pipeline.getLocation();
            telemetry.addData("Location",location);
            telemetry.update();
        }

        if(!opModeIsActive() || isStopRequested()) return;

        waitForStart();

        switch(location) {
            case NO_ALLIANCE:
            case ALLIANCE_FIRST: {
                chosenArmPos = Utility_Constants.THIRD_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.THIRD_LEVEL_POWER;
                chosenTrajectoryX = -23;
                chosenTrajectoryY = -39.5;
                telemetry.addData("OpenCV","Third Level Detected");
                telemetry.update();
                break;
            }
            case ALLIANCE_SECOND: {
                chosenArmPos = Utility_Constants.SECOND_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.SECOND_LEVEL_POWER;
                chosenTrajectoryX = -25.5;
                chosenTrajectoryY = -43.5;
                telemetry.addData("OpenCV","Second Level Detected");
                telemetry.update();
                break;
            }
            case ALLIANCE_THIRD: {
                chosenArmPos = Utility_Constants.FIRST_LEVEL_POS-60;
                chosenArmSpeed = Utility_Constants.FIRST_LEVEL_POWER;
                chosenTrajectoryX = -27.5;
                chosenTrajectoryY = -44;
                telemetry.addData("OpenCV","First Level Detected");
                telemetry.update();
                break;
            }
            default: {
                chosenArmPos = Utility_Constants.THIRD_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.THIRD_LEVEL_POWER;
                chosenTrajectoryX = -23;
                chosenTrajectoryY = -39.5;
                telemetry.addData("OpenCV","Defaulted to Third Level");
                telemetry.update();
            }
        }
        currentState = AUTO_STATE.MOVING_TO_HUB;

        TrajectorySequence preloadSequence = chassis.trajectorySequenceBuilder(new Pose2d(-34, -63, Math.toRadians(90)))
                .waitSeconds(5)
                .addDisplacementMarker(() -> {
                    mechanisms.moveIntake(0.4);
                    mechanisms.rotateArm(chosenArmPos,chosenArmSpeed);
                }).addSpatialMarker(new Vector2d(chosenTrajectoryX, chosenTrajectoryY-0.25), () -> {
                    mechanisms.moveIntake(0);
                    mechanisms.releaseServoMove(0.3);
                }).lineToSplineHeading(new Pose2d(chosenTrajectoryX,chosenTrajectoryY,Math.toRadians(225)))
                .build();
        TrajectorySequence moveToCarouselSequence = chassis.trajectorySequenceBuilder(preloadSequence.end())
                .lineToSplineHeading(new Pose2d(-64,-57,Math.toRadians(0))).build();
        chassis.followTrajectorySequenceAsync(moveToCarouselSequence);
        TrajectorySequence warehouseSequence = chassis.trajectorySequenceBuilder(moveToCarouselSequence.end())
                .strafeTo(new Vector2d(-70,-34))
                /* .waitSeconds(7)
                .splineToConstantHeading(new Vector2d(-20,-40),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60,-35),Math.toRadians(0)) */
                .build();

        mechanisms.releaseServoMove(Utility_Constants.RELEASE_SERVO_DEFAULT);
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        chassis.followTrajectorySequenceAsync(preloadSequence);

        master:while(opModeIsActive() && !isStopRequested()) {
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
                        currentState = AUTO_STATE.DELIVERING_FREIGHT;
                        //mechanisms.releaseServoMove(0.3);
                    }
                    break;
                }
                case DELIVERING_FREIGHT: {
                    if(timeout.milliseconds() >= Utility_Constants.FLICKER_TIME+300) {
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
                    mechanisms.rotateCarousel(Utility_Constants.INIT_POWER+0.1);
                    if(carouselTimeout.milliseconds() >= Utility_Constants.MILLI_END+650) {
                        telemetry.addData("State Machine","Moved to ROTATING_CAROUSEL");
                        telemetry.update();
                        currentState = AUTO_STATE.MOVING_TO_WAREHOUSE;
                        chassis.followTrajectorySequenceAsync(warehouseSequence);
                    }
                    break;
                }
                case MOVING_TO_WAREHOUSE: {
                    mechanisms.rotateCarousel(0.0);
                    if(!chassis.isBusy()) {
                        telemetry.addData("State Machine","Moved to PATH_FINISHED");
                        telemetry.update();
                        currentState = AUTO_STATE.PATH_FINISHED;
                    }
                    break;
                }
                case PATH_FINISHED: {
                    break master;
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