package org.firstinspires.ftc.Team19567.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.Team19567.drive.SlowMecanumDriveCancelable;
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

/**
 * This is the class for the blue warehouse side. <br>
 * It utilizes a finite state machine (FSM), just like the other opmodes. <br>
 * Alignment: <a href="https://discord.com/channels/873012716757995540/912908534071558144/9462033760797410482">Discord</a>
 */

@Autonomous(name="Blue Warehouse FSM", group="Dababy")

//TODO: Create a base autonomous class and have all specific autonomous classes extend it
public class BlueWarehouseFSM extends LinearOpMode {
    //Declare opmode members
    //TODO: In the future, these variables should all be situated in a base autonomous class

    //TIMEOUTS
    /** ElapsedTime() representing time for flicker to release freight */
    private ElapsedTime timeout = new ElapsedTime();
    /** ElapsedTime() representing maximum time to spend intaking freight */
    private ElapsedTime intakeTimeout = new ElapsedTime();

    //CUSTOM
    /** TSE OpenCV Pipeline */
    private final greenPipeline pipeline = new greenPipeline(telemetry);
    /** Location of TSE (defaults to ALLIANCE_THIRD) */
    private LOCATION location = LOCATION.ALLIANCE_THIRD;
    /** State of state machine (defaults to DETECTING_OPENCV) */
    private AUTO_STATE currentState = AUTO_STATE.DETECTING_OPENCV;
    /** Mechanisms of the opmode */
    private Mechanisms mechanisms = null; //Will be properly initialized later

    //HARDWARE
    /** The blinkin */
    private RevBlinkinLedDriver blinkin = null;
    /** The force sensor */
    private AnalogInput forceSensor = null;
    /** The distance sensor */
    private DistanceSensor distanceSensor = null;

    //PRELOAD
    /** Position to set the arm to (for preload) */
    private int chosenArmPos = Utility_Constants.THIRD_LEVEL_POS;
    /** Speed to run the arm at (for preload) */
    private double chosenArmSpeed = Utility_Constants.THIRD_LEVEL_POWER;

    /** Amount of extra freight that have been delivered */
    private int freightCount = 0;
    //Function run when the init button is pressed
    @Override //Annotation used to inform the compiler that the superclass's element is being overriden by the sub class's element
    public void runOpMode() {
        telemetry.addData("Status", "Initialized"); //Indicate that the program has initialized (convention more than anything)
        telemetry.update(); //Update the telemetry (clear the screen and write new data)

        SlowMecanumDriveCancelable chassis = new SlowMecanumDriveCancelable(hardwareMap); //Create new SlowMecanumDriveCancelable from the robot's hardwareMap

        mechanisms = new Mechanisms(hardwareMap,telemetry); //Create new mechanisms object from the robot's hardwareMap and telemetry
        mechanisms.setModes(); //Set modes of mechanisms (e.g. run modes and directions of motors)

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class,"blinkin"); //Properly initialize the blinkin

        //This pose estimate should be the physical location of the robot on the field and should NEVER be changed as a "bandage solution" to driving inaccuracies (which should be fixed by modifying the trajectory sequences themselves)
        chassis.setPoseEstimate(new Pose2d(12, 63, Math.toRadians(-90))); //Set Roadrunner's pose estimate of the robot to the starting position.

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1"); //Get the name of the webcam
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName); //Use the name of the webcam to create the camera to be used

        //Opens the camera device asynchronously (opens it in a new thread so that the main thread is not interrupted)
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            //Runs if OpenCV loads successfully (which is also indicated when the webcam's light turns on)
            @Override
            public void onOpened() {
                camera.setPipeline(pipeline); //Sets the camera's pipeline once it is opened (which may take a few seconds)
                camera.startStreaming(544,288, OpenCvCameraRotation.UPRIGHT); //Sets the dimensions and orientation of the camera's stream (which streams to the Driver Station's viewport)
                telemetry.addData("OpenCV","OpenCV actually connected wow"); //Tells the driver that OpenCV has connected (been opened successfully)
                telemetry.update();
            }
            //Runs if OpenCV fails to load (which happens surprisingly often)
            @Override
            public void onError(int errorCode) {
                telemetry.addData("OpenCV","OpenCV failed to load :( Error Code: " + errorCode); //Tells the driver that OpenCV has failed to load, as well as why
                telemetry.update();
            }
        });

        //These sequences are created as soon as the opmode is initialized in order to reduce the time after the start button is pressed for the robot to move
        TrajectorySequence firstHubSplineSequence = chassis.trajectorySequenceBuilder(new Pose2d(59.5, 70,0))
                .addTemporalMarker(Utility_Constants.INTAKE_TIME,() -> { mechanisms.moveIntake(0); })
                .addSpatialMarker(new Vector2d(15.5,60), () -> {
                    mechanisms.moveIntake(0.4);
                    mechanisms.rotateArm(Utility_Constants.THIRD_LEVEL_POS, Utility_Constants.THIRD_LEVEL_POWER); })
                .addSpatialMarker(new Vector2d(2.5,40.5),() -> {
                    mechanisms.releaseServoMove(0.3);
                    mechanisms.moveIntake(0.0); })
                .strafeTo(new Vector2d(12,71)).lineToSplineHeading(new Pose2d(2.5,40,Math.toRadians(45)))
                .build();
                /* More advanced spline code that was too inconsistent to use:
                .setReversed(true).splineTo(new Vector2d(16, -61),Math.toRadians(150)).splineTo(new Vector2d(11.5,-40.5),Math.toRadians(135))
                .setReversed(false).build(); */
        //Its in the name
        TrajectorySequence secondReturnSplineSequence = chassis.trajectorySequenceBuilder(firstHubSplineSequence.end())
                .addSpatialMarker(new Vector2d(20,64), () -> {
                    mechanisms.moveIntake(1.0); })
                .addSpatialMarker(new Vector2d(10, 50),() -> {
                    mechanisms.releaseServoMove(Utility_Constants.RELEASE_SERVO_DEFAULT);})
                /* More advanced spline code that was too inconsistent to use:
                .splineTo(new Vector2d(14, -62),Math.toRadians(-30))
                .splineTo(new Vector2d(72,-63),Math.toRadians(0)) */
                .lineToSplineHeading(new Pose2d(12,72,0)).strafeTo(new Vector2d(59.5, 72))
                .build();

        telemetry.addData("Status","Finished loading Roadrunner splines");
        telemetry.update();

        //Creates a loop that runs from init until the start button is pressed
        while(!opModeIsActive()) {
            location = pipeline.getLocation(); //Update the location of the TSE each loop
            telemetry.addData("Location",location); //Inform the driver of the detected location
            telemetry.update();
        }
        if(!opModeIsActive() || isStopRequested()) return; //Exits the function (and by extension, the program) if the stop button is pressed (more of a failsafe than anything)

        waitForStart(); /* Wait for the driver to press start (necessary for the program not to crash, even if it is technically redundant)

        THE FOLLOWING CODE RUNS AFTER THE DRIVER PRESSES START

        Changes variables based on the ascertained location of the TSE */

        //x coordinate for the robot to drive to (for the preload)
        double chosenTrajectoryX;
        //y coordinate for the robot to drive to (for the preload)
        double chosenTrajectoryY;

        switch(location) {
            //Code to run if the second level is detected
            case ALLIANCE_SECOND: {
                chosenArmPos = Utility_Constants.SECOND_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.SECOND_LEVEL_POWER;
                chosenTrajectoryX = 2.5;
                chosenTrajectoryY = 43;
                telemetry.addData("OpenCV","Second Level Detected");
                telemetry.update();
                break;
            }
            //Technically first level, (due to webcam positioning and some positions sometimes being flipped), although the third level is being detected
            case ALLIANCE_THIRD: {
                chosenArmPos = Utility_Constants.FIRST_LEVEL_POS-20;
                chosenArmSpeed = Utility_Constants.FIRST_LEVEL_POWER;
                chosenTrajectoryX = 3.75;
                chosenTrajectoryY = 45.5;
                telemetry.addData("OpenCV","First Level Detected");
                telemetry.update();
                break;
            }
            //Technically third level, although the first level (or no level) is being detected
            case ALLIANCE_FIRST:
            case NO_ALLIANCE: {
                chosenArmPos = Utility_Constants.THIRD_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.THIRD_LEVEL_POWER;
                chosenTrajectoryX = 2;
                chosenTrajectoryY = 37;
                telemetry.addData("OpenCV","Third Level Detected");
                telemetry.update();
                break;
            }
            //Code to run by default (failsafe)
            default: {
                chosenArmPos = Utility_Constants.THIRD_LEVEL_POS;
                chosenArmSpeed = Utility_Constants.THIRD_LEVEL_POWER;
                chosenTrajectoryX = 2;
                chosenTrajectoryY = 37.5;
                telemetry.addData("OpenCV","Third Level Detected");
                telemetry.update();
            }
        }

        //Changes the current state from detecting the TSE to moving to the alliance hub (delivering the preload)
        currentState = AUTO_STATE.MOVING_TO_HUB;

        //Roadrunner TrajectorySequence followed when the robot is delivering the preload
        TrajectorySequence SplineSequence = chassis.trajectorySequenceBuilder(new Pose2d(12,63,Math.toRadians(-90))) //Sets initial pose, which is equal to the robot's pose estimate, *this is important*
                //Creates a marker to run immediately after the previous command has finished running
                .addDisplacementMarker(() -> {
                    mechanisms.moveIntake(0.4); //Moves the intake so the arm doesn't get stuck
                    mechanisms.rotateArm(chosenArmPos,chosenArmSpeed); //Begins moving the arm
                }).addSpatialMarker(new Vector2d(chosenTrajectoryX, chosenTrajectoryY +0.4), () -> { //Creates a marker to run approximately 0.4 inches before the specified coordinates have been reached (in this case, the alliance hub)
                    mechanisms.releaseServoMove(0.3); //Begins moving the release servo
                    mechanisms.moveIntake(0); //Stops moving the intake
                }).lineToSplineHeading(new Pose2d(chosenTrajectoryX, chosenTrajectoryY,Math.toRadians(45))) //Begins moving the robot towards the alliance hub (see https://learnroadrunner.com/trajectorybuilder-functions.html for what this function does)
                .build(); //Builds the trajectory sequence (necessary)
        //Its in the name
        TrajectorySequence firstReturnSplineSequence = chassis.trajectorySequenceBuilder(SplineSequence.end())
                .addSpatialMarker(new Vector2d(20,64), () -> {
                    mechanisms.moveIntake(1.0); })
                .addSpatialMarker(new Vector2d(10, 50),() -> {
                    mechanisms.releaseServoMove(Utility_Constants.RELEASE_SERVO_DEFAULT);})
                .lineToSplineHeading(new Pose2d(12,69,0)).strafeTo(new Vector2d(59.5, 68.5))
                /* More advanced spline code that was too inconsistent too use:
                .splineTo(new Vector2d(14, -62),Math.toRadians(-30))
                .splineTo(new Vector2d(67,-62.5),Math.toRadians(0)) */
                .build();

        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        mechanisms.releaseServoMove(Utility_Constants.RELEASE_SERVO_DEFAULT);
        chassis.followTrajectorySequenceAsync(SplineSequence);

        master:while(opModeIsActive()) { //State machine loop to constantly run as long as the stop button is not pressed; the loop is labelled master
            Pose2d poseEstimate = chassis.getPoseEstimate(); //Gets the robot's estimated pose to display to telemetry
            //Self explanatory
            telemetry.addData("Pose X",poseEstimate.getX());
            telemetry.addData("Pose Y",poseEstimate.getY());
            telemetry.addData("Pose Heading",poseEstimate.getHeading());

            /*
            Switch for the state machine's states
            Each case represents a possible state the robot can be in
             */
            switch(currentState) {
                case MOVING_TO_HUB: {
                    //Condition is true if the Roadrunner sequence finishes
                    if(!chassis.isBusy()) {
                        timeout.reset(); //Resets the flicker timeout
                        telemetry.addData("State Machine","Moved to DELIVERING_FREIGHT"); //Informs the driver that the state machine changed state
                        telemetry.update();
                        currentState = AUTO_STATE.DELIVERING_FREIGHT; //Changes the current state to delivering freight
                        freightCount++;
                    }
                    break;
                }
                case DELIVERING_FREIGHT: {
                    //Condition is true once the designated flicker time has expired
                    if(timeout.milliseconds() >= Utility_Constants.FLICKER_TIME) {
                        telemetry.addData("State Machine","Moved to MOVING_TO_WAREHOUSE");
                        telemetry.update();
                        currentState = AUTO_STATE.MOVING_TO_WAREHOUSE;
                        chassis.followTrajectorySequenceAsync(firstReturnSplineSequence); //Begins asynchronously following the moveToCarousel sequence
                    }
                    break;
                }
                case MOVING_TO_WAREHOUSE: {
                    mechanisms.rotateArm(0,Utility_Constants.GOING_DOWN_POWER);
                    //Condition is true if an extra freight has already been delivered (this can be adjusted to whatever)
                    if(freightCount >= 2) {
                        telemetry.addData("State Machine","Moved to PATH_FINISHED");
                        telemetry.update();
                        currentState = AUTO_STATE.PATH_FINISHED;
                    }
                    //Condition is true if the Roadrunner sequence finishes
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
                        mechanisms.moveIntake(-0.5);
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
                    //If the path is finished, then the robot breaks out of the master (state machine) loop and will basically stop running
                    break master;
                }
                default: {
                    currentState = AUTO_STATE.MOVING_TO_HUB;
                }
            }
            mechanisms.maintainBalance(); //Every loop, make sure that the box is as parallel to the ground as possible
            chassis.update(); //Updates the robot's estimated pose to use for Roadrunner (necessary)
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