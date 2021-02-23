package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HelperClasses.WayPoint;
import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;

/**
 * Created by 7592 RoarBots
 */
public abstract class UltimateGoalAutoBase extends LinearOpMode {
    protected static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    protected static final String LABEL_FIRST_ELEMENT = "Quad";
    protected static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY_OLD = "ATaHrPr/////AAAAGYhG118G0EZgjFy6T7Snt3otqlgNSultuXDM66X1x1QK3ov5GUJcqL/9RTkdWkDlZDRxBKTAWm/szD7VmJteuQd2WfAk1t8qraapAsr2b4H5k5r4IpIO0UZghwNqhUqfZnCYl3e9tmmuocgZlfLXt4Xw+IAGxZ5e9MaQLR5lTv9/aFO1/CnH9/8jvnSq5NGeLrCHA6BtvqS30sAv7NYX8gz79MHaNiGZvyrUXZslbp2HHkehCocBbc080NrnYCouuUCqIbaMFl4ei8/ViSvdvtJDks4ox5KynBth4HaLHYpYkK3T2XJ1dBab6KfrWn6dm8ug7tfHTy68wLqWev7IWB0oPcqGOY+bZiz343VteHzk";
    private static final String VUFORIA_KEY = "AV9qQjH/////AAABmevyHOWysUHMk13tNwTMMwZGklBtzRfo6k5H4WmEo/nTR/rzB/toJF0mfAo2cyyN03uGAUOsAaTDb5Wr45yCf38AZzefFajJ1L/5g8cHxHFy84n0iuJ+kDSdxb0AjRTq5phTrC68nqR/agCDkV7UEdsop3dpeU0LXyP9cdhhulwfLliolsMBAkemUjxroUtpgs1vd57cyvvB1mJCGylGHBWFaIxjxmY51u0HVqu+X3W8Jp+e1dURqNRBA4SQn5bsFPJy1s/crYZnd2wPys8PSkOlT7NvMCGH3q5EPxdtl2Ee/BB/GdgclUX8tllA3A9WQ4cmwniK6hOZtHhQDcMbu46JNLZ6lUD0n3B2FCLXnhF1";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    protected VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    protected TFObjectDetector tfod;

    protected ElapsedTime timer;

    public static float MM_PER_INCH = (float)25.4;
    public static float mmPerInch = UltimateGoalAutoBase.MM_PER_INCH;
    public static float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    public static float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
    protected boolean skipThis = false;
    protected boolean integrated = false;

    // The curve points we are going to use to do the whole auto.  Set in the alliance
    // specific autonomous.
    // This is the field location the bot starts at.
    protected WayPoint startLocation;

    // These waypoints maneuver the robot around the starting ring stack.
    protected WayPoint aroundStartingStack1;
    protected WayPoint aroundStartingStack2;

    // To deposit the first wobble goal, set by vision.
    protected WayPoint targetZone1;

    // The powershots
    protected WayPoint powerShotFirst;
    protected WayPoint powerShotSecond;
    protected WayPoint powerShotThird;

    // Starting stack pick up line up
    protected WayPoint beforeStack;
    protected WayPoint collectStack;

    // Pickup the second wobble goal.
    protected WayPoint wobble2PickupLineup;
    protected WayPoint wobble2Pickup;

    // Collect the starting ring stack.
    protected WayPoint collectStartingStack;

    // Shoot the collected rings in the high goal.
    protected WayPoint highGoal;

    // Deliver the second wobble goal.
    protected WayPoint targetZone2;

    // End of auto parking spot.
    protected WayPoint park;

    protected ElapsedTime autoTimer = new ElapsedTime();
    protected ElapsedTime autoTaskTimer = new ElapsedTime();

    UltimateGoalRobot robot = new UltimateGoalRobot();

    /**
     */
    public void setupRobotParameters() {
        robot.init(hardwareMap);
        timer = new ElapsedTime();

        robot.resetEncoders();
        robot.setInputShaping(false);
    }

    protected void updatePosition() {
        // Allow the robot to read sensors again
        robot.resetReads();
        MyPosition.giveMePositions(robot.getLeftEncoderWheelPosition(),
                robot.getRightEncoderWheelPosition(),
                robot.getStrafeEncoderWheelPosition());

        // Progress the robot actions.
        performRobotActions();
    }

    protected void performRobotActions() {
        robot.resetReads();
        robot.performInjecting();
        robot.performTripleInjecting();
        robot.performRotatingArm();
        robot.performClawToggle();
        robot.updateShooterStability();
    }

    protected void driveToWayPoint(WayPoint destination, boolean passThrough) {
        // Loop until we get to destination.
        updatePosition();
        double allowedError = 2.0;
        if(passThrough) {
            allowedError = 7.0;
        }
        while(!robot.driveToXY(destination.x, destination.y, destination.angle,
                robot.MIN_DRIVE_RATE, destination.speed, 0.014, allowedError, passThrough)
                && opModeIsActive()) {
            updatePosition();
        }
    }

    protected void rotateToWayPointAngle(WayPoint destination) {
        // Move the robot away from the wall.
        updatePosition();
        robot.rotateToAngle(destination.angle, true);
        // Loop until we get to destination.
        updatePosition();
        while(!robot.rotateToAngle(destination.angle, false) && opModeIsActive()) {
            updatePosition();
        }
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    protected void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    protected void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}