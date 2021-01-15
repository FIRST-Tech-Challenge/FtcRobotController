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
    private static final String VUFORIA_KEY = "ATaHrPr/////AAAAGYhG118G0EZgjFy6T7Snt3otqlgNSultuXDM66X1x1QK3ov5GUJcqL/9RTkdWkDlZDRxBKTAWm/szD7VmJteuQd2WfAk1t8qraapAsr2b4H5k5r4IpIO0UZghwNqhUqfZnCYl3e9tmmuocgZlfLXt4Xw+IAGxZ5e9MaQLR5lTv9/aFO1/CnH9/8jvnSq5NGeLrCHA6BtvqS30sAv7NYX8gz79MHaNiGZvyrUXZslbp2HHkehCocBbc080NrnYCouuUCqIbaMFl4ei8/ViSvdvtJDks4ox5KynBth4HaLHYpYkK3T2XJ1dBab6KfrWn6dm8ug7tfHTy68wLqWev7IWB0oPcqGOY+bZiz343VteHzk";
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
    // This is for coop to move to so it can sample.
    protected WayPoint sampleLocation;

    // This is if we want to pull the robot away from the wall before maneuvering
    protected WayPoint distanceFromWall;

    // This is the spot to align with the stone.
    protected WayPoint positionToGrabSkystone1;
    // This is the spot to grab the stone.
    protected WayPoint grabSkystone1;
    // This is where we want to pull back to to run with the stone.
    protected WayPoint pullBackSkystone1;

    // This might go away, but is intended to jog to run lane from pulling the foundation and not
    // hit partner.
    protected WayPoint buildSiteDodgingPartner;
    // This is a place to eject stones when they are still detected after attempting to place on
    // the foundation.
    protected WayPoint buildSiteEjectingStone;
    // This is under the bridge on the building side and should be far enough in to be a delivery
    protected WayPoint buildSiteUnderBridge;
    // This gets near the foundation starting position so we can grab it slowly
    protected WayPoint snuggleFoundation;
    // This drives into the foundation for grabbing
    protected WayPoint grabFoundation;
    // This pulls the foundation out from the starting position
    protected WayPoint pullFoundation;
    // This pushes the foundation back against the wall.  Only time it is used is to set the
    // foundation angle.
    protected WayPoint pushFoundation;
    // This is where the robot waits for the lift to go down to go under the bridge, should be
    // extending intake over line to park.
    protected WayPoint buildSiteReadyToRun;

    // This is at the top of the skystone line on the quarry side.
    protected WayPoint quarryUnderBridge;

    // This is the position to line up with the second skystone
    protected WayPoint positionToGrabSkystone2;
    // This is moving forward to grab the skystone.
    protected WayPoint grabSkystone2;
    // This is pulling back with the skystone to run.
    protected WayPoint pullBackSkystone2;

    // This is the location to put stones on the foundation
    protected WayPoint foundationDeposit;
    // This might go away if we can get buildSiteReadyToRun working.
    protected WayPoint park;

    // This is the position to line up with the first stone that isn't a skystone.
    protected WayPoint positionToGrabMundanestone1;
    // This is moving forward to grab the stone.
    protected WayPoint grabMundanestone1;
    // This is pulling back with the stone to run.
    protected WayPoint pullBackMundanestone1;

    // This is the position to line up with the second stone that isn't a skystone.
    protected WayPoint positionToGrabMundanestone2;
    // This is moving forward to grab the stone.
    protected WayPoint grabMundanestone2;
    // This is pulling back with the stone to run.
    protected WayPoint pullBackMundanestone2;

    protected ElapsedTime autoTimer = new ElapsedTime();
    protected ElapsedTime autoTaskTimer = new ElapsedTime();

    UltimateGoalRobot robot = new UltimateGoalRobot();

	// Have to set this when we start motions
	public boolean liftIdle = true;

    /**
     */
    public void setupRobotParameters() {
        robot.init(hardwareMap);
        timer = new ElapsedTime();

        robot.resetEncoders();
        robot.setInputShaping(false);
    }

    public void collectStoneFoundation(WayPoint positionToGrabStone, WayPoint grabStone,
                                       WayPoint pullBackStone, boolean moveFoundation) {

        // If the stone didn't get put on the foundation, eject it here.
//        if(robot.stonePresent()) {
//            driveToWayPoint(buildSiteEjectingStone, false, false);
//            robot.startEjecting();
//            while(robot.ejectState != HardwareOmnibot.EjectActivity.IDLE) {
//                updatePosition();
//            }
//            rotateToWayPointAngle(buildSiteReadyToRun, false);
//        }

        // Make sure we don't have a stone before going to collect one.
//        if(!robot.stonePresent()) {
            // Starting point is approaching bridge from the build plate.  buildSiteReadyToRun is
            // supposed to be close enough to score parking.
            driveToWayPointMindingLift(buildSiteReadyToRun);
            // Make sure the lift is down before going under bridge
//            while (robot.stackStone != HardwareOmnibot.StackActivities.IDLE && opModeIsActive()) {
//                updatePosition();
//            }

            // Go under the bridge
            driveToWayPoint(quarryUnderBridge, true, false);

            // Start the intake spinning
//            robot.startIntake(false);

            // Make sure we are at the right angle
            driveToWayPoint(positionToGrabStone, false, false);
            rotateToWayPointAngle(positionToGrabStone, false);
            driveToWayPoint(grabStone, false, false);
            driveToWayPoint(pullBackStone, true, false);

            driveToWayPoint(quarryUnderBridge, true, false);

            // Stop the intake
  //          robot.stopIntake();
            // Drive under the bridge with our skystone.  buildSiteUnderBridge should be far enough to
            // score delivery points.
            driveToWayPoint(buildSiteUnderBridge, true, false);

            // Start the second skystone deposit
//            if (robot.stonePresent()) {
//                if (!skipThis) {
//                    robot.liftTargetHeight = HardwareOmnibot.LiftPosition.STONE_AUTO;
//                    robot.startStoneStacking();
//                }
//            }
            if (moveFoundation) {
                driveToWayPoint(pushFoundation, true, true);
            }
            driveToWayPoint(foundationDeposit, false, false);
            // Make sure we have released the skystone before leaving
//            while ((robot.liftState != HardwareOmnibot.LiftActivity.IDLE ||
//                    robot.releaseState != HardwareOmnibot.ReleaseActivity.IDLE) && opModeIsActive()) {
//                updatePosition();
//            }
//        } else {
//            if (moveFoundation) {
//                driveToWayPoint(pushFoundation, true, true);
//            }
//            driveToWayPoint(foundationDeposit, false, false);
//        }
    }

    public void collectStoneDelivery(WayPoint positionToGrabStone, WayPoint grabStone, WayPoint pullBackStone) {
        // Starting point is approaching bridge from the build plate.  buildSiteReadyToRun is
        // supposed to be close enough to score parking.
        driveToWayPointMindingLift(buildSiteReadyToRun);
        // Make sure the lift is down before going under bridge
//        while (robot.stackStone != HardwareOmnibot.StackActivities.IDLE && opModeIsActive()) {
//            updatePosition();
//        }

        // Go under the bridge
        driveToWayPoint(quarryUnderBridge, true, false);

        // Start the intake spinning
//        robot.startIntake(false);

        // Make sure we are at the right angle
        driveToWayPoint(positionToGrabStone, false, false);
        rotateToWayPointAngle(positionToGrabStone, false);
        driveToWayPoint(grabStone, false, false);
        driveToWayPoint(pullBackStone, true, false);

        driveToWayPoint(quarryUnderBridge, true, false);

        // Stop the intake
//        robot.stopIntake();
        // Drive under the bridge with our skystone.  buildSiteUnderBridge should be far enough to
        // score delivery points.
        driveToWayPoint(buildSiteUnderBridge, false, false);
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
//        robot.performExtendingIntake();
//        robot.performStowing();
//        robot.performLifting();
//        robot.performReleasing();
//        robot.performStoneStacking();
//        robot.performEjecting();
//        liftIdle = robot.stackStone == HardwareOmnibot.StackActivities.IDLE;
    }

    protected void driveToWayPoint(WayPoint destination, boolean passThrough, boolean pullingFoundation) {
        // Loop until we get to destination.
        updatePosition();
        while(!robot.driveToXY(destination.x, destination.y, destination.angle,
                destination.speed, passThrough, pullingFoundation)
                && opModeIsActive()) {
            updatePosition();
        }
    }

    // This is a special case where we want to pass through a point if we get
    // the lift down in time.
    protected void driveToWayPointMindingLift(WayPoint destination) {
        // Loop until we get to destination.
        updatePosition();
        // When the lift is idle (true) we want pass through to be true
        // When the lift is not idle (false) we want pass through to be false.
        while(!robot.driveToXY(destination.x, destination.y, destination.angle,
                destination.speed, liftIdle, false)
                && opModeIsActive()) {
            updatePosition();
        }
    }

    protected void rotateToWayPointAngle(WayPoint destination, boolean pullingFoundation) {
        // Move the robot away from the wall.
        updatePosition();
        robot.rotateToAngle(destination.angle, pullingFoundation, true);
        // Loop until we get to destination.
        updatePosition();
        while(!robot.rotateToAngle(destination.angle, pullingFoundation, false) && opModeIsActive()) {
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