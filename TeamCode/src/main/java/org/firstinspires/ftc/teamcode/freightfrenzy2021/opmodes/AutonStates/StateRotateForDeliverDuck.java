package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotsutil.Pose;
import org.firstinspires.ftc.teamcode.ebotsutil.PoseError;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.ebotsutil.UtilFuncs;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.AutonDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateRotateForDeliverDuck implements EbotsAutonState{


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    private EbotsAutonOpMode autonOpMode;
    private Telemetry telemetry;
    private AutonDrive motionController;
    // State used for updating telemetry
    private double targetHeadingDeg;

    long stateTimeLimit = 2000;
    StopWatch stopWatch = new StopWatch();

    private double headingErrorDeg;
    private boolean wasTargetPoseAchieved;
    private StopWatch stopWatchPoseAchieved = new StopWatch();
    private long targetDurationMillis = 250;

    private String logTag = "EBOTS";
    private boolean firstPass = true;
    private final Pose currentPose;
    private final Pose targetPose;
    private final PoseError poseError;
    private final EbotsImu ebotsImu;
    private int loopCount = 0;

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public StateRotateForDeliverDuck(EbotsAutonOpMode autonOpMode){
        Log.d(logTag, "In Constructor for StateRotateForDeliverDuck");
        this.autonOpMode = autonOpMode;
        telemetry = autonOpMode.telemetry;
        this.motionController = autonOpMode.getMotionController();
//        this.motionController = new AutonDrive(autonOpMode);
        this.motionController.setSpeed(Speed.FAST);

//        targetHeadingDeg = (AllianceSingleton.isBlue() ? 0 : 0);
        targetHeadingDeg = 0;
        ebotsImu = EbotsImu.getInstance(autonOpMode.hardwareMap);
        currentPose = autonOpMode.getCurrentPose();
        targetPose = new Pose(currentPose.getX(),currentPose.getY(),0);
        poseError = new PoseError(currentPose, targetPose, autonOpMode);

    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    @Override
    public boolean shouldExit() {
        currentPose.setHeadingDeg(ebotsImu.getCurrentFieldHeadingDeg(true));
        poseError.calculateError(currentPose, targetPose, 0);
        headingErrorDeg = poseError.getHeadingErrorDeg();
        if (firstPass){
            Log.d(logTag, "heading error is " + String.format("%.2f", headingErrorDeg));
            firstPass = false;
        }
        double acceptableError = 3;

        boolean isTargetHeadingAchieved = Math.abs(headingErrorDeg) < acceptableError;

        boolean isTargetHeadingSustained = isTargetHeadingSustained(isTargetHeadingAchieved);

        boolean stateTimedOut = stopWatch.getElapsedTimeMillis() >= stateTimeLimit;

        if (isTargetHeadingSustained) Log.d(logTag, "Exited because target heading sustained");
        if (stateTimedOut) Log.d(logTag, "Exited because timed out");
        if (!autonOpMode.opModeIsActive()) {
            Log.d(logTag, "opMode not active");
            Log.d(logTag, autonOpMode.getClass().getSimpleName());
        }
        updateTelemetry();
        loopCount++;

        return isTargetHeadingSustained | stateTimedOut | !autonOpMode.opModeIsActive();
//        return isTargetHeadingSustained | stateTimedOut;
    }

    @Override
    public void performStateActions() {
//        Log.d(logTag, "in state actions for " + autonOpMode.getClass().getSimpleName());
//        motionController.rotateToFieldHeadingFromError(headingErrorDeg);
        motionController.calculateDriveFromError(poseError);
        boolean debugOn = (loopCount % 10 == 0);
        if (debugOn){
            Log.d("EBOTS", poseError.toString());
        }

    }

    @Override
    public void performTransitionalActions() {
        Log.d(logTag, "Performing transitional actions for StateDeliverDuck");
        Log.d(logTag, "Current heading is: " + String.format("%.2f", EbotsImu.getInstance(autonOpMode.hardwareMap).getCurrentFieldHeadingDeg(false)) +
                " Error is: " + (String.format("%.2f", EbotsImu.getInstance(autonOpMode.hardwareMap).getCurrentFieldHeadingDeg(false) - targetHeadingDeg)));
        motionController.stop();

        // update robot's pose in autonOpMode
        autonOpMode.getCurrentPose().setHeadingDeg(EbotsImu.getInstance(autonOpMode.hardwareMap).getCurrentFieldHeadingDeg(true));
        telemetry.addLine("Exiting RotateForDeliverDuck");
        telemetry.update();
        Log.d(logTag, "Exiting RotateForDeliverDuck, robot's pose: " + autonOpMode.getCurrentPose().toString());
    }

    private boolean isTargetHeadingSustained(boolean isTargetPoseAchieved){
        boolean isTargetPoseSustained = false;
        if (isTargetPoseAchieved && !wasTargetPoseAchieved){
            // if pose newly achieved
            stopWatchPoseAchieved.reset();
            wasTargetPoseAchieved = true;
        } else if (isTargetPoseAchieved && wasTargetPoseAchieved){
            // pose is achieved was correct the previous loop
            if (stopWatchPoseAchieved.getElapsedTimeMillis() >= targetDurationMillis){
                // see how long the pose has been achieved
                isTargetPoseSustained = true;
            }
        } else{
            // target pose is not achieved
            wasTargetPoseAchieved = false;
        }

        return isTargetPoseSustained;
    }

    private void updateTelemetry(){
        telemetry.addData("Current Heading", String.format("%.2f", EbotsImu.getInstance(autonOpMode.hardwareMap).getCurrentFieldHeadingDeg(false)));
        telemetry.addData("Target Heading", targetHeadingDeg);

    }
}