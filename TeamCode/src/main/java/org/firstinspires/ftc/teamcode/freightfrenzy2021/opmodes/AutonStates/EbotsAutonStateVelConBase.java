package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ebotsenums.Accuracy;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsImu;
import org.firstinspires.ftc.teamcode.ebotsutil.FieldPosition;
import org.firstinspires.ftc.teamcode.ebotsutil.Pose;
import org.firstinspires.ftc.teamcode.ebotsutil.PoseError;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.motioncontrollers.AutonDrive;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public abstract class EbotsAutonStateVelConBase implements EbotsAutonState{
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Attributes
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    protected EbotsAutonOpMode autonOpMode;
    protected Telemetry telemetry;

    protected int targetClicks;
    protected long stateTimeLimit;
    protected StopWatch stopWatchState;
    protected StopWatch stopWatchLoop;
    protected long loopDuration = 0L;

    protected AutonDrive motionController;
    protected EbotsImu ebotsImu;

    protected final String logTag = "EBOTS";
    protected final String intFmt = "%d";
    protected final String oneDec = "%.1f";
    protected final String twoDec = "%.2f";

    protected boolean firstPass = true;
    protected static double travelDistance = 0.0;
    protected double travelDirectionDeg = 0;
    protected double targetHeadingDeg = 0;
    protected boolean rotationOnly = false;
    Accuracy accuracy = Accuracy.STANDARD;

    protected Pose currentPose;
    protected Pose targetPose;
    protected PoseError poseError;
    protected boolean wasRotationAchieved = false;

    protected int lastAvgClicks = 0;

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Constructors
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    public EbotsAutonStateVelConBase(EbotsAutonOpMode autonOpMode){
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");
        this.autonOpMode = autonOpMode;
        this.telemetry = autonOpMode.telemetry;
        motionController = new AutonDrive(autonOpMode);
        autonOpMode.setMotionController(motionController);
        ebotsImu = EbotsImu.getInstance(autonOpMode.hardwareMap);

        currentPose = autonOpMode.getCurrentPose();

        stopWatchState = new StopWatch();
        stopWatchLoop = new StopWatch();

        Log.d(logTag, "Constructor complete");

    }

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Getters & Setters
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    public static double getTravelDistance() {
        return travelDistance;
    }

    public static void setTravelDistance(double distance){
        travelDistance = distance;
    }
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Class Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    
    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    Instance Methods
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

    @Override
    public boolean shouldExit(){
        if(firstPass){
            Log.d(logTag, "Inside shouldExit of abstract class...");
            firstPass = false;
        }
        updateLocationAndError();

        boolean exitVerdict = false;
        if (!rotationOnly){
            exitVerdict = translateExitTest();
        } else {
            exitVerdict = rotateExitTest();
        }
        return exitVerdict;
    }

    @Override
    public void performStateActions() {
        motionController.calculateDriveFromError(poseError);
        telemetry.addData("Avg Clicks", motionController.getAverageClicks());
        telemetry.addData("Position Reached", motionController.isTargetReached());
        telemetry.addLine(stopWatchState.toString());
    }

    @Override
    public void performTransitionalActions() {
        Log.d(logTag, "Inside transitional Actions of abstract class..." + this.getClass().getSimpleName());
        motionController.stop();
        motionController.logAllEncoderClicks();
        Log.d(logTag, "Elapsed time at exit: " + String.format(intFmt, stopWatchState.getElapsedTimeMillis()) +
                " of stateTimeLimit: " + String.format(intFmt, stateTimeLimit));
        updateLocationAndError();

        Log.d(logTag, "Average Error: " + String.format(intFmt, motionController.getAverageError()));
        Log.d(logTag, "Pose when exiting " + this.getClass().getSimpleName() + ": "
                + autonOpMode.getCurrentPose().toString());

        telemetry.addLine("Exiting " + this.getClass().getSimpleName());
        telemetry.update();
    }

    protected void initAutonState(){
        calculateTravel();
        updateError();
        calculateTimeLimit();
    }

    protected void calculateTravel(){
        if (travelDistance == 0.0 && targetHeadingDeg != currentPose.getHeadingDeg()){
            rotationOnly = true;
        }
        double xTravel = Math.cos(Math.toRadians(travelDirectionDeg)) * travelDistance;
        double yTravel = Math.sin(Math.toRadians(travelDirectionDeg)) * travelDistance;
        FieldPosition requestedTravel = new FieldPosition(xTravel, yTravel);
        FieldPosition targetFieldPosition = currentPose.getFieldPosition().offsetFunc(requestedTravel);
        targetPose = new Pose(targetFieldPosition, targetHeadingDeg);
        Log.d(logTag, "Target pose for state created: " + targetPose.toString());
    }

    protected void updateError(){
        poseError = new PoseError(currentPose, targetPose, autonOpMode);
        Log.d(logTag, "Starting Error for state: \n" + poseError.toString());
    }

    protected void calculateTimeLimit(){
        stateTimeLimit = motionController.calculateTimeLimitMillis(poseError);
    }

    protected void setDriveTarget(){
        Log.d(logTag, "Setting drive target with travelDistance: " +
                String.format(twoDec, travelDistance) + ", " +
                "travelFieldHeading: " + String.format(twoDec, travelDirectionDeg) + ", " +
                "targetHeading: " + String.format(twoDec, targetHeadingDeg));
        // Note, the auton motion controller is limited to travel in 4 cardinal directions
        motionController.setEncoderTarget(poseError);
    }

    protected boolean translateExitTest(){
        boolean stateTimedOut = stopWatchState.getElapsedTimeMillis() > stateTimeLimit;
        boolean targetTravelCompleted = isTargetFieldPositionAchieved() && isTargetRotationAchieved();
        if (stateTimedOut) Log.d(logTag, "Exited because timed out. ");
        if (targetTravelCompleted) Log.d(logTag, "Exited because travel completed");
        if (!autonOpMode.opModeIsActive()) Log.d(logTag, "Exited because opMode is no longer active");
        return stateTimedOut | targetTravelCompleted | !autonOpMode.opModeIsActive();
    }

    protected boolean rotateExitTest(){
        boolean stateTimedOut = stopWatchState.getElapsedTimeMillis() > stateTimeLimit;
        boolean targetRotationCompleted = isTargetRotationSustained();

        if (stateTimedOut) Log.d(logTag, "Exited because timed out. ");
        if (targetRotationCompleted) Log.d(logTag, "Exited because rotation angle achieved.  " +
                        "Target: " + String.format(oneDec, targetPose.getHeadingDeg()) +
                        " Achieved: " + String.format(oneDec, currentPose.getHeadingDeg()) +
                        " Heading error: " + String.format(oneDec, poseError.getHeadingErrorDeg()));
        if (!autonOpMode.opModeIsActive()) Log.d(logTag, "Exited because opMode is no longer active");
        return stateTimedOut | targetRotationCompleted | !autonOpMode.opModeIsActive();
    }

    protected void updateLocationAndError(){
        boolean debugOn = false;

        // Manage the loop duration for integrator error term
        loopDuration = stopWatchLoop.getElapsedTimeMillis();
        stopWatchLoop.reset();

        if (travelDistance > 0.0) {
            FieldPosition fieldPositionChange = calculateChangeInFieldPosition();
            // offset currentPose field position
            currentPose.getFieldPosition().offsetInPlace(fieldPositionChange);
        }

        // update heading with last imu reading
        double newHeading = ebotsImu.getCurrentFieldHeadingDeg(false);
        currentPose.setHeadingDeg(newHeading);

        // refresh the pose error
        poseError.calculateError(currentPose, targetPose, loopDuration);


        if (debugOn){
            Log.d(logTag, currentPose.toString());
            Log.d(logTag, poseError.toString());
        }
    }

    private FieldPosition calculateChangeInFieldPosition(){
        // get the current avg clicks and figure out translation distance
        int currentAvgClicks = motionController.getAverageClicks();
        double translationInches = (currentAvgClicks - lastAvgClicks) / motionController.getClicksPerInch();

        // calculate change in field position
        // this is a simplified calculation and only works in the 4 cardinal directions
        double xTravel = Math.cos(Math.toRadians(travelDirectionDeg)) * translationInches;
        double yTravel = Math.sin(Math.toRadians(travelDirectionDeg)) * translationInches;
        FieldPosition currentTravel = new FieldPosition(xTravel, yTravel);

        // update last average clicks
        lastAvgClicks = currentAvgClicks;

        return currentTravel;
    }

    public boolean isTargetFieldPositionAchieved(){
        double currentPositionError = poseError.getMagnitude();
        return currentPositionError <= accuracy.getPositionalAccuracy();
    }
    public boolean isTargetRotationAchieved(){
        double allowableHeadingErrorDeg = accuracy.getHeadingAccuracyDeg();
        double headingError = poseError.getHeadingErrorDeg();
        boolean headingAchieved =  Math.abs(headingError) <= allowableHeadingErrorDeg;
        return headingAchieved;
    }

    private boolean isTargetRotationSustained(){
        boolean rotationAchieved = isTargetRotationAchieved();
        boolean sustainedVerdict = false;
        if (rotationAchieved && wasRotationAchieved){
            // if currently achieved and previously achieved
            sustainedVerdict = true;
            wasRotationAchieved = true;
        } else if(rotationAchieved){
            // is currently achieved but not previously achieved
            wasRotationAchieved = true;
        } else{
            // not currently achieved
            wasRotationAchieved = false;
        }
        return sustainedVerdict;
    }
}
