package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.BucketState;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotssensors.EbotsBlinkin;
import org.firstinspires.ftc.teamcode.ebotsutil.Pose;
import org.firstinspires.ftc.teamcode.ebotsutil.PoseError;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Bucket;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Intake;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.opencvpipelines.FreightDetector;
import org.openftc.easyopencv.OpenCvCamera;

public class StateEnterWarehouseAndCollectFreightWithVelocityControl extends EbotsAutonStateVelConBase{
    FreightDetector freightDetector;
    boolean freightPresent = false;
    OpenCvCamera camera;
    private static boolean exitedWithFreight = false;

    private static double stateUndoTravelDistance = 0.0;
    private final Pose startPose;

    private boolean isSpeedSlow = false;
    private int ballSightingScore = 0;
    private int boxSightingScore = 0;
    private int sightingThreshold = 2;
    private final double enterWarehouseDistance = 24.0;
    private final double collectDistance = 36.0;



    public StateEnterWarehouseAndCollectFreightWithVelocityControl(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");

        // Make sure the bucket is ready for collect
        Bucket bucket = Bucket.getInstance(autonOpMode);
        if (bucket.getBucketState() != BucketState.COLLECT) {
            bucket.setState(BucketState.COLLECT);
        }

        exitedWithFreight = false;

        startPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeadingDeg());

        // Must define



        travelDistance = enterWarehouseDistance + collectDistance;
        travelDirectionDeg = 0.0;
        targetHeadingDeg = 0.0;

        initAutonState();
        setDriveTarget();

        stateTimeLimit = 5500;

        Intake.getInstance(autonOpMode.hardwareMap).fullPower();
        freightDetector = autonOpMode.getFreightDetector();

        // turn on the lights
        EbotsBlinkin.getInstance(autonOpMode.hardwareMap).lightsOn();

        Log.d(logTag, "Constructor complete");

    }

    public static double getStateUndoTravelDistance() {
        return stateUndoTravelDistance;
    }

    public static boolean getExitedWithFreight(){
        return exitedWithFreight;
    }

    private void updateFreightPresent(){
        if (!freightDetector.isReadingConsumed()) {
            freightDetector.markReadingAsConsumed();
            if (freightDetector.getIsBall()) {
                Log.d(logTag, "!!! Ball Detected !!!");
                ballSightingScore++;
            } else {
                ballSightingScore--;
                ballSightingScore = Math.max(ballSightingScore, 0);
            }

            if (freightDetector.getIsBox()) {
                Log.d(logTag, "!!! Box Detected !!!");
                boxSightingScore++;
            } else {
                boxSightingScore--;
                boxSightingScore = Math.max(boxSightingScore, 0);
            }
            freightPresent = ballSightingScore >= sightingThreshold | boxSightingScore >= sightingThreshold;

        }
        telemetry.addData("Box Present", freightDetector.getIsBox());
        telemetry.addData("Ball Present", freightDetector.getIsBall());
    }

    @Override
    public boolean shouldExit() {
        // standardExitConditions include opMode inactivated, travel complete, state timed out.
        updateLocationAndError();
        boolean freightDetectionLockOut = stopWatchState.getElapsedTimeMillis() <= 500;
        if (!freightDetectionLockOut) updateFreightPresent();


        boolean stateTimedOut = stopWatchState.getElapsedTimeMillis() > stateTimeLimit;
        boolean targetTravelCompleted = isTargetFieldPositionAchieved() && isTargetRotationAchieved();
        if (stateTimedOut) Log.d(logTag, "Exited because timed out. ");
        if (targetTravelCompleted) Log.d(logTag, "Target travel completed but not exiting. ");

        if(freightPresent) {
            Log.d(logTag, "Exiting because freight detected during " + this.getClass().getSimpleName());
            exitedWithFreight = true;
        }

        return freightPresent | stateTimedOut;
    }

    @Override
    public void performStateActions() {
        super.performStateActions();
        double distanceTraveled = new PoseError(startPose, currentPose, autonOpMode).getMagnitude();

        if (!isSpeedSlow && distanceTraveled > enterWarehouseDistance) {
            motionController.setSpeed(Speed.SLOW);
            Log.d(logTag, "Shifted to slower for collect " +
                    String.format(twoDec, distanceTraveled) +
                    " of target travel " + String.format(twoDec, travelDistance));
            isSpeedSlow = true;
        }

    }

    @Override
    public void performTransitionalActions() {
        super.performTransitionalActions();
        EbotsBlinkin.getInstance(autonOpMode.hardwareMap).lightsOff();

        // move the bucket to travel position
        Bucket.getInstance(autonOpMode).setState(BucketState.TRAVEL);

        Intake.getInstance(autonOpMode.hardwareMap).stop();

        stateUndoTravelDistance = new PoseError(startPose, currentPose, autonOpMode).getMagnitude();
        Log.d(logTag, "Setting stateUndoTravelDistance to: " + String.format(twoDec, stateUndoTravelDistance));

        // if failed to collect freight, then clear the remaining autonstates and just park
        if(!exitedWithFreight){
            autonOpMode.clearRemainingItinerary();
            Log.d(logTag, "Exiting state without freight, itinerary cleared!");
        }

        telemetry.addLine("Exiting " + this.getClass().getSimpleName());
        telemetry.update();
    }
}
