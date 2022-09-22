package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.BarCodePosition;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateStrafeToAlignTSEVelocityControl extends EbotsAutonStateVelConBase{

    // this static class must be created specific for this state, otherwise will fall to abstract class
    private static double stateUndoTravelDistance = 0.0;
    private static double stateStrafeDistance = 4.2;

    /**
     * Within this constructor the following variables must be set:
     * @implSpec travelDistance
     * @implSpec travelFieldHeading
     * @implSpec targetHeadingDeg

     * @param autonOpMode
     */
    public StateStrafeToAlignTSEVelocityControl(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");

        // Must define

        BarCodePosition barCodePosition = autonOpMode.getBarCodePosition();
        double multiplier = 0.0;
        if (barCodePosition == BarCodePosition.MIDDLE){
            multiplier = 1.0;
        }

        Log.d(logTag, "In StateStrafeToAlignTSEVelocityControl barCodePosition: " + barCodePosition.name() +
                " multiplier: " + String.format(oneDec, multiplier) +
                " travelDistance: " + String.format(twoDec, stateStrafeDistance * multiplier));

        travelDistance = stateStrafeDistance * multiplier;

        stateUndoTravelDistance = stateStrafeDistance - travelDistance;
        travelDirectionDeg = 0.0;
        targetHeadingDeg = AllianceSingleton.getDriverFieldHeadingDeg();


        initAutonState();
        setDriveTarget();

        Log.d(logTag, "Constructor complete");

    }

    // Must be created specific for this class to not invoke abstract class method
    public static double getStateUndoTravelDistance() {
        return stateUndoTravelDistance;
    }

    public static double getStateStrafeDistance() {
        return stateStrafeDistance;
    }

    @Override
    public boolean shouldExit() {
        return super.shouldExit();
    }

    @Override
    public void performStateActions() {
        super.performStateActions();
    }

    @Override
    public void performTransitionalActions() {
        super.performTransitionalActions();
    }

}
