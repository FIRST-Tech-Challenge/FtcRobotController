package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.StartingSide;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StatePushOffCarouselWithVelocityControl extends EbotsAutonStateVelConBase{

    private static double stateUndoTravelDistance = 4.0;
    /**
     * Within this constructor the following variables must be set:
     * @implSpec travelDistance
     * @implSpec travelFieldHeading
     * @implSpec targetHeadingDeg

     * @param autonOpMode
     */

    public StatePushOffCarouselWithVelocityControl(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");

        // Must define
        boolean isCarouselSide = autonOpMode.getStartingSide() == StartingSide.CAROUSEL;
        boolean isBlue = AllianceSingleton.isBlue();

        if (isCarouselSide && isBlue){
            travelDistance = 8.0;
        } else if(isCarouselSide && !isBlue){
            travelDistance = 8.0;
        }

        stateUndoTravelDistance = travelDistance;

        travelDirectionDeg = isBlue ? -90.0 : 90.0;
        targetHeadingDeg = isBlue ? AllianceSingleton.getDriverFieldHeadingDeg() : 0.0;

        initAutonState();
        setDriveTarget();

        Log.d(logTag, "Constructor complete");

    }

    public static double getStateUndoTravelDistance() {
        return stateUndoTravelDistance;
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
