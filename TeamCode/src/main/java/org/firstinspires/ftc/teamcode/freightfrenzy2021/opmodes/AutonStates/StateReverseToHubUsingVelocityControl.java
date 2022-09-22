package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotsenums.StartingSide;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateReverseToHubUsingVelocityControl extends EbotsAutonStateVelConBase{


    public StateReverseToHubUsingVelocityControl(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");

        // Must define
        motionController.setSpeed(Speed.FAST);

        StartingSide startingSide = autonOpMode.getStartingSide();
        boolean isCarousel = autonOpMode.getStartingSide() == StartingSide.CAROUSEL;
        boolean isBlue = AllianceSingleton.isBlue();

        if(isCarousel && isBlue) {
            travelDistance = 27.14;
        } else if (isCarousel && !isBlue){
            travelDistance = 26.58;

        } else if(!isCarousel && isBlue) {
            // starting side is Warehouse
            double additionalTravel = 1.0;
            travelDistance = additionalTravel + StateStrafeToAlignTSEVelocityControl.getStateStrafeDistance();
        } else if(!isCarousel && !isBlue) {
            // starting side is Warehouse
            double additionalTravel = -1.0;
            travelDistance = additionalTravel + StateStrafeToAlignTSEVelocityControl.getStateStrafeDistance();
        }
        travelDirectionDeg = isCarousel ? 0.0 : 180.0;
        targetHeadingDeg = isCarousel ? 180.0 : 0.0;

        initAutonState();
        setDriveTarget();

        Log.d(logTag, "Constructor complete");
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
