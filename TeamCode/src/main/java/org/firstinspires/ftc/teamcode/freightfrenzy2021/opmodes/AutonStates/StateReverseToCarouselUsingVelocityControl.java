package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotsenums.StartingSide;
import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateReverseToCarouselUsingVelocityControl extends EbotsAutonStateVelConBase{


    private final boolean isCarouselSide;
    private final boolean isBlue;

    public StateReverseToCarouselUsingVelocityControl(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");

        // Must define
        motionController.setSpeed(Speed.MEDIUM);
        isCarouselSide = autonOpMode.getStartingSide() == StartingSide.CAROUSEL;
        isBlue = AllianceSingleton.isBlue();
        if(isCarouselSide && isBlue) {
            travelDistance = 17.38;
        } else if (isCarouselSide && !isBlue) {
            travelDistance = 10.22;
        }

        travelDirectionDeg = 180.0;
        targetHeadingDeg = 0.0;

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
