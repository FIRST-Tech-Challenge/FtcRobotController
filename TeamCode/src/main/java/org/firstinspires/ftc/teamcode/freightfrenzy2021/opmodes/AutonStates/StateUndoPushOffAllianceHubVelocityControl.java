package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotsenums.StartingSide;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateUndoPushOffAllianceHubVelocityControl extends EbotsAutonStateVelConBase{

    /**
     * Within this constructor the following variables must be set:
     * @implSpec travelDistance
     * @implSpec travelFieldHeading
     * @implSpec targetHeadingDeg

     * @param autonOpMode
     */
    public StateUndoPushOffAllianceHubVelocityControl(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");

        // Must define

        motionController.setSpeed(Speed.SLOW);
        travelDistance = StatePushOffWithVelocityControl.getTravelDistance();
        travelDirectionDeg = autonOpMode.getStartingSide() == StartingSide.CAROUSEL ? 0.0 : 180.0;
        targetHeadingDeg = autonOpMode.getStartingSide() == StartingSide.CAROUSEL ? 180.0 : 0.0;

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
