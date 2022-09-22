package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsutil.AllianceSingleton;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateStrafeToAllowTurnToAllianceHubVelocityControl extends EbotsAutonStateVelConBase{

    /**
     * Within this constructor the following variables must be set:
     * @implSpec travelDistance
     * @implSpec travelFieldHeading
     * @implSpec targetHeadingDeg

     * @param autonOpMode
     */
    public StateStrafeToAllowTurnToAllianceHubVelocityControl(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");

        // Must define

        travelDistance = StateStrafeToAlignTSEVelocityControl.getStateUndoTravelDistance();
        Log.d(logTag, "travelDistance received from StateStrafeToAlignTSEVelocityControl" +
                String.format(twoDec, travelDistance));
        travelDirectionDeg = 0.0;
        targetHeadingDeg = AllianceSingleton.getDriverFieldHeadingDeg();

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
