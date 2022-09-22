package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Intake;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateUndoCollectTravelWithVelocityControl extends EbotsAutonStateVelConBase{

    public StateUndoCollectTravelWithVelocityControl(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");

        // Reverse the intake during this movement
        Intake.getInstance(autonOpMode.hardwareMap).purge();


        // Must define



        travelDistance = StateCollectFreightWithVelocityControl.getStateUndoTravelDistance();
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
        // Stop the intake purge
        Intake.getInstance(autonOpMode.hardwareMap).stop();
        telemetry.addLine("Exiting " + this.getClass().getSimpleName());
        telemetry.update();
    }
}
