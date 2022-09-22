package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotsutil.Pose;
import org.firstinspires.ftc.teamcode.ebotsutil.PoseError;
import org.firstinspires.ftc.teamcode.ebotsutil.StopWatch;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Intake;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateUndoEnterWarehouseAndCollectFreightWithVelocityControl extends EbotsAutonStateVelConBase{
    private StopWatch stopWatchPurge = new StopWatch();
    private long purgeTimeLimit = 1000;
    boolean purgeEnded = false;
    boolean isSpeedSlow = false;
    private final Pose startPose;


    public StateUndoEnterWarehouseAndCollectFreightWithVelocityControl(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");

        startPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeadingDeg());

        // Must define

        travelDistance = StateEnterWarehouseAndCollectFreightWithVelocityControl.getStateUndoTravelDistance();
        travelDirectionDeg = 180.0;
        targetHeadingDeg = 0.0;

        initAutonState();
        setDriveTarget();

        Intake.getInstance(autonOpMode.hardwareMap).purge();

        Log.d(logTag, "Constructor complete");
    }


    @Override
    public boolean shouldExit() {
        return super.shouldExit();
    }

    @Override
    public void performStateActions() {
        super.performStateActions();
        if (!purgeEnded && stopWatchPurge.getElapsedTimeMillis() > purgeTimeLimit){
            Intake.getInstance(autonOpMode.hardwareMap).stop();
            purgeEnded = true;
            Log.d(logTag, "Intake purge ended " + stopWatchPurge.toString());
        }
        double distanceTraveled = new PoseError(startPose, currentPose, autonOpMode).getMagnitude();

        if (!isSpeedSlow && distanceTraveled > (travelDistance * 0.8)) {
            motionController.setSpeed(Speed.SLOW);
            Log.d(logTag, "Shifted to slower for undo enter warehouse and collect " +
                    String.format(twoDec, distanceTraveled) +
                    " of target travel " + String.format(twoDec, travelDistance));
            isSpeedSlow = true;
        }

    }

    @Override
    public void performTransitionalActions() {
        super.performTransitionalActions();
        if (!purgeEnded) Intake.getInstance(autonOpMode.hardwareMap).stop();
    }
}
