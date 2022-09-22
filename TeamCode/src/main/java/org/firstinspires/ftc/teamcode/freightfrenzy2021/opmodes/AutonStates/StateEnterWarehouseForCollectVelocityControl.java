package org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.AutonStates;

import android.util.Log;

import org.firstinspires.ftc.teamcode.ebotsenums.BucketState;
import org.firstinspires.ftc.teamcode.ebotsenums.Speed;
import org.firstinspires.ftc.teamcode.ebotsutil.Pose;
import org.firstinspires.ftc.teamcode.ebotsutil.PoseError;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.manips2021.Bucket;
import org.firstinspires.ftc.teamcode.freightfrenzy2021.opmodes.EbotsAutonOpMode;

public class StateEnterWarehouseForCollectVelocityControl extends EbotsAutonStateVelConBase{
    private static double stateUndoTravelDistance;

    Pose startPose;
    public StateEnterWarehouseForCollectVelocityControl(EbotsAutonOpMode autonOpMode){
        super(autonOpMode);
        Log.d(logTag, "Entering " + this.getClass().getSimpleName() + " constructor");

        // Must define
        startPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeadingDeg());
        motionController.setSpeed(Speed.MEDIUM);
        travelDistance = 38.0;
        stateUndoTravelDistance = travelDistance;
        travelDirectionDeg = 0.0;
        targetHeadingDeg = 0.0;

        initAutonState();
        setDriveTarget();

        Bucket bucket = Bucket.getInstance(autonOpMode);
        bucket.setState(BucketState.COLLECT);

        Log.d(logTag, "Constructor complete");

    }

    public static double getStateUndoTravelDistance(){
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

        // Now pass the strafe clicks to the opmode for processing
        // calculate the distance traveled using poseError
        PoseError distanceTraveledPoseError = new PoseError(startPose, currentPose, autonOpMode);
        stateUndoTravelDistance = distanceTraveledPoseError.getMagnitude();

        Log.d(logTag, "Just set stateUndoTravelDistance to " + String.format("%.2f", stateUndoTravelDistance));

        telemetry.addLine("Exiting " + this.getClass().getSimpleName());
        telemetry.update();
    }
}
