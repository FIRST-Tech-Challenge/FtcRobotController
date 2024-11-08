package org.firstinspires.ftc.teamcode.opmodes.autonomous.movement;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;

public class ForwardMovement extends BaseMovement {

    public ForwardMovement(double distance) {
        super(distance);
    }

    @Override
    protected void updateDrivingParameters() {
        strafeSpeed = getMoveErrorCorrection(previousPose.getY() - startingPose.getY());
        forwardSpeed = getMainMovement(distance-accumulatedChanges);
        Log.i(LOG_TAG, "error correction speed: " + getRotErrorCorrection(startingPose.getHeading() - previousPose.getHeading()));
        rotationSpeed = getRotErrorCorrection(startingPose.getHeading() - previousPose.getHeading()) * .05;
    }

    @Override
    protected double getCurrentDrivingReading(Pose2d pose) {
        return pose.getX();
    }
}
