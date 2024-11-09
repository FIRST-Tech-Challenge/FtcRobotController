package org.firstinspires.ftc.teamcode.opmodes.autonomous.movement;

import com.arcrobotics.ftclib.geometry.Pose2d;

public class StrafeMovement extends BaseMovement {
    public StrafeMovement(double distance) {
        super(distance);
    }

    @Override
    protected void updateDrivingParameters() {
        strafeSpeed = getMainMovement(distance-accumulatedChanges);
        forwardSpeed = getMoveErrorCorrection(startingPose.getX() - previousPose.getX());
        rotationSpeed = -getRotErrorCorrection(previousPose.getHeading() - startingPose.getHeading()) * directionFactor;
    }

    @Override
    protected double getCurrentDrivingReading(Pose2d pose) {
        return pose.getY();
    }
}
