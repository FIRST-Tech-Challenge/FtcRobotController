package org.firstinspires.ftc.teamcode.opmodes.autonomous.movement;

import com.arcrobotics.ftclib.geometry.Pose2d;

public class StrafeMovement extends BaseMovement {
    public StrafeMovement(double distance) {
        super(distance);
    }

    @Override
    protected void updateDrivingParameters() {
        strafeSpeed = isCloseToEnd() || isJustStarted() ? .2 : getMainMovement(distance-accumulatedChanges);
        forwardSpeed = isCloseToEnd() || isJustStarted() ? 0 : getMoveErrorCorrection(startingPose.getX() - previousPose.getX());
        rotationSpeed = isCloseToEnd() || isJustStarted() ? 0 : getRotErrorCorrection(startingPose.getHeading() - previousPose.getHeading());
    }

    @Override
    protected double getCurrentDrivingReading(Pose2d pose) {
        return pose.getY();
    }
}
