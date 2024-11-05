package org.firstinspires.ftc.teamcode.opmodes.autonomous.movement;

import com.arcrobotics.ftclib.geometry.Pose2d;

public class ForwardMovement extends BaseMovement {

    public ForwardMovement(double distance) {
        super(distance);
    }

    @Override
    protected void updateDrivingParameters() {
        strafeSpeed = isCloseToEnd() || isJustStarted() ? 0 : getMoveErrorCorrection(previousPose.getY() - startingPose.getY());
        forwardSpeed = isCloseToEnd() || isJustStarted() ? .2 : getMainMovement(distance-accumulatedChanges);
        rotationSpeed = isCloseToEnd() || isJustStarted() ? 0 : getRotErrorCorrection(startingPose.getHeading() - previousPose.getHeading());
    }

    @Override
    protected double getCurrentDrivingReading(Pose2d pose) {
        return pose.getX();
    }
}
