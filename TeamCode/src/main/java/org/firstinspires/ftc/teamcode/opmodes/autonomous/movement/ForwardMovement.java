package org.firstinspires.ftc.teamcode.opmodes.autonomous.movement;

import com.arcrobotics.ftclib.geometry.Pose2d;

public class ForwardMovement extends BaseMovement {

    public ForwardMovement(double distance) {
        super(distance);
    }

    @Override
    protected void updateDrivingParameters() {
        strafeSpeed = getMoveErrorCorrection(previousPose.getY() - startingPose.getY());
        forwardSpeed = getMainMovement(distance-accumulatedChanges);
        rotationSpeed = getRotErrorCorrection(startingPose.getHeading() - previousPose.getHeading());
    }

    @Override
    protected double getCurrentDrivingReading(Pose2d pose) {
        return pose.getX();
    }
}
