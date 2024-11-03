package org.firstinspires.ftc.teamcode.opmodes.autonomous.movement;

import com.arcrobotics.ftclib.geometry.Pose2d;

public class StrafeMovement extends BaseMovement {
    public StrafeMovement(double distance) {
        super(distance);
    }

    @Override
    protected void updateDrivingParameters() {
        strafeSpeed = currentSpeed;
        forwardSpeed = getErrorCorrectionSpeed(startingPose.getX() - previousPose.getX());
        rotationSpeed = getErrorCorrectionSpeed(startingPose.getHeading() - previousPose.getHeading());
    }

    @Override
    protected double getCurrentDrivingReading(Pose2d pose) {
        return pose.getY();
    }
}
