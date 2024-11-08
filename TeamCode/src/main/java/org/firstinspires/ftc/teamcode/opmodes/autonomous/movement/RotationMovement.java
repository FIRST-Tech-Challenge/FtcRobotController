package org.firstinspires.ftc.teamcode.opmodes.autonomous.movement;

import com.arcrobotics.ftclib.geometry.Pose2d;

public class RotationMovement extends BaseMovement {
    public RotationMovement(double distance) {
        super(distance);
    }

    @Override
    protected void updateDrivingParameters() {
        rotationSpeed = getMainMovement(distance-accumulatedChanges);
        strafeSpeed = 0;
        forwardSpeed = 0;

    }

    @Override
    protected double getCurrentDrivingReading(Pose2d pose) {
        return pose.getRotation().getRadians();
    }

//    @Override
//    protected double getNormalSpeed() {
//        return .;
//    }
//
//    @Override
//    protected double getSlowSpeed() {
//        return .1;
//    }
//
//    @Override
//    protected double getErrorCorrectionSpeed() {
//        return .1;
//    }
}
