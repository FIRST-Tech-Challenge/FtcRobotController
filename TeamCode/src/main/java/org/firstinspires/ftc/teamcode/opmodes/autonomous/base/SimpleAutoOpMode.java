package org.firstinspires.ftc.teamcode.opmodes.autonomous.base;

import com.arcrobotics.ftclib.geometry.Pose2d;

/**
 * Simple non PID auto up mode. This class implements the {@link #updateTargetPose(Pose2d, Pose2d)} to
 * use a simple kP to calculate the target position
 */
public abstract class SimpleAutoOpMode extends AutoOpMode {

    @Override
    protected void updateTargetPose(Pose2d currentPose, Pose2d desiredPose) {
        double dx = desiredPose.getX() - currentPose.getX();
        double dy = desiredPose.getY() - currentPose.getY();
        double dtheta = desiredPose.getRotation().minus(currentPose.getRotation()).getRadians();

        // Use a simple P controller to move towards the desired pose
        double kP = 0.5; // TODO: [tune]

        targetX = dx * kP;
        targetY = dy * kP;
        targetRotationSpeed = rotateAngleToSpeed(dtheta * kP);
    }
}
