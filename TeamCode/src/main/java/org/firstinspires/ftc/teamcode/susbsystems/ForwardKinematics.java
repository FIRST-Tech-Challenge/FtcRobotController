package org.firstinspires.ftc.teamcode.susbsystems;

import static org.firstinspires.ftc.teamcode.Constants.TURRENT_TO_CAMERA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Forward kinematics for the arm geometry.
 *
 * Current arm position is the pre-wrist point. End effector pose is the
 * treatment nozzle/tip after applying the wrist model.
 */
class ForwardKinematics {
    private ForwardKinematics() {
    }

    static Position computeArmPosition(
            double extensionLength,
            double shoulderLength,
            double shoulderHeight,
            Position turretOffset) {
        Position armPosition = new Position(DistanceUnit.INCH, 0, 0, 0, System.nanoTime());

        // x left/right of robot, right is positive. More negative X means farther out.
        armPosition.x = -(extensionLength + shoulderLength - turretOffset.x) + TURRENT_TO_CAMERA.x + 10;
        armPosition.y = turretOffset.y + TURRENT_TO_CAMERA.y;
        armPosition.z = shoulderHeight + TURRENT_TO_CAMERA.z;
        return armPosition;
    }

    static Position computeEndEffectorPose(
            double extensionLength,
            double shoulderLength,
            double shoulderHeight,
            double wristLength,
            double wristHeight,
            Position turretOffset) {
        Position preWrist = computeArmPosition(extensionLength, shoulderLength, shoulderHeight, turretOffset);

        Position endEffectorPose = new Position(DistanceUnit.INCH, 0, 0, 0, System.nanoTime());
        // Wrist length is signed. Negative wrist length means farther out in robot X.
        endEffectorPose.x = preWrist.x + wristLength;
        endEffectorPose.y = preWrist.y;
        endEffectorPose.z = preWrist.z + wristHeight;
        return endEffectorPose;
    }
}
