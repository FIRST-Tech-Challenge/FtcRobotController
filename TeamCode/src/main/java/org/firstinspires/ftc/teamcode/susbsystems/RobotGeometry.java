package org.firstinspires.ftc.teamcode.susbsystems;

import static org.firstinspires.ftc.teamcode.Constants.TURRENT_TO_CAMERA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * Shared physical geometry helpers.
 *
 * Keep coordinate sign conventions here so FK, IK, automation, and telemetry
 * use the same robot model.
 */
class RobotGeometry {
    static final double SHOULDER_RADIUS = 19;
    static final double TURRET_LENGTH = 10.2362;
    static final double TEST_IK_TREATMENT_Y_WINDOW = 5;
    static final double TEST_IK_WRIST_RADIUS = 7.338;
    // Geometry pose for a level/horizontal nozzle. The command angle is calculated from shoulder angle by Wrist.
    static final double TEST_IK_LEVEL_WRIST_POSE_ANGLE = 183.88;
    // Test IK uses the calibrated wrist angle range: vertical is about 93.85, horizontal is about 183.88.
    static final double TEST_IK_WRIST_MIN_ANGLE = 93.85;
    static final double TEST_IK_WRIST_MAX_ANGLE = 183.88;
    static final double TEST_IK_WRIST_ANGLE_STEP = 5;
    // Test IK should not command outside the mechanically verified shoulder range.
    static final double TEST_IK_SHOULDER_MIN_ANGLE = -60;
    static final double TEST_IK_SHOULDER_MAX_ANGLE = 40;
    // Temporary camera correction: camera reads about 2 inches too far in negative X.
    static final double TEST_IK_CAMERA_X_CORRECTION = 3.25;
    static final int[] TEST_IK_TURRET_DEGREES = {45, 90, 135, 180, 225};

    private RobotGeometry() {
    }

    static Position turretOffset(double turretDegrees) {
        double turretRadians = Math.toRadians(turretDegrees);
        return new Position(
                DistanceUnit.INCH,
                TURRET_LENGTH * Math.cos(turretRadians),
                TURRET_LENGTH * Math.sin(turretRadians),
                0,
                System.nanoTime());
    }

    static double turretTargetY(double turretDegrees) {
        return turretOffset(turretDegrees).y + TURRENT_TO_CAMERA.y;
    }

    static double turretTargetX(double turretDegrees) {
        return turretOffset(turretDegrees).x + TURRENT_TO_CAMERA.x;
    }

    static Position correctedIKTarget(Position cameraTarget) {
        return new Position(
                DistanceUnit.INCH,
                cameraTarget.x + TEST_IK_CAMERA_X_CORRECTION,
                cameraTarget.y,
                cameraTarget.z,
                System.nanoTime());
    }

    static double shoulderMinAngle(double shoulderMinHeight) {
        return Math.toDegrees(Math.asin(shoulderMinHeight / SHOULDER_RADIUS));
    }

    static double shoulderMaxAngle(double shoulderMaxHeight) {
        return Math.toDegrees(Math.asin(shoulderMaxHeight / SHOULDER_RADIUS));
    }

    static double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }
}
