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
    static final double SHOULDER_RADIUS = 21.5;
    static final double TURRET_LENGTH = 10.2362;
    static final double TEST_IK_TREATMENT_Y_WINDOW = 5;
    static final double TEST_IK_WRIST_RADIUS = 7.338;
    // Geometry pose for a level/horizontal nozzle. The command angle is calculated from shoulder angle by Wrist.
    static final double TEST_IK_LEVEL_WRIST_POSE_ANGLE = 183.33;
    // Test IK uses the calibrated wrist angle range: vertical is about 93.85, horizontal is about 183.33.
    static final double TEST_IK_WRIST_MIN_ANGLE = 93.85;
    static final double TEST_IK_WRIST_MAX_ANGLE = 183.33;
    static final double TEST_IK_WRIST_ANGLE_STEP = 5;
    // Test IK should not command outside the mechanically verified shoulder range.
    static final double TEST_IK_SHOULDER_MIN_ANGLE = -60;
    static final double TEST_IK_SHOULDER_MAX_ANGLE = 40;
    static final double TEST_IK_SHOULDER_STEP_DEGREES = 0.5;
    static final double TEST_IK_Z_TOLERANCE = 0.5;
    // Temporary camera correction: camera reads about 2 inches too far in negative X.
    static final double TEST_IK_CAMERA_X_CORRECTION = 3.25;
    // Temporary camera/geometry correction: measured FK ends about 1 inch high,
    // so lower the IK target by 1 inch until the physical Z reference is retuned.
    static final double TEST_IK_CAMERA_Z_CORRECTION = 0.0;
    // Continuous turret search for IK. Keep the first pass in the proven working range.
    static final double TEST_IK_TURRET_MIN_DEGREES = 45;
    static final double TEST_IK_TURRET_MAX_DEGREES = 225;
    static final double TEST_IK_TURRET_STEP_DEGREES = 5;

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

    static double levelWristLength() {
        // The wrist command angle changes to cancel shoulder motion, but the
        // nozzle is intended to stay level. Use this world-space projection for
        // IK/FK instead of splitting the command angle into length and height.
        return TEST_IK_WRIST_RADIUS * Math.cos(Math.toRadians(TEST_IK_LEVEL_WRIST_POSE_ANGLE));
    }

    static double levelWristHeight() {
        // Small calibrated vertical offset of the level nozzle pose.
        return TEST_IK_WRIST_RADIUS * Math.sin(Math.toRadians(TEST_IK_LEVEL_WRIST_POSE_ANGLE));
    }

    static Position correctedIKTarget(Position cameraTarget) {
        return new Position(
                DistanceUnit.INCH,
                cameraTarget.x + TEST_IK_CAMERA_X_CORRECTION,
                cameraTarget.y,
                cameraTarget.z + TEST_IK_CAMERA_Z_CORRECTION,
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
