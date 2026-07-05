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
    static final double TEST_IK_WRIST_MIN_ANGLE = 0;
    static final double TEST_IK_WRIST_MAX_ANGLE = 90;
    static final double TEST_IK_WRIST_ANGLE_STEP = 5;
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
