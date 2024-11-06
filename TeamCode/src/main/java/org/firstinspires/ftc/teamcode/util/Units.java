package org.firstinspires.ftc.teamcode.util;

public class Units {
    private static final double INCHES_PER_METER = 39.3701;

    private static final double INCHES_PER_MM = 0.0393701;

    public static double inchesToMeters(double inches) {
        return inches / INCHES_PER_METER;
    }

    public static double inchesToMMs(double inches) {
        return inches / INCHES_PER_MM;
    }

    public static double metersToInches(double meters) {
        return meters * INCHES_PER_METER;
    }

    public static double degreesToRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double radiansToDegrees(double radians) {
        return Math.toDegrees(radians);
    }

    public static double normalizeAngleDifference(double diff) {
        // Normalize to -π to π
        if (diff > Math.PI) {
            diff -= 2 * Math.PI;
        } else if (diff < -Math.PI) {
            diff += 2 * Math.PI;
        }

        return diff;
    }
}
