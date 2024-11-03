package org.firstinspires.ftc.teamcode.util;

public class Units {
    private static final double INCHES_PER_METER = 39.3701;

    public static double inchesToMeters(double inches) {
        return inches / INCHES_PER_METER;
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
}
