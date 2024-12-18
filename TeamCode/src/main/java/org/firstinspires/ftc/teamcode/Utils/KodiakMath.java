package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

public class KodiakMath {
    public static double INCH_PER_METER = 39.3701;

    public static double max(double x1, double x2, double x3, double x4) {
        double max = x1;

        if (x2 > max)
            max = x2;
        if (x3 > max)
            max = x3;
        if (x4 > max)
            return x4;

        return max;
    }

    public static double max(MecanumDriveWheelSpeeds wheelSpeeds) {
        double max = max(wheelSpeeds.frontLeftMetersPerSecond, wheelSpeeds.frontRightMetersPerSecond, wheelSpeeds.rearLeftMetersPerSecond, wheelSpeeds.rearRightMetersPerSecond);

        return max;
    }

    public static int minIndex(char[] array) {
        char minIndex = 0;
        for (char i = 1; i < array.length; i++) {
            if (array[i] < array[minIndex])
                minIndex = i;
        }
        return (int) minIndex;
    }

    public static double toInches(double distance_m) {
        return distance_m * INCH_PER_METER;
    }

    public static double toMeters(double distance_in) {
        return distance_in / INCH_PER_METER;
    }

    public static MecanumDriveWheelSpeeds multiply(MecanumDriveWheelSpeeds wheelSpeeds, double multiplier) {
        return new MecanumDriveWheelSpeeds(
                wheelSpeeds.frontLeftMetersPerSecond * multiplier,
                wheelSpeeds.frontRightMetersPerSecond * multiplier,
                wheelSpeeds.rearLeftMetersPerSecond * multiplier,
                wheelSpeeds.rearRightMetersPerSecond * multiplier
        );
    }

    public static double getAverage(double a, double b) {
        return (a + b) / 2.0;
    }

    public static int between(int x, int min, int max) {
        if (x > max)
            return max;
        else
            return Math.max(x, min);
    }

    public static double between(double x, double min, double max) {
        if (x > max)
            return max;
        else
            return Math.max(x, min);
    }

    public static boolean isBetween(double x, double min, double max) {
        return (min < x) && (x < max);
    }

    public static double roundNumber(double a) {
        return Math.round(a * 100000) / 100000.0;
    }
}
