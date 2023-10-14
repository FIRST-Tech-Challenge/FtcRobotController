package org.firstinspires.ftc.teamcode.lib.util;

public class Util {
    private static final double kEpsilon = 1E-12;

    private Util() {}

    public static double limit(double x, double maxMagnitude) {
        return limit(x, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double x, double min, double max) {
        return Math.min(max, Math.max(min, x));
    }

    public static boolean inRange(double x, double maxMagnitude) {
        return inRange(x, -maxMagnitude, maxMagnitude);
    }

    public static boolean inRange(double x, double min, double max) {
        return x > min && x < max;
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0d, 1d);
        return a + (b - a) * x;
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static double getEpsilon() {
        return kEpsilon;
    }
}
