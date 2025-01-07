package org.firstinspires.ftc.teamcode.utils;

public class MathUtils {
    public static final double TWO_PI = Math.PI * 2;
    public static final double HALF_PI = Math.PI / 2;

    public static double normalizeAngle(double angle) {
        return angle - TWO_PI * Math.floor((angle + Math.PI) / TWO_PI);
    }

    public static double round(double num, int digits) {
        return Math.round(num * Math.pow(10, digits)) / Math.pow(10, digits);
    }

    public static double normalize(double value, double lowIn, double highIn, double lowOut, double highOut) {
        return (Math.max(Math.min(value, highIn), lowIn) - lowIn) / (highIn - lowIn) * (highOut - lowOut) + lowOut;
    }

    public static double lawOfCosines(double a, double b, double c) {
        // c = sqrt(a^2 + b^2 - 2ab cos C)
        // c^2 = a^2 + b^2 - 2ab cos C
        // 2ab cos C = a^2 + b^2 - c^2
        // cos C = (a^2 + b^2 - c^2)/2ab
        return Math.acos((a*a + b*b - c*c) / (2*a*b));
    }
}
