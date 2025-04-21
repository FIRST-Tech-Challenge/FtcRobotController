package org.firstinspires.ftc.team00000.v2.util;

public final class MathUtil {
    private MathUtil() {}

    public static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
