package org.firstinspires.ftc.team12397.v2.cameraSoftware.util;

public final class MathUtil {
    private MathUtil() {}

    public static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
