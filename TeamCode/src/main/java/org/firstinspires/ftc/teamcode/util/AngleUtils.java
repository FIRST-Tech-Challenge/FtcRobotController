package org.firstinspires.ftc.teamcode.util;

public class AngleUtils {
    /**
     * Returns the minimal difference between two angles (in radians), wrapped to [-PI, PI].
     */
    public static double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}
