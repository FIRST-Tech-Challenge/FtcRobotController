package org.firstinspires.ftc.teamcode.shared.util;

public class MathUtil {
    /**
     * Takes in a value and clamps it between a min and max value.
     * @param value - The value to be clamped.
     * @param min - The minimum value.
     * @param max - The maximum value.
     * @return A value between min and max inclusive.
     */
    public static double clamp(double value, double min, double max) {
        if(value >= max) {
            return max;
        } else if(value <= min) {
            return min;
        } else {
            return value;
        }
    }
}
