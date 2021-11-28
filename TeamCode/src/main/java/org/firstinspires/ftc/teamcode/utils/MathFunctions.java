package org.firstinspires.ftc.teamcode.utils;

public class MathFunctions {

    /** Raises a to the b power while keeping the sign and preventing domain errors.
     * @param a  the base
     * @param b  the exponent
     * @return   the result
     */
    public static double safeSquare(double a, double b) {
        return a * Math.pow(Math.abs(a), b-1);
    }

    /** Clamps a double between to values
     * @param a     The value
     * @param max   The maximum
     * @param min   The minimum
     * @return      The clamped value
     */
    public static double clamp(double a, double max, double min) {
        return Math.max(min, Math.min(a, max));
    }

    public static boolean epsEquals(double a, double b) {
        return Math.abs(a - b) <= 1E-5;
    }

}
