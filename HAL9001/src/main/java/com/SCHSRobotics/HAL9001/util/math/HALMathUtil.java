package com.SCHSRobotics.HAL9001.util.math;

import static java.lang.Math.round;

/**
 * A static math utility class for miscellaneous math operations used in HAL.
 * <p>
 * Creation Date: 5/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.1.0
 */
public class HALMathUtil {
    //A constant used to fix floating point values.
    private static final double FLOATING_POINT_FIXER_CONSTANT = 1e9;

    /**
     * A private constructor for HALMathUtil. Used to make the class static
     */
    private HALMathUtil() {
    }

    /**
     * Calculates x mod m. This correctly supports negative input values, the standard java % operator does not.
     *
     * @param x The input value.
     * @param m The modulus.
     * @return The input mod the modulus.
     */
    public static int mod(int x, int m) {
        return (int) mod((double) x, (double) m);
    }

    /**
     * Calculates x mod m. This correctly supports negative input values, the standard java % operator does not.
     *
     * @param x The input value.
     * @param m The modulus.
     * @return The input mod the modulus.
     */
    public static double mod(double x, int m) {
        return mod(x, (double) m);
    }

    /**
     * Calculates x mod m. This correctly supports negative input values, the standard java % operator does not.
     *
     * @param x The input value.
     * @param m The modulus.
     * @return The input mod the modulus.
     */
    public static int mod(int x, double m) {
        return (int) mod((double) x, m);
    }

    /**
     * Calculates x mod m. This correctly supports negative input values, the standard java % operator does not.
     *
     * @param x The input value.
     * @param m The modulus.
     * @return The input mod the modulus.
     */
    public static double mod(double x, double m) {
        return (x % m + m) % m;
    }

    /**
     * Fixes any floating point errors in the input value.
     *
     * @param value The input value to be fixed.
     * @return The fixed version of the input value.
     */
    public static double floatingPointFix(double value) {
        return round(value * FLOATING_POINT_FIXER_CONSTANT) / FLOATING_POINT_FIXER_CONSTANT;
    }
}