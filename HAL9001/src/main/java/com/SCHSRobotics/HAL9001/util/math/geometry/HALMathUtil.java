package com.SCHSRobotics.HAL9001.util.math.geometry;

import com.SCHSRobotics.HAL9001.util.exceptions.ExceptionChecker;

import java.math.BigDecimal;
import java.math.RoundingMode;

public class HALMathUtil {
    public static final double FLOATING_POINT_FIXER_CONSTANT = 1e9;

    private HALMathUtil() {
    }

    public static int mod(int x, int m) {
        return (int) mod((double) x, (double) m);
    }

    public static double mod(double x, int m) {
        return mod(x, (double) m);
    }

    public static double mod(int x, double m) {
        return mod((double) x, m);
    }

    public static double mod(double x, double m) {
        return (x % m + m) % m;
    }

    public static double round(double value, int places) {
        ExceptionChecker.assertTrue(places >= 0, new ArithmeticException("You cannot round to a negative number of decimal places."));

        BigDecimal bd = new BigDecimal(Double.toString(value));
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }
}
