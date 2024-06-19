package org.rustlib.utils;

public class MathHelpers {
    public static double interpolate(double t, double first, double second) {
        return first + t * (second - first);
    }

    public static double getTValue(double first, double second, double position) {
        return (position - first) / (second - first);
    }
}
