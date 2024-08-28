package org.firstinspires.ftc.team417.utilities;

public class FTCMath {
    public final static double EPSILON = 0.0001;
    public static boolean isEpsilonEquals(double a, double b) {
        return (Math.abs(a) + EPSILON >= Math.abs(b) && Math.abs(a) - EPSILON <= Math.abs(b));
    }
}
