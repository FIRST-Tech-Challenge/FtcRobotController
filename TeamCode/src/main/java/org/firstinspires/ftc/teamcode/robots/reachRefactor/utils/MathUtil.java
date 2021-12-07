package org.firstinspires.ftc.teamcode.robots.reachRefactor.utils;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.FFConstants.*;

public class MathUtil {
    public static int metersToTicks(double meters) {
        double circumference = 2 * Math.PI * WHEEL_RADIUS;
        double revolutions = meters / circumference;
        return (int) (revolutions * DRIVETRAIN_TICKS_PER_REVOLUTION);
    }

    public static double ticksToMeters(double ticks) {
        double revolutions = ticks / DRIVETRAIN_TICKS_PER_REVOLUTION;
        double circumference = 2 * Math.PI * WHEEL_RADIUS;
        return revolutions * circumference;
    }

    public static double wrapAngle(double angle1, double angle2){
        return (angle1 + angle2) % 360;
    }

    public static double wrapAngleMinus(double angle1, double angle2) {
        return 360-((angle1 + angle2) % 360);
    }

    public static double deadZone(double x, double threshold) {
        return Math.abs(x) < threshold ? 0 : x;
    }

    public static boolean approxEquals(double x, double y) {
        return Math.abs(x - y) < EPSILON;
    }

    public static double max(double... values) {
        double max = Double.NEGATIVE_INFINITY;
        for(double value : values)
            if(value > max)
                max = value;
        return max;
    }
}