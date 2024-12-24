package org.firstinspires.ftc.teamcode.Hardware.Util;

public class PosChecker {

    public static boolean atAngularPos(double pos1, double pos2, double tolerance) {
        double delta = Math.min(pos1, pos2) - Math.max(pos1, pos2) + 360.00;

        delta = delta > 180.00 ? 360-delta : delta;

        return delta <= tolerance;
    }

    public static boolean atLinearPos(double pos1, double pos2, double tolerance) {
        double delta = Math.abs(pos1 - pos2);

        return delta <= tolerance;
    }
}
