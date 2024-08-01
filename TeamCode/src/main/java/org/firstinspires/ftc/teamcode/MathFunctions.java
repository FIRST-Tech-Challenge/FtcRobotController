package org.firstinspires.ftc.teamcode;

public class MathFunctions {
    public static double angleWrapDeg(double angle) {
        double moddedAngle = angle % 360;
        while (moddedAngle > 180) {
            return moddedAngle - 360;
        }
        while (moddedAngle <= -180) {
            return moddedAngle + 360;
        }
        return moddedAngle;
    }
    public static double angleWrapRad(double angle) {
        double moddedAngle = angle % (2 * Math.PI);
        while (moddedAngle > Math.PI) {
            return moddedAngle -= (2 * Math.PI);
        }
        while (moddedAngle <= -Math.PI) {
            return moddedAngle + (2 * Math.PI);
        }
        return moddedAngle;
    }
}
