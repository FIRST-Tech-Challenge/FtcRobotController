package org.firstinspires.ftc.teamcode.math;



public class MathFunctions {
    public static double angleWrapRad(double angle) {
        while (angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI){
            angle -= 2 * Math.PI;
        }

        return angle;
    }

    public static double maxAbsValueDouble(double a, double... others) {

        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }

        return Math.abs(max);
    }
}
