package org.firstinspires.ftc.purepursuit;

public class MathFunctions {

    public static double angleWrap(double angle) {
        while(angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while(angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }
}
