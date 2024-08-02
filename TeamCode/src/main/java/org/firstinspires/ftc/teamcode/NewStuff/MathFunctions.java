package org.firstinspires.ftc.teamcode.NewStuff;

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
}
