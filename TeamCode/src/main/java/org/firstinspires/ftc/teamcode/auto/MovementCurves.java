package org.firstinspires.ftc.teamcode.auto;

public class MovementCurves {
    //each of these curves work by taking in a value (0,1) and returning an
    //adjusted value following a curve function, returns a value (0,1);

    public static double sinCurve(double x) {
        if(x < 0 || x > 1) {
            x = 0;
        }
        return Math.sin(Math.PI*(x));
    }
    public static double circleArc(double x) {
        if(x < 0 || x > 1) {
            x = 0;
        }
        return Math.sqrt(1 - Math.pow((1 - x), 2));
    }

}
