package org.firstinspires.ftc.teamcode.auto;

public class MovementCurves {
    //each of these curves work by taking in a value (0,1) and returning an
    //adjusted value following a curve function, returns a value (0,1);

    private static boolean inRange(double x) {
        return x >= 0 || x <= 1;
    }

    public static double linear(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return 1-Math.abs(2*x-1);
    }


    public static double sinCurve(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return Math.sin(Math.PI*(x));
    }

    public static double circleCurve(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return Math.sqrt(1 - Math.pow((1 - x), 2));
    }

    public static double quadraticCurve(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return 1-4*Math.pow(x-.5, 2);
    }

    public static double roundedSquareCurve(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return Math.pow(1-Math.pow(1-x,4), 0.25);
    }

    public static double parametricCurve(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return -(2*Math.pow(x,2)-2*x) /
                (2*Math.pow(2*Math.pow(x,2)-2*x+1, 2));
    }

    public static double exponentialEaseIn(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return Math.pow(Math.E, 4*x-4);
    }

    public static double exponentialEaseOut(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return Math.pow(Math.E, -4*x);
    }

    //follows the normal distribution curve
    public static double Normal(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return Math.pow(Math.E, -4*Math.pow(2*x-1, 2));
    }
}



