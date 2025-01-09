package org.firstinspires.ftc.teamcode.libraries.MovementCurves;

public class MovementCurves {
    //each of these curves work by taking in a value (0,1) and returning an
    //adjusted value following a curve function, returns a value (0,1);
    static final public int CONSTANT = 0;
    static final public int LINEAR = 1;
    static final public int SIN = 2;
    static final public int CIRCLE = 3;
    static final public int QUADRATIC = 4;
    static final public int ROUNDEDSQUARE = 5;
    static final public int PARAMETRIC = 6;
    static final public int NORMAL = 7;
    static final public int EXPEASEIN = 8;
    static final public int EXPEASEOUT = 9;

    //inrange checker
    private static boolean inRange(double x) {
        return x >= 0 || x <= 1;
    }
    //follows an absolute value equation
    public static double linear(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return 1-Math.abs(2*x-1);
    }

    //follows a sin curve from trough to trough
    public static double sinCurve(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return Math.sin(2 * Math.PI*(x) - Math.PI / 2) / 2 + 0.5;
    }
    //follows the arc of a circle
    public static double circleCurve(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return Math.sqrt(1 - Math.pow((1 - x), 2));
    }
    //follows a quadratic equation
    public static double quadraticCurve(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return 1-4*Math.pow(x-.5, 2);
    }
    //follows the shape of a rounded square, is fast for most of the time
    //then quickly drops off in speed towards each end
    public static double roundedSquareCurve(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return Math.pow(1-Math.pow(1-2*x,4), 0.25);
    }
    //follows a parametric curve
    public static double parametricCurve(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return -(2*Math.pow(x,2)-2*x) /
                (2*Math.pow(2*Math.pow(x,2)-2*x+1, 2));
    }

    //follows the normal distribution curve
    public static double normalCurve(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return Math.pow(Math.E, -4*Math.pow(2*x-1, 2));
    }
    //follows an exponential curve, starts slow ends fast
    public static double exponentialEaseIn(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return Math.pow(Math.E, 4*x-4);
    }
    //follows an exponential curve, starts fast ends slow
    public static double exponentialEaseOut(double x) {
        if(!inRange(x)) {
            return -1;
        }
        return Math.pow(Math.E, -4*x);
    }


}



