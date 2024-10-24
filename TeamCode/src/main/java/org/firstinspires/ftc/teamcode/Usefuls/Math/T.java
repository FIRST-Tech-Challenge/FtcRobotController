package org.firstinspires.ftc.teamcode.Usefuls.Math;

//T for Trigonometry !!
public final class T {

    public static double hypot(double a, double b) {
        return Math.hypot(a,b);
    }
    //hypotenuse legnth

    //forward trig functions (outputs numbers)

    public static double sin(double rad) {
        return Math.sin(rad);
    }

    public static double sin(double opposite, double hypotenuse){
        if (hypotenuse == 0) {
            throw new IllegalArgumentException("Hypotenuse cannot be zero");
        }
        return opposite/hypotenuse;
    }
    //sin

    public static double cos(double rad) {
        return Math.cos(rad);
    }

    public static double cos(double adjacent, double hypotenuse){
        if (hypotenuse == 0) {
            return Double.NaN;
        }
        return adjacent/hypotenuse;
    }
    //cos

    public static double tan(double rad) {
        return Math.tan(rad);
    }

    public static double tan(double opposite, double adjacent){
        if (adjacent == 0) {
            return Double.NaN;
        }
        return opposite/adjacent;
    }
    //tan

    //inverse trig functions (output angles)

    public static double arctan(double x) {
        return Math.atan(x);
    }

    //arc tangent, only 1 vector, and domain restriction from -pi/2 to pi/2

    public static double arctan2(double y, double x) {
        return Math.atan2(y,x);
    }
    //arc tangent but better, y,x, and domain restricted to -pi and pi

    public static double arccos(double v) {
        if (v < -1 || v > 1) {
            return Double.NaN;
        }
        return Math.acos(v);
    }
    //arc cosine

    public static double arcsin(double v) {
        if (v < -1 || v > 1) {
            return Double.NaN;
        }
        return Math.asin(v);
    }
    //arc sin

    public double angleWrap(double inputRad) {
        int inputSign = 1;
        if(inputRad < 0) { inputSign = -1; }
        inputRad = Math.abs(inputRad);

        inputRad += Math.PI;
        inputRad %= 2*Math.PI;
        inputRad *= inputSign;
        inputRad -= Math.PI;

        return inputRad;
    }
}
