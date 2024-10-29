package org.firstinspires.ftc.teamcode;

public class Utilize {
    public static boolean AtTargetRange(double number, double target, double range) {
        return target - range < number && number < target + range;
    }

    public static double WrapRads(double rads) {
        if (rads >  Math.PI) return rads - (2 * Math.PI);
        if (rads < -Math.PI) return rads + (2 * Math.PI);
        return rads;
    }

    public static double WrapDegs(double degs) {
        if (degs >  180) return degs - 360;
        if (degs < -180) return degs + 360;
        return degs;
    }

    public static double NormalizeRads(double rads) {
        rads = rads % (2 * Math.PI);
        if (rads == (2 * Math.PI)) return 0;
        return rads;
    }

    public static double NormalizeDegs(double degs) {
        degs = degs % 360;
        if (degs == 360) return 0;
        return degs;
    }

    public static byte SigNum(double number) {
        return (byte) (number == 0 ? 0 : (number < 0 ? -1 : 1));
    }

    public static double[] XY2Base(double base, double x, double y) {
        double absX = Math.abs(x);
        double absY = Math.abs(y);
        return new double[]{absX > absY ? base : (x / absY) * base,
                            absX < absY ? base : (y / absX) * base};
    }
    public  static double toRadian(double degs){
        return Math.toRadians(degs) * -1;
    }
    public  static double toDegree(double rads){
        return Math.toDegrees(rads) * -1;
    }
}