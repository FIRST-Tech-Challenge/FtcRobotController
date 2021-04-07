package util;

public class Geometry {

    public static double lawOfCosinesC(double a, double b, double rad){
        return Math.sqrt(a*a+b*b-(2*a*b*Math.cos(rad)));
    }
    public static double lawOfSinesAngle(double a, double b, double bAng){
        return Math.asin(a*Math.sin(bAng)*(1/b));
    }
    public static double pythagoreanC(double a, double b){
        return Math.sqrt(a*a + b*b);
    }

}
