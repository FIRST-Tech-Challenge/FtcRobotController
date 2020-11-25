package util;

public class Geometry {

    public double lawOfCosinesC(double a, double b, double rad){
        return Math.sqrt(a*a+b*b-(2*a*b*Math.cos(rad)));
    }
    public double lawOfSinesAngle(double a, double b, double rad){
        return Math.asin(a*Math.sin(rad)*(1/b));
    }

}
