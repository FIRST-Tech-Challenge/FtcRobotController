package util;

public class Geometry {
    //Calculates law of cosines for C knowing a,b and the angle between them
    public static double lawOfCosinesC(double a, double b, double rad){
        return Math.sqrt(a*a+b*b-(2*a*b*Math.cos(rad)));
    }
    //Calculates law of sines for the angle knowing a,b and the angle oposite to b
    public static double lawOfSinesAngle(double a, double b, double bAng){
        return Math.asin(a*Math.sin(bAng)*(1/b));
    }
    //Calculates the pythagorean theorom for the c knowing a and b
    public static double pythagoreanC(double a, double b){
        return Math.sqrt(a*a + b*b);
    }

}
