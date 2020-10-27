package util;

public class Vector {
    public double x;
    public double y;
    public double theta;

    public Vector(double x1, double y1){
        x = x1;
        y = y1;
        theta = Math.atan2(y, x);
    }

//    public double[] getPos(){
//        double[] out = new double[2];
//        out[0] = x;
//        out[1] = y;
//        return out;
//    }
//
    public Vector getRotatedVector(double deg){
        double ang  = theta + Math.toRadians(deg);
        double radius = Math.sqrt(x*x + y*y);
        return new Vector(Math.cos(ang)*radius, Math.sin(ang)*radius);
    }

}
