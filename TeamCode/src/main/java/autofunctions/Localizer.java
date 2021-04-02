package autofunctions;

import util.Geometry;
import util.Vector;

public class Localizer {

    public double l2 = 0;
    public double theta = 0;

    public final double robotWidth = 30;
    public final double robotLength = 33;
    public final double robotRadius = Math.sqrt(Math.pow(robotWidth/2, 2) + Math.pow(robotLength/2, 2));
    public final double centerTheta = Math.atan2(robotLength,robotWidth);
    public final double n9 = Math.toRadians(90);

    public Geometry geometry = new Geometry();

    public void update(double l2, double heading){
        theta = heading;

        this.l2 = l2;

    }

    public double getX(){
        double a = robotRadius;
        double cent = n9-centerTheta;
        double d = geometry.lawOfCosinesC(l2,a,cent+n9);
        double phi = geometry.lawOfSinesAngle(a,d,cent+n9);
        double x = d*Math.cos(Math.toRadians(theta)+phi);

        return x;

    }

}
