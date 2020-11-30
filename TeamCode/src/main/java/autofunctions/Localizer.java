package autofunctions;

import java.util.ArrayList;

import telefunctions.AutoModule;
import util.Geometry;
import util.Vector;

public class Localizer {

    public double r1 = 0;
    public double l1 = 0;
    public double r2 = 0;
    public double l2 = 0;
    public double theta = 0;

    public double r1fn = 0;
    public double l1fn = 0;
    public double r2fn = 0;
    public double l2fn = 0;

    public double lastL2 = 0;
    public double lastAngle = 0;
    public double lastX = 0;
    public double lastY = 0;
    public double calTheta = 0;

    public boolean isCal = false;


    public int counter = 0;

    public final double robotWidth = 30;
    public final double robotLength = 33;
    public final double robotRadius = Math.sqrt(Math.pow(robotWidth/2, 2) + Math.pow(robotLength/2, 2));
    public final double centerTheta = Math.atan2(robotLength,robotWidth);
    public final double n9 = Math.toRadians(90);

    public double numGets = 5;

    public Geometry geometry = new Geometry();

    public Vector sub;


//    public void update(double r1, double l1, double r2, double l2, double heading){
//        theta = heading;
//
//        r1fn += r1/numGets;
//        l1fn += l1/numGets;
//        r2fn += r2/numGets;
//        l2fn += l2/numGets;
//
//        counter++;
//        if(counter == numGets){
//            counter = 0;
//            this.r1 = r1fn;
//            this.r2 = r2fn;
//            this.l1 = l1fn;
//            this.l2 = l2fn;
//            r1fn = 0;
//            l1fn = 0;
//            r2fn = 0;
//            l2fn = 0;
//
//        }
//
//
//    }
    public void update(double l2, double heading){
        theta = heading;

        this.l2 = l2;

//        r1fn += r1/numGets;
//        l1fn += l1/numGets;
//        r2fn += r2/numGets;
//        l2fn += l2/numGets;
//
//        counter++;
//        if(counter == numGets){
//            counter = 0;
//            this.r1 = r1fn;
//            this.r2 = r2fn;
//            this.l1 = l1fn;
//            this.l2 = l2fn;
//            r1fn = 0;
//            l1fn = 0;
//            r2fn = 0;
//            l2fn = 0;
//
//        }
//

    }

    public void updateHeading(double head){
        theta = head;
    }

//    public double getAngle(){
//        double cd = l1 - r1;
//        double out = Math.atan2(cd, robotWidth);
//        return Math.toDegrees(out);
//    }
    public double getAngle(double dis, boolean sign){
        double out = Math.atan2(dis, robotWidth);
        if(sign) {
            return Math.toDegrees(out);
        }else{
            return -Math.toDegrees(out);
        }
    }


    public double getX(){
        double a = robotRadius;
        double cent = n9-centerTheta;
        double d = geometry.lawOfCosinesC(l2,a,cent+n9);
        double phi = geometry.lawOfSinesAngle(a,d,cent+n9);
        double x = d*Math.cos(Math.toRadians(theta)+phi);

        return x;

    }
//    public double getY(){
//        double a = robotRadius;
//        double d = geometry.lawOfCosinesC(l1,a,centerTheta+n9);
//        double phi = geometry.lawOfSinesAngle(a,d,centerTheta+n9);
//        double y = d*Math.cos(Math.toRadians(theta)-phi);
//
//        double d1 = geometry.lawOfCosinesC(r1,a,centerTheta+n9);
//        double phi2 = geometry.lawOfSinesAngle(a,d1,centerTheta+n9);
//        double y2 = d*Math.cos(Math.toRadians(theta)+phi2);
//
//        return -(y+y2)/2;
//    }

    public void startCalibrating(double l2dis, double startAngle, double x, double y){
        lastL2 = l2dis;
        lastAngle = startAngle;
        lastX = x;
        lastY = y;
    }

    public void stopCalibrating(double l2dis, double stopAngle, double x, double y){
        l2 = l2dis;
        double angle = Math.toRadians(stopAngle-lastAngle);

        Vector vec = new Vector(-robotWidth/2, -robotLength/2);
        Vector rot = vec.getRotatedVec(-Math.toDegrees(angle), Vector.angle.DEGREES);
        sub = vec.subtractVector(rot);
        double cx = (x-lastX) + sub.x;
        double cy = (y-lastY) + sub.y;
        double extFirst = Math.abs(cy)/Math.tan(angle);
        double extSecond = Math.abs(cy)/Math.sin(angle);
        lastL2 += extFirst+cx;
        l2dis += extSecond;
        double c = geometry.lawOfCosinesC(lastL2, l2dis, angle);
        double the = geometry.lawOfSinesAngle(lastL2, c,  angle);
        calTheta =  90-Math.toDegrees(the);
        isCal = true;
    }
    public double getCalibratedTheta(){
        return calTheta;
    }


}
