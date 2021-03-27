package developing;

import java.util.ArrayList;

import global.Constants;
import util.Vector;

public class Odometry2 {

    public double x = 0; // total x
    public double y = 0; // total y
    public double h = 0; // heading

    public double rp = 0; // right pos
    public double lp = 0; // left pos
    public double cp = 0; // center pos

    public double deltaRP = 0; // change in right pos
    public double deltaLP = 0; // change in left pos
    public double deltaCP = 0; // change in center pos
    public double deltaH = 0; //change in heading
    public double deltaX = 0; //change in x (robot frame)
    public double deltaY = 0; //change in y (robot frame)

    public final double d = Constants.HALF_DIS_BETWEEN_ENCS;
    public final double c = Constants.RADIUS_CENTER_TO_ENC;

    public ArrayList<Double> Ydebug = new ArrayList<>();
    public ArrayList<Double> Xdebug = new ArrayList<>();
    public ArrayList<Double> Hdebug = new ArrayList<>();


    public Vector dl = new Vector(0, 0);
    public Vector dr = new Vector(0, 0);
    public double s = 0;
    public double c1 = 0;


    //resets positions to 0
    public void reset() {
        x = 0;
        y = 0;
        h = 0;
    }

    //resets positions to desired values
    public void reset(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    //Make sure to call update encoder positions once before using this to reset them

    //Update global positions using encoder readings
    public void updateGlobalPosition(double l, double c, double r) {
        updateEncoderPositions(l, c, r);
        calcDeltaH();
        updateCS();
        calcDVectors();
        updateDeltaXY();
        updateGlobalXYH();

    }

    //Er/l is right/left change in encoder positions, theta is change in heading
    //This function should return 0 if theta is the correct value
    public double thetaFunction(double theta) {
        return ign0(theta, (deltaRP - deltaLP) / (4 * d) + (Math.cos(theta) - 1) / theta);
    }

    //Derivative of theta function, accepts change in heading
    //Used in newtons method
    public double derivativeOfThetaFunction(double theta) {
        return ign0(theta, (1 - Math.cos(theta) - theta * Math.sin(theta)) / (Math.pow(theta, 2)));
    }

    //NewtonsMethodOfThetaFunction used to calculate theta to a greater accuracy
    public double nmotf(double theta) {
        return ign0(theta, theta - (thetaFunction(theta) / derivativeOfThetaFunction(theta)));
    }

    //Calculated change in heading using newtons method
    public void calcDeltaH() {
        deltaH = nmotf(nmotf(nmotf((deltaRP - deltaLP) / (2 * d))));
    }

    //Calculate distance vectors for left and right odometry that measure the change in coordinates from the center odometry at (0,0)
    public void calcDVectors() {
        dl.setXY((-d * c1) - (c * s), (-c * c1) + (d * s));
        dr.setXY((d * c1) - (c * s), (-c * c1) - (d * s));
    }

    //Calculate x using encoder position and d vectors, inputs encoder reading and d vector
    public double calcX(double E, Vector d) {
        return ign0(c1, 0.5 * ((deltaCP * deltaH * s) / c1) - (E * deltaH) + (d.x * c1) - (d.y * s));
    }

    //Calculate y using x
    public double calcY(double x) {
        return ign0(c1, ((deltaCP * deltaH) - (x * s)) / c1);
    }

    //calculate average x and use that to calculate average y and then update
    public void updateDeltaXY() {
        deltaX = 0.5 * (calcX(deltaLP, dl) + calcX(deltaRP, dr));
        deltaY = calcY(deltaX);
    }

    //update global x y and h
    public void updateGlobalXYH() {

        double oldY = ign0(deltaH, ((deltaRP + deltaLP) * s )/ (2 * deltaH));
        double oldX = ign0(deltaH, deltaCP - (oldY * c1 / deltaH));
        double oldH = (deltaRP - deltaLP) / (2 * d);



        Ydebug.add(deltaY - oldY);
        Xdebug.add(deltaX - oldX);
        Hdebug.add(deltaH - oldH);

//        deltaH = oldH;

        deltaY = oldY;
        deltaX = oldX;

        Vector deltaGlobalPos = new Vector(deltaX, deltaY).getRotatedVec(h, Vector.angle.DEGREES);
        x += deltaGlobalPos.x;
        y += deltaGlobalPos.y;
        h += Math.toDegrees(deltaH);
    }

    //Update c1 and s
    public void updateCS() {
        s = Math.sin(deltaH);
        c1 = 1 - Math.cos(deltaH);
    }

    //Convert ticks to cm
    public double ticksToCm(double ticks) {
        return (ticks / Constants.CM_TO_TICKS);
    }

    //update encoder positions
    public void updateEncoderPositions(double l, double c, double r) {
        r = ticksToCm(r);
        l = ticksToCm(l);
        c = ticksToCm(c);
        deltaRP = r - rp; // change since last
        deltaLP = l - lp; // change since last
        deltaCP = c - cp; // change since last
        rp = r;
        lp = l;
        cp = c;
    }

    //returns out if inp is not 0
    public double ign0(double inp, double out) {
        if (inp != 0) {
            return out;
        } else {
            return 0;
        }
    }


//    //convert cm to ticks
//    public double CmToTicks(double cm) {
//        return (cm * Constants.CM_TO_TICKS);
//    }
//    //return x in cm
//    public double getX() {
//        return x;
//    }
//
//    //return y in cm
//    public double getY() {
//        return y;
//    }
//
//    //return heading in degrees
//    public double getHeading() {
//        return h;
//    }
//
//    //get position as an array
//    public double[] getPos() {
//        return new double[]{getX(), getY(), getHeading()};
//    }
//    public void updateGlobalPosition(double l, double c , double r, double heading){
//        deltaRP = r-rp; // change since last
//        deltaLP = l-lp; // change since last
//        deltaCP = c-cp; // change since last
//        deltaTheta = Math.toRadians(theta - heading); // change since last - in radians
//
//        // updating encoder positions and current heading variables
//        updateEncoderPositions(l, c, r);
//        theta = heading;
//
//        // forward movement
//        forward = (deltaLP+deltaRP)/2;
//        // strafe movement
//        strafe = deltaCP;
//
//        // change in theta according to encoder
//        deltaThetaEnc = (deltaRP - deltaLP) / (Constants.CM_TO_TICKS * Constants.DIS_BEWTEEN_ENCS);
//
//        if(deltaTheta == 0) {
//            if(deltaThetaEnc != 0.0) {
//                // gyro has failed; using encoders
//                // forcor looks good
//                forcor = forward * (Math.sin(deltaThetaEnc) / deltaThetaEnc);
//                strcor = strafe -  (deltaThetaEnc * Constants.CM_TO_TICKS * Constants.RADIUS_CENTER_TO_ENC) + (forward * ((1 - Math.cos(deltaThetaEnc)) / deltaThetaEnc));
//            }else{
//                // forcor and strcor look good
//                forcor = forward;
//                strcor = strafe;
//            }
//        }else{
//            // everything is fine
//            // forcor looks good
//            forcor = forward * (Math.sin(deltaTheta) / deltaTheta);
//            strcor = strafe - (deltaTheta*Constants.CM_TO_TICKS*Constants.RADIUS_CENTER_TO_ENC) + (forward * ((1 - Math.cos(deltaTheta)) / deltaTheta));
//        }
//
//        Vector deltaGlobalPos = new Vector(strcor, forcor).getRotatedVec(theta, Vector.angle.DEGREES);
//
//        tx += deltaGlobalPos.x;
//        ty += deltaGlobalPos.y;
//
//    }


}
