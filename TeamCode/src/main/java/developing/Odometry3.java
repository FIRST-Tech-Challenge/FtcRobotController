package developing;

import java.util.ArrayList;

import global.Constants;
import util.Vector;

public class Odometry3 {

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

    public final double d = Constants.HALF_DIS_BETWEEN_ENCS; //Looks like this
    //  L |-----|------| R
    public final double dc = Constants.DIS_CENTER_ENC_TO_CENTER;//Looks like this
    //          C
    // L--------|--|-----------R
    public final double dh = Constants.RADIUS_CENTER_TO_ENC;
    // C
    // ___
    //  |
    // ___

    public ArrayList<Double> Ydebug = new ArrayList<>();
    public ArrayList<Double> Xdebug = new ArrayList<>();
    public ArrayList<Double> Hdebug = new ArrayList<>();

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
        updateDeltaXY();
        updateGlobalXYH();
    }

    //Calculates deltah using right and left encoder readings
    public void calcDeltaH() {
        deltaH = Math.asin((deltaRP - deltaLP) / (2 * d));
    }

    //update delta x and y
    public void updateDeltaXY() {
        deltaY = deltaLP + (s * d);
        deltaX = deltaCP + (dc * c1) - (dh * s);
    }


    //update global x y and h
    public void updateGlobalXYH() {

//        double oldY = ign0(deltaH, ((deltaRP + deltaLP) * s )/ (2 * deltaH));
        double oldY = ign0(deltaH, (deltaRP + deltaLP) / 2);
        double oldX = ign0(deltaH, deltaCP - (deltaH * Constants.RADIUS_CENTER_TO_ENC) + (oldY * c1 / deltaH));
        double oldH = (deltaRP - deltaLP) / (2*d);


        Ydebug.add(deltaY - oldY);
        Xdebug.add(deltaX - oldX);
        Hdebug.add(deltaH - oldH);

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

    //Convert ticks to cm
    public double ticksToCm(double ticks) {
        return (ticks / Constants.CM_TO_TICKS);
    }

    //convert cm to ticks
    public double CmToTicks(double cm) {
        return (cm * Constants.CM_TO_TICKS);
    }

}
