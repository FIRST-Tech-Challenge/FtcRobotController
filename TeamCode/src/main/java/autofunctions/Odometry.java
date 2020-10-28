package autofunctions;

import global.TerraBot;
import util.Vector;

public class Odometry {

    public double tx = 0; //total x
    public double ty = 0; //total y
    double rp = 0; //right pos
    double lp = 0; //left pos
    double cp = 0; //center pos
    double deltaRP = 0; //change in right pos
    double deltaLP = 0; //change in left pos
    double deltaCP = 0; //change in center pos

    public double forward = 0;
    public double turn = 0;
    public double strafe = 0;
    public double theta = 0;
    public double sr = 0;
    public double sl = 0;
    public double sc = 0;
    public double cr = 0;
    public double cl = 0;
    public double cc = 0;

    public double vx = 0;


    public final double TICKS_FOR_ODOMETRY =  8192;
    public final double ENCODER_WHEEL_RADIUS = 1.75; // in cm

    public final double LEFT_OVER_RIGHT = 1.070175;
    public final double LEFT_OVER_TOTAL = LEFT_OVER_RIGHT/(LEFT_OVER_RIGHT+1);

    public void init(double l, double c , double r){
        updateEncoderPositions(l, c, r);
    }

    public void reset(double l, double c , double r){
        tx = 0;
        ty = 0;
        theta = 0;
        updateEncoderPositions(l, c, r);
    }

    public void updateGlobalPosition(double l, double c , double r, double heading){
        deltaRP = r-rp;
        deltaLP = l-lp;
        deltaCP = c-cp;

        updateEncoderPositions(l, c, r);

        theta = heading;


        forward = deltaLP+((deltaRP-deltaLP)*LEFT_OVER_TOTAL);
        strafe = deltaCP;

        Vector cur = new Vector(strafe, forward);

        Vector globalCur = cur.getRotatedVec(-theta, Vector.angle.DEGREES);

        tx += globalCur.x;
        ty += globalCur.y;

    }

    public double ticksToCm(double ticks){
        return (ticks/TICKS_FOR_ODOMETRY)*(2*Math.PI*ENCODER_WHEEL_RADIUS);
    }


    public void updateEncoderPositions(double l, double c, double r){
        rp = r;
        lp = l;
        cp = c;
    }

    public double getX(){
        return ticksToCm(tx);
    }

    public double getY(){
        return ticksToCm(ty);
    }

    public double getTheta(){
        return theta;
    }

}
