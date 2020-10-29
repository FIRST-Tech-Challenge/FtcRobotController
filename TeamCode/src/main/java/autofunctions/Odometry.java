package autofunctions;

import global.TerraBot;
import util.Vector;

public class Odometry {

    public double tx = 0; //total x
    public double ty = 0; //total y
    public double rp = 0; //right pos
    public double lp = 0; //left pos
    public double cp = 0; //center pos
    public double deltaRP = 0; //change in right pos
    public double deltaLP = 0; //change in left pos
    public double deltaCP = 0; //change in center pos
    public  double deltaTheta = 0;

    public double forward = 0;
    public double strafe = 0;
    public double theta = 0;

    double forcor = 0;
    double strcor = 0;

    public double testx = 0;
    public double testy = 0;


    public final double TICKS_FOR_ODOMETRY =  8192;
    public final double ENCODER_WHEEL_RADIUS = 1.75; // in cm

    public final double RADIUS_CENTER_TO_ENC = 2.5; // in cm

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
        deltaTheta = theta - heading;
        deltaTheta = Math.toRadians(deltaTheta);

        updateEncoderPositions(l, c, r);
        theta = heading;


        forward = (deltaLP+deltaRP)/2;
        strafe = deltaCP;

        if(deltaTheta != 0.0) {
            forcor = forward; //* (Math.sin(deltaTheta) / deltaTheta);
            strcor = strafe  -  (deltaTheta*cmToTicks(RADIUS_CENTER_TO_ENC)) ;//+ (forward * ((1 - Math.cos(deltaTheta)) / deltaTheta));
        }else{
            forcor = forward;
            strcor = strafe;
        }


        Vector cur = new Vector(strcor, forcor);

        Vector globalCur = cur.getRotatedVec(-theta, Vector.angle.DEGREES);

        tx += globalCur.x;
        ty += globalCur.y;

        testx += strcor;
        testy += forcor;

    }

    public double ticksToCm(double ticks){
        return (ticks/TICKS_FOR_ODOMETRY)*(2*Math.PI*ENCODER_WHEEL_RADIUS);
    }

    public double cmToTicks(double cm){
        return (cm*TICKS_FOR_ODOMETRY)/(2*Math.PI*ENCODER_WHEEL_RADIUS);
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

    public double[] getPos(){
        return new double[]{getX(), getY(), getTheta()};
    }

}
