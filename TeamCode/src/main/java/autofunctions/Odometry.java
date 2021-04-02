

package autofunctions;

import global.Constants;
import util.Vector;

public class Odometry {

    public double tx = 0; // total x
    public double ty = 0; // total y
    public double rp = 0; // right pos
    public double lp = 0; // left pos
    public double cp = 0; // center pos
    public double deltaRP = 0; // change in right pos
    public double deltaLP = 0; // change in left pos
    public double deltaCP = 0; // change in center pos
    public double deltaTheta = 0; // change in robot theta

    public double forward = 0;
    public double strafe = 0;
    public double theta = 0;

    public double deltaThetaEnc = 0;

    public double forcor = 0;
    public double strcor = 0;

    public void init(double l, double c, double r) {
        updateEncoderPositions(l, c, r);
    }

    public void reset(double l, double c, double r) {
        tx = 0;
        ty = 0;
        theta = 0;
        updateEncoderPositions(l, c, r);
    }

    public void updateGlobalPosition(double l, double c, double r, double heading) {
        deltaRP = r - rp; // change since last
        deltaLP = l - lp; // change since last
        deltaCP = c - cp; // change since last
        deltaTheta = Math.toRadians(theta - heading); // change since last - in radians

// updating encoder positions and current heading variables
        updateEncoderPositions(l, c, r);
        theta = heading;

// forward movement
        forward = (deltaLP + deltaRP) / 2;
// strafe movement
        strafe = deltaCP;

// change in theta according to encoder
        deltaThetaEnc = (deltaRP - deltaLP) / (Constants.CM_TO_TICKS * Constants.DIS_BEWTEEN_ENCS);

        if (deltaTheta == 0) {
            if (deltaThetaEnc != 0.0) {
// gyro has failed; using encoders
// forcor looks good
                forcor = forward * (Math.sin(deltaThetaEnc) / deltaThetaEnc);
                strcor = strafe - (deltaThetaEnc * Constants.CM_TO_TICKS * Constants.RADIUS_CENTER_TO_ENC) + (forward * ((1 - Math.cos(deltaThetaEnc)) / deltaThetaEnc));
            } else {
// forcor and strcor look good
                forcor = forward;
                strcor = strafe;
            }
        } else {
// everything is fine
// forcor looks good
            forcor = forward * (Math.sin(deltaTheta) / deltaTheta);
            strcor = strafe - (deltaTheta * Constants.CM_TO_TICKS * Constants.RADIUS_CENTER_TO_ENC) + (forward * ((1 - Math.cos(deltaTheta)) / deltaTheta));
        }

        Vector deltaGlobalPos = new Vector(strcor, forcor).getRotatedVec(theta, Vector.angle.DEGREES);

        tx += deltaGlobalPos.x;
        ty += deltaGlobalPos.y;

    }

    //Convert ticks to cm
    public double ticksToCm(double ticks) {
        return (ticks / Constants.CM_TO_TICKS);
    }

    //convert cm to ticks
    public double cmToTicks(double cm) {
        return (cm * Constants.CM_TO_TICKS);
    }


    public void updateEncoderPositions(double l, double c, double r) {
        rp = r;
        lp = l;
        cp = c;
    }

    public double getX() {
        return ticksToCm(tx);
    }

    public double getY() {
        return ticksToCm(ty);
    }

    public double getTheta() {
        return theta;
    }

    public double[] getPos() {
        return new double[]{getX(), getY(), getTheta()};
    }

    public void setTheta(double theta) {
        this.theta = theta;
        deltaTheta = 0;
        deltaThetaEnc = 0;
        deltaRP = 0;
        deltaCP = 0;
        deltaLP = 0;
    }

    public double getTVel(){
        return 0;
    }
    public double getYVel(){
        return 0;
    }
    public double getXVel(){
        return 0;
    }

    public double[] getVels(){
        return new double[]{0,0,0};
    }

}
