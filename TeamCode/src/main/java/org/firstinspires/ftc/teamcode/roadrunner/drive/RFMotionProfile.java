package org.firstinspires.ftc.teamcode.roadrunner.drive;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import static java.lang.Double.min;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class RFMotionProfile {
    double curviness, c,velo1,velo2;
    double MIN_DECEL = -5;
    public RFMotionProfile(Vector2d p_startPos, Vector2d p_startVel, Vector2d p_endVel, Vector2d p_endPos, double p_curviness){
        curviness = p_curviness;
        //convert from percentage to slope
        c = 2*MAX_ACCEL*MAX_ACCEL*(1-curviness)/(curviness*MAX_VEL);
        velo1=MAX_ACCEL*MAX_ACCEL/(2*c*c*c);
        velo2=MAX_VEL-velo1;
    }

    public double getInstantaneousTargetAcceleration(double dist){
        double velo = currentVelocity.vec().norm(), targetAccel;
        if(velo<velo1){
            targetAccel = c*Math.sqrt(2*(velo)*c);
        }
        else if(velo<velo2){
            targetAccel = MAX_ACCEL;
        }
        else if (velo>velo2&&velo<MAX_VEL){
            targetAccel=MAX_ACCEL-c*Math.sqrt(2*(velo-velo2));
        }
        else{
            targetAccel=0;
        }
        //if u should decel
        if(dist<getDecelDist()){
            targetAccel = min(targetAccel*-1,MIN_DECEL);
        }
        else{
            //nothing
        }
        return targetAccel;
    }

    public double getTargetVelocity(){
        return 0;
    }

    public double getTargetAcceleration(){
        return 0;
    }

    public double motionProfileDistToTime(double dist){
        return 0;
    }

    public double getDecelTime(){
        return currentVelocity.vec().norm()/((1-curviness*0.5)*MAX_ACCEL);
    }

    public double getDecelDist(){
        return getDecelTime()*currentVelocity.vec().norm()*0.5;
    }
}
