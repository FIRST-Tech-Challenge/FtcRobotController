package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDFController;

public class MotionProfiler {
    //trustttt i will change based on actual measurements
    double maxVelocity;
    double maxAccel;
    double distance;
    double timeElapsed;

    public MotionProfiler(double maxVelocity, double maxAcceleration, double distance, double timeElapsed){
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAcceleration;
        this.distance = distance;
        this.timeElapsed = timeElapsed;
        //should I not make time elapsed an instance field??
    }

    public double motion_profile(){
        //time to accelerate to max velocity
        double acceleration_dt = maxVelocity/maxAccel;
        double halfway_dist = distance/2;
        double accelerationDistance = 0.5 * maxAccel * Math.pow(acceleration_dt,2);
        if(accelerationDistance>halfway_dist){
            acceleration_dt = Math.sqrt(halfway_dist/(0.5*maxAccel));
            //accelerates as much as it can if it can't accelerate
        }
        this.maxVelocity = maxAccel * acceleration_dt;
        double deacceleration = acceleration_dt;

    }



}
