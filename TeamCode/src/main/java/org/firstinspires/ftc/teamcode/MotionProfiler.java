package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.controller.PIDFController;

public class MotionProfiler {

    double maxVelocity, maxAccel, distance, totalDt, accelerationDt, halfwayDist, accelerationDistance, deaccelerationDt, cruiseDistance, cruiseDt, deaccelerationTime;
    double cruiseCurrentDt;
    boolean isDone=false;


    public MotionProfiler(double maxVelocity, double maxAcceleration, double distance){
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAcceleration;
        this.distance= distance;
    }

    public MotionProfiler(double maxVelocity, double maxAccel){
        this.maxVelocity = maxVelocity;
        this.maxAccel = maxAccel;
    }

    public void updateDistance(double distance){
        this.distance = distance;
    }

        public double profileMotion(double timeElapsed){

        //accelDt = time to accelerate to max velocity
        accelerationDt = maxVelocity/maxAccel;
        halfwayDist = distance/2;
        accelerationDistance = 0.5 * maxAccel * Math.pow(accelerationDt,2);
        //distance of acceleration: 1/2 a delta t^2 (more kinematic equations!!!!)
        if((accelerationDistance)>halfwayDist){
            accelerationDt = Math.sqrt(halfwayDist/(0.5*maxAccel));
            //If we can't accelerate to max velocity in given distance, will accelerate as much as possible
        }
        maxVelocity = maxAccel * accelerationDt;
        deaccelerationDt = accelerationDt;
        //acceleration time is same as deacceleration time

        cruiseDistance = distance - 2 * accelerationDistance;
        //cruising distance is twice as much as acceleration distance
        cruiseDt = cruiseDistance / maxVelocity;
        deaccelerationTime = accelerationDt + cruiseDt;

        totalDt = accelerationDt + cruiseDt + deaccelerationDt;
        if (timeElapsed > totalDt) {
            isDone = true;
            return 0;
        }

        if (timeElapsed < accelerationDt)
            // use the kinematic equation for acceleration
            return 0.5 * maxAccel * Math.pow(timeElapsed,2);

        else if (timeElapsed < deaccelerationTime) {
            accelerationDistance = 0.5 * maxAccel * Math.pow(accelerationDt,2);
            cruiseCurrentDt = timeElapsed - accelerationDt;

            //constant velocity
            return accelerationDistance + maxVelocity * cruiseCurrentDt;

    } // deacceleration
        else{
            accelerationDistance = 0.5 * maxAccel * Math.pow(accelerationDt,2);
            cruiseDistance = maxVelocity * cruiseDt;
            deaccelerationTime = timeElapsed - deaccelerationTime;

            // use the kinematic equations to calculate the instantaneous desired position
            return accelerationDistance + cruiseDistance + maxVelocity * deaccelerationTime - 0.5 * maxAccel * Math.pow(deaccelerationTime,2);
        }

        //method returns the distance you need to go to reach next target

    }



}


