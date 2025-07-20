package org.firstinspires.ftc.teamcode.Mechanisms.Utils.Planners;

import com.acmerobotics.dashboard.config.Config;


@Config
public class MotionProfile {

    // Motion Profile parameters
    double targetDistance; // Total distance the robot should move
    double accelerationTime; // Time spent accelerating
    double decelerationTime; // Time spent decelerating
    double accelerationDistance; // Distance covered during acceleration
    double decelerationDistance; // Distance covered during deceleration
    double cruiseDistance; // Distance covered at constant max speed
    double cruiseTime; // Time spent at a constant speed
    double totalTime; // Total time for the motion
    double maxVelocity; // Max allowed velocity
    double maxAcceleration; // Max allowed acceleration
    double maxDeceleration; // Max allowed deceleration
    int reverse; //

    /**
     * Generates a motion profile based on distance and motion constraints.
     * It creates either a trapezoidal profile (acceleration → cruise → deceleration)
     * or a triangular profile (acceleration → deceleration only if not enough distance).
     *
     * @param targetDistance  Distance to move
     * @param maxVelocity     Max velocity during motion
     * @param maxAcceleration Max acceleration
     * @param maxDeceleration Max deceleration
     * @param reverse         Whether to reverse the direction of motion
     */

    public MotionProfile(double targetDistance, double maxVelocity, double maxAcceleration, double maxDeceleration, boolean reverse){

        // Calculate time to reach max velocity (acceleration and deceleration phases)
        double accelerationTime = maxVelocity/maxAcceleration;
        double decelerationTime = maxVelocity/maxDeceleration;

        // Calculate distances covered during acceleration and deceleration
        double accelerationDistance = maxAcceleration*Math.pow(accelerationTime,2)/2;
        double decelerationDistance = maxDeceleration*Math.pow(decelerationTime,2)/2;

        // If the distance is too short to reach full speed, switch to triangular profile
        if (targetDistance<accelerationDistance+decelerationDistance) {
            decelerationTime = Math.sqrt((2*maxAcceleration*targetDistance)/(Math.pow(maxDeceleration,2)+maxAcceleration*maxDeceleration));
            accelerationTime = (maxDeceleration/maxAcceleration)*decelerationTime;

            // Recalculate distances for triangular profile
            accelerationDistance = maxAcceleration*Math.pow(accelerationTime,2)/2;
            decelerationDistance = maxDeceleration*Math.pow(decelerationTime,2)/2;

            // Max velocity achieved will be less than input max velocity
            maxVelocity = maxAcceleration * accelerationTime;
        }

        // Distance and time for the cruising (constant velocity) phase
        double cruiseDistance = targetDistance-accelerationDistance-decelerationDistance;
        double cruiseTime = cruiseDistance/maxVelocity;

        // Total time for the full motion
        double totalTime = accelerationTime+cruiseTime+decelerationTime;

        // Stores all values in classes
        this.targetDistance = targetDistance;
        this.maxVelocity = maxVelocity;
        this.accelerationDistance = accelerationDistance;
        this.decelerationDistance = decelerationDistance;
        this.maxAcceleration = maxAcceleration;
        this.maxDeceleration = maxDeceleration;
        this.cruiseDistance = cruiseDistance;
        this.cruiseTime = cruiseTime;
        this.totalTime = totalTime;
        this.accelerationTime = accelerationTime;
        this.decelerationTime = decelerationTime;

        // Set direction multiplier based on `reverse`
        if (reverse){
            this.reverse=-1;
        } else {
            this.reverse=1;
        }

    }

    /**
     * Returns the position (distance traveled) at a given time.
     *
     * @param time Time since start of motion
     * @return Position in distance units (positive or negative based on direction)
     */

    public double getPosition(double time){
        double cruiseTimePassed = time -accelerationTime;
        double timeDeceleration = time -accelerationTime-cruiseTime;

        // If motion is complete, return to final target distance
        if (time > totalTime){
            return targetDistance*reverse;
        }

        // Acceleration phase
        else if (time <= accelerationTime) {
            return maxAcceleration*Math.pow(time,2)/2*reverse;
        }

        // Constant velocity (cruise) phase
        else if (accelerationTime< time && time <=accelerationTime+cruiseTime){
            return accelerationDistance+maxVelocity*cruiseTimePassed*reverse;
        }

        // Deceleration phase
        else if (accelerationTime+cruiseTime<time &&time <=totalTime){
            return (accelerationDistance+cruiseDistance+maxVelocity*timeDeceleration-maxDeceleration*Math.pow(timeDeceleration,2)/2)*reverse;
        }

        // Should not reach here unless input is invalid
        return -1;
    }

    /**
     * Returns the velocity at a given time.
     *
     * @param time Time since start of motion
     * @return Velocity (positive or negative based on direction)
     */

    public double getVelocity(double time){
        if (time>totalTime){
            return 0;
        }

        // Acceleration phase
        else if (time<accelerationTime){
            return maxAcceleration*time*reverse;
        }

        // Constant velocity (cruise) phase
        else if (accelerationTime<time&&time<=accelerationTime+cruiseTime){
            return maxVelocity*reverse;
        }

        // Deceleration phase
        else if (accelerationTime+cruiseTime<time&&time<=totalTime){
            double timeDeceleration = time-accelerationTime-cruiseTime;
            return (maxVelocity-maxDeceleration*timeDeceleration)*reverse;
        }
        return 10000;
    }

    /**
     * Returns the acceleration at a given time.
     *
     * @param t Time since start of motion
     * @return Acceleration (positive, zero, or negative depending on phase)
     *
     * I'm pretty sure that t is time, but I'm unsure how to change the variable, because whenever
     * I attempt to change it, it brings up errors
     */

    public double getAcceleration(double t){
        if (t>totalTime){
            return 0;
        }

        // Acceleration phase
        else if (t<=accelerationTime){
            return maxAcceleration*reverse;
        }

        // Cruise phase
        else if (accelerationTime<t&&t<=accelerationTime+cruiseTime){
            return 0;
        }

        // Deceleration phase
        else if (accelerationTime+cruiseTime<t&&t<=totalTime){
            return -maxDeceleration*reverse;
        }

        return -1; // Fallback for invalid input
    }

    /**
     * @return Total time of the motion profile
     */

    public double getTime(){
        return totalTime;
    }

    /**
     * @return Direction of the profile: 1 = forward, -1 = reverse
     */

    public int isReverse(){
        return reverse;
    }
}
