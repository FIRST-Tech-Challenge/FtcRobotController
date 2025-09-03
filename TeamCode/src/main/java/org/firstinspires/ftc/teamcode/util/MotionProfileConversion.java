package org.firstinspires.ftc.teamcode.util;


public class MotionProfileConversion {

    // Maximum velocity the motion profile can reach
    private double maxVel;

    // Maximum acceleration allowed in the profile
    private double maxAccel;

    // Starting position of the motion
    private double initialPos;

    // Constructor to initialize the motion profile parameters
    public MotionProfileConversion(double maxVel, double maxAccel, double initialPos) {
        this.maxVel = maxVel;       // Set max velocity
        this.maxAccel = maxAccel;   // Set max acceleration
        this.initialPos = initialPos; // Set the initial position
    }

    // Method to calculate velocity and acceleration at a given current position toward a maxVelPoint
    public VelAccelPair getTrapezoidalConversion(double currentPos, double maxVelPoint) {

        // Total distance from current position to target point
        double d_total = maxVelPoint - currentPos;

        // Default acceleration and deceleration distances
        double d_accel = 30;
        double d_decel = d_accel;

        // If total distance is less than needed for full accel/decel, scale them down equally
        if(d_total < 2 * d_accel) {
            d_accel = d_total / 2;
            d_decel = d_accel;
        }

        // Compute position where acceleration should end
        double accelRange = initialPos + d_accel;

        // Check if we are still in the acceleration phase
        if (Math.abs(currentPos) < Math.abs(accelRange)) {
            // Return calculated velocity using kinematic equation and positive acceleration
            return new VelAccelPair(
                    Math.signum(currentPos - initialPos) * Math.sqrt(Math.abs(2 * maxAccel * currentPos - initialPos)),
                    maxAccel
            );
        }

        // Compute position where deceleration should start
        double decelRange = maxVelPoint - d_decel;

        // Check if we are in the deceleration phase
        if (Math.abs(currentPos) > Math.abs(decelRange)) {
            // If so, decelerate; however, this line is commented out and replaced with a stop
//            return new VelAccelPair(Math.signum(maxVelPoint - currentPos)*Math.sqrt(Math.abs(2 * maxAccel * maxVelPoint - currentPos)), -maxAccel);

            // Currently returns velocity 0 and acceleration 0 (full stop)
            return new VelAccelPair(0, 0);
        }

        // If neither accelerating nor decelerating, we are in the cruising phase at constant velocity
        return new VelAccelPair(maxVel, 0);
    }

    // Setter for initial position in case it needs to be updated during runtime
    public void setInitialPos(double initialPos){
        this.initialPos = initialPos;
    }
}