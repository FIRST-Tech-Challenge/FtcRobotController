package org.firstinspires.ftc.teamcode.trajectory;

/** Generates an S-curve motion profile
 * @author TheConverseEngineer
 */
public abstract class MotionProfileFactory {

    private final double ACCEL_TIME;
    private final double MAX_VELOCITY;
    private final double ACCEL_DISTANCE;

    public MotionProfileFactory(double accelTime, double maxVelocity) {
        this.ACCEL_TIME = accelTime;
        this.MAX_VELOCITY = maxVelocity;
        this.ACCEL_DISTANCE = (maxVelocity*accelTime) / 1.9992;

    }

    public MotionProfile generateStandardProfile(double distance) {
        MotionProfile profile;
        if (distance > 2*this.ACCEL_DISTANCE) { // Standard Trajectory
            profile = new MotionProfile.Builder()
                    .addAccelPhase(ACCEL_TIME, MAX_VELOCITY, 0)
                    .addSteadyPhase(MAX_VELOCITY, (distance - (2*this.ACCEL_DISTANCE)) / this.MAX_VELOCITY)
                    .addDeaccelPhase(ACCEL_TIME, MAX_VELOCITY, 0)
                    .build();
        } else {  // Degenerate trajectory
            double newAccelTime = (distance*ACCEL_TIME) / (2*ACCEL_DISTANCE);
            double newMaxVelo = (distance * 0.996) / newAccelTime;
            profile = new MotionProfile.Builder()
                    .addAccelPhase(newAccelTime, newMaxVelo, 0)
                    .addDeaccelPhase(newAccelTime, newMaxVelo, 0)
                    .build();
        }
        return profile;
    }

    public MotionProfile generateAccelOnlyProfile(double distance) {
        MotionProfile profile;
        if (distance > this.ACCEL_DISTANCE) { // Standard Trajectory
            profile = new MotionProfile.Builder()
                    .addAccelPhase(ACCEL_TIME, MAX_VELOCITY, 0)
                    .addSteadyPhase(MAX_VELOCITY, (distance - this.ACCEL_DISTANCE) / this.MAX_VELOCITY)
                    .build();
        } else {  // Degenerate trajectory (Will be longer than needed and won't finish)
            profile = new MotionProfile.Builder()
                    .addAccelPhase(ACCEL_TIME, MAX_VELOCITY, 0)
                    .build();
        }
        return profile;
    }

    public MotionProfile generateDeaccelOnlyProfile(double distance) {
        MotionProfile profile;
        if (distance > this.ACCEL_DISTANCE) { // Standard Trajectory
            profile = new MotionProfile.Builder()
                    .addSteadyPhase(MAX_VELOCITY, (distance - this.ACCEL_DISTANCE) / this.MAX_VELOCITY)
                    .addDeaccelPhase(ACCEL_TIME, MAX_VELOCITY, 0)
                    .build();
        } else {  // Degenerate trajectory (Will be longer than needed and won't finish)
            profile = new MotionProfile.Builder()
                    .addDeaccelPhase(ACCEL_TIME, MAX_VELOCITY, 0)
                    .build();
        }
        return profile;
    }

    public MotionProfile generateSteadyProfile(double distance) {
        return new MotionProfile.Builder().addSteadyPhase(MAX_VELOCITY, distance / MAX_VELOCITY).build();
    }

}
