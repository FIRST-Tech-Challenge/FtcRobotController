package org.rustlib.control;

public class MotionProfileSetpoint {
    public final double position;
    public final double velocity;
    public final double acceleration;

    public MotionProfileSetpoint(double position, double velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }
}
