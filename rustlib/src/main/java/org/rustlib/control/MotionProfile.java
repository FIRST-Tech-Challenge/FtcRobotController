package org.rustlib.control;

public interface MotionProfile {
    MotionProfileSetpoint sample(double t);
}
