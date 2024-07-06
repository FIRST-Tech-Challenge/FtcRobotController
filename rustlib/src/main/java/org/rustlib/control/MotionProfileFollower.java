package org.rustlib.control;

import org.rustlib.control.PIDController.PIDGains;

import java.util.function.DoubleSupplier;

public class MotionProfileFollower {
    public final MotionProfile profile;
    public final double kV;
    public final double kA;
    private final PIDController controller;
    private final DoubleSupplier position;

    public MotionProfileFollower(MotionProfile profile, double kV, double kA, PIDGains pidGains, DoubleSupplier position) {
        this.profile = profile;
        this.kV = kV;
        this.kA = kA;
        controller = new PIDController(pidGains);
        this.position = position;
    }

    public double calculate(double t) {
        MotionProfileSetpoint setpoint = profile.sample(t);
        double[] pidTerms = controller.calculateTerms(position.getAsDouble(), setpoint.position);
        return kV * setpoint.velocity + kA * setpoint.acceleration + pidTerms[0] + pidTerms[1] + pidTerms[2] - controller.getGains().kD * setpoint.velocity;
    }
}
