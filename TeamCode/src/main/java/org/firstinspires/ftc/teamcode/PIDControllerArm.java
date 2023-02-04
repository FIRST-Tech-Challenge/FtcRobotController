package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

public class PIDControllerArm extends PIDController {
    public final double ksin;

    /**
     * Construct the PID controller
     * @param p - Proportional coefficient
     * @param i - Integral coefficient
     * @param d - Derivative coefficient
     * @param sin - The factor referenced to sin of the angle the arm is at
     */
    public PIDControllerArm(double p, double i, double d, double sin) {
        super(p, i, d);
        ksin = sin;
    }
    /**
     * Update the PID output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     */
    public double update (double target, double state) {
        return super.update(target, state) + ksin * sin(toRadians(state));
    }
}
