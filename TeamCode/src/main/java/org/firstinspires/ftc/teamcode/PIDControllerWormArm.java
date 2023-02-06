package org.firstinspires.ftc.teamcode;

import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

public class PIDControllerWormArm {
    public final PIDController pidLift;
    public final double ksinLift;
    public final double kStaticLift;
    public final PIDController pidLower;
    public final double ksinLower;
    public final double kStaticLower;
    public PIDController pidCurrent;
    public double ksinCurrent;
    public double kStaticCurrent;

    /**
     * Construct the PID controller
     * @param pLift - Proportional coefficient against gravity
     * @param iLift - Integral coefficient against gravity
     * @param dLift - Derivative coefficient against gravity
     * @param sinLift - The factor referenced to sin of the angle the arm is at against gravity
     * @param staticLift - The minimal constant power to almost move the arm against gravity
     * @param pLower - Proportional coefficient with gravity
     * @param iLower - Integral coefficient with gravity
     * @param dLower - Derivative coefficient with gravity
     * @param sinLower - The factor referenced to sin of the angle the arm is at with gravity
     * @param staticLower - The minimal constant power to almost move the arm with gravity
     */
    public PIDControllerWormArm(double pLift, double iLift, double dLift, double sinLift, double staticLift,
                                double pLower, double iLower, double dLower, double sinLower, double staticLower) {
        pidLift = new PIDController(pLift, iLift, dLift);
        ksinLift = sinLift;
        kStaticLift = staticLift;
        pidLower = new PIDController(pLower, iLower, dLower);
        ksinLower = sinLower;
        kStaticLower = staticLower;
    }
    /**
     * Update the PID output based on lifting, lowering, front, back
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     */
    public double update (double target, double state) {
        double result;
        // Front is 0 to +180 degrees, back is 0 to -180 degrees
        if(state >= 0) {
            // Front
            if(target >= state) {
                // Lowering
                pidCurrent = pidLower;
                kStaticCurrent = kStaticLower;
                ksinCurrent = ksinLower;
            } else {
                // Lifting
                pidCurrent = pidLift;
                kStaticCurrent = kStaticLift;
                ksinCurrent = ksinLift;
            }
        } else {
            // Back
            if(target > state) {
                // Lifting
                pidCurrent = pidLift;
                kStaticCurrent = kStaticLift;
                ksinCurrent = ksinLift;
            } else {
                // Lowering
                pidCurrent = pidLower;
                kStaticCurrent = kStaticLower;
                ksinCurrent = ksinLower;
            }
        }
        result = pidCurrent.update(target, state) + Math.signum(-pidCurrent.error) *
                (kStaticCurrent + ksinCurrent * sin(toRadians(state)));
        return result;
    }

    /**
     * Reset the PID for a new target
     */
    public void reset() {
        pidLift.reset();
        pidLower.reset();
    }
}
