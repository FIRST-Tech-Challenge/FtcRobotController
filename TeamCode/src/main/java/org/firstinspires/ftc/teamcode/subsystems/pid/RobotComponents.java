package org.firstinspires.ftc.teamcode.subsystems.pid;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/** Initialize a motor as a RobotComponent with its values for PID functionality and extra features */
public class RobotComponents {
    private final DcMotorEx motor;
    private final double ticks_per_degree;
    private final double F;
    private final PIDController controller;
    private double target;

    /**
     * Constructor for RobotComponents
     *
     * @param motor              Motor Declaration
     * @param ticks_per_rotation Value of ticks for specific motor RPM. Can be found on the manufacture's website (GoBuilda, Rev, etc)
     * @param p                  The Proportional Value tuned after F (if applicable) until the motor can barely get to its target position. Typically under 0.1, but may be higher.
     * @param i                  The Integral Value tuned last, only if steady state error continues. Typically relatively unnecessary unless extreme precision is needed and typically of a value in the tenths of decimals. Be sure to re-tune D afterwards.
     * @param d                  The Derivative Value tuned after P, this value helps reduce oscillation in reaching a target and should result in a smoothed out curve when going to a target. Extremely small and sensitive, typically around 0.001. Re-tune after adjusting I.
     * @param f                  The Feed-forward value, tuned before anything else by increasing until the motor can hold itself against gravity at any position. Only use if you want a motor to hold its position against gravity and never use on locking systems like worm gears (in this case, set to 0)! Value is variable based on necessary motor power, but should be low (under 0.1) to avoid motor overheating and power draw.
     */
    RobotComponents(DcMotorEx motor, double ticks_per_rotation, double p, double i, double d, double f) {
        this.motor = motor;
        this.F = f;
        ticks_per_degree = ticks_per_rotation / 360.0;
        target = 0;

        controller = new PIDController(p, i, d);
    }

    /** Get the number of encoder ticks per degree */
    public double getTicksPerDegree() {
        return ticks_per_degree;
    }

    /** Get the current encoder target */
    public double getTarget() {
        return target;
    }

    /**
     * Set the encoder target to a new value
     *
     * @param newTarget The new encoder target value
     */
    public void setTarget(double newTarget) {
        target = newTarget;
    }

    /**
     * Modify the encoder target by a set value rather than resetting it, positive to increase, negative to decrease
     *
     * @param increment The value to increment (positively or negatively) the current encoder target
     */
    public void incrementTarget(double increment) {
        target += increment;
        moveUsingPID();
    }

    /**
     * Sends power to the RobotComponent's motor to smoothly and automatically reach currentTarget (or whatever its called, might be different) using the PID loop. Must be called every code loop!
     */
    public void moveUsingPID() {

        controller.reset();
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        int armPos = motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_per_degree)) * F;
        double power = pid + ff;

        motor.setPower(power);

    }

    /**
     * Returns true if the motor's current encoder position is within the specified range of currentTarget and false otherwise (e.g. true for a pos of 167 with a target of 170 and a range of 5, false for the same conditions and a pos of 200)
     *
     * @param range The permissible range of encoder positions relative to the current target (e.g. target of 170 with a range of 5 will accept positions from 165 to 175)
     */
    public boolean motorCloseEnough(int range) {
        return (target - range <= motor.getCurrentPosition()) && (target + range >= motor.getCurrentPosition());
    }
}