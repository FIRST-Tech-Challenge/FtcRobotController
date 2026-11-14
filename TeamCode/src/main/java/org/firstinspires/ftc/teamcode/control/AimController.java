package org.firstinspires.ftc.teamcode.control;

import org.firstinspires.ftc.teamcode.AimConfig;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/*
 * Turns the smoothed Limelight tx/ty into drive powers. Pure logic, no hardware.
 * Call calculate() every loop, then read forward() and turn().
 * Values are in Drivetrain.drive() terms: forward + is forward, turn + is clockwise.
 */
public class AimController {

    private double forward;
    private double turn;

    /**
     * @param txDegrees       smoothed horizontal angle to the tag cluster
     * @param tyDegrees       smoothed vertical angle to the tag cluster
     * @param turnRateRadPerS robot turn rate from Odometry (counterclockwise +), used for damping
     */
    public void calculate(double txDegrees, double tyDegrees, double turnRateRadPerS) {
        double yawError = MathUtil.deadband(txDegrees, AimConfig.YAW_DEADBAND);
        double axialError = MathUtil.deadband(tyDegrees, AimConfig.AXIAL_DEADBAND);

        turn = MathUtil.clamp(yawError * AimConfig.YAW_GAIN
                        + turnRateRadPerS * AimConfig.ROTATION_VELOCITY_GAIN,
                -AimConfig.MAX_YAW, AimConfig.MAX_YAW);

        forward = MathUtil.clamp((-axialError + AimConfig.AXIAL_ADJUST) * AimConfig.AXIAL_GAIN,
                -AimConfig.MAX_AXIAL, AimConfig.MAX_AXIAL);
    }

    public double forward() {
        return forward;
    }

    public double turn() {
        return turn;
    }

    /** True when both powers are small enough that the robot is lined up. */
    public boolean isAimed() {
        return Math.abs(forward) <= AimConfig.AIMED_POWER && Math.abs(turn) <= AimConfig.AIMED_POWER;
    }
}
