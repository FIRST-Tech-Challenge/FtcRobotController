package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PIDFMotorController {

    private PIDController pidController;
    public DcMotorEx motor;

    private double p, i, d, f;
    private final double ticksInDegrees;

    public PIDFMotorController(DcMotorEx motor, double p, double i, double d, double f, double ticksInDegrees) {
        this.motor = motor;
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.ticksInDegrees = ticksInDegrees;

        // Initialize the PIDController with the provided PID values
        this.pidController = new PIDController(p, i, d);
    }

    /**
     * Updates the PIDF values, useful if you want to adjust them dynamically
     */
    public void setPIDF(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.pidController.setPID(p, i, d);
    }

    /**
     * Drives the motor to the specified target position using PIDF control.
     * @param targetPosition Target position in ticks.
     */
    public void driveToPosition(int targetPosition) {
        int currentPosition = motor.getCurrentPosition();

        // Calculate the PID output
        double pidOutput = pidController.calculate(currentPosition, targetPosition);

        // Calculate feed-forward component
        double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees)) * f;

        // Sum PID and feed-forward for final motor power
        double power = pidOutput + ff;

        // Set motor power
        motor.setPower(power);
    }
}
