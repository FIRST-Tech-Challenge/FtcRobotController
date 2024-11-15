package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PIDFMotorController {

    private final PIDController pidController;
    private final DcMotorEx motor;
    private final double f;
    private final double ticksInDegrees;
    private final double maxSpeed;
    private int targetPosition;

    public PIDFMotorController(DcMotorEx motor, double p, double i, double d, double f, double ticksInDegrees, double maxSpeed) {
        this.motor = motor;
        this.f = f;
        this.ticksInDegrees = ticksInDegrees;
        this.maxSpeed = maxSpeed;

        this.pidController = new PIDController(p, i, d);
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void resetMotorEncoder(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double runIteration() {
        int currentPosition = motor.getCurrentPosition();

        // Calculate the PID output
        double pidOutput = pidController.calculate(currentPosition, targetPosition);

        // Calculate feed-forward component
        double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees)) * f;

        // Sum PID and feed-forward for final motor power
        double power = pidOutput + ff;
        power = limitPower(power);

        // Set motor power
        motor.setPower(power);
        return power;
    }

    private double limitPower(double power){
        if (power > maxSpeed){
            return maxSpeed;
        }
        return Math.max(power, -maxSpeed);
    }
}
