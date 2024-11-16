package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PIDFMotorController {

    private final PIDController pidController;
    private final DcMotorEx motor;
    private final double f;
    private final double ticksInDegrees;
    private double maxSpeed;
    private final int initialPositionForFF;
    private Integer targetPosition = null;

    public PIDFMotorController(DcMotorEx motor, double p, double i, double d, double f, double ticksInDegrees, double maxSpeed) {
        this.motor = motor;
        this.f = f;
        this.ticksInDegrees = ticksInDegrees;
        this.maxSpeed = maxSpeed;
        this.initialPositionForFF = 0;

        this.pidController = new PIDController(p, i, d);
    }

    public PIDFMotorController(DcMotorEx motor, double p, double i, double d, double f, double ticksInDegrees, double maxSpeed, int initialPositionForFF) {
        this.motor = motor;
        this.f = f;
        this.ticksInDegrees = ticksInDegrees;
        this.maxSpeed = maxSpeed;
        this.initialPositionForFF = initialPositionForFF;

        this.pidController = new PIDController(p, i, d);
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void resetMotorEncoder(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getCurrentPosition(){
        return motor.getCurrentPosition();
    }

    public void setMaxSpeed(double maxSpeed){
        this.maxSpeed = maxSpeed;
    }

    public MotorData runIteration() {
        if (targetPosition == null){
            int currentPosition = motor.getCurrentPosition();
            return new MotorData(0, currentPosition, 0);
        }

        int currentPosition = motor.getCurrentPosition();

        // Calculate the PID output
        double pidOutput = pidController.calculate(currentPosition, targetPosition);

        // Calculate feed-forward component
        double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegrees) - Math.toRadians(initialPositionForFF) + Math.toRadians(90)) * f;

        // Sum PID and feed-forward for final motor power
        double power = pidOutput + ff;
        power = limitPower(power);

        // Set motor power
        motor.setPower(power);
        return new MotorData(targetPosition, currentPosition, power);
    }

    private double limitPower(double power){
        if (power > maxSpeed){
            return maxSpeed;
        }
        return Math.max(power, -maxSpeed);
    }

    public class MotorData{
        public double TargetPosition;
        public double CurrentPosition;
        public double SetPower;
        public MotorData(double targetPosition,double currentPosition, double setPower){
            this.TargetPosition = targetPosition;
            this.CurrentPosition = currentPosition;
            this.SetPower = setPower;
        }
    }
}
