package org.firstinspires.ftc.team5898;

public class PIDController {
    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;
    private double maxOutput = 1.0; // Maximum motor power output

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setMaxOutput(double max) {
        this.maxOutput = max;
    }

    public double calculate(double target, double current) {
        double error = target - current;
        integralSum += error;
        double derivative = error - lastError;
        lastError = error;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);
        return Math.max(-maxOutput, Math.min(maxOutput, output)); // Clamp output
    }
}