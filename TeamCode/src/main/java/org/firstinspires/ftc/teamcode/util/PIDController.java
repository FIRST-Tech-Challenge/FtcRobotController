package org.firstinspires.ftc.teamcode.util;

public class PIDController {

    private double kP;
    private double kI;
    private double kD;

    private double integralSum = 0;
    private double lastError = 0;

    private double outputMin = -1;
    private double outputMax = 1;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setOutputLimits(double min, double max) {
        outputMin = min;
        outputMax = max;
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
    }

    public double calculate(double error) {
        integralSum += error;
        double derivative = error - lastError;
        lastError = error;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);
        output = Math.max(outputMin, Math.min(outputMax, output));
        return output;
    }
}
