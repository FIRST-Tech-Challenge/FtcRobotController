package org.firstinspires.ftc.team417_CENTERSTAGE;


public class SimplePIDF {
    private double P;
    private double I;
    private double D;
    private double F;

    private double maxI = 1;
    private double minI = -1;
    private double maxOutput = 1;
    private double minOutput = -1;
    private boolean continuous = false;

    private double error = 0;
    private double integral = 0;
    private double derivative = 0;
    private double feedforward = 0;

    private double lastInput = Double.NaN;

    public SimplePIDF(double P, double I, double D, double F) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.F = F;
    }

    public double calculate(double targetPosition) {
        double currentPosition = getCurrentPosition();
        error = targetPosition - currentPosition;

        // Integral windup prevention
        if (continuous) {
            if (Math.abs(error) > (maxI - minI) / 2) {
                error = (error > 0) ? error - maxI + minI : error + maxI - minI;
            }
        }

        double dt = 0.02; // Assume a constant time step for simplicity
        integral += error * dt;

        derivative = (lastInput == Double.NaN) ? 0 : (currentPosition - lastInput) / dt;

        feedforward = F * targetPosition;

        double output = P * error + I * integral + D * derivative + feedforward;

        // Output saturation
        if (output > maxOutput) {
            output = maxOutput;
        } else if (output < minOutput) {
            output = minOutput;
        }

        lastInput = currentPosition;
        return output;
    }

    private double getCurrentPosition() {
        // Assume some way to get the current position, replace this with your actual implementation
        return 0;
    }

    public static void main(String[] args) {
        // Example usage
        SimplePIDF pidfController = new SimplePIDF(0.1, 0.01, 0.05, 0.2);

        double targetPosition = 10.0;

        for (int i = 0; i < 100; i++) {
            double output = pidfController.calculate(targetPosition);
            // Apply the output to your system, replace this with your actual implementation
            System.out.println("Output: " + output);
        }
    }
}