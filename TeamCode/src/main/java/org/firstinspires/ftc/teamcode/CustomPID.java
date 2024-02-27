package org.firstinspires.ftc.teamcode;

public class CustomPID{
    private final double kp;
    private final double ki;
    private final double kd;
    private double setpoint;
    private double integral = 0;
    private double prevError = 0;

    private long lastTime; // Time of the previous iteration in nanoseconds

    public CustomPID(double[] PidConstants) {
        this.kp = PidConstants[0];
        this.ki = PidConstants[1];
        this.kd = PidConstants[2];
        this.lastTime = System.nanoTime();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double[] calculateGivenRaw(double processVariable) {
        long currentTime = System.nanoTime();
        double loopTime = (currentTime - lastTime) / 1e9; // Convert to seconds
        lastTime = currentTime;

        // Calculate error
        double error = setpoint - processVariable;

        // Proportional term
        double proportional = kp * error;


        // Integral term
        integral += error * loopTime; // Adjust for loop time
        double integralMin = -0.2;
        double integralMax = 0.2;
        integral = clamp(integral, integralMin, integralMax); // Apply clamping
        double integralTerm = ki * integral;

        // Derivative term
        double derivative = kd * (error - prevError) / loopTime; // Adjust for loop time
        prevError = error;

        // Calculate the raw control signal
        double rawControlSignal = proportional + integralTerm + derivative;

        // Scale the output to fit within the specified range

        double outputMin = -0.8;
        double outputMax = 0.8;
        return new double[]{scaleToRange(rawControlSignal, outputMin, outputMax), proportional, integralTerm, derivative};
    }
    public double[] calculateGivenError(double error) {
        long currentTime = System.nanoTime();
        double loopTime = (currentTime - lastTime) / 1e9; // Convert to seconds
        lastTime = currentTime;

        // Proportional term
        double proportional = kp * error;
        // Integral term
        integral += error * loopTime; // Adjust for loop time
        double integralMin = -0.2;
        double integralMax = 0.2;
        integral = clamp(integral, integralMin, integralMax); // Apply clamping
        double integralTerm = ki * integral;

        // Derivative term
        double derivative = kd * (error - prevError) / loopTime; // Adjust for loop time
        prevError = error;

        // Calculate the raw control signal
        double rawControlSignal = proportional + integralTerm + derivative;

        // Scale the output to fit within the specified range

        double outputMin = -0.8;
        double outputMax = 0.8;
        return new double[]{scaleToRange(rawControlSignal, outputMin, outputMax), proportional, integralTerm, derivative};
    }

    private double scaleToRange(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

}
