package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class customPID{
    private final double kp;
    private final double ki;
    private final double kd;
    //test
    private double setpoint;
    private double integral = 0;
    private double prevError = 0;

    private long lastTime; // Time of the previous iteration in nanoseconds

    public customPID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.lastTime = System.nanoTime();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double calculate(double processVariable) {
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
        return scaleToRange(rawControlSignal, outputMin, outputMax);
    }

    private double scaleToRange(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

}
