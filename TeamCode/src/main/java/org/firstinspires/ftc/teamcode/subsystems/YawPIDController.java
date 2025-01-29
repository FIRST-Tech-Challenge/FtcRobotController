package org.firstinspires.ftc.teamcode.subsystems;

public class YawPIDController {
    private final double kp;
    private final double ki;
    private final double kd;
    private double previousError = 0;
    private double integral = 0;

    public YawPIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double getCorrection(double targetYaw, double currentYaw) {
        // Calculate the error
        double error = targetYaw - currentYaw;

        while (error > 180) {
            error -= 360;
        }
        while (error < -180) {
            error += 360;
        }
        // P
        double proportional = kp * error;
        // I
        integral += ki * error;
        // D
        double derivative = kd * (error - previousError);

        double correction = proportional + integral + derivative;

        previousError = error;

        return correction;
    }
    public void reset() {
        previousError = 0;
        integral = 0;
    }
}
