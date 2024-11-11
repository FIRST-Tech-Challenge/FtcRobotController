package com.kalipsorobotics.PID;

import android.os.SystemClock;

public class PIDController {
    private final double Kp;
    private final double Ki;
    private final double Kd;

    private double integralError;
    private double lastError;
    private double lastTime;

    public PIDController(double P, double I, double D) {
        Kp = P;
        Ki = I;
        Kd = D;

        integralError = 0;
        lastError = 0;
        lastTime = SystemClock.elapsedRealtimeNanos();
    }

    public double calculate(double current, double target) {
        double currentTime = SystemClock.elapsedRealtimeNanos();
        double timeDelta = (currentTime - lastTime) / 1e9;

        double error = target - current;
        integralError += error * timeDelta;

        double proportional = Kp * error;
        double integral = Ki * integralError;
        double derivative = Kd * (error - lastError) / timeDelta;

        lastTime = currentTime;
        lastError = error;

        return proportional + integral + derivative;
    }
}
