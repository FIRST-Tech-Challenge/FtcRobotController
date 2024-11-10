package com.kalipsorobotics.PID;

import android.os.SystemClock;

public class PIDController {
    private final double Kp;
    private final double Ki;
    private final double Kd;

    private double errorIntegral;
    private double lastError;
    private double lastTime;

    public PIDController(double P, double I, double D) {
        this.Kp = P;
        this.Ki = I;
        this.Kd = D;

        this.lastError = 0;
        this.lastTime = SystemClock.elapsedRealtimeNanos();
    }

    public double calculate(double current, double target) {
        double currentTime = SystemClock.elapsedRealtimeNanos();
        double timeDelta = currentTime - lastTime;

        double error = target - current;

        double proportional = Kp * error;
        double integral = Ki * error * timeDelta;
        double derivative = Kd * (error - lastError) / timeDelta;

        lastTime = currentTime;
        lastError = error;

        return proportional + integral + derivative;
    }

}
