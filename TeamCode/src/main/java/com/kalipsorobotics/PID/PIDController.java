package com.kalipsorobotics.PID;

import android.annotation.SuppressLint;
import android.os.SystemClock;

import androidx.annotation.NonNull;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;

    private double integralError;
    private double lastError;
    private double lastTime;

    private final String name;

    public PIDController(double P, double I, double D, String controllerName) {
        Kp = P;
        Ki = I;
        Kd = D;

        integralError = 0;
        lastError = 0;
        lastTime = SystemClock.elapsedRealtimeNanos();

        name = controllerName;
    }

    public void reset() {
        integralError = 0;
        lastError = 0;
        lastTime = SystemClock.elapsedRealtimeNanos();
    }

    public double calculate(double current, double target) {
        double error = target - current;
        return calculate(error);
    }

    public double calculate(double error) {
        double currentTime = SystemClock.elapsedRealtimeNanos();
        double timeDelta = (currentTime - lastTime) / 1e9;
        integralError += error * timeDelta;

        double proportional = Kp * error;
        double integral = Ki * integralError;
        double derivative = Kd * (error - lastError) / timeDelta;

        lastTime = currentTime;
        lastError = error;

        return proportional + integral + derivative;
    }

    public double chKp(double delta) {
        return Kp += delta;
    }

    public double chKi(double delta) {
        return Ki += delta;
    }

    public double chKd(double delta) {
        return Kd += delta;
    }

    public double setKp(double val) {
        return Kp = val;
    }

    public double setKi(double val) {
        return Ki = val;
    }

    public double setKd(double val) {
        return Kd = val;
    }

    public double getKp() {
        return Kp;
    }

    public double getKi() {
        return Ki;
    }

    public double getKd() {
        return Kd;
    }

    public String getName() {
        return name;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format("%s with Kp %f, Ki %f, Kd %f", name, Kp, Ki, Kd);
    }
}
