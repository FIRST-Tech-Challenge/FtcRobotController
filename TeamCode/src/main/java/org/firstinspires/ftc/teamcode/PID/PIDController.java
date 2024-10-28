package org.firstinspires.ftc.teamcode.PID;

import android.os.SystemClock;

public class PIDController {
    private final double Kp;
    private final double Kd;

    private double lastError;
    private double lastTime;

    public PIDController(double P, double D) {
        this.Kp = P;
        this.Kd = D;

        this.lastError = 0;
        this.lastTime = SystemClock.elapsedRealtimeNanos();
    }

    public double calculate(double current, double target) {
        double currentTime = SystemClock.elapsedRealtimeNanos();
        double timeDelta = currentTime - lastTime;
        double error = target - current;

        double proportional = Kp * error;
        double derivative = Kd * (error - lastError) / timeDelta;

        lastTime = currentTime;
        lastError = error;

        return proportional + derivative;
    }

}
