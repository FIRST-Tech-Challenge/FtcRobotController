package org.firstinspires.ftc.teamcode.NewStuff.Localization;

import android.os.SystemClock;

public class PidNav {
    private final double P;
    private final double I;
    private final double D;

    private double errorIntegral = 0;
    private double lastTime = SystemClock.elapsedRealtimeNanos();
    private double lastError = 0;

    public PidNav(double P, double I, double D){
        this.P = P;
        this.I = I;
        this.D = D;
    }

    public double getPower(double error){
        double currentTime = SystemClock.elapsedRealtimeNanos();
        double deltaTime = currentTime - lastTime;

        errorIntegral += error * deltaTime;
        double errorDerivative = (error - lastError)/deltaTime;

        lastTime = currentTime;
        lastError = error;
        return P * error + I * errorIntegral + D * errorDerivative;
    }
}