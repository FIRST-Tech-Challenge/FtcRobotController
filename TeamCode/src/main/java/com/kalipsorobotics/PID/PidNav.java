package com.kalipsorobotics.PID;

import android.os.SystemClock;

import androidx.annotation.NonNull;

public class PidNav {


    private double P;
    private double I;
    private double D;

    private double errorIntegral = 0;
    private double lastTime = System.currentTimeMillis();
    private double lastError = 0;

    public PidNav(double P, double I, double D){
        this.P = P;
        this.I = I;
        this.D = D;
    }

    public double getPower(double error){
        double currentTime = System.currentTimeMillis();
        double deltaTime = currentTime - lastTime;

        errorIntegral += error * (deltaTime/100);
        double errorDerivative = (error - lastError)/deltaTime;

        lastTime = currentTime;
        lastError = error;
        return (P * error) + (I * errorIntegral) + (D * errorDerivative);
    }

    public double getP() {
        return P;
    }
    public double getI() {
        return I;
    }
    public double getD() {
        return D;
    }
    public void setP(double p) {
        P = p;
    }
    public void setI(double i) {
        I = i;
    }
    public void setD(double d) {
        D = d;
    }

    public void setErrorIntegral(double errorIntegral) {
        this.errorIntegral = errorIntegral;
    }

    @Override
    public String toString() {
        return "PidNav{" +
                "D=" + D +
                ", I=" + I +
                ", P=" + P +
                '}';
    }
}